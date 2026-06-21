#include "DS18B20.h"
#include <string.h>

// データシート Figure 11 (CRC Generator) に基づく 1-Wire CRC-8 の計算ルーチン
// 生成多項式: X^8 + X^5 + X^4 + 1 (ビット反転表現: 0x8C)
static uint8_t calculateCRC8(const uint8_t *data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        uint8_t inbyte = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix) {
                crc ^= 0x8C;
            }
            inbyte >>= 1;
        }
    }
    return crc;
}

DS18B20::DS18B20(uint8_t pin) 
    : _pin(pin), _ow(NULL), _deviceCount(0), _parasitePowerDetected(false) 
{
    // スレッド安全性のためのミューテックスを作成
    _mutex = xSemaphoreCreateMutex();
    for (int i = 0; i < MAX_DEVICES; i++) {
        _lastErrors[i] = ERR_NONE;
    }
}

DS18B20::~DS18B20() {
    if (_ow) {
        delete _ow;
    }
    if (_mutex) {
        vSemaphoreDelete(_mutex);
    }
}

bool DS18B20::begin() {
    if (!_mutex) return false;

    if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
        _deviceCount = 0;
        _parasitePowerDetected = false;
        for (int i = 0; i < MAX_DEVICES; i++) {
            _lastErrors[i] = ERR_NONE;
        }

        // インスタンスがなければ生成する（再呼び出し時のヒープ断片化防止）
        if (!_ow) {
            _ow = new OneWireNg_CurrentPlatform(_pin, false);
        }
        
        if (_ow) {
            // バスリセットを試み、応答がなければ即座に終了する
            if (_ow->reset() != OneWireNg::EC_SUCCESS) {
                xSemaphoreGive(_mutex);
                return false;
            }

            // バス上のデバイスをスキャンしてDS18B20のみを抽出
            // OneWireNgのイテレータを使用
            for (const auto& id : *_ow) {
                if (_deviceCount >= MAX_DEVICES) break;

                // バイト0はファミリーコード。DS18B20 / DS18B20U+ は 0x28 (データシート参照)
                // 通信ノイズ対策として、ROMアドレスのCRC8も検証する
                if (id[0] == 0x28 && OneWireNg::checkCrcId(id) == OneWireNg::EC_SUCCESS) {
                    for (int i = 0; i < 8; i++) {
                        _devices[_deviceCount][i] = id[i];
                    }
                    _lastErrors[_deviceCount] = ERR_NONE;
                    _deviceCount++;
                }
            }

            // デバイスが検出された場合、電源供給モード（3線モード）の確認を行う
            if (_deviceCount > 0) {
                if (_ow->reset() == OneWireNg::EC_SUCCESS) {
                    _ow->addressAll();
                    _ow->writeByte(0xB4); // Read Power Supply コマンド
                    // 0が返ってきた場合は、少なくとも1台が寄生電源動作（VDDピン断線の疑い）
                    if (_ow->readBit() == 0) {
                        _parasitePowerDetected = true;
                    } else {
                        _parasitePowerDetected = false;
                    }
                }
            }
        }
        xSemaphoreGive(_mutex);
        return (_deviceCount > 0);
    }
    return false;
}

bool DS18B20::checkPowerSupply() {
    if (!_ow || !_mutex) return false;

    bool allExternal = true;
    if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
        if (_ow->reset() == OneWireNg::EC_SUCCESS) {
            _ow->addressAll();
            _ow->writeByte(0xB4);
            if (_ow->readBit() == 0) {
                _parasitePowerDetected = true;
                allExternal = false;
            } else {
                _parasitePowerDetected = false;
                allExternal = true;
            }
        } else {
            allExternal = false;
        }
        xSemaphoreGive(_mutex);
    }
    return allExternal;
}

bool DS18B20::startConversion() {
    if (!_ow || !_mutex) return false;

    if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
        // バスリセットを試み、失敗した場合は即座に終了する
        if (_ow->reset() != OneWireNg::EC_SUCCESS) {
            xSemaphoreGive(_mutex);
            return false;
        }

        // バス上のすべてのデバイスに対して一斉に温度変換を開始する
        // OneWireNgの最適化APIを使用してSkip ROMを実行
        _ow->addressAll();
        _ow->writeByte(0x44); // Convert T (データシート: 温度変換開始コマンド)
        
        xSemaphoreGive(_mutex);
        return true;
    }
    return false;
}

bool DS18B20::readTemperature(int romIndex, float &tempCelsius) {
    if (!_ow || !_mutex || romIndex < 0 || romIndex >= _deviceCount) return false;

    bool success = false;
    if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
        _lastErrors[romIndex] = ERR_NONE;

        // バスリセットを試み、失敗した場合は即座に終了する
        if (_ow->reset() != OneWireNg::EC_SUCCESS) {
            _lastErrors[romIndex] = ERR_BUS_RESET;
            xSemaphoreGive(_mutex);
            return false;
        }

        // OneWireNgの最適化APIを使用してMatch ROMを実行
        if (_ow->addressSingle(_devices[romIndex]) != OneWireNg::EC_SUCCESS) {
            _lastErrors[romIndex] = ERR_MATCH_ROM;
            xSemaphoreGive(_mutex);
            return false;
        }
        
        _ow->writeByte(0xBE); // Read Scratchpad (データシート: スクラッチパッド読出)

        // スクラッチパッドデータ (9バイト) を一括で読み出す (readBytesによる最適化)
        uint8_t scratchpad[9];
        _ow->readBytes(scratchpad, 9);

        // バスのショート判定（全バイト 0x00 の場合、バスがGNDに短絡していると判定）
        bool allZeros = true;
        for (int i = 0; i < 9; i++) {
            if (scratchpad[i] != 0x00) {
                allZeros = false;
                break;
            }
        }
        if (allZeros) {
            _lastErrors[romIndex] = ERR_SHORT_GND;
            xSemaphoreGive(_mutex);
            return false;
        }

        // バスの断線または未接続判定（全バイト 0xFF の場合、バスが浮いているか切断されていると判定）
        bool allOnes = true;
        for (int i = 0; i < 9; i++) {
            if (scratchpad[i] != 0xFF) {
                allOnes = false;
                break;
            }
        }
        if (allOnes) {
            _lastErrors[romIndex] = ERR_DISCONNECTED;
            xSemaphoreGive(_mutex);
            return false;
        }

        // データシート基準のCRC-8検証
        // 9バイトすべてのCRC計算結果が0になればデータ破損がないことを示す
        if (calculateCRC8(scratchpad, 9) != 0) {
            _lastErrors[romIndex] = ERR_CRC;
            xSemaphoreGive(_mutex);
            return false;
        }

        // スクラッチパッドのバイト0(LSB), バイト1(MSB), バイト4(CFG)を抽出して温度計算
        tempCelsius = calculateTemperature(scratchpad[0], scratchpad[1], scratchpad[4]);
        
        // 85℃固定出力問題（初期値エラーまたは未完了エラー）の検出と処理
        if (tempCelsius == 85.0f) {
            // スクラッチパッドのバイト6（予約領域）は、パワーオンリセット初期状態では 0x0C となっています。
            // 温度変換が一度も行われていないか、あるいは給電寸断でセンサーが再起動した場合は 0x0C のままです。
            // 一度でも温度変換が正常に行われると、バイト6は温度に応じた動的な値（カウント残値）に更新されます。
            // これにより、実際の「正常な85.0℃」と「起動直後/異常リセットの85.0℃」を正確に区別できます。
            if (scratchpad[6] == 0x0C) {
                if (_parasitePowerDetected) {
                    _lastErrors[romIndex] = ERR_PARASITE_POWER;
                } else {
                    _lastErrors[romIndex] = ERR_STUCK_85C;
                }
                xSemaphoreGive(_mutex);
                return false;
            }
        }

        success = true;
        xSemaphoreGive(_mutex);
    }
    return success;
}

DS18B20::ErrorType DS18B20::getLastError(int romIndex) const {
    if (romIndex >= 0 && romIndex < _deviceCount) {
        return _lastErrors[romIndex];
    }
    return ERR_NOT_FOUND;
}

const char* DS18B20::getErrorString(ErrorType err) {
    switch (err) {
        case ERR_NONE:
            return "正常 (エラーなし)";
        case ERR_NOT_FOUND:
            return "デバイスが見つかりません";
        case ERR_BUS_RESET:
            return "バスリセット失敗 (応答なし)";
        case ERR_MATCH_ROM:
            return "Match ROM コマンド送信失敗";
        case ERR_SHORT_GND:
            return "DQラインGNDショート検知 (全0x00)";
        case ERR_DISCONNECTED:
            return "断線または未接続 (全0xFF)";
        case ERR_CRC:
            return "CRCエラー (受信データ破損)";
        case ERR_STUCK_85C:
            return "85℃初期値固定エラー (未変換または電源寸断によるリセット)";
        case ERR_PARASITE_POWER:
            return "寄生電源検知 (3線モードでのVDD未接続・浮き)";
        default:
            return "未知のエラー";
    }
}

bool DS18B20::readAllTemperatures(float *tempArray) {
    if (!tempArray || _deviceCount == 0) return false;

    // 1. 温度変換開始 (非同期コマンド送信)
    if (!startConversion()) return false;

    // 2. データシートに基づく変換待機
    // 12bit解像度時の最大変換時間は 750ms。RTOSのタスクを yield して他のタスクに時間を与える。
    // マージンをとって 780ms 待機。
    vTaskDelay(pdMS_TO_TICKS(780));

    // 3. 各センサーから順番にデータを読み出す
    bool atLeastOneSuccess = false;
    for (int i = 0; i < _deviceCount; i++) {
        float temp = -999.0f;
        if (readTemperature(i, temp)) {
            tempArray[i] = temp;
            atLeastOneSuccess = true;
        } else {
            tempArray[i] = -999.0f; // エラー値
        }
    }
    return atLeastOneSuccess;
}

void DS18B20::getDeviceAddress(int romIndex, DeviceAddress destAddress) const {
    if (romIndex >= 0 && romIndex < _deviceCount) {
        memcpy(destAddress, _devices[romIndex], 8);
    } else {
        memset(destAddress, 0, 8);
    }
}

float DS18B20::calculateTemperature(uint8_t lsb, uint8_t msb, uint8_t cfg) {
    // データシート Figure 4 (Temperature Register Format) に基づく算出
    // LSBとMSBを合わせた16bit符号付き2の補数表現を取得
    int16_t raw = (msb << 8) | lsb;

    // デバイス解像度（設定レジスタのビット6:5 (R1, R0)）に基づき未定義ビットをマスク
    // 00: 9-bit (下位3ビット未定義)
    // 01: 10-bit (下位2ビット未定義)
    // 10: 11-bit (下位1ビット未定義)
    // 11: 12-bit (全ビット有効)
    uint8_t res = (cfg >> 5) & 0x03;
    if (res == 0x00) {
        raw &= ~0x07; // 下位3ビットをクリア
    } else if (res == 0x01) {
        raw &= ~0x03; // 下位2ビットをクリア
    } else if (res == 0x02) {
        raw &= ~0x01; // 下位1ビットをクリア
    }

    // 12bit解像度時の重みは 1/16℃ (0.0625℃) (データシート Table 1)
    // 16.0f で割ることで小数点以下の温度を正しく復元
    return (float)raw / 16.0f;
}
