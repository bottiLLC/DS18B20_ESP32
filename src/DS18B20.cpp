#include "DS18B20.h"
#include <string.h>

// ESP32のタイミングにクリティカルな処理のためのスピンロック
static portMUX_TYPE owMux = portMUX_INITIALIZER_UNLOCKED;

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
    : _pin(pin), _deviceCount(0), _parasitePowerDetected(false) 
{
    // スレッド安全性のためのミューテックスを作成
    _mutex = xSemaphoreCreateMutex();
    for (int i = 0; i < MAX_DEVICES; i++) {
        _lastErrors[i] = ERR_NONE;
    }
}

DS18B20::~DS18B20() {
    if (_mutex) {
        vSemaphoreDelete(_mutex);
    }
}

// 1-Wire 低レベル通信関数

void DS18B20::ow_write_bit(uint8_t bit) {
    portENTER_CRITICAL(&owMux);
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
    if (bit) {
        delayMicroseconds(5);  // Write 1: LOW 5us
        pinMode(_pin, INPUT);  // リリース (プルアップでHIGH)
        delayMicroseconds(55); // 残りの時間を待機
    } else {
        delayMicroseconds(60); // Write 0: LOW 60us
        pinMode(_pin, INPUT);  // リリース
        delayMicroseconds(5);  // 回復時間
    }
    portEXIT_CRITICAL(&owMux);
}

uint8_t DS18B20::ow_read_bit() {
    uint8_t bit = 0;
    portENTER_CRITICAL(&owMux);
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
    delayMicroseconds(2);      // Read slot: LOW 2us
    pinMode(_pin, INPUT);      // リリース
    delayMicroseconds(10);     // 10us待機してサンプリング (合計12us時点)
    if (digitalRead(_pin)) {
        bit = 1;
    }
    delayMicroseconds(48);     // タイムスロットの残りを待機 (合計60us)
    portEXIT_CRITICAL(&owMux);
    return bit;
}

void DS18B20::ow_write_byte(uint8_t byte) {
    for (uint8_t i = 0; i < 8; i++) {
        ow_write_bit(byte & 0x01);
        byte >>= 1;
    }
}

uint8_t DS18B20::ow_read_byte() {
    uint8_t byte = 0;
    for (uint8_t i = 0; i < 8; i++) {
        byte >>= 1;
        if (ow_read_bit()) {
            byte |= 0x80;
        }
    }
    return byte;
}

bool DS18B20::ow_reset() {
    uint8_t presence = 0;
    
    // リセットパルス: LOW 480us
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
    delayMicroseconds(480);
    
    // バスをリリースしてデバイスの応答を待つ
    portENTER_CRITICAL(&owMux);
    pinMode(_pin, INPUT);
    delayMicroseconds(70); // デバイスは15〜60us以内にDQをLOWに引くので、70us時点で確認
    if (!digitalRead(_pin)) {
        presence = 1;
    }
    portEXIT_CRITICAL(&owMux);
    
    delayMicroseconds(410); // 残りの復旧時間 (合計480us)
    return (presence == 1);
}

// 外部ライブラリ依存ゼロで全バス異常を判定するリセット診断ルーチン
bool DS18B20::ow_reset_with_diagnosis(ErrorType &busError) {
    busError = ERR_NONE;
    
    // 1. 事前のバスレベル確認（プルアップによりHIGHになっているはず）
    pinMode(_pin, INPUT);
    delayMicroseconds(10);
    if (!digitalRead(_pin)) {
        // すでにLOWになっている ＝ バスGNDショートの可能性
        busError = ERR_SHORT_GND;
        return false;
    }

    // 2. リセットパルスの送信 (LOW 480us)
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
    delayMicroseconds(480);
    
    // 3. リリースし、ピンの立ち上がりをタイミング保護下で確認
    portENTER_CRITICAL(&owMux);
    pinMode(_pin, INPUT);
    
    // デバイスがプレゼンスを送信し始める前のタイミング(10us後)で
    // バスが一旦HIGHへ立ち上がったかを確認（これで永続的なGNDショートを検出）
    delayMicroseconds(10);
    if (!digitalRead(_pin)) {
        portEXIT_CRITICAL(&owMux);
        busError = ERR_SHORT_GND;
        return false;
    }
    
    // 4. プレゼンスパルス（デバイスがLOWに引く）のサンプリング
    // リリース開始から合計70us後（すでに10us経過しているため残り60us）
    delayMicroseconds(60); 
    uint8_t presence = digitalRead(_pin);
    portEXIT_CRITICAL(&owMux);
    
    // 5. リセットパルスのタイムスロット完了まで待機 (残り340us)
    delayMicroseconds(340);
    
    if (presence != 0) {
        // LOWに引くデバイスが存在しない ＝ センサー未接続または信号線断線
        busError = ERR_DISCONNECTED;
        return false;
    }
    
    return true;
}

void DS18B20::ow_select(const DeviceAddress addr) {
    ow_write_byte(0x55); // Match ROM
    for (int i = 0; i < 8; i++) {
        ow_write_byte(addr[i]);
    }
}

void DS18B20::ow_skip() {
    ow_write_byte(0xCC); // Skip ROM
}

// 1-Wire Search ROM アルゴリズムの実装
bool DS18B20::search(DeviceAddress &address, SearchState &state) {
    uint8_t id_bit_number = 1;
    uint8_t last_zero = 0;
    uint8_t rom_byte_number = 0;
    uint8_t rom_byte_mask = 1;
    bool search_result = false;
    uint8_t id_bit, cmp_id_bit;
    uint8_t search_direction;

    if (!state.last_device_flag) {
        if (!ow_reset()) {
            state.last_discrepancy = 0;
            state.last_device_flag = false;
            state.last_family_discrepancy = 0;
            return false;
        }

        ow_write_byte(0xF0); // SEARCH ROM

        do {
            id_bit = ow_read_bit();
            cmp_id_bit = ow_read_bit();

            if ((id_bit == 1) && (cmp_id_bit == 1)) {
                // デバイス応答なし
                break;
            } else {
                if (id_bit != cmp_id_bit) {
                    search_direction = id_bit;
                } else {
                    if (id_bit_number < state.last_discrepancy) {
                        search_direction = ((state.rom_number[rom_byte_number] & rom_byte_mask) > 0);
                    } else {
                        search_direction = (id_bit_number == state.last_discrepancy);
                    }

                    if (search_direction == 0) {
                        last_zero = id_bit_number;
                        if (last_zero < 9) {
                            state.last_family_discrepancy = last_zero;
                        }
                    }
                }

                if (search_direction == 1) {
                    state.rom_number[rom_byte_number] |= rom_byte_mask;
                } else {
                    state.rom_number[rom_byte_number] &= ~rom_byte_mask;
                }

                ow_write_bit(search_direction);

                id_bit_number++;
                rom_byte_mask <<= 1;

                if (rom_byte_mask == 0) {
                    rom_byte_number++;
                    rom_byte_mask = 1;
                }
            }
        } while (rom_byte_number < 8);

        if (!(id_bit_number < 65)) {
            state.last_discrepancy = last_zero;
            if (state.last_discrepancy == 0) {
                state.last_device_flag = true;
            }
            search_result = true;
        }
    }

    if (!search_result || (state.rom_number[0] == 0)) {
        state.last_discrepancy = 0;
        state.last_device_flag = false;
        state.last_family_discrepancy = 0;
        search_result = false;
    } else {
        for (int i = 0; i < 8; i++) {
            address[i] = state.rom_number[i];
        }
    }

    return search_result;
}

bool DS18B20::begin() {
    if (!_mutex) return false;

    if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
        _deviceCount = 0;
        _parasitePowerDetected = false;
        for (int i = 0; i < MAX_DEVICES; i++) {
            _lastErrors[i] = ERR_NONE;
        }

        // 初期状態でバスピンをプルアップの浮いた入力状態にする
        pinMode(_pin, INPUT);

        SearchState s_state;
        memset(&s_state, 0, sizeof(SearchState));
        s_state.last_discrepancy = 0;
        s_state.last_device_flag = false;
        s_state.last_family_discrepancy = 0;

        DeviceAddress addr;

        while (search(addr, s_state)) {
            if (_deviceCount >= MAX_DEVICES) break;

            // DS18B20ファミリーコード 0x28 の検証
            // 自前の CRC-8 算出で ROM の整合性を検査
            if (addr[0] == 0x28 && calculateCRC8(addr, 7) == addr[7]) {
                memcpy(_devices[_deviceCount], addr, 8);
                _lastErrors[_deviceCount] = ERR_NONE;
                _deviceCount++;
            }
        }

        // 電源供給モード（3線モード）の確認を行う
        if (_deviceCount > 0) {
            if (ow_reset()) {
                ow_skip();
                ow_write_byte(0xB4); // Read Power Supply コマンド
                // 0が返ってきた場合は、少なくとも1台が寄生電源動作
                if (ow_read_bit() == 0) {
                    _parasitePowerDetected = true;
                } else {
                    _parasitePowerDetected = false;
                }
            }
        }

        xSemaphoreGive(_mutex);
        return (_deviceCount > 0);
    }
    return false;
}

bool DS18B20::checkPowerSupply() {
    if (!_mutex) return false;

    bool allExternal = true;
    if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
        if (ow_reset()) {
            ow_skip();
            ow_write_byte(0xB4);
            if (ow_read_bit() == 0) {
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
    if (!_mutex) return false;

    if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
        if (!ow_reset()) {
            xSemaphoreGive(_mutex);
            return false;
        }

        ow_skip();
        ow_write_byte(0x44); // Convert T
        
        xSemaphoreGive(_mutex);
        return true;
    }
    return false;
}

// 3線式での温度変換中ポーリング判定処理 (Convert T後のステータス確認)
// 外部電源使用時、デバイスは温度変換中に 0 を出力し、完了すると 1 を出力し続けます。
bool DS18B20::isConversionComplete() {
    if (!_mutex) return false;
    
    bool complete = false;
    if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
        // リードタイムスロットを実行し、結果が1になれば完了
        complete = (ow_read_bit() == 1);
        xSemaphoreGive(_mutex);
    }
    return complete;
}

bool DS18B20::readTemperature(int romIndex, float &tempCelsius) {
    if (!_mutex || romIndex < 0 || romIndex >= _deviceCount) return false;

    bool success = false;
    if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
        _lastErrors[romIndex] = ERR_NONE;

        // 詳細なバス診断を伴うリセット
        ErrorType busErr = ERR_NONE;
        if (!ow_reset_with_diagnosis(busErr)) {
            _lastErrors[romIndex] = busErr;
            xSemaphoreGive(_mutex);
            return false;
        }

        ow_select(_devices[romIndex]);
        ow_write_byte(0xBE); // Read Scratchpad

        uint8_t scratchpad[9];
        for (int i = 0; i < 9; i++) {
            scratchpad[i] = ow_read_byte();
        }

        // バスのショート判定（全バイト 0x00 の場合）
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

        // バスの断線判定（全バイト 0xFF の場合）
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

        // --- 全エラー検知：データシート仕様に基づくスクラッチパッド予約領域・整合性チェック ---
        // 1. Byte 5 は Reserved (常に 0xFF) である必要があります
        if (scratchpad[5] != 0xFF) {
            _lastErrors[romIndex] = ERR_MEM_CORRUPT;
            xSemaphoreGive(_mutex);
            return false;
        }
        
        // 2. Byte 7 は Reserved (常に 0x10) である必要があります
        if (scratchpad[7] != 0x10) {
            _lastErrors[romIndex] = ERR_MEM_CORRUPT;
            xSemaphoreGive(_mutex);
            return false;
        }

        // 3. Byte 4 (Configuration Register) の下位5ビットは常に 11111 (0x1F) である必要があります (Figure 10参照)
        if ((scratchpad[4] & 0x1F) != 0x1F) {
            _lastErrors[romIndex] = ERR_MEM_CORRUPT;
            xSemaphoreGive(_mutex);
            return false;
        }

        // 4. データシート基準のCRC-8検証
        if (calculateCRC8(scratchpad, 8) != scratchpad[8]) {
            _lastErrors[romIndex] = ERR_CRC;
            xSemaphoreGive(_mutex);
            return false;
        }

        // スクラッチパッドから温度を算出
        tempCelsius = calculateTemperature(scratchpad[0], scratchpad[1], scratchpad[4]);
        
        // 5. ハードウェア測定限界値（-55.0℃ 〜 +125.0℃）の範囲外判定
        if (tempCelsius < -55.0f || tempCelsius > 125.0f) {
            _lastErrors[romIndex] = ERR_OUT_OF_RANGE;
            xSemaphoreGive(_mutex);
            return false;
        }

        // 6. 85℃初期値固着 (未変換または電源寸断によるリセット検知)
        if (tempCelsius == 85.0f) {
            // スクラッチパッドのバイト6（予約領域）は、パワーオンリセット初期状態では 0x0C となっています。
            // これにより、実際の「測定環境が正常に85.0℃」なのか「リセットによる85.0℃」かを切り分けます。
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
            return "DQラインGNDショート検知 (全0x00 / 応答レベル低)";
        case ERR_DISCONNECTED:
            return "断線または未接続 (全0xFF / プレゼンスパルスなし)";
        case ERR_CRC:
            return "CRCエラー (受信データ破損)";
        case ERR_STUCK_85C:
            return "85℃初期値固定エラー (未変換または電源寸断によるリセット)";
        case ERR_PARASITE_POWER:
            return "寄生電源検知 (3線モードでのVDD未接続・浮き)";
        case ERR_OUT_OF_RANGE:
            return "測定可能範囲外エラー (-55℃〜+125℃の範囲外値を検出)";
        case ERR_MEM_CORRUPT:
            return "スクラッチパッド破損・定義外ビット検出";
        case ERR_CONVERSION_TIMEOUT:
            return "温度変換タイムアウトエラー (3線式応答タイムアウト)";
        default:
            return "未知のエラー";
    }
}

// 3線式専用の動的な温度変換完了ポーリング待機処理
bool DS18B20::readAllTemperatures(float *tempArray) {
    if (!tempArray || _deviceCount == 0) return false;

    // 1. 温度変換開始
    if (!startConversion()) return false;

    // 2. 最大800msの間、10msごとに完了ステータスをポーリング
    // (通常、室温付近では750msよりも早く、およそ600ms〜700msで完了します)
    int timeout_ms = 800;
    int elapsed_ms = 0;
    bool completed = false;

    while (elapsed_ms < timeout_ms) {
        vTaskDelay(pdMS_TO_TICKS(10));
        elapsed_ms += 10;
        if (isConversionComplete()) {
            completed = true;
            break;
        }
    }

    // タイムアウトした場合はエラーを設定
    if (!completed) {
        for (int i = 0; i < _deviceCount; i++) {
            _lastErrors[i] = ERR_CONVERSION_TIMEOUT;
            tempArray[i] = -999.0f;
        }
        return false;
    }

    // 3. 各センサーから順番にデータを読み出す
    bool atLeastOneSuccess = false;
    for (int i = 0; i < _deviceCount; i++) {
        float temp = -999.0f;
        if (readTemperature(i, temp)) {
            tempArray[i] = temp;
            atLeastOneSuccess = true;
        } else {
            tempArray[i] = -999.0f;
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
    int16_t raw = (msb << 8) | lsb;

    uint8_t res = (cfg >> 5) & 0x03;
    if (res == 0x00) {
        raw &= ~0x07;
    } else if (res == 0x01) {
        raw &= ~0x03;
    } else if (res == 0x02) {
        raw &= ~0x01;
    }

    return (float)raw / 16.0f;
}
