/*
 * MIT License
 * 
 * Copyright (c) 2026 Yoshimasa Fujisawa
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "DS18B20.h"
#include <string.h>
#include "soc/gpio_struct.h"
#include "soc/gpio_reg.h"

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
    : _pin(pin), _deviceCount(0) 
{
    // スレッド安全性のためのミューテックスを作成
    _mutex = xSemaphoreCreateMutex();
    for (int i = 0; i < MAX_DEVICES; i++) {
        _lastErrors[i] = ERR_NONE;
        _parasitePowerDetected[i] = false;
    }
}

DS18B20::~DS18B20() {
    if (_mutex) {
        vSemaphoreDelete(_mutex);
    }
}

// 1-Wire 低レベル通信関数

void DS18B20::ow_write_bit(uint8_t bit) {
    const uint32_t pin_mask = (1UL << _pin);
    if (bit) {
        portENTER_CRITICAL(&owMux);
        GPIO.out_w1tc.val = pin_mask;     // 常にLOWに落とす準備
        GPIO.enable_w1ts.val = pin_mask;   // OUTPUT有効 (バスをLOWへ)
        delayMicroseconds(5);          // Write 1: LOW 5us
        GPIO.enable_w1tc.val = pin_mask;   // OUTPUT無効 (リリース、プルアップによりHIGHへ)
        portEXIT_CRITICAL(&owMux);
        delayMicroseconds(55);         // 残りの時間は割り込みを許可して待機
    } else {
        portENTER_CRITICAL(&owMux);
        GPIO.out_w1tc.val = pin_mask;     // 常にLOWに落とす準備
        GPIO.enable_w1ts.val = pin_mask;   // OUTPUT有効 (バスをLOWへ)
        delayMicroseconds(60);         // Write 0: LOW 60us
        GPIO.enable_w1tc.val = pin_mask;   // OUTPUT無効 (リリース)
        portEXIT_CRITICAL(&owMux);
        delayMicroseconds(5);          // 回復時間は割り込みを許可して待機
    }
}

uint8_t DS18B20::ow_read_bit() {
    uint8_t bit = 0;
    const uint32_t pin_mask = (1UL << _pin);
    portENTER_CRITICAL(&owMux);
    GPIO.out_w1tc.val = pin_mask;     // LOWに落とす準備
    GPIO.enable_w1ts.val = pin_mask;   // OUTPUT有効 (Read開始: バスをLOWへ)
    delayMicroseconds(2);          // Read slot: LOW 2us
    GPIO.enable_w1tc.val = pin_mask;   // OUTPUT無効 (リリース)
    delayMicroseconds(10);         // 10us待機してサンプリング (合計12us時点)
    if (GPIO.in.val & pin_mask) {      // ピン状態の直接読み込み
        bit = 1;
    }
    portEXIT_CRITICAL(&owMux);     // サンプリング直後に割り込みを許可
    delayMicroseconds(48);         // タイムスロットの残りを待機 (合計60us)
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
    ErrorType dummy;
    return ow_reset_with_diagnosis(dummy);
}

// 外部ライブラリ依存ゼロで全バス異常を判定するリセット診断ルーチン
bool DS18B20::ow_reset_with_diagnosis(ErrorType &busError) {
    busError = ERR_NONE;
    const uint32_t pin_mask = (1UL << _pin);
    
    // 1. 事前のバスレベル確認（プルアップによりHIGHになっているはず）
    GPIO.enable_w1tc.val = pin_mask;   // INPUTモード
    delayMicroseconds(10);
    if (!(GPIO.in.val & pin_mask)) {
        // すでにLOWになっている ＝ バスGNDショートの可能性
        busError = ERR_SHORT_GND;
        return false;
    }

    // 2. リセットパルスの送信 (LOW 480us)
    GPIO.out_w1tc.val = pin_mask;     // LOWレベル準備
    GPIO.enable_w1ts.val = pin_mask;   // OUTPUT有効
    delayMicroseconds(480);
    
    // 3. リリースし、ピンの立ち上がりをタイミング保護下で確認
    portENTER_CRITICAL(&owMux);
    GPIO.enable_w1tc.val = pin_mask;   // OUTPUT無効 (リリース)
    
    // デバイスがプレゼンスを送信し始める前のタイミング(10us後)で
    // バスが一旦HIGHへ立ち上がったかを確認（これで永続的なGNDショートを検出）
    delayMicroseconds(10);
    if (!(GPIO.in.val & pin_mask)) {
        portEXIT_CRITICAL(&owMux);
        busError = ERR_SHORT_GND;
        return false;
    }
    
    // 4. プレゼンスパルス（デバイスがLOWに引く）のサンプリング
    // リリース開始から合計70us後（すでに10us経過しているため残り60us）
    delayMicroseconds(60); 
    uint8_t presence = (GPIO.in.val & pin_mask) ? 1 : 0;
    portEXIT_CRITICAL(&owMux);
    
    // 5. リセットパルスのタイムスロット完了まで待機 (残り410us)
    delayMicroseconds(410);
    
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
                    }
                }

                if (search_direction == 1) {
                    state.rom_number[rom_byte_number] |= rom_byte_mask;
                } else {
                    state.rom_number[rom_byte_number] &= ~rom_byte_mask;
                }

                ow_write_bit(search_direction);

                id_bit_number++;
                if (id_bit_number >= 65) {
                    break;
                }
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
        search_result = false;
    } else {
        for (int i = 0; i < 8; i++) {
            address[i] = state.rom_number[i];
        }
    }

    return search_result;
}

bool DS18B20::begin_internal() {
    _deviceCount = 0;
    for (int i = 0; i < MAX_DEVICES; i++) {
        _lastErrors[i] = ERR_NONE;
        _parasitePowerDetected[i] = false;
    }

    // 初期状態でバスピンをプルアップの浮いた入力状態にする
    pinMode(_pin, INPUT);

    SearchState s_state;
    memset(&s_state, 0, sizeof(SearchState));
    s_state.last_discrepancy = 0;
    s_state.last_device_flag = false;

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

    // 各デバイスの電源供給モードを個別に確認
    for (int i = 0; i < _deviceCount; i++) {
        if (ow_reset()) {
            ow_select(_devices[i]);
            ow_write_byte(0xB4); // Read Power Supply コマンド
            if (ow_read_bit() == 0) {
                _parasitePowerDetected[i] = true;
                _lastErrors[i] = ERR_PARASITE_POWER; // 初期診断時に寄生電源ならエラーをセット
            } else {
                _parasitePowerDetected[i] = false;
            }
        }
    }

    return (_deviceCount > 0);
}

bool DS18B20::begin() {
    if (!_mutex) return false;
    bool success = false;
    if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
        success = begin_internal();
        xSemaphoreGive(_mutex);
    }
    return success;
}

bool DS18B20::checkPowerSupply_internal() {
    bool allExternal = true;
    for (int i = 0; i < _deviceCount; i++) {
        if (ow_reset()) {
            ow_select(_devices[i]);
            ow_write_byte(0xB4);
            if (ow_read_bit() == 0) {
                _parasitePowerDetected[i] = true;
                _lastErrors[i] = ERR_PARASITE_POWER;
                allExternal = false;
            } else {
                _parasitePowerDetected[i] = false;
            }
        } else {
            _lastErrors[i] = ERR_DISCONNECTED;
            allExternal = false;
        }
    }
    return allExternal;
}

bool DS18B20::checkPowerSupply() {
    if (!_mutex) return false;
    bool allExternal = false;
    if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
        allExternal = checkPowerSupply_internal();
        xSemaphoreGive(_mutex);
    }
    return allExternal;
}

bool DS18B20::startConversion_internal() {
    ErrorType busErr = ERR_NONE;
    if (!ow_reset_with_diagnosis(busErr)) {
        // バス診断失敗時はすべてのデバイスにエラーを設定
        for (int i = 0; i < _deviceCount; i++) {
            _lastErrors[i] = busErr;
        }
        return false;
    }

    ow_skip();
    ow_write_byte(0x44); // Convert T
    return true;
}

bool DS18B20::startConversion() {
    if (!_mutex) return false;
    bool success = false;
    if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
        success = startConversion_internal();
        xSemaphoreGive(_mutex);
    }
    return success;
}

bool DS18B20::isConversionComplete_internal() {
    return (ow_read_bit() == 1);
}

bool DS18B20::isConversionComplete() {
    if (!_mutex) return false;
    bool complete = false;
    if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
        complete = isConversionComplete_internal();
        xSemaphoreGive(_mutex);
    }
    return complete;
}

bool DS18B20::readTemperature_internal(int romIndex, float &tempCelsius) {
    if (romIndex < 0 || romIndex >= _deviceCount) return false;
    _lastErrors[romIndex] = ERR_NONE;

    // 詳細なバス診断を伴うリセット
    ErrorType busErr = ERR_NONE;
    if (!ow_reset_with_diagnosis(busErr)) {
        _lastErrors[romIndex] = busErr;
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
        return false;
    }

    // --- 全エラー検知：データシート仕様に基づくスクラッチパッド予約領域・整合性チェック ---
    // 1. Byte 5 は Reserved (常に 0xFF) である必要があります
    if (scratchpad[5] != 0xFF) {
        _lastErrors[romIndex] = ERR_MEM_CORRUPT;
        return false;
    }
    
    // 2. Byte 7 は Reserved (常に 0x10) である必要があります
    if (scratchpad[7] != 0x10) {
        _lastErrors[romIndex] = ERR_MEM_CORRUPT;
        return false;
    }

    // 3. Byte 4 (Configuration Register) の下位5ビットは常に 11111 (0x1F) である必要があります (Figure 10参照)
    if ((scratchpad[4] & 0x1F) != 0x1F) {
        _lastErrors[romIndex] = ERR_MEM_CORRUPT;
        return false;
    }

    // 4. データシート基準のCRC-8検証
    if (calculateCRC8(scratchpad, 8) != scratchpad[8]) {
        _lastErrors[romIndex] = ERR_CRC;
        return false;
    }

    // スクラッチパッドから温度を算出
    tempCelsius = calculateTemperature(scratchpad[0], scratchpad[1], scratchpad[4]);
    
    // 5. ハードウェア測定限界値（-55.0℃ 〜 +125.0℃）の範囲外判定
    if (tempCelsius < -55.0f || tempCelsius > 125.0f) {
        _lastErrors[romIndex] = ERR_OUT_OF_RANGE;
        return false;
    }

    // 6. 85℃初期値固着 (未変換または電源寸断によるリセット検知)
    if (tempCelsius == 85.0f) {
        // スクラッチパッドのバイト6（予約領域）は、パワーオンリセット初期状態では 0x0C となっています。
        // これにより、実際の「測定環境が正常に85.0℃」なのか「リセットによる85.0℃」かを切り分けます。
        if (scratchpad[6] == 0x0C) {
            // デバイス個別の電源供給モードをその場で再確認
            bool isParasite = false;
            if (ow_reset()) {
                ow_select(_devices[romIndex]);
                ow_write_byte(0xB4);
                if (ow_read_bit() == 0) {
                    isParasite = true;
                    _parasitePowerDetected[romIndex] = true;
                } else {
                    _parasitePowerDetected[romIndex] = false;
                }
            }
            
            if (isParasite) {
                _lastErrors[romIndex] = ERR_PARASITE_POWER;
            } else {
                _lastErrors[romIndex] = ERR_STUCK_85C;
            }
            return false;
        }
    }

    return true;
}

bool DS18B20::readTemperature(int romIndex, float &tempCelsius) {
    if (!_mutex) return false;
    bool success = false;
    if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
        success = readTemperature_internal(romIndex, tempCelsius);
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
    if (!tempArray || _deviceCount == 0 || !_mutex) return false;

    bool atLeastOneSuccess = false;
    if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
        // 1. 温度変換開始
        if (!startConversion_internal()) {
            xSemaphoreGive(_mutex);
            return false;
        }

        // 2. 最大800msの間、10msごとに完了ステータスをポーリング
        int timeout_ms = 800;
        int elapsed_ms = 0;
        bool completed = false;

        while (elapsed_ms < timeout_ms) {
            vTaskDelay(pdMS_TO_TICKS(10));
            elapsed_ms += 10;
            if (isConversionComplete_internal()) {
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
            xSemaphoreGive(_mutex);
            return false;
        }

        // 3. 各センサーから順番にデータを読み出す
        for (int i = 0; i < _deviceCount; i++) {
            float temp = -999.0f;
            if (readTemperature_internal(i, temp)) {
                tempArray[i] = temp;
                atLeastOneSuccess = true;
            } else {
                tempArray[i] = -999.0f;
            }
        }
        xSemaphoreGive(_mutex);
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
    uint16_t raw_u = (msb << 8) | lsb;

    uint8_t res = (cfg >> 5) & 0x03;
    if (res == 0x00) {
        raw_u &= ~0x07;
    } else if (res == 0x01) {
        raw_u &= ~0x03;
    } else if (res == 0x02) {
        raw_u &= ~0x01;
    }

    int16_t raw = (int16_t)raw_u;
    return (float)raw / 16.0f;
}
