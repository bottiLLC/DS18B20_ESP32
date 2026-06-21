#include <DS18B20.h>

// 1-Wireバス接続用GPIOピン
#define ONE_WIRE_BUS_PIN 4

// 期待するDS18B20の接続台数
#define EXPECTED_DEVICE_COUNT 2

// DS18B20ライブラリのインスタンス作成
DS18B20 sensor(ONE_WIRE_BUS_PIN);

// タスクハンドル
TaskHandle_t xSensorTaskHandle = NULL;
TaskHandle_t xHeartbeatTaskHandle = NULL;

/**
 * @brief DS18B20温度センサー計測用のFreeRTOSタスク
 */
void vSensorTask(void *pvParameters) {
    Serial.println("[SensorTask] センサー計測タスクを開始しました。");
    Serial.printf("[SensorTask] 設定ピン: GPIO%d, 期待するセンサー台数: %d台\n", ONE_WIRE_BUS_PIN, EXPECTED_DEVICE_COUNT);
    
    // センサーバスの初期化と探索
    while (!sensor.begin()) {
        Serial.println("[SensorTask] 【警告】DS18B20センサーが見つかりません。5秒後に再スキャンします...");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    
    int deviceCount = sensor.getDeviceCount();
    Serial.printf("[SensorTask] 接続検出: %d 個のDS18B20センサーを検出しました。\n", deviceCount);
    
    // 期待する台数と検出数が一致しない場合の警告
    if (deviceCount != EXPECTED_DEVICE_COUNT) {
        Serial.printf("[SensorTask] 【警告】検出されたセンサー数(%d台)が期待する台数(%d台)と異なります。接続を確認してください。\n",
                      deviceCount, EXPECTED_DEVICE_COUNT);
    }
    
    // 3線モードにおける電源供給状態(VDD断線)の検知
    if (!sensor.checkPowerSupply()) {
        Serial.println("[SensorTask] 【重要警告】寄生電源モードが検出されました！");
        Serial.println("[SensorTask]          3線モードの電源線(VDD)が断線または接触不良を起こしている可能性があります。");
        Serial.println("[SensorTask]          これにより温度変換が失敗し、「85℃固定」等の異常出力になる恐れがあります。");
    } else {
        Serial.println("[SensorTask] [電源状態確認] すべてのセンサーが外部電源(3線モード)で正常に動作しています。");
    }
    
    // 各センサーのROMアドレスを出力
    for (int i = 0; i < deviceCount; i++) {
        DS18B20::DeviceAddress addr;
        sensor.getDeviceAddress(i, addr);
        Serial.printf("  センサー [%d] ROMコード: ", i);
        for (int j = 0; j < 8; j++) {
            Serial.printf("%02X", addr[j]);
        }
        Serial.println();
    }

    while (true) {
        Serial.println("[SensorTask] ==================================");
        Serial.println("[SensorTask] 温度計測トランザクションを開始します。");
        
        // 3線モードにおける電源供給状態(VDD断線)の常時監視
        if (!sensor.checkPowerSupply()) {
            Serial.println("[SensorTask] 【警告】寄生電源モードが検出されました。VDD電源線が断線・接触不良を起こしている可能性があります！");
        }

        // 1. 温度変換の開始 (RMTによる非同期パルス送信)
        if (sensor.startConversion()) {
            Serial.println("[SensorTask] 一括温度変換コマンド(Convert T)を送信しました。");
            Serial.println("[SensorTask] 変換待ち時間 (780ms) に入ります。(vTaskDelayでCPUを解放します)...");
            
            // データシート仕様に基づき、12bit解像度での温度変換時間(最大750ms)を満たす待機を
            // vTaskDelayで行います。この待機中、本タスクはBlocked状態になり、
            // ハートビートタスクやWi-Fiなどのシステムバックグラウンド処理にCPUが完全に割り当てられます。
            vTaskDelay(pdMS_TO_TICKS(780));
            
            Serial.println("[SensorTask] 待機時間が完了しました。各センサーから測定結果を読み出します。");
            
            // 2. センサー毎にスクラッチパッドを読み出し、温度と各種エラーを検証
            for (int i = 0; i < deviceCount; i++) {
                float temp = 0.0f;
                if (sensor.readTemperature(i, temp)) {
                    Serial.printf("[SensorTask] センサー [%d] 測定値: %.4f ℃\n", i, temp);
                } else {
                    // エラーハンドリング：詳細なエラー理由の出力
                    DS18B20::ErrorType err = sensor.getLastError(i);
                    Serial.printf("[SensorTask] 【エラー】センサー [%d] のデータ取得に失敗しました。原因: %s (エラーコード: %d)\n", 
                                  i, DS18B20::getErrorString(err), err);
                    
                    if (err == DS18B20::ERR_STUCK_85C) {
                        Serial.println("[SensorTask]   └─ [対策] 85℃はDS18B20の電源投入時初期値です。温度変換が正常に行われなかった、");
                        Serial.println("[SensorTask]             または測定中にセンサーへの給電が寸断されてリセットがかかった可能性があります。");
                    } else if (err == DS18B20::ERR_PARASITE_POWER) {
                        Serial.println("[SensorTask]   └─ [対策] VDDピンの配線が切断されているか浮いています。接続状態を再度確認してください。");
                    }
                }
            }
        } else {
            Serial.println("[SensorTask] 【エラー】温度変換開始コマンドの送信に失敗しました。");
        }
        
        Serial.println("[SensorTask] トランザクション完了。次回計測まで2秒間待機します。");
        Serial.println("[SensorTask] ==================================");
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/**
 * @brief RTOSの健全性を視覚化するためのハートビートタスク
 * 
 * 1-Wireの変換待ち時間（780ms）の間にもブロックされずに並行して動作し、
 * FreeRTOSの並行処理が正常に機能していることをシリアル出力で実証します。
 */
void vHeartbeatTask(void *pvParameters) {
    // 一般的なESP32モジュールを使用し、オンボードLEDなどが搭載されていない前提です。
    // そのためGPIO制御は行わず、シリアル出力のみで並行動作を確認します。
    while (true) {
        // タイムスタンプ付きでシリアルに生存シグナルを出力
        Serial.printf("[Heartbeat] システム生存中 / Tick: %lu ms\n", millis());
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // コンソールログが見やすくなるよう1秒間隔に変更
    }
}

void setup() {
    Serial.begin(115200);
    // ESP32のシリアル通信準備完了を少し待つ
    delay(1000);
    Serial.println("\n");
    Serial.println("==================================================");
    Serial.println(" ESP32 DS18B20 & FreeRTOS 制御プログラム");
    Serial.println("==================================================");
    Serial.println("※3線式(外部電源モード)専用設計。バスエラー検知＆診断機能付き。");
    Serial.println("※タイミング制御とvTaskDelay()により、RTOSへの悪影響を排除しています。");

    // 1. センサータスクの生成 (優先度: 2)
    xTaskCreate(
        vSensorTask,
        "SensorTask",
        4096,
        NULL,
        2,
        &xSensorTaskHandle
    );

    // 2. ハートビートタスクの生成 (優先度: 1)
    xTaskCreate(
        vHeartbeatTask,
        "HeartbeatTask",
        2048,
        NULL,
        1,
        &xHeartbeatTaskHandle
    );
}

void loop() {
    // loopタスクは使用せず、作成した各FreeRTOSタスクにすべての処理を委ねます。
    // CPU使用率を最小限に抑えるため、loopタスクはYieldし続けます。
    vTaskDelay(pdMS_TO_TICKS(1000));
}
