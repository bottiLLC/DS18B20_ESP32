#include <DS18B20.h>

/**
 * DS18B20 シンプル測定サンプル
 * 
 * ESP32-C3 の GPIO 直接制御を使用した高精度測定デモです。
 * 設定項目を変更することで、ピン番号や測定間隔を簡単に調整できます。
 */

// ============================================================
// 設定項目
// ============================================================
#define DQ_PIN 4              // DQ（データ線）を接続した GPIO ピン番号
#define EXPECTED_SENSORS 1    // 接続しているセンサーの予定数
#define MEASURE_INTERVAL 5000 // 測定間隔（ミリ秒）
// ============================================================

DS18B20 ds(DQ_PIN);

void setup() {
    Serial.begin(115200);
    while(!Serial) delay(10);
    
    Serial.println("\n========================================");
    Serial.println("DS18B20 高精度・レジスタ制御サンプル");
    Serial.println("========================================");

    // 1-Wire バスの初期化とデバイス探索
    if (ds.begin()) {
        int foundCount = ds.getDeviceCount();
        Serial.printf("探索完了: %d 台のセンサーが見つかりました。\n", foundCount);
        if (foundCount < EXPECTED_SENSORS) {
            Serial.printf("警告: 期待される数 (%d) よりも少ないです。配線を確認してください。\n", EXPECTED_SENSORS);
        }
    } else {
        Serial.println("エラー: センサーが1台も見つかりませんでした。");
    }
}

void loop() {
    int deviceCount = ds.getDeviceCount();
    
    if (deviceCount > 0) {
        float temps[deviceCount];
        
        // 全センサーの温度を一括読み取り（動的ポーリング待機内蔵）
        // 読み取りに成功したデバイスが1台以上あれば true が返ります
        if (ds.readAllTemperatures(temps)) {
            Serial.printf("[%lu] 測定結果:\n", millis() / 1000);
            
            for (int i = 0; i < deviceCount; i++) {
                // センサーの固有ID (64-bit ROM Address) の取得
                DS18B20::DeviceAddress addr;
                ds.getDeviceAddress(i, addr);
                
                // IDの表示
                Serial.print("  ID: ");
                for (int j = 0; j < 8; j++) {
                    Serial.printf("%02X", addr[j]);
                }
                
                // 温度の表示
                if (temps[i] != -999.0f) {
                    Serial.printf(" -> 温度: %.2f ℃\n", temps[i]);
                } else {
                    // 個別のエラー内容を表示
                    DS18B20::ErrorType err = ds.getLastError(i);
                    Serial.printf(" -> エラー: %s\n", DS18B20::getErrorString(err));
                }
            }
        } else {
            Serial.println("エラー: 温度データの読み取りに失敗しました。バスを確認してください。");
        }
    } else {
        Serial.println("センサーが接続されていません。再スキャンを試みます...");
        ds.begin();
    }

    Serial.println("----------------------------------------");
    
    // 指定した間隔だけ待機
    delay(MEASURE_INTERVAL);
}
