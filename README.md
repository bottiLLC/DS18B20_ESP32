# DS18B20 for ESP32 Arduino Library

ESP32 の GPIO 直接制御（ビットバンギング）を使用した、外部ライブラリ依存関係のない FreeRTOS 向け非ブロッキング 1-Wire DS18B20温度センサーライブラリです。

## 概要
本ライブラリは、ESP32 などの FreeRTOS 環境において、温度変換（約750ms）に伴う待機時間を非ブロッキングで処理（`vTaskDelay` を利用）し、CPUリソースを他のタスクに解放することで、マルチタスク制御との親和性を高めたライブラリです。
また、堅牢性を重視し、バスの異常検知（GNDショート、断線、3線モードのVDD浮き）や、電源寸断による85℃固着（POR）状態を識別できる強力な自己診断機能を備えています。

## 公式データシート・製品情報
* **アナログ・デバイセズ（旧ダラス・マキシム）公式製品ページ**:
  [https://www.analog.com/jp/products/ds18b20.html](https://www.analog.com/jp/products/ds18b20.html)
  ※データシートや詳細な技術仕様は上記の公式サイトからご確認ください。

## 主な特徴
1. **FreeRTOS 非ブロッキング設計 & 3線式最適化**:
   * 温度変換の待機中（最大750ms以上）、固定時間の単純スリープではなく、**3線式（外部電源モード）ならではの機能である「温度変換ステータスの動的ポーリング」**を実装しています。変換が完了した瞬間に非ブロッキング（`vTaskDelay`併用）で復帰するため、測定時間を大幅に短縮できます。
2. **外部ライブラリへの依存なし (スタンドアロン)**:
   * 1-Wire 通信プロトコルおよび探索（Search ROM）アルゴリズムを自前で実装しているため、`OneWire` や `OneWireNg` などの外部ライブラリをインストールする必要はありません。
3. **自己診断および詳細エラー検知 (全エラー検出)**:
   * **バスレベル診断**: リセット時にバスの状態を段階的にサンプリングすることで、DQ線の「GNDへのショート」と「断線・プルアップ抵抗抜け」を明確に判別します。
   * **3線式（外部電源モード）確認**: センサーが安定した外部電源で動作しているかをチェックし、VDDの断線や接触不良（寄生電源モードへの望まないフォールバック）を検出します。
   * **スクラッチパッド整合性チェック**: データシート仕様に基づき、スクラッチパッドの予約領域（Byte 5 = 0xFF, Byte 7 = 0x10, Byte 4の下位5桁 = 0x1F）を毎回検証し、データの微細な破損を検知します。
   * **85℃固着（POR）検出**: 電源投入時初期値である `85℃` のまま値が変化しない不具合を検出し、リセットがかかった形跡を追跡します。
   * **測定限界値チェック**: ハードウェア仕様限界である `-55℃` 〜 `+125℃` を超える測定値を異常として検出します。

## 依存関係
* **なし**（本ライブラリ単体で動作します）

## 回路図（推奨接続）
本ライブラリは**3線式（外部電源モード）専用設計**です。
```text
      [ESP32]                      [DS18B20]
       GPIO Pin (例: 4) <----------> DQ (2pin)
                                     |
                                  [4.7kΩ Pull-up]
                                     |
       3.3V / 5.0V      <-----------> VDD (3pin)
       GND              <-----------> GND (1pin)
```

## クイックスタート

```cpp
#include <DS18B20.h>

#define ONE_WIRE_BUS_PIN 4  // 1-Wireバス接続GPIO

DS18B20 sensor(ONE_WIRE_BUS_PIN);

void setup() {
    Serial.begin(115200);
    
    // センサーの初期化とスキャン
    if (sensor.begin()) {
        Serial.printf("%d台のセンサーを検出しました。\n", sensor.getDeviceCount());
    } else {
        Serial.println("センサーが見つかりません。");
    }
}

void loop() {
    // 1. 温度変換開始（非同期）
    if (sensor.startConversion()) {
        
        // 3線式での温度変換完了をポーリングチェック (RTOSタスクの場合は vTaskDelay を推奨)
        while (!sensor.isConversionComplete()) {
            delay(10);
        }
        
        // 2. 温度データの読み取り
        for (int i = 0; i < sensor.getDeviceCount(); i++) {
            float temp = 0.0f;
            if (sensor.readTemperature(i, temp)) {
                Serial.printf("センサー [%d]: %.2f ℃\n", i, temp);
            } else {
                // エラーコードの取得
                DS18B20::ErrorType err = sensor.getLastError(i);
                Serial.printf("エラー [%d]: %s (Code: %d)\n", 
                              i, DS18B20::getErrorString(err), err);
            }
        }
    }
    delay(2000);
}
```

## 主要APIリファレンス

### `DS18B20` クラス

#### メソッド

* **`DS18B20(uint8_t pin)`**
  * コンストラクタ。1-Wireバスに使用するGPIOピン番号を指定します。
* **`bool begin()`**
  * バスおよびセンサーを初期化し、接続されているデバイスをスキャンします。成功した場合は `true` を返します。
* **`bool startConversion()`**
  * 接続されているすべてのDS18B20に対し、一括で温度変換の開始コマンド（Convert T）を送信します。
* **`bool isConversionComplete()`**
  * 温度変換が完了しているか判定ステータス（3線式外部電源モードでのみ動作）を読み込みます。完了している場合は `true` を返します。
* **`bool readTemperature(int romIndex, float &tempCelsius)`**
  * 指定したインデックス（`0` 〜 `getDeviceCount()-1`）のセンサーからデータを読み込み、参照引数 `tempCelsius` に結果を代入します。成功した場合は `true` を返します。
* **`bool readAllTemperatures(float *tempArray)`**
  * バス上の全センサーから一括で温度を読み取り、配列に格納します（動的ポーリング待機・vTaskDelay内蔵）。
* **`int getDeviceCount() const`**
  * スキャンによって検出されたセンサーの総数を返します（最大 8 台）。
* **`bool checkPowerSupply()`**
  * 接続されたセンサーが外部電源（3線モード）で動作しているかチェックします。もし1台でも寄生電源で動作している（VDDが未接続・浮いているなど）場合は `false` を返します。
* **`ErrorType getLastError(int romIndex) const`**
  * 指定したインデックスのセンサーの直近のエラーコードを取得します。
* **`static const char* getErrorString(ErrorType err)`**
  * エラーコード（`ErrorType`）を人間が読める日本語のエラー説明文字列に変換します。

#### エラーコード (`ErrorType`)

| エラー名 | 値 | 説明 |
| :--- | :---: | :--- |
| `ERR_NONE` | 0 | 正常終了。エラーはありません。 |
| `ERR_NOT_FOUND` | 1 | 該当インデックスのデバイスが見つかりません。 |
| `ERR_BUS_RESET` | 2 | 1-Wireバスのリセットパルスに対するプレゼンスパルスがありません。 |
| `ERR_MATCH_ROM` | 3 | ROMコードの選択および照合に失敗しました。 |
| `ERR_SHORT_GND` | 4 | バスの信号線がGNDにショートしているか、立ち上がりレベルが低すぎます。 |
| `ERR_DISCONNECTED` | 5 | 断線またはプルアップ抵抗の欠落（全0xFFデータ受信/リセット時の応答なし）しています。 |
| `ERR_CRC` | 6 | 受信したスクラッチパッドのCRCチェックに失敗しました。 |
| `ERR_STUCK_85C` | 7 | 電源投入時の初期値である 85.0℃ が検出されました。正常に温度変換が完了していない、または計測中にリセットされた可能性があります。 |
| `ERR_PARASITE_POWER` | 8 | 外部電源ライン（VDD）が断線・接触不良のため、寄生電源で動作しています。 |
| `ERR_OUT_OF_RANGE` | 9 | 温度測定可能範囲（-55℃〜+125℃）を外れた異常値を検出しました。 |
| `ERR_MEM_CORRUPT` | 10 | スクラッチパッドの仕様上の予約領域（Byte 5, Byte 7）や定義レジスタの固定ビットに異常値を検出し、メモリ破損・通信崩壊と判定しました。 |
| `ERR_CONVERSION_TIMEOUT`| 11 | 温度変換が制限時間（800ms）以内に完了しなかったことを検知しました。 |

## サンプルプログラム
`examples/ComprehensiveDemo/ComprehensiveDemo.ino` に、FreeRTOSタスクを生成して並行して動作させる包括的なデモが用意されています。
