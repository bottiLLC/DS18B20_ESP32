#ifndef DS18B20_H
#define DS18B20_H

#include <Arduino.h>

/**
 * @brief DS18B20 / DS18B20U+ 専用の温度測定管理クラス (ESP32-C3・3線式専用)
 * 
 * FreeRTOS環境において、タイミングの精密なGPIO制御を行い、
 * 温度変換待ち時間（750ms）に vTaskDelay() を用いて他のタスクを動作させます。
 * バスの異常検知機能、電源断検出機能を内蔵しています。
 */
class DS18B20 {
public:
    static const int MAX_DEVICES = 8;  // バス上に接続可能な最大センサー数
    typedef uint8_t DeviceAddress[8];

    /**
     * @brief コンストラクタ
     * @param pin 1-Wire バスに接続する GPIO ピン番号
     */
    DS18B20(uint8_t pin);

    /**
     * @brief デストラクタ
     */
    ~DS18B20();

    /**
     * @brief 1-Wireバスおよびセンサーの初期化と探索
     * @return 初期化およびデバイス発見に成功した場合 true
     */
    bool begin();

    /**
     * @brief バス上のすべてのDS18B20に対して温度変換を開始する (非同期)
     * @return 変換開始コマンドの送信に成功した場合 true
     */
    bool startConversion();

    /**
     * @brief バス上で発見された指定インデックスのデバイスの温度データを読み取る
     * 
     * @param romIndex 発見されたデバイスのインデックス (0 〜 getDeviceCount()-1)
     * @param tempCelsius 読み取った温度を格納する参照変数
     * @return 温度読み取りおよび各種エラー・CRCチェックに成功した場合 true
     */
    bool readTemperature(int romIndex, float &tempCelsius);

    /**
     * @brief バス上で発見された全デバイスの温度データを一括で読み取る (ブロッキング・vTaskDelay内蔵)
     * @param tempArray 温度結果を格納する float 配列 (要素数は getDeviceCount() 以上必要)
     * @return 少なくとも1つのデバイスから正常に読み取れた場合 true
     */
    bool readAllTemperatures(float *tempArray);

    /**
     * @brief バス上で発見されたDS18B20のデバイス数を取得する
     * @return 発見されたデバイス数
     */
    int getDeviceCount() const { return _deviceCount; }

    /**
     * @brief 指定したインデックスのデバイスアドレス(ROM)を取得する
     * @param romIndex デバイスインデックス
     * @param destAddress コピー先のアドレス配列
     */
    void getDeviceAddress(int romIndex, DeviceAddress destAddress) const;

    /**
     * @brief バス上のデバイスが外部電源(3線モード)で動作しているか確認する
     * 
     * もし 0 (寄生電源) が検知された場合、VDD線が断線または浮いている状態（フォールバック）と判定します。
     * @return すべてのデバイスが外部電源で動作している場合 true
     */
    bool checkPowerSupply();

private:
    // 1-Wire 探索状態管理構造体
    struct SearchState {
        uint8_t rom_number[8];
        int last_discrepancy;
        int last_family_discrepancy;
        bool last_device_flag;
    };

    // 1-Wire 低レベル通信関数
    void ow_write_bit(uint8_t bit);
    uint8_t ow_read_bit();
    void ow_write_byte(uint8_t byte);
    uint8_t ow_read_byte();
    bool ow_reset();
    void ow_select(const DeviceAddress addr);
    void ow_skip();
    bool search(DeviceAddress &address, SearchState &state);

    uint8_t _pin;
    DeviceAddress _devices[MAX_DEVICES];
    int _deviceCount;
    bool _parasitePowerDetected; // 寄生電源駆動のデバイスが検出されたかどうかのフラグ
    SemaphoreHandle_t _mutex;  // 複数タスクからのアクセスを保護するためのMutex

public:
    enum ErrorType {
        ERR_NONE = 0,
        ERR_NOT_FOUND,
        ERR_BUS_RESET,
        ERR_MATCH_ROM,
        ERR_SHORT_GND,        // 全0x00
        ERR_DISCONNECTED,     // 全0xFF
        ERR_CRC,              // CRCエラー
        ERR_STUCK_85C,        // 85℃初期値固定（未変換または電源寸断によるリセット）
        ERR_PARASITE_POWER    // 寄生電源検知（3線モードでのVDD未接続・浮き）
    };

    /**
     * @brief 指定したインデックスのデバイスの最後のエラーを取得する
     * @param romIndex デバイスインデックス
     * @return エラーの種類 (ErrorType)
     */
    ErrorType getLastError(int romIndex) const;

    /**
     * @brief エラーコードに対応する日本語のメッセージを取得する
     * @param err エラーコード
     * @return エラー内容を表す文字列
     */
    static const char* getErrorString(ErrorType err);

private:
    ErrorType _lastErrors[MAX_DEVICES];

    // データシートに基づき、スクラッチパッドから摂氏温度を計算するヘルパー
    float calculateTemperature(uint8_t lsb, uint8_t msb, uint8_t cfg);
};

#endif // DS18B20_H
