/*
 M5_GPS_Avatar_Combined_Multi_Geofence_LoRaWAN.ino
 
   ポリゴンエリア（敷地全体）：エリアから「外に出た」ときに一度だけ発話
   円形エリア（POI）：エリアに「中に入った」ときに一度だけ発話
   LoRaWANによるGPS位置情報の送信
 という複合的なロジックを実装したバージョン。
*/
// 必要なライブラリをインクルード
#include <M5Unified.h>
#include <M5GFX.h>
#include <cmath>
#include "MultipleSatellite.h"
#include "AquesTalkTTS.h"
#include <Avatar.h>
#include <tasks/LipSync.h>
using namespace m5avatar;
// --- GPS設定 ---
static const int GPS_RX_PIN = 7, GPS_TX_PIN = 6;
static const uint32_t GPS_BAUDRATE = 115200;
MultipleSatellite gps(Serial1, GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
// --- AquesTalk & Avatar設定 ---
Avatar avatar;
int speed = 100; // 話すスピード
// --- 判定エリアと発話メッセージの設定 ---
// 緯度経度を扱うための構造体
struct Point {
double lat;
double lng;
};
// --- 1. 多角形エリア（敷地全体）の設定 ---
// このエリアは「外に出た時」に発話します
const Point targetPolygon[] = {
{36.318671, 138.989292}, {36.320361, 138.987463},
{36.319030, 138.985521}, {36.318468, 138.985822},
{36.317664, 138.986782}, {36.317993, 138.987265},
{36.317470, 138.987908}
};
const int numPolygonVertices = sizeof(targetPolygon) / sizeof(targetPolygon);
static bool wasLastKnownStateInsidePolygon = false; // 前回、範囲内にいたかのフラグ
const char* POLYGON_EXIT_MESSAGE = "高崎高校の、敷地の、外に出ました。気をつけてくださいね";
// --- 2. 円形エリア（注目地点: Point of Interest）の設定 ---
// このエリアは「中に入った時」に発話します
const double POI_RADIUS_METERS = 15.0; // 半径15m
// 注目地点を管理するための構造体
struct PointOfInterest {
Point location;
const char* message;
bool hasSpoken; // この地点で発話済みかどうかのフラグ
};
// 判定したい地点のリスト
PointOfInterest pois[] = {
{ {36.318715, 138.986685}, "ここは、理科棟、だよ。物理部が、すごく遅くまで、活動しているよ。", false },
{ {36.319748, 138.987455}, "ここは、翠巒会館、だよ。同窓会の、事務局も、あるよ。", false }
};
const int numPois = sizeof(pois) / sizeof(pois);
// =================================================================
// LoRaWAN 設定
// =================================================================
const char* TTN_DEVICE_EUI = "8C1F64B910010080";
const char* TTN_APP_EUI    = "A840410000000101";
const char* TTN_APP_KEY    = "53E252A69E536B636222D4FAE44EA962";
// LoRaモジュール (RAK3172) のシリアル設定
#define LORA_SERIAL      Serial2
#define LORA_UART_TX_PIN 17
#define LORA_UART_RX_PIN 18
#define LORA_UART_BAUDRATE 9600
const char* LORAWAN_REGION = "2"; // AS923
bool lora_joined = false;
unsigned long last_lora_send_time = 0;
const unsigned long LORA_SEND_INTERVAL_MS = 60000; // 60秒ごとに送信
// --- 関数のプロトタイプ宣言 ---
void printGpsInfoToSerial();
void checkGpsAndSpeak();
bool isInsidePolygon(Point p, const Point polygon[], int n);
double distanceBetweenPoints(Point p1, Point p2);
// LoRaWAN関連関数のプロトタイプ宣言
String bytesToHexString(const byte* bytes, size_t len);
String hexToUtf8String(const String& hex);
bool send_at_command(const String& command, const String& expected_reply, unsigned long timeout_ms = 5000, String* response_out = nullptr);
bool lora_module_setup();
bool lora_join_network(int retries = 3, int retry_delay_s = 10);
bool lora_send_data(const byte* payload, size_t len, bool confirmed = false);
void setup() {
auto cfg = M5.config();
cfg.external_speaker.atomic_spk = true;
M5.begin(cfg);
M5.Lcd.setBrightness(100);
// M5.Lcd.clear(); // アバターが描画されるため不要
// M5.Lcd.setTextSize(2); // ディスプレイにテキストを表示しないため不要
Serial.begin(115200);
Serial.println(F("GPS Avatar LoRaWAN Demo Start"));

// --- AquesTalk & Avatar 初期化 ---
if (int iret = TTS.createK()) {
    const char* err_msg = "ERR:TTS.createK()";
    if(iret == 501) { err_msg = "ERR: SD card not found or dictionary file is missing."; }
    Serial.println(err_msg); 
    M5.Display.println(err_msg); // エラー表示はM5ディスプレイにも残す
    while(1) delay(1);
}
M5.Speaker.setVolume(255);

avatar.init();
avatar.addTask(lipSync, "lipSync");

// --- GPS 初期化 ---
Serial.println("Initializing GPS...");
gps.begin();
gps.setSystemBootMode(BOOT_FACTORY_START);
delay(100);
gps.setSatelliteMode(SATELLITE_MODE_GPS);
Serial.println("GPS Initialized.");
delay(1000); // GPSモジュールが安定するのを待つ

// --- LoRaWAN 初期化 ---
// M5.Lcd.fillScreen(BLACK); // アバターが描画されるため不要
// M5.Lcd.setCursor(0, 0);   // アバターが描画されるため不要
LORA_SERIAL.begin(LORA_UART_BAUDRATE, SERIAL_8N1, LORA_UART_RX_PIN, LORA_UART_TX_PIN);
if (!lora_module_setup()) {
    Serial.println("LoRa module setup failed. Halting.");
    // エラー表示はSerialのみにする
    while(1) { delay(1000); }
}

Serial.println("Initialization complete.");
delay(2000);

TTS.playK("アバタースタート", speed);

}
void loop() {
M5.update();
gps.updateGPS();
// LoRaWANネットワークへの接続確認と再接続
if (!lora_joined) {
    // M5.Lcd.fillScreen(BLACK); // アバターが描画されるため不要
    // M5.Lcd.setCursor(0, 0);   // アバターが描画されるため不要
    Serial.println("--- LoRaWAN Join ---");
    if (!lora_join_network()) {
        Serial.println("Could not join network.");
        Serial.println("Retrying in 60 seconds.");
        delay(5000); // 接続失敗時は少し待つ
        return; // 次のループで再試行
    }
}

// GPS情報の表示 (Serialのみ)
// M5.Lcd.fillScreen(BLACK); // アバターが描画されるため不要
// M5.Lcd.setCursor(0, 0);   // アバターが描画されるため不要
Serial.println("--- GPS LoRaWAN Tracker ---");
Serial.printf("LoRa Status: %s\n", lora_joined ? "Joined" : "Not Joined");
Serial.printf("GPS Sats: %d\n", gps.satellites.value());

// GPSが有効な場合のみ、アバターの発話とLoRaWAN送信を行う
if (gps.location.isUpdated() && gps.location.isValid()) {
    double lat_raw = gps.location.lat();
    double lon_raw = gps.location.lng();
    
    Serial.printf("Lat: %.8f\n", lat_raw);
    Serial.printf("Lon: %.8f\n", lon_raw);
    // --- アバターの発話ロジック ---
    checkGpsAndSpeak();
    // --- LoRaWANデータ送信ロジック (非同期) ---
    unsigned long current_millis = millis();
    if (lora_joined && (current_millis - last_lora_send_time >= LORA_SEND_INTERVAL_MS)) {
        const double ENCODE_SCALE = 10000000.0;
        int32_t encoded_lat = (int32_t)(lat_raw * ENCODE_SCALE);
        int32_t encoded_lon = (int32_t)(lon_raw * ENCODE_SCALE);
        const uint16_t deviceId = 12345;
        const size_t payloadSize = 10;
        byte payload[payloadSize];
        size_t offset = 0;
        payload[offset++] = (uint8_t)(deviceId & 0xFF);
        payload[offset++] = (uint8_t)((deviceId >> 8) & 0xFF);
        payload[offset++] = (uint8_t)(encoded_lat & 0xFF);
        payload[offset++] = (uint8_t)((encoded_lat >> 8) & 0xFF);
        payload[offset++] = (uint8_t)((encoded_lat >> 16) & 0xFF);
        payload[offset++] = (uint8_t)((encoded_lat >> 24) & 0xFF);
        payload[offset++] = (uint8_t)(encoded_lon & 0xFF);
        payload[offset++] = (uint8_t)((encoded_lon >> 8) & 0xFF);
        payload[offset++] = (uint8_t)((encoded_lon >> 16) & 0xFF);
        payload[offset++] = (uint8_t)((encoded_lon >> 24) & 0xFF);
        
        Serial.println("Sending packet...");
        Serial.printf("\n--- Sending Payload (High Precision) ---\n");
        Serial.printf("Original -> Lat: %.8f, Lon: %.8f\n", lat_raw, lon_raw);
        Serial.printf("Encoded (scaled by %.0f) -> Lat: %d, Lon: %d\n", ENCODE_SCALE, encoded_lat, encoded_lon);
        Serial.print("Payload Hex: " + bytesToHexString(payload, payloadSize) + "\n");
        if (lora_send_data(payload, payloadSize, false)) {
            Serial.println("Send command successful.");
        } else {
            Serial.println("Send failed. Conn. lost?");
            lora_joined = false; // 送信失敗時は再接続を試みる
        }
        last_lora_send_time = current_millis; // 最終送信時刻を更新
    }
} else {
    Serial.println("Lat: (waiting for fix...)");
    Serial.println("Lon: (waiting for fix...)");
    Serial.println("Waiting for GPS fix...");
}
// ループの最後に短いディレイを入れて、CPU負荷を軽減
delay(100);
}

// 2点間の距離を計算する（ハーバーサイン公式）
double distanceBetweenPoints(Point p1, Point p2) {
const double R = 6371000.0; // 地球の半径 (メートル)
double lat1_rad = p1.lat * M_PI / 180.0;
double lon1_rad = p1.lng * M_PI / 180.0;
double lat2_rad = p2.lat * M_PI / 180.0;
double lon2_rad = p2.lng * M_PI / 180.0;
double d_lat = lat2_rad - lat1_rad;
double d_lon = lon2_rad - lon1_rad;
double a = sin(d_lat / 2.0) * sin(d_lat / 2.0) + cos(lat1_rad) * cos(lat2_rad) * sin(d_lon / 2.0) * sin(d_lon / 2.0);
double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
return R * c;
}
// 点が多角形の内側にあるか判定する (レイキャスティング法)
bool isInsidePolygon(Point p, const Point polygon[], int n) {
bool inside = false;
for (int i = 0, j = n - 1; i < n; j = i++) {
if (((polygon[i].lat > p.lat) != (polygon[j].lat > p.lat)) &&
(p.lng < (polygon[j].lng - polygon[i].lng) * (p.lat - polygon[i].lat) / (polygon[j].lat - polygon[i].lat) + polygon[i].lng)) {
inside = !inside;
}
}
return inside;
}

// GPS情報をシリアルモニターに出力
void printGpsInfoToSerial() {
Serial.printf("Satellites: %d | ", gps.satellites.value());
if (gps.location.isUpdated() && gps.location.isValid()) {
Serial.printf("Location: Lat=%.8f, Lng=%.8f", gps.location.lat(), gps.location.lng());
} else { Serial.print("Location: INVALID"); }
Serial.println();
}
/**
 @brief GPS位置を判定し、条件に応じて発話する
*/
void checkGpsAndSpeak() {
if (!gps.location.isValid() || TTS.isPlay()) {
// GPSが無効な場合、またはTTSが再生中の場合は、発話をスキップ
return;
}
Point currentPoint = { gps.location.lat(), gps.location.lng() };
// --- 1. 多角形エリアから「出た」ときの判定 ---
bool isCurrentlyInsidePolygon = isInsidePolygon(currentPoint, targetPolygon, numPolygonVertices);
// 前回は範囲内で、かつ今回は範囲外になった瞬間か？
if (wasLastKnownStateInsidePolygon && !isCurrentlyInsidePolygon) {
Serial.println("=> Moved out of polygon area. Speaking...");
TTS.playK(POLYGON_EXIT_MESSAGE, speed);
wasLastKnownStateInsidePolygon = isCurrentlyInsidePolygon;
return; // 発話したので、この回のチェックは終了
}
// 次のループのために、今回の状態を「前回の状態」として保存する
wasLastKnownStateInsidePolygon = isCurrentlyInsidePolygon; // <-- ここを修正
// --- 2. 各POIの円形エリアに「入った」ときの判定 ---
for (int i = 0; i < numPois; i++) {
double distance = distanceBetweenPoints(currentPoint, pois[i].location);
 if (distance <= POI_RADIUS_METERS) {
     // 範囲内にいる
     if (!pois[i].hasSpoken) {
         // まだ発話していない場合のみ発話
         Serial.printf("=> Entered POI area: %s. Speaking...\n", pois[i].message);
         TTS.playK(pois[i].message, speed);
         pois[i].hasSpoken = true; // 発話済みフラグを立てる
         return; // 発話したので、この回のチェックは終了
     }
 } else {
     // 範囲外にいる
     if (pois[i].hasSpoken) {
         // 一度範囲外に出たら、発話済みフラグをリセット
         Serial.printf("=> Exited POI area: %d. Resetting flag.\n", i);
         pois[i].hasSpoken = false;
     }
 }
 
}
}
// =================================================================
// LoRaWAN関連ヘルパー関数
// =================================================================
// byte配列を16進数文字列に変換
String bytesToHexString(const byte* bytes, size_t len) {
String hexString = "";
hexString.reserve(len * 2);
for (size_t i = 0; i < len; i++) {
char hex;
sprintf(hex, "%02X", bytes[i]);
hexString += hex;
}
return hexString;
}

// 16進数文字列をUTF-8文字列に変換（ダウンリンク用）
String hexToUtf8String(const String& hex) {
String utf8String = "";
for (size_t i = 0; i < hex.length(); i += 2) {
String byteString = hex.substring(i, i + 2);
char byte = (char)strtol(byteString.c_str(), NULL, 16);
utf8String += byte;
}
return utf8String;
}

// ATコマンドを送信し、応答を確認する
bool send_at_command(const String& command, const String& expected_reply, unsigned long timeout_ms, String* response_out) {
while (LORA_SERIAL.available()) { LORA_SERIAL.read(); }
Serial.printf("TX: %s\n", command.c_str()); // Serialに出力
LORA_SERIAL.println(command);
unsigned long start_time = millis();
String response = "";
bool found_expected_reply = false;
while (millis() - start_time < timeout_ms) {
if (LORA_SERIAL.available()) {
String line = LORA_SERIAL.readStringUntil('\n');
line.trim();
if (line.length() > 0) {
Serial.printf("RX: %s\n", line.c_str()); // Serialに出力
response += line + "\n";
if (line.indexOf(expected_reply) != -1) {
found_expected_reply = true;
break;
}
}
}
}
if (response_out != nullptr) { *response_out = response; }
if (!found_expected_reply) {
Serial.println("Timeout or unexpected reply."); // Serialに出力
}
return found_expected_reply;
}
// LoRaモジュールの初期設定
bool lora_module_setup() {
lora_joined = false;
Serial.println("--- LoRa Module Init ---"); // Serialに出力
send_at_command("AT+CGMM?", "+CGMM");
delay(100);
if (!send_at_command("AT+CCLASS=0", "OK")) return false;
delay(100);
if (!send_at_command("AT+RREGION=" + String(LORAWAN_REGION), "OK")) return false;
delay(100);
if (!send_at_command("AT+CDEVEUI=" + String(TTN_DEVICE_EUI), "OK")) return false;
delay(100);
if (!send_at_command("AT+CAPPEUI=" + String(TTN_APP_EUI), "OK")) return false;
delay(100);
if (!send_at_command("AT+CAPPKEY=" + String(TTN_APP_KEY), "OK")) return false;
delay(100);
if (!send_at_command("AT+CJOINMODE=0", "OK")) return false;
delay(100);
if (!send_at_command("AT+CADR=1", "OK")) return false;
delay(100);
if (!send_at_command("AT+CAPPPORT=10", "OK")) return false;
delay(100);
Serial.println("Saving settings..."); // Serialに出力
if (!send_at_command("AT+CSAVE", "OK")) return false;
delay(2000);
Serial.println("--- LoRa Module Init OK ---"); // Serialに出力
return true;
}
// LoRaWANネットワークへの接続試行
bool lora_join_network(int retries, int retry_delay_s) {
Serial.println("--- Joining LoRaWAN Network ---"); // Serialに出力
for (int attempt = 0; attempt < retries; attempt++) {
Serial.printf("Join attempt %d/%d\n", attempt + 1, retries); // Serialに出力
if (send_at_command("AT+DJOIN=1,0,8,2", "OK", 10000)) {
Serial.println("Join command sent. Waiting for status..."); // Serialに出力
unsigned long join_wait_start = millis();
while (millis() - join_wait_start < 60000) { // 60秒待機
String status_response;
if (send_at_command("AT+DULSTAT?", "+DULSTAT", 2000, &status_response)) {
if (status_response.indexOf("+DULSTAT:04") != -1) {
Serial.println("LoRaWAN Joined Successfully!"); // Serialに出力
lora_joined = true;
return true;
}
if (status_response.indexOf("+DULSTAT:05") != -1) {
Serial.println("LoRaWAN Join Failed by module."); // Serialに出力
break;
}
}
delay(5000);
}
} else {
Serial.println("Failed to send JOIN command."); // Serialに出力
}
if (attempt < retries - 1) {
Serial.printf("Retrying in %d seconds...\n", retry_delay_s); // Serialに出力
delay(retry_delay_s * 1000);
}
}
Serial.println("--- LoRaWAN Join Failed ---"); // Serialに出力
lora_joined = false;
return false;
}
// LoRaWANでデータを送信
bool lora_send_data(const byte* payload, size_t len, bool confirmed) {
if (!lora_joined) {
Serial.println("Cannot send: not joined."); // Serialに出力
return false;
}
String payload_hex = bytesToHexString(payload, len);
String confirm_val = confirmed ? "1" : "0";
String command = "AT+DTRX=" + confirm_val + ",10," + String(len) + "," + payload_hex;
String expected_send_reply = confirmed ? "OK+SENT" : "OK+SEND";
String response;
if (send_at_command(command, expected_send_reply, 15000, &response)) {
Serial.printf("Data sent: %s\n", payload_hex.c_str());
int pos = response.indexOf("OK+RECV=");
if (pos != -1 && response.indexOf("OK+RECV=00,00,00") == -1) {
String recv_line = response.substring(pos);
int first_comma = recv_line.indexOf(',');
int second_comma = recv_line.indexOf(',', first_comma + 1);
if (first_comma != -1 && second_comma != -1) {
String port_str = recv_line.substring(recv_line.indexOf('=') + 1, first_comma);
String dl_payload_hex = recv_line.substring(second_comma + 1);
dl_payload_hex.trim();
if (dl_payload_hex != "00") {
String decoded_str = hexToUtf8String(dl_payload_hex); // <-- ここを修正
Serial.printf("DL on Port %s: %s (Hex: %s)\n", port_str.c_str(), decoded_str.c_str(), dl_payload_hex.c_str()); // Serialに出力
}
}
}
return true;
} else {
Serial.println("Failed to send data."); // Serialに出力
return false;
}
}