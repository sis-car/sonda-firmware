/*
  sonda_51_codemax.ino
  Firmware corregido y modular conforme al diagrama:
  - Medición con periféricos encendidos, WiFi desconectado.
  - Conexión y envío posterior; en fallo guarda en "mochila".
  - I2C inicializado una vez en setup() para evitar freezes por reinit.
  - Recuperación I2C por GPIO como salvaguarda.
  - Código organizado en funciones para facilitar lectura y mantenimiento.
*/

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <Adafruit_MAX31865.h>
#include <LittleFS.h>
#include <RTClib.h>
#include <Preferences.h>
#include <Adafruit_MAX1704X.h>   // MAX17048 fuel gauge (reemplaza INA219)
#include <WiFiClientSecure.h>         // OTA sobre HTTPS
#include <HTTPUpdate.h>               // OTA via HTTP
#include "driver/gpio.h"

#include <WebServer.h>
#include <DNSServer.h>

// --- Credenciales del Portal ---
const char* valid_username = "admin";
const char* valid_password = "Sonda123";
const char* cookie_name    = "ESPSESSIONID";

// --- Configuración de Hardware ---
const int PIN_BOTON_CONFIG = 4; // Cambialo al pin que uses (con pull-up)

// --- Objetos y Variables de Configuración ---
WebServer server(80);
DNSServer dnsServer;
Preferences preferences;
// --- Agrega esto que te faltaba (MQTT_PORT) ---
const int MQTT_PORT = 1883; 



String p_ssid, p_pass, p_sonda_num, p_srv_host, p_mqtt_host;
int p_srv_port;
bool modoConfigActivo = false;

// Variables que usa tu código principal
char* WIFI_SSID;
char* WIFI_PASSWORD;
char* SONDA_ID;
char* SERVER_HOST;
int   SERVER_PORT;
char* MQTT_HOST;
// ⭐ AGREGAR DESPUÉS de las otras constantes globales (~línea 80)
const wifi_power_t WIFI_MAX_POWER = WIFI_POWER_19_5dBm;
// ⭐ AGREGAR con las otras constantes RSSI (~línea 90)
const int WIFI_RSSI_EXCELENTE = -50;
const int WIFI_RSSI_BUENO     = -65;
const int WIFI_RSSI_REGULAR   = -75;
const int WIFI_RSSI_DEBIL     = -85;
// ---------------- CONFIG ----------------
//const char* WIFI_SSID       = "CORT";
//const char* WIFI_PASSWORD   = "venezuela1871";
//const char* SONDA_ID        = "sonda_03";
////const char* SERVER_HOST = "sonda-server.local";
//const int   SERVER_PORT     = 5000;
//const char* MQTT_HOST = "sonda-server.local";
//const int   MQTT_PORT   = 1883;
IPAddress serverIp;
bool serverIpOk = false;
// --- Agrega estas líneas al principio del código ---
const uint32_t MIN_LIVE_GAP_MS       = 15000;  // Mínimo 15 seg entre envíos LIVE
const uint32_t PAYLOAD_DUP_WINDOW_MS = 60000;  // Ventana de 1 min para detectar duplicados
const int   CS_PIN       = 23;  // MAX31865 CS
const int   THERMO_CLK_PIN = 5;  // MAX31865 CLK
const int   THERMO_SDO_PIN = 18; // MAX31865 SDO → MISO (sensor→ESP32)
const int   THERMO_SDI_PIN = 19; // MAX31865 SDI → MOSI (ESP32→sensor)
const int   LED_PIN = 27;  // LED de estado

// --- OTA via GitHub (ajustar a tu repositorio) ---
#define FIRMWARE_VERSION "1.0.0"
const char* OTA_VERSION_URL  = "https://raw.githubusercontent.com/sis-car/sonda-firmware/refs/heads/main/firmware/version.txt";
const char* OTA_FIRMWARE_URL = "https://raw.githubusercontent.com/sis-car/sonda-firmware/refs/heads/main/firmware/sonda_14.ino";

const uint32_t T_GRACIA_INICIAL_MS = 5 * 60 * 1000UL;
const uint32_t T_GRACIA_FINAL_MS   = 1 * 60 * 1000UL;
// Nuevas variables de umbral
float T_ALERTA = 35.0f;
float T_CRITICA = 40.0f;
uint32_t T_REPORT_NORMAL_MS = 60000UL;
uint32_t T_REPORT_CRITICAL_MS = 15000UL;
uint32_t T_REPORT_ALERTA_MS = 30000UL; // Los 10 segundos que pides
bool modoOnlineForzado = false; // Para T > T_ALERTA
// tiempos y límites
const uint32_t WIFI_CONNECT_TIMEOUT_MS = 8000UL;
const uint32_t MAX_FLUSH_TIME_MS = 7000UL;
const int      MAX_LINES_PER_FLUSH = 12;
const int      MAX_LINE_TRIES = 3;
const unsigned long STUCK_RESET_MS = 120000UL;

// power control
const int PWR_CTRL_PIN = 14;
const int PWR_ON_LEVEL = LOW;
const int PWR_OFF_LEVEL = (PWR_ON_LEVEL == HIGH) ? LOW : HIGH;
const unsigned long PWR_STABILIZE_MS = 1000UL;

// I2C pins
const int SDA_MAIN = 21;
const int SCL_MAIN = 22;
const int SDA_INA  = 25;
const int SCL_INA  = 26;
bool enGracia= false;
// averaging
const int SAMPLES_AVG = 5;
const int SAMPLE_DELAY_MS = 100;

// hardware objects
TwoWire I2C_INA = TwoWire(1);
Adafruit_MAX17048 maxlipo;
Adafruit_MAX31865 thermo(CS_PIN, THERMO_SDI_PIN, THERMO_SDO_PIN, THERMO_CLK_PIN);
RTC_DS3231 rtc;

WiFiClient mqttNet;
WiFiClient httpNet;
PubSubClient mqttClient(mqttNet);

// ---------------- BATTERY MODEL ----------------
const float BATTERY_CAPACITY_mAh = 5200.0f;
const float I_CONSUMO_MEDIO_mA = 15.0f; // ajustar con medición de campo

// estados de batería (globales)
float lastBatteryV = NAN;
float lastBatteryPct = NAN;
float lastBatteryHours = NAN;


volatile bool pendingFinalize = false;
volatile uint32_t pendingFinalizePrevRun = 0;
volatile bool pendingWakeupReport = false;

volatile bool pendingStart = false;
volatile uint32_t pendingStartRun = 0;


// ---------------- STATE ----------------
struct PersistentData { uint32_t run_id; uint32_t id_anterior; uint32_t t_reporte_ms; bool sonda_colocada; uint32_t last_seq; };
RTC_DATA_ATTR PersistentData estado = {0,0,40000UL,false,0};

bool rtcPresente = false;
bool ina_available = false;
bool peripherals_on = false;
bool mochilaHasData = false;
bool flushedThisBoot = false;

float lastTemp = NAN;
unsigned long lastTempMillis = 0;

char lastSentKey[128] = {0};
unsigned long lastSentKeyMillis = 0;
unsigned long mqttSubscribedAt = 0;

unsigned long lastProgressMillis = 0;
unsigned long lastRuntimeReportMillis = 0;
unsigned long lastLiveSendMillis = 0;
uint32_t lastLiveSeq = 0;
float lastLiveTemp = NAN;
uint32_t lastLiveRun = 0;
unsigned long lastLiveMillis = 0;
 unsigned long proxRep  =0;
bool keepWifiDuringGrace = false; // si true, desconexión se omite
bool abortarGracia = false;
// 1. El texto del Login
const char login_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  body { font-family: sans-serif; background: #2c3e50; display: flex; justify-content: center; align-items: center; height: 100vh; margin: 0; }
  .login-card { background: white; padding: 30px; border-radius: 15px; box-shadow: 0 10px 20px rgba(0,0,0,0.3); width: 100%; max-width: 300px; text-align: center; }
  input { width: 100%; padding: 12px; margin: 10px 0; border: 1px solid #ccc; border-radius: 8px; box-sizing: border-box; }
  button { width: 100%; padding: 12px; background: #28a745; color: white; border: none; border-radius: 8px; cursor: pointer; font-weight: bold; }
</style></head>
<body><div class="login-card"><h2>Iniciar Sesión</h2><form action="/login" method="POST">
  <input type="text" name="username" placeholder="Usuario" required>
  <input type="password" name="password" placeholder="Contraseña" required>
  <button type="submit">INGRESAR</button>
</form></div></body></html>
)rawliteral";

// 2. La función que genera el formulario
String getHTML() {
  String v_ssid = (p_ssid == "") ? "" : p_ssid;
  String v_pass = (p_pass == "") ? "" : p_pass;
  String v_snum = (p_sonda_num == "") ? "" : p_sonda_num;
  String v_shost = (p_srv_host == "") ? "server.local" : p_srv_host;
  int v_sport = (p_srv_port == 0) ? 5000 : p_srv_port;
  String v_mhost = (p_mqtt_host == "") ? "server.local" : p_mqtt_host;

  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>body{font-family:sans-serif;background:#2c3e50;padding:20px;display:flex;justify-content:center;} .card{background:white;padding:25px;border-radius:15px;box-shadow:0 10px 20px rgba(0,0,0,0.3);width:100%;max-width:350px;} h2{color:#333;text-align:center;border-bottom:2px solid #007bff;padding-bottom:10px;} label{font-size:0.85em;font-weight:bold;color:#555;} input{width:100%;padding:10px;margin:5px 0 15px 0;border:1px solid #ccc;border-radius:8px;box-sizing:border-box;} button{width:100%;padding:12px;background:#007bff;color:white;border:none;border-radius:8px;cursor:pointer;font-weight:bold;} .btn-exit{background:#6c757d;margin-top:10px;}</style></head><body>";
  html += "<div class='card'><h2>⚙️ Config Sonda</h2><form action='/save' method='POST'>";
  html += "<label>WiFi SSID</label><input name='s' value='"+v_ssid+"'>";
  html += "<label>WiFi Pass</label><input name='p' value='"+v_pass+"'>";
  html += "<label>ID Sonda (Número)</label><input name='id' type='number' value='"+v_snum+"'>";
  html += "<label>Servidor API</label><input name='sh' value='"+v_shost+"'>";
  html += "<label>Puerto API</label><input name='sp' type='number' value='"+String(v_sport)+"'>";
  html += "<label>Broker MQTT</label><input name='mh' value='"+v_mhost+"'>";
  html += "<button type='submit'>✅ GUARDAR Y REINICIAR</button>";
  html += "<button type='button' class='btn-exit' onclick='location.href=\"/exit\"'>❌ SALIR</button>";
  html += "</form></div></body></html>";
  return html;
}
// ---------------- FORWARD DECLARATIONS ----------------
String formatFechaSQL(const DateTime &dt);
float soc_from_voltage_table(float v);
float estimate_hours_left_from_voltage(float soc_pct, float capacity_mAh, float I_prom_mA);
void peripheralsPowerInitPin();
void peripheralsPowerOn();
void peripheralsPowerOff();
bool measureOnce(float &temp_out, float &batt_out);
bool vaciarMochila_limited(uint32_t maxTimeMs, int maxLines, bool managePower);
// En la parte de arriba del archivo:
bool enviarDatoAlServidor(float t, float battery_v, float battery_pct, float battery_hours, const char* fecha_custom, uint32_t runIdToSend, uint32_t measurement_id, const char* source, bool force = false);
bool serverReachable(const char* host, uint16_t port, uint32_t timeoutMs);
void syncRTCFromNTP();
void sincronizarConServidor(float tempActual = lastTemp, bool forzarReporte = false);
void mqttCallback(char* topic, byte* payload, unsigned int length);
void connectWiFiIfNeeded();
void disconnectWiFiIfAllowed();
void guardarEnMochila(uint32_t seq, float temp, uint32_t rid, uint32_t ts, float bV, float bP, float bH);
uint32_t nextMeasurementSeq();
void enviarReporteActual(float t_hint, float v_hint, uint32_t ts_hint, bool force = false);
void enviarReporteActual(float t_hint, float v_hint);
void enviarReporteActual(float t_hint);
void applyConfig(uint32_t i_normal_s, uint32_t i_crit_s, float alerta, float critica, bool colocada, bool fromPrefs=false);
void maybeAdjustIntervalByTemp(float tref);
void forceDisconnectWiFi();
void ledBlink(int times, int onMs, int offMs);
void checkAndApplyOTA();

bool ensureValidRTC(uint32_t& ts_out) {
  if (!peripherals_on) {
    peripheralsPowerOn();
    delay(100);
  }
  
  if (!rtc.begin()) {
    safeLog(">>> [RTC] begin() FAIL\n");
    return false;
  }
  
  DateTime ahora = rtc.now();
  if (ahora.unixtime() < 1700000000UL) {  // Antes de 2023-11
    safeLog(">>> [RTC] Fecha inválida: %lu → Intentando NTP\n", ahora.unixtime());
    
    if (WiFi.status() == WL_CONNECTED) {
      syncRTCFromNTP();
      if (rtc.begin()) ahora = rtc.now();
    }
  }
  
  ts_out = ahora.unixtime();
  safeLog(">>> [RTC] Válido: %lu (%s)\n", ts_out, formatFechaSQL(ahora).c_str());
  return true;
}
bool isRTCValid() {
  peripheralsPowerOn();
  delay(100);
  
  if (!rtc.begin()) {
    peripheralsPowerOff();
    return false;
  }
  
  DateTime ahora = rtc.now();
  bool valid = (ahora.year() >= 2024 && ahora.year() <= 2030);
  
  safeLog(">>> [RTC-CHECK] %04d-%02d-%02d %02d:%02d:%02d -> %s\n",
          ahora.year(), ahora.month(), ahora.day(),
          ahora.hour(), ahora.minute(), ahora.second(),
          valid ? "VALID" : "INVALID");
  
  peripheralsPowerOff();
  return valid;
}
String getRssiQuality(int rssi) {
  if (rssi >= WIFI_RSSI_EXCELENTE) return "EXCELENTE";
  if (rssi >= WIFI_RSSI_BUENO) return "BUENO";
  if (rssi >= WIFI_RSSI_REGULAR) return "REGULAR";
  if (rssi >= WIFI_RSSI_DEBIL) return "DÉBIL";
  return "MUY DÉBIL";
}
// ---------------- UTIL ----------------
void safeLog(const char *fmt, ...) {
  char buf[512];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.print(buf);
}
inline void touchProgress() { lastProgressMillis = millis(); }
inline void tickInLongLoop() { if (WiFi.status() == WL_CONNECTED) mqttClient.loop(); yield(); touchProgress(); }
void checkSoftWatchdog() { if (lastProgressMillis==0) return; if ((millis()-lastProgressMillis) > STUCK_RESET_MS) { safeLog(">>> [WATCHDOG] reset\n"); delay(100); esp_restart(); } }

// ---------------- LED ----------------
// Patrón:  1 flash = arranque  |  2 flashes = WiFi OK  |  3 flashes = WiFi FAIL
//          5 rápidos = batería baja (<20%)  |  3 lentos = OTA descargando
void ledBlink(int times, int onMs, int offMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(onMs);
    digitalWrite(LED_PIN, LOW);
    if (i < times - 1) delay(offMs);
  }
  touchProgress();
}

void setLedState(bool on) {
  digitalWrite(LED_PIN, on ? HIGH : LOW);
}

void updateLedByMode(bool inConfigMode, bool inGrace, bool wifiConnected) {
  if (inConfigMode) {
    setLedState(true);
    return;
  }

  if (inGrace) {
    static unsigned long lastGraceToggle = 0;
    static bool graceLedState = false;
    const unsigned long gracePeriodMs = 1000;

    if (millis() - lastGraceToggle >= gracePeriodMs) {
      lastGraceToggle = millis();
      graceLedState = !graceLedState;
      setLedState(graceLedState);
    }
    return;
  }

  if (!wifiConnected) {
    static unsigned long lastNoWifiToggle = 0;
    static bool noWifiLedState = false;
    const unsigned long noWifiPeriodMs = 200;

    if (millis() - lastNoWifiToggle >= noWifiPeriodMs) {
      lastNoWifiToggle = millis();
      noWifiLedState = !noWifiLedState;
      setLedState(noWifiLedState);
    }
    return;
  }

  setLedState(false);
}

// ---------------- OTA via GitHub ----------------
// Sube firmware/version.txt y firmware/sonda_14.bin a tu repo y cambia las URLs arriba.
void checkAndApplyOTA() {
  if (WiFi.status() != WL_CONNECTED) return;
  safeLog(">>> [OTA] Verificando actualizacion (local=%s)...\n", FIRMWARE_VERSION);

  WiFiClientSecure verClient;
  verClient.setInsecure(); // sin verificacion de certificado (practico en embedded)
  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.begin(verClient, OTA_VERSION_URL);
  http.addHeader("Cache-Control", "no-cache");
  http.addHeader("Pragma", "no-cache");
  int code = http.GET();
  if (code != 200) {
    safeLog(">>> [OTA] version check fail code=%d\n", code);
    http.end(); return;
  }
  String remoteVer = http.getString();
  remoteVer.trim();
  http.end();

  safeLog(">>> [OTA] Local=%s Remote=%s\n", FIRMWARE_VERSION, remoteVer.c_str());
  if (remoteVer == FIRMWARE_VERSION) { safeLog(">>> [OTA] Firmware al dia\n"); return; }

  safeLog(">>> [OTA] Actualizando a v%s...\n", remoteVer.c_str());
  ledBlink(3, 200, 200); // 3 destellos lentos = descargando OTA
  touchProgress();

  WiFiClientSecure otaClient;
  otaClient.setInsecure();
  httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  t_httpUpdate_return ret = httpUpdate.update(otaClient, OTA_FIRMWARE_URL);
  switch (ret) {
    case HTTP_UPDATE_FAILED:
      safeLog(">>> [OTA] FALLO: %s\n", httpUpdate.getLastErrorString().c_str());
      break;
    case HTTP_UPDATE_NO_UPDATES:
      safeLog(">>> [OTA] Sin actualizacion disponible\n");
      break;
    case HTTP_UPDATE_OK:
      safeLog(">>> [OTA] OK - reiniciando...\n");
      break; // httpUpdate llama ESP.restart() automaticamente
  }
}

// --------- 1) Nueva función: validar fecha SQL ----------
/*
  Valida una cadena "YYYY-MM-DD HH:MM:SS".
  Devuelve true si los campos están dentro de rangos razonables.
*/
bool isValidSQLDatetime(const char *s) {
  if (!s) return false;
  // Formato esperado: 19 caracteres "YYYY-MM-DD HH:MM:SS"
  int y, mo, d, hh, mm, ss;
  int n = sscanf(s, "%d-%d-%d %d:%d:%d", &y, &mo, &d, &hh, &mm, &ss);
  if (n != 6) return false;
  if (y < 2000 || y > 2100) return false;          // rango razonable para el sistema
  if (mo < 1 || mo > 12) return false;
  if (d < 1 || d > 31) return false;
  if (hh < 0 || hh > 23) return false;
  if (mm < 0 || mm > 59) return false;
  if (ss < 0 || ss > 59) return false;
  // Validación simple; se puede reforzar checando días por mes/bisiesto.
  return true;
}
// ---------------- I2C / power helpers ----------------
// Pulsar SCL vía GPIO para intentar liberar SDA atascada.
bool recoverI2CBus(int sdaPin, int sclPin) {
  pinMode(sdaPin, INPUT_PULLUP);
  pinMode(sclPin, OUTPUT);
  digitalWrite(sclPin, HIGH);
  delay(1);

  if (digitalRead(sdaPin) == HIGH) {
    safeLog(">>> [I2C RECOVER] SDA already HIGH (pin %d)\n", sdaPin);
    return true;
  }

  safeLog(">>> [I2C RECOVER] SDA stuck LOW (pin %d), pulsing SCL\n", sdaPin);
  for (int i = 0; i < 12; ++i) {
    touchProgress();
    digitalWrite(sclPin, LOW);
    delayMicroseconds(300);
    digitalWrite(sclPin, HIGH);
    delayMicroseconds(300);
    if (digitalRead(sdaPin) == HIGH) {
      safeLog(">>> [I2C RECOVER] SDA released after %d pulses\n", i+1);
      return true;
    }
  }
  safeLog(">>> [I2C RECOVER] SDA still LOW after pulses\n");
  return false;
}

// Inicializa I2C y drivers UNA sola vez mediante un power-on controlado.
void initI2CAndDriversOnce() {
  safeLog(">>> [SETUP] temporal power-on para init I2C y drivers\n");
  pinMode(PWR_CTRL_PIN, OUTPUT);
  digitalWrite(PWR_CTRL_PIN, PWR_ON_LEVEL);
  delay(PWR_STABILIZE_MS + 200);

  Wire.begin(SDA_MAIN, SCL_MAIN);
  I2C_INA.begin(SDA_INA, SCL_INA);
  I2C_INA.setClock(100000);

  if (!rtcPresente) {
    if (rtc.begin()) { rtcPresente = true; safeLog(">>> [SETUP] RTC OK\n"); }
  }
  if (!ina_available) {
    if (maxlipo.begin(&I2C_INA)) {
      delay(300); // MAX17048 necesita ~250ms para primer ciclo ADC valido
      ina_available = true;
      safeLog(">>> [SETUP] MAX17048 OK (V=%.3fV SOC=%.1f%%)\n", maxlipo.cellVoltage(), maxlipo.cellPercent());
    } else {
      safeLog(">>> [SETUP] MAX17048 init FAIL\n");
    }
  }
  thermo.begin(MAX31865_3WIRE);

  digitalWrite(PWR_CTRL_PIN, PWR_OFF_LEVEL);
  delay(40);
  safeLog(">>> [SETUP] I2C y drivers inicializados una vez\n");
}

// Encender periféricos: NO reinit I2C (está inicializado en setup)
void peripheralsPowerInitPin() {
  pinMode(PWR_CTRL_PIN, OUTPUT);
  digitalWrite(PWR_CTRL_PIN, PWR_OFF_LEVEL);
  peripherals_on = false;
}

void peripheralsPowerOn() {
  if (peripherals_on) return;
  safeLog(">>> [PWR] POWER ON\n");

  // Liberar holds aplicados en peripheralsPowerOff() para que Wire.begin()
  // pueda reconfigurar los pines. Sin esto el driver I2C queda con handle
  // inválido → crash (LoadProhibited) en la segunda llamada a peripheralsPowerOn().
  gpio_hold_dis((gpio_num_t)LED_PIN);
  gpio_hold_dis((gpio_num_t)SDA_INA);
  gpio_hold_dis((gpio_num_t)SCL_INA);

  pinMode(PWR_CTRL_PIN, OUTPUT);   // ← necesario: peripheralsPowerOff deja el pin en INPUT
  digitalWrite(PWR_CTRL_PIN, PWR_ON_LEVEL);
  delay(350); 

  Wire.begin(SDA_MAIN, SCL_MAIN); 
  thermo.begin(MAX31865_3WIRE);

  // ✅ SOLO re-sincronizar RTC si está inválido
  if (rtcPresente) {
    if (rtc.lostPower()) {
      safeLog(">>> [PWR] RTC lost power → re-sync\n");
      rtcPresente = rtc.begin();
    } else {
      safeLog(">>> [PWR] RTC válido, NO re-sync\n");
    }
  } else {
    rtcPresente = rtc.begin();
  }
  ina_available = false;
  I2C_INA.begin(SDA_INA, SCL_INA);   // re-init bus tras flotar pines en power-off
  I2C_INA.setClock(100000);
  delay(50); // estabilizar bus
  if (maxlipo.begin(&I2C_INA)) {
    delay(300); // MAX17048 necesita ~250ms para primer ciclo ADC valido tras reinit
    float _v = maxlipo.cellVoltage();
    float _s = maxlipo.cellPercent();
    ina_available = true;
    safeLog(">>> [PWR] MAX17048 OK (V=%.3fV SOC=%.1f%%)\n", _v, _s);
  } else {
    safeLog(">>> [PWR] MAX17048 no responde\n");
  }
  peripherals_on = true;
  safeLog(">>> [PWR] peripherals_on=true\n");
}
void peripheralsPowerOff() {
  if (!peripherals_on) return;
  safeLog(">>> [PWR] POWER OFF\n");

  // Flotar bus principal (los periféricos del rail controlado se apagan)
  pinMode(SDA_MAIN, INPUT);
  pinMode(SCL_MAIN, INPUT);
  
  // Configurar pull-up interno en bus INA para que MAX17048 no drene corriente
  pinMode(SDA_INA, INPUT_PULLUP);
  pinMode(SCL_INA, INPUT_PULLUP);
  
  // Apagar el rail de periféricos:
  // AO3401A (P-ch) Source=3.3V (después del LDO). Cuando GPIO=INPUT (alta-Z),
  // el pull-up 10kΩ Gate-Source lleva Vgs→0V → corte total garantizado.
  delay(10);
  digitalWrite(PWR_CTRL_PIN, PWR_OFF_LEVEL); // preparar nivel antes de soltar
  pinMode(PWR_CTRL_PIN, INPUT);              // ← alta impedancia: pull-up 10k hace Vgs=0V

  // --- Congelar pines RTC para retener su estado en Deep Sleep ---
  // PWR_CTRL_PIN: NO se hace hold; el pull-up 10kΩ mantiene Vgs=0 (MOSFET OFF)
  // CS_PIN (GPIO5): NO es RTC IO → gpio_hold_en no aplica, se omite
  gpio_hold_en((gpio_num_t)LED_PIN);  // LED apagado durante sleep (GPIO27 = RTC IO)
  gpio_hold_en((gpio_num_t)SDA_INA); // (GPIO25 = RTC IO)
  gpio_hold_en((gpio_num_t)SCL_INA); // (GPIO26 = RTC IO)
  gpio_deep_sleep_hold_en(); // Habilita retención global durante Deep Sleep

  peripherals_on = false;
  safeLog(">>> [PWR] peripherals_off\n");
}

// ---------------- WiFi / MQTT helpers ----------------
void mqtt_subscribe_topics() {
  String base = String("sonda/") + SONDA_ID;
  mqttClient.subscribe((base + "/cmd").c_str());
  mqttClient.subscribe((base + "/state").c_str());
  mqttSubscribedAt = millis();
}
void connectWiFiIfNeeded() { 
  if (WiFi.status() == WL_CONNECTED) return;

  safeLog(">>> [WIFI-SMART] Conexión inteligente (TX=19.5dBm)...\n");
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.mode(WIFI_STA);
  
  // ⭐ RSSI PREDICTIVO: Escanea ANTES de conectar (200ms)
  int n = WiFi.scanNetworks(1);  // Solo 1 red = súper rápido
  int targetRssi = -100;
  
  for (int i = 0; i < n; i++) {
    if (WiFi.SSID(i) == String(WIFI_SSID)) {
      targetRssi = WiFi.RSSI(i);
      break;
    }
  }
  
  // ⭐ ADAPTATIVO: Señal → Retries
  int maxRetries = (targetRssi > -70) ? 1 : 
                   (targetRssi > -85) ? 2 : 3;
                   
  safeLog(">>> [WIFI-SMART] SSID='%s' RSSI_pred=%ddBm → %d retries\n", 
          WIFI_SSID, targetRssi, maxRetries);

  // ⭐ BUCLE ROBUSTO (adaptativo)
  for (int attempt = 1; attempt <= maxRetries; ++attempt) {
    safeLog(">>> [WIFI] Intento %d/%d...\n", attempt, maxRetries);

    safeLog(">>> [WIFI] connecting...\n");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < WIFI_CONNECT_TIMEOUT_MS) {
      checkSoftWatchdog();
      delay(200);
    }

    if (WiFi.status() != WL_CONNECTED) {
      safeLog(">>> [WIFI] connect timeout (intento %d)\n", attempt);
      if (attempt < maxRetries) delay(1000);
      continue;
    }

    safeLog(">>> [WIFI] connected IP=%s\n", WiFi.localIP().toString().c_str());

    // ---- DNS con timeout (mejora) ----
serverIpOk = WiFi.hostByName(SERVER_HOST, serverIp);
    safeLog(">>> [DNS] %s -> %s (%s)\n",
            SERVER_HOST,
            serverIpOk ? serverIp.toString().c_str() : "FAIL",
            serverIpOk ? "OK" : "ERR");

    // ---- MQTT + Presence (LWT) ----
    if (serverIpOk) mqttClient.setServer(serverIp, MQTT_PORT);
    else            mqttClient.setServer(MQTT_HOST, MQTT_PORT);

    mqttClient.setCallback(mqttCallback);
    mqttClient.setKeepAlive(10);

    if (!mqttClient.connected()) {
      String willTopic = String("sonda/") + SONDA_ID + "/presence";

      bool ok = mqttClient.connect(
        SONDA_ID,
        willTopic.c_str(),
        1,
        true,
        "offline"
      );

      if (ok) {
        mqtt_subscribe_topics();
        mqttClient.publish(willTopic.c_str(), "online", true);
        safeLog(">>> [MQTT] connected and subscribed (presence=online)\n");
      } else {
        safeLog(">>> [MQTT] connect failed rc=%d\n", mqttClient.state());
      }
    }
    
    safeLog(">>> [WIFI-SMART] ✅ RSSI=%ddBm\n", WiFi.RSSI());
    ledBlink(2, 150, 100); // 2 flashes = WiFi conectado
    WiFi.scanDelete(); // Limpia scan previo
    return;  // ÉXITO
  }
  
  safeLog(">>> [WIFI-SMART] ❌ Falló todos los intentos (RSSI_pred=%ddBm)\n", targetRssi);
  ledBlink(3, 100, 100); // 3 flashes rapidos = WiFi falló
  WiFi.scanDelete(); // Limpieza
}
void disconnectWiFiIfAllowed() {
  if (keepWifiDuringGrace || enGracia || modoOnlineForzado) {
    safeLog(">>> [WIFI] keep ON for GRACE/ONLINE\n");
    return;
  }

  // ---- Presence OFFLINE (apagado "limpio") ----
  if (mqttClient.connected()) {
    String willTopic = String("sonda/") + SONDA_ID + "/presence";
    mqttClient.publish(willTopic.c_str(), "offline", true); // retained
    mqttClient.loop();   // intenta enviar antes de cortar
    delay(50);
  }

  if (mqttClient.connected()) {
    mqttClient.disconnect();
    safeLog(">>> [MQTT] disconnected\n");
  }

  if (WiFi.status() == WL_CONNECTED) {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    safeLog(">>> [WIFI] disconnected\n");
  }
}
void maybeAdjustIntervalByTemp(float tref) {
  // Sin temperatura: aplicar intervalo NORMAL del servidor de todas formas.
  // Sin este fallback, el intervalo descargado del servidor nunca se aplica
  // cuando el MAX31865 no responde y tref queda NAN.
  if (isnan(tref)) {
    modoOnlineForzado = false;
    estado.t_reporte_ms = T_REPORT_NORMAL_MS;
    safeLog(">>> [ESTADO: NORMAL] T:NAN (sin lectura). Reporte: %ums\n", T_REPORT_NORMAL_MS);
    return;
  }

  // 1. ALERTA (Lo más grave: >= T_ALERTA, ej. 60°C)
  // En este modo la sonda debe quedarse ONLINE reportando frecuentemente
  if (tref >= T_ALERTA) {
    modoOnlineForzado = true;   // FIX: activa modo online continuo
    keepWifiDuringGrace = true; // FIX: no desconectar WiFi
    estado.t_reporte_ms =  T_REPORT_ALERTA_MS;
    safeLog(">>> [ESTADO: ALERTA] T:%.2f >= %.2f. Reporte: %ums (Online FORZADO)\n", tref, T_ALERTA, T_REPORT_ALERTA_MS);
  } 
  // 2. CRÍTICO (Intermedio: >= T_CRITICA)
  else if (tref >= T_CRITICA) {
    modoOnlineForzado = false;
    
    estado.t_reporte_ms = T_REPORT_CRITICAL_MS; // 15s
    safeLog(">>> [ESTADO: CRITICO] T:%.2f >= %.2f. Reporte: %ums\n", tref, T_CRITICA, T_REPORT_CRITICAL_MS);
  } 
  // 3. NORMAL (< T_CRITICA)
  else {
    modoOnlineForzado = false;
    
    estado.t_reporte_ms = T_REPORT_NORMAL_MS; // 60s
    safeLog(">>> [ESTADO: NORMAL] T:%.2f. Reporte: %ums\n", tref, T_REPORT_NORMAL_MS);
  }
}
void forceDisconnectWiFi() {
  if (mqttClient.connected()) {
    mqttClient.disconnect();
    safeLog(">>> [WIFI] mqtt disconnected (force)\n");
  }
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    safeLog(">>> [WIFI] disconnected (force)\n");
  }
  touchProgress();
}

// ---------------- MOCHILA / HTTP ----------------
uint32_t nextMeasurementSeq() {
  estado.last_seq++;
  preferences.begin("sonda", false);
  preferences.putUInt("last_seq", estado.last_seq);
  preferences.end();
  return estado.last_seq;
}

void guardarEnMochila(uint32_t seq, float temp, uint32_t rid, uint32_t ts, float bV, float bP, float bH) {
  // ✅ NUEVO: NO guardar si ts inválido
  if (ts < 1700000000UL) {
    safeLog(">>> [MOCHILA] ts inválido %u → SKIP\n", ts);
    return;
  }
  
  File f = LittleFS.open("/mochila.txt", FILE_APPEND);
  if (!f) {
    safeLog(">>> [MOCHILA] open FAIL\n");
    return;
  }

  char line[256];
  int n = snprintf(line, sizeof(line), "%u,%u,%.4f,%u,%.3f,%.2f,%.2f\n", 
                   seq, rid, temp, ts, bV, bP, bH);
  
  if (n > 0) f.print(line);
  f.close();
  safeLog(">>> [MOCHILA] OK seq=%u T=%.2f ts=%u\n", seq, temp, ts);
}


bool enviarDatoAlServidor(float t, float battery_v, float battery_pct, float battery_hours, const char* fecha_custom, uint32_t runIdToSend, uint32_t measurement_id, const char* source, bool force) {
  touchProgress();
  if (WiFi.status() != WL_CONNECTED) { safeLog(">>> [HTTP][%s] no wifi\n", source); return false; }

  // --- BLOQUE DE CONTROL DE FRECUENCIA ---
  if (strcmp(source, "LIVE") == 0) {
    if (!force) { 
      if (lastLiveSendMillis != 0 && (millis() - lastLiveSendMillis) < MIN_LIVE_GAP_MS) {
        safeLog(">>> [HTTP] skip recent\n");
        return true;
      }
    }
  }

  const int MAX_TRIES = 3; unsigned long delayMs = 300;
  uint32_t run_to_send = (runIdToSend != 0) ? runIdToSend : estado.run_id;

  // --- PREPARACIÓN DEL JSON ---
  StaticJsonDocument<512> doc; // Aumentado un poco para metadatos extra
  doc["id_sonda"] = SONDA_ID;
  doc["temperatura"] = isnan(t) ? 0.0f : t;
  
  // Voltaje puro de batería
  float v_final = (!isnan(battery_v)) ? battery_v : (!isnan(lastBatteryV) ? lastBatteryV : 0.0f);
  doc["bateria_v"] = v_final; 

  // SOC del MAX17048 (fuel gauge de hardware - más preciso que interpolación por voltaje)
  // lastBatteryPct se actualiza en measureOnce() con maxlipo.cellPercent()
  if (!isnan(lastBatteryPct) && lastBatteryPct >= 0.0f) {
    doc["bateria_pct"] = lastBatteryPct;
  }

  // Metadatos para la predicción inteligente
  doc["rssi"] = WiFi.RSSI(); 
  doc["uptime_s"] = millis() / 1000;
  doc["bateria_nominal_mAh"] = BATTERY_CAPACITY_mAh;
  
  if (run_to_send > 0) doc["run_id"] = run_to_send;
  if (measurement_id != 0) doc["measurement_id"] = measurement_id;
  if (fecha_custom && strcmp(fecha_custom, "NULL") != 0) doc["fecha_custom"] = fecha_custom;

  String body; serializeJson(doc, body);

  // --- LÓGICA DE DEDUPLICACIÓN (KeyBuf) ---
  char keyBuf[128];
  if (measurement_id != 0) snprintf(keyBuf, sizeof(keyBuf), "%s|%u|%u", SONDA_ID, run_to_send, measurement_id);
  else snprintf(keyBuf, sizeof(keyBuf), "%s|%u|%.4f|%s", SONDA_ID, run_to_send, isnan(t) ? 0.0f : t, (fecha_custom ? fecha_custom : "NULL"));

  if (strlen(lastSentKey) > 0 && strcmp(keyBuf, lastSentKey) == 0 && (millis() - lastSentKeyMillis) < PAYLOAD_DUP_WINDOW_MS) {
    safeLog(">>> [HTTP] dedupe skip\n");
    lastSentKeyMillis = millis();
    return true;
  }

  // --- BUCLE DE ENVÍO CON REINTENTOS ---
  for (int attempt = 1; attempt <= MAX_TRIES; ++attempt) {
    touchProgress();
    if (!serverReachable(SERVER_HOST, SERVER_PORT, 1500)) { 
      safeLog(">>> [HTTP] server not reachable attempt %d\n", attempt); 
      delay(delayMs); tickInLongLoop(); delayMs = min(delayMs * 2, 2000UL); continue; 
    }

    HTTPClient http;
String host = serverIpOk ? serverIp.toString() : String(SERVER_HOST);
String url  = String("http://") + host + ":" + String(SERVER_PORT) + "/api/reporte_sonda";
http.begin(httpNet, url);
    http.addHeader("Content-Type", "application/json");
    http.setTimeout(8000);

    int code = http.POST((uint8_t*)body.c_str(), body.length());
    String resp = http.getString();
    http.end();

    if (code == 200 || code == 201) {
      strncpy(lastSentKey, keyBuf, sizeof(lastSentKey) - 1); 
      lastSentKey[sizeof(lastSentKey) - 1] = 0; 
      lastSentKeyMillis = millis();
      
      safeLog(">>> [HTTP] sent OK run=%u T=%.2f V=%.3f RSSI=%d (try %d)\n", 
              run_to_send, isnan(t) ? 0.0f : t, v_final, (int)doc["rssi"], attempt);
      
      if (measurement_id != 0) { 
        lastLiveSeq = measurement_id; 
        lastLiveTemp = t; 
        lastLiveRun = run_to_send; 
        lastLiveMillis = millis(); 
      }
      if (strcmp(source, "LIVE") == 0) { 
        lastLiveSendMillis = millis(); 
        lastRuntimeReportMillis = millis(); 
      }
      return true;
    } else {
      safeLog(">>> [HTTP] POST fail code=%d (try %d)\n", code, attempt);
    }
    delay(delayMs); tickInLongLoop(); delayMs = min(delayMs * 2, 2000UL);
  }

  safeLog(">>> [HTTP] all attempts failed -> mochila\n");
  return false;
}
// Función auxiliar para verificar si el servidor está escuchando
bool serverReachable(const char* host, uint16_t port, uint32_t timeoutMs) {
  WiFiClient checkClient;
  // Intenta conectar al socket del servidor
  if (checkClient.connect(host, port, timeoutMs)) {
    checkClient.stop();
    return true;
  }
  return false;
}
// ---------------- vaciarMochila_limited ----------------
bool vaciarMochila_limited(uint32_t maxTimeMs, int maxLines, bool managePower) {
  touchProgress();
  if (WiFi.status() != WL_CONNECTED) return false;
  if (!LittleFS.exists("/mochila.txt")) { mochilaHasData = false; return true; }
  
  // Verificación rápida del servidor
  if (!serverReachable(SERVER_HOST, SERVER_PORT, 1500)) return false;

  File f = LittleFS.open("/mochila.txt", FILE_READ);
  if (!f) return true;
  File tmp = LittleFS.open("/mochila.tmp", FILE_WRITE);
  if (!tmp) { f.close(); return false; }

  safeLog(">>> [MOCHILA] Flushing data (safe mode)...\n");
  unsigned long start = millis(); 
  int sent = 0;
  bool limitReached = false;

  while (f.available()) {
    checkSoftWatchdog();
    
    String line = f.readStringUntil('\n'); 
    line.trim();
    if (line.length() == 0) continue;

    // Si ya llegamos al límite de tiempo o líneas, activamos el modo "solo copia"
    if (!limitReached) {
      if ((millis() - start) >= maxTimeMs || sent >= maxLines) {
        limitReached = true;
      }
    }

    // SI NO HEMOS LLEGADO AL LÍMITE: Intentamos enviar
    if (!limitReached) {
      uint32_t seq, rId, tsVal;
      float tVal, bV, bP, bH;
      
      int found = sscanf(line.c_str(), "%u,%u,%f,%u,%f,%f,%f", &seq, &rId, &tVal, &tsVal, &bV, &bP, &bH);

      if (found < 3) {
        // Si la línea está mal, no la enviamos pero la mantenemos para no perder nada
        tmp.println(line); 
        continue; 
      }

      char fechaBufTmp[32] = "NULL";
     if (tsVal > 1600000000UL && tsVal < 2147483647UL) {
  // ❌ PROBLEMA: ts_local = tsVal - 10800UL (DOUBLE CORRECTION!)
  
  // ✅ CORRECCIÓN: El RTC ya está en UTC-3, NO restar más
  DateTime dt(tsVal);  // tsVal ya está en zona local (UTC-3)
  snprintf(fechaBufTmp, sizeof(fechaBufTmp), "%04d-%02d-%02d %02d:%02d:%02d",
           dt.year(), dt.month(), dt.day(), dt.hour(), dt.minute(), dt.second());
}

      bool ok = enviarDatoAlServidor(tVal, bV, bP, bH, fechaBufTmp, rId, seq, "FLUSH");
      
      if (ok) {
        sent++;
        lastLiveSeq = seq; 
      } else {
        // Si el envío falla (ej. timeout), guardamos la línea para el próximo intento
        tmp.println(line);
      }
    } 
    else {
      // SI YA LLEGAMOS AL LÍMITE: Copiamos directo al temporal sin intentar enviar
      tmp.println(line);
    }
  }

  f.close(); 
  tmp.close();

  // Reemplazo seguro del archivo
  LittleFS.remove("/mochila.txt"); 
  if (LittleFS.exists("/mochila.tmp")) {
    File checkTmp = LittleFS.open("/mochila.tmp", FILE_READ);
    if (checkTmp.size() > 0) {
      checkTmp.close();
      LittleFS.rename("/mochila.tmp", "/mochila.txt");
    } else {
      checkTmp.close();
      LittleFS.remove("/mochila.tmp");
    }
  }

  mochilaHasData = LittleFS.exists("/mochila.txt");
  safeLog(">>> [MOCHILA] Fin: %d enviados. Sigue con datos: %s\n", sent, mochilaHasData ? "SI" : "NO");
  
  return !mochilaHasData;
}



// measure once (encender gestionado por caller)
bool measureOnce(float &temp_out, float &batt_out) {
  float tSum = 0, vSum = 0; 
  int cntT = 0, cntIna = 0;
  float vReposoclean = NAN;

  // 1. LECTURA DE REPOSO
  if (ina_available) {
    delay(10); // estabilizar MAX17048
    vReposoclean = maxlipo.cellVoltage();
  }

  // 2. INICIALIZACIÓN DE SENSORES
  thermo.begin(MAX31865_3WIRE);
  I2C_INA.setClock(100000);

  // 3. BUCLE DE MUESTREO PROMEDIADO (Tu lógica original)
  for (int s = 0; s < SAMPLES_AVG; ++s) {
    touchProgress();
    delay(SAMPLE_DELAY_MS);

    // Medición de Temperatura
    float t = thermo.temperature(100.0, 430.0);
    uint8_t fault = thermo.readFault();
    if (!fault && !isnan(t) && t > -50.0f && t < 300.0f) {
      tSum += t;
      ++cntT;
    } else {
      safeLog(">>> [THERMO] fault=0x%02X T=%.3f (sample %d)\n", fault, t, s);
      thermo.clearFault();
    }

    // Medición de Batería (durante el proceso)
    if (ina_available) {
      float batt = maxlipo.cellVoltage();
      if (!isnan(batt) && batt > 0.5f && batt < 6.0f) {
        vSum += batt;
        ++cntIna;
      }
    }
    checkSoftWatchdog();
  }

  // 4. PROCESAMIENTO DE RESULTADOS
  temp_out = (cntT > 0) ? (tSum / (float)cntT) : NAN;
  
  // Para batt_out preferimos la lectura "limpia" inicial, 
  // si falló, usamos el promedio del bucle.
  if (!isnan(vReposoclean) && vReposoclean > 0.5f) {
    batt_out = vReposoclean;
  } else {
    batt_out = (cntIna > 0) ? (vSum / (float)cntIna) : NAN;
  }

  // 5. ACTUALIZACIÓN DE GLOBALES
  if (!isnan(temp_out)) { 
    lastTemp = temp_out; 
    lastTempMillis = millis(); 
  }
  if (!isnan(batt_out)) {
    lastBatteryV = batt_out;
    if (ina_available) {
      float soc = maxlipo.cellPercent();
      if (!isnan(soc) && soc >= 0.0f) lastBatteryPct = soc;
    }
  }

  return (cntT > 0) || (cntIna > 0);
}
void enviarReporteActual(float t_hint, float v_hint, uint32_t ts_hint, bool force) {
  touchProgress();

  if (estado.run_id == 0 && !force) return;

  float temp_avg = t_hint;
  float batt_v_avg = v_hint;
  uint32_t ts_capturado = ts_hint;
  char fechaBuf[32] = "NULL";

  // 1. Medición (mantener MQTT vivo mientras medimos)
  // Solo re-medir si falta la temperatura. La batería tiene fallback (lastBatteryV)
  // y no debe forzar un measureOnce() extra cuando temp_avg ya es válido.
  if (isnan(temp_avg)) {
    tickInLongLoop();

    peripheralsPowerOn();
    tickInLongLoop();

    measureOnce(temp_avg, batt_v_avg);
    tickInLongLoop();

    if (ts_capturado == 0) {
  ensureValidRTC(ts_capturado);
}

    peripheralsPowerOff();
    tickInLongLoop();
  }

  if (isnan(temp_avg)) temp_avg = lastTemp;
  if (isnan(batt_v_avg)) batt_v_avg = lastBatteryV;

  // --- AJUSTE DE HORA PARA EL SERVIDOR ---
 if (ts_capturado > 0) {
  // ❌ PROBLEMA: uint32_t ts_local = ts_capturado - 10800UL;
  
  // ✅ CORRECCIÓN:
  DateTime dt(ts_capturado);  // RTC ya está en UTC-3
  
  snprintf(fechaBuf, sizeof(fechaBuf), "%04d-%02d-%02d %02d:%02d:%02d",
           dt.year(), dt.month(), dt.day(), dt.hour(), dt.minute(), dt.second());
}

  tickInLongLoop();
  connectWiFiIfNeeded();
  tickInLongLoop();

  uint32_t seq = nextMeasurementSeq();
  uint32_t id_a_enviar = estado.run_id;

  if (WiFi.status() != WL_CONNECTED) {
    if (id_a_enviar > 0) {
      guardarEnMochila(seq, temp_avg, id_a_enviar, ts_capturado, batt_v_avg, 0, 0);
    }
    return;
  }

  // Mantener MQTT vivo durante flush/HTTP
  tickInLongLoop();
  vaciarMochila_limited(60000, 600, false);
  tickInLongLoop();

  bool sent = enviarDatoAlServidor(temp_avg, batt_v_avg, 0, 0, fechaBuf, id_a_enviar, seq, "LIVE", force);
  tickInLongLoop();

  if (!sent) {
    if (id_a_enviar > 0) {
      guardarEnMochila(seq, temp_avg, id_a_enviar, ts_capturado, batt_v_avg, 0, 0);
    }
  } else {
    lastTemp = temp_avg;
    lastBatteryV = batt_v_avg;
    lastLiveMillis = millis();
    flushedThisBoot = true;
  }

  // Si estamos en GRACE o modo online, NO cortes MQTT/WiFi después de un reporte inmediato.
  if (!enGracia && !keepWifiDuringGrace && !modoOnlineForzado) {
    disconnectWiFiIfAllowed();
  } else {
    safeLog(">>> [WIFI] keep ON (post-report)\n");
  }

  touchProgress();
}

// Sobrecargas (Mantienen la lógica de llamar con force = false por defecto)
void enviarReporteActual(float t_hint, float v_hint) {
  enviarReporteActual(t_hint, v_hint, 0, false);
}

void enviarReporteActual(float t_hint) {
  enviarReporteActual(t_hint, NAN, 0, false);
}
void syncRTCFromNTP() {
  if (WiFi.status() != WL_CONNECTED) {
    safeLog(">>> [RTC] No WiFi para NTP\n");
    return;
  }
  
  safeLog(">>> [RTC] NTP sync start\n");
  
  // Configurar zona horaria UTC-3 (Argentina/Chile)
  configTime(-3 * 3600, 0, "pool.ntp.org", "time.nist.gov", "south-america.pool.ntp.org");
  
  // Esperar sincronización NTP (máximo 15 segundos)
  unsigned long start = millis();
  struct tm timeinfo;
  
  while ((millis() - start) < 15000) {
    if (getLocalTime(&timeinfo)) {
      safeLog(">>> [RTC] NTP sincronizado: %04d-%02d-%02d %02d:%02d:%02d\n",
              timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
              timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
      
      // Power ON y ajustar RTC con hora LOCAL (-3 UTC)
      peripheralsPowerOn();
      delay(200); // Estabilizar
      
      if (rtc.begin()) {
        rtc.adjust(DateTime(
          timeinfo.tm_year + 1900,
          timeinfo.tm_mon + 1,
          timeinfo.tm_mday,
          timeinfo.tm_hour,
          timeinfo.tm_min,
          timeinfo.tm_sec
        ));
        rtcPresente = true;
        safeLog(">>> [RTC] Ajustado OK\n");
        peripheralsPowerOff();
        return;
      }
      peripheralsPowerOff();
      safeLog(">>> [RTC] NTP OK pero RTC adjust FAIL\n");
      return;
    }
    delay(500);
    checkSoftWatchdog();
  }
  
  safeLog(">>> [RTC] NTP timeout\n");
}
// ---------------- sincronizarConServidor ----------------
// Usamos un valor por defecto (lastTemp) para que sea compatible con llamadas vacías
void sincronizarConServidor(float tempActual, bool forzarReporte) { 
  touchProgress();
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http; 
  // --- PASO 1: Obtener Umbrales y Tiempos ---
 String host = serverIpOk ? serverIp.toString() : String(SERVER_HOST);

String urlConfig = "http://" + host + ":" + String(SERVER_PORT) + "/api/reporte_sonda";
http.begin(httpNet, urlConfig); 
  http.addHeader("Content-Type", "application/json");
  // Enviamos temperatura solo para que el server sepa que estamos vivos
  String jsonPayload = "{\"id_sonda\":\"" + String(SONDA_ID) + "\", \"temperatura\":" + String(tempActual) + ", \"solo_config\":true}";
  
  if (http.POST(jsonPayload) == 200) {
    StaticJsonDocument<512> res; 
    deserializeJson(res, http.getString());
    if (res.containsKey("config")) {
      T_ALERTA  = res["config"]["T_ALERTA"].as<float>();
      T_CRITICA = res["config"]["T_CRITICA"].as<float>();
      T_REPORT_NORMAL_MS   = res["config"]["T_REPORT_NORMAL_S"].as<uint32_t>() * 1000UL;
      // FIX: también actualizamos los intervalos crítico y alerta si el servidor los manda
      if (res["config"].containsKey("T_REPORT_CRITICAL_S"))
        T_REPORT_CRITICAL_MS = res["config"]["T_REPORT_CRITICAL_S"].as<uint32_t>() * 1000UL;
      if (res["config"].containsKey("T_REPORT_ALERTA_S"))
        T_REPORT_ALERTA_MS   = res["config"]["T_REPORT_ALERTA_S"].as<uint32_t>() * 1000UL;
      safeLog(">>> [SYNC] Config OK. Umbrales: %.1f/%.1f Intervalos: %u/%u/%u ms\n",
              T_ALERTA, T_CRITICA, T_REPORT_NORMAL_MS, T_REPORT_CRITICAL_MS, T_REPORT_ALERTA_MS);
    }
  }
  http.end();

  // --- PASO 2: Verificar ID Real ---
  
String urlEstado = "http://" + host + ":" + String(SERVER_PORT) + "/api/estado/" + String(SONDA_ID);
http.begin(httpNet, urlEstado);
  
  if (http.GET() == 200) {
    StaticJsonDocument<256> resEstado;
    deserializeJson(resEstado, http.getString());
    if (resEstado.containsKey("prueba_activa")) {
      // Actualizamos el ID local
      estado.run_id = resEstado["prueba_activa"].as<uint32_t>();
    }
  }
  http.end();
// --- PASO 3: Persistencia de Configuración ---
  preferences.begin("sonda", false);
  preferences.putFloat("t_alerta", T_ALERTA);
  preferences.putFloat("t_critica", T_CRITICA);
  
  // ¡FUNDAMENTAL! Guardar los intervalos que bajaste del server
  preferences.putUInt("t_norm_ms", T_REPORT_NORMAL_MS);
  preferences.putUInt("t_crit_ms", T_REPORT_CRITICAL_MS);
  preferences.putUInt("t_aler_ms", T_REPORT_ALERTA_MS);
  
  preferences.putUInt("run_id", estado.run_id);
  // También guardamos el intervalo actual decidido por la temperatura
  preferences.putUInt("t_reporte_ms", estado.t_reporte_ms); 
  preferences.end();
  
  safeLog(">>> [PERSISTENCIA] Intervalos guardados en Flash.\n");
}
// ---------------- MQTT callback ----------------
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  touchProgress();
  StaticJsonDocument<512> doc; 
  DeserializationError err = deserializeJson(doc, payload, length);
  
  String cmd;
  if (!err && doc.containsKey("cmd")) {
    cmd = String(doc["cmd"].as<const char*>());
  } else {
    String s;
    for (unsigned int i = 0; i < length; i++) s += (char)payload[i];
    cmd = s;
  }
  cmd.toLowerCase();
  safeLog(">>> [MQTT] cmd=%s\n", cmd.c_str());

 if (cmd == "iniciar") {
  uint32_t id = 0;
  if (!err && doc.containsKey("id_prueba")) id = doc["id_prueba"].as<uint32_t>();

  if (id) {
    safeLog(">>> [MQTT] iniciar (deferred) RUN=%u\n", id);
    pendingStartRun = id;
    pendingStart = true;
    keepWifiDuringGrace = true; // importante: no cortar WiFi mientras procesamos
  }
}
else if (cmd == "finalizar") {
  safeLog(">>> [MQTT] finalizar (deferred)\n");

  pendingFinalizePrevRun = estado.run_id; // run actual para cerrar bien
  pendingFinalize = true;
  keepWifiDuringGrace = true;

  // QUITA ESTO: no lo manejes desde el callback
  // enGracia = true;
}
else if (cmd == "wakeup" || cmd == "wakeup_report" || cmd == "actualizar") {
  safeLog(">>> [MQTT] wakeup_report/actualizar (deferred)\n");
  pendingWakeupReport = true;
  keepWifiDuringGrace = true;
}
else if (cmd == "config_changed") {
  safeLog(">>> [MQTT] config_changed -> Sincronizando con servidor...\n");

  keepWifiDuringGrace = true;   // importante si estás en GRACE
  connectWiFiIfNeeded();

  // 1) bajar config + run_id desde server
  sincronizarConServidor(lastTemp, true);

  // 2) recalcular intervalo local con los nuevos umbrales
  maybeAdjustIntervalByTemp(lastTemp);

  // 3) REPORTE INMEDIATO con la config ya aplicada
  safeLog(">>> [MQTT] config_changed -> enviando reporte inmediato (intervalo=%u ms)\n", estado.t_reporte_ms);
  enviarReporteActual(NAN, NAN, 0, true);   // force=true para que salga sí o sí
proxRep = millis() + estado.t_reporte_ms;
  safeLog(">>> [MQTT] proxRep RESET a %ums (config nueva aplicada)\n", estado.t_reporte_ms);
  // 4) opcional: limpiar retained state (como ya hacías)
  String stateTopic = String("sonda/") + SONDA_ID + "/state";
  mqttClient.publish(stateTopic.c_str(), "", true);
}
  else if (cmd == "clear_mochila") {
    if (LittleFS.exists("/mochila.txt")) {
      LittleFS.remove("/mochila.txt");
      mochilaHasData = false;
      safeLog(">>> [MQTT] Mochila borrada.\n");
    } else {
      safeLog(">>> [MQTT] Mochila ya estaba vacía.\n");
    }
  } 
  else if (cmd.length() > 0) {
  safeLog(">>> [MQTT] cmd desconocido: %s\n", cmd.c_str());
}
  touchProgress();
}

// ---------------- formatFechaSQL ----------------
String formatFechaSQL(const DateTime &dt) {
  char b[25];
  snprintf(b, sizeof(b), "%04d-%02d-%02d %02d:%02d:%02d", dt.year(), dt.month(), dt.day(), dt.hour(), dt.minute(), dt.second());
  return String(b);
}

bool is_authenticated() {
  if (server.hasHeader("Cookie")) {
    if (server.header("Cookie").indexOf(cookie_name) != -1) return true;
  }
  return false;
}

void handleSave() {
  if (!is_authenticated()) { server.sendHeader("Location", "/login"); server.send(302); return; }
  
  preferences.begin("sonda_wifi", false);
  preferences.putString("ssid", server.arg("s"));
  preferences.putString("pass", server.arg("p"));
  preferences.putString("snum", server.arg("id"));
  preferences.putString("srv_h", server.arg("sh"));
  preferences.putInt("srv_p", server.arg("sp").toInt());
  preferences.putString("mqtt_h", server.arg("mh"));
  preferences.end();
  
  server.send(200, "text/html", "<html><body><h1>Guardado. Reiniciando...</h1></body></html>");
  delay(2000);
  esp_restart();
}

void setupAP() {
WiFi.mode(WIFI_AP);
WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
WiFi.softAP("ExoTerLog_Pro_Config", "Sonda123");
setLedState(true);

// El asterisco "*" significa: "Cualquier URL que busquen, mandalos a mi IP"
dnsServer.start(53, "*", IPAddress(192, 168, 4, 1));

  const char * headerkeys[] = {"Cookie"};
  server.collectHeaders(headerkeys, 1);

  server.on("/", []() {
    if (!is_authenticated()) { server.sendHeader("Location", "/login"); server.send(302); }
    else { server.send(200, "text/html", getHTML()); }
  });

  server.on("/login", HTTP_GET, []() { server.send(200, "text/html", login_html); });
  
  server.on("/login", HTTP_POST, []() {
    if (server.arg("username") == valid_username && server.arg("password") == valid_password) {
      server.sendHeader("Set-Cookie", String(cookie_name) + "=1");
      server.sendHeader("Location", "/");
      server.send(302);
    } else {
      server.send(200, "text/html", login_html); // Podrías agregar un error aquí
    }
  });

  server.on("/save", handleSave);
  server.on("/exit", []() { server.send(200, "text/html", "Reinicio..."); delay(1000); esp_restart(); });
  //server.onNotFound([]() { server.sendHeader("Location", "/", true); server.send(302); });
  server.onNotFound([]() {
    server.sendHeader("Location", "/login", true); 
    server.send(302, "text/plain", ""); 
  });
  server.begin();
  modoConfigActivo = true;
}

void setup() {

  gpio_deep_sleep_hold_dis();
  gpio_hold_dis((gpio_num_t)PWR_CTRL_PIN);
  gpio_hold_dis((gpio_num_t)LED_PIN);
  gpio_hold_dis((gpio_num_t)CS_PIN);
  gpio_hold_dis((gpio_num_t)SDA_INA);
  gpio_hold_dis((gpio_num_t)SCL_INA);
    pinMode(0, INPUT_PULLUP);
  delay(200);  // ← Estabiliza boot
Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(PIN_BOTON_CONFIG, INPUT_PULLUP);

  // 1. Cargar preferencias
  preferences.begin("sonda_wifi", true);
  p_ssid = preferences.getString("ssid", "");
  p_pass = preferences.getString("pass", "");
  p_sonda_num = preferences.getString("snum", "");
  p_srv_host = preferences.getString("srv_h", "sonda-server.local");
  p_srv_port = preferences.getInt("srv_p", 5000);
  p_mqtt_host = preferences.getString("mqtt_h", "sonda-server.local");
  preferences.end();

  // 2. Asignar a variables de trabajo (con concatenación del ID)
  // Formato: sonda_01..sonda_09 para num 1-9, sonda_10+ para >= 10
  String full_id = (p_sonda_num.toInt() < 10) ? ("sonda_0" + p_sonda_num) : ("sonda_" + p_sonda_num);
  SONDA_ID = strdup(full_id.c_str());
  WIFI_SSID = strdup(p_ssid.c_str());
  WIFI_PASSWORD = strdup(p_pass.c_str());
  SERVER_HOST = strdup(p_srv_host.c_str());
  SERVER_PORT = p_srv_port;
  MQTT_HOST = strdup(p_mqtt_host.c_str());

  // 3. Verificar si el técnico apretó el botón al encender
  if (digitalRead(PIN_BOTON_CONFIG) == LOW) {
    Serial.println(">>> MODO CONFIGURACION ACTIVADO");
    setupAP();
    while (true) { // Bucle infinito de configuración
      dnsServer.processNextRequest();
      server.handleClient();
      updateLedByMode(true, false, false);
      delay(10);
    }
  }




  delay(200);
  touchProgress();
  safeLog(">>> [BOOT] setup start\n");



  //esp_sleep_wakeup_cause_t caus = esp_sleep_get_wakeup_cause();
  //if (caus != ESP_SLEEP_WAKEUP_TIMER) {
   // setupAP(); // Activa el portal solo si es arranque manual (Cold Boot)
  //}

  setenv("TZ", "<-03>3", 1);
  tzset();
  btStop();
  ledBlink(1, 80, 0); // 1 flash = arranque
  if (!LittleFS.begin(true)) safeLog(">>> [ERROR] LittleFS init failed\n");
  
  peripheralsPowerInitPin();
  initI2CAndDriversOnce();
  bool fin_pend = false;

  // --- 1. PREFERENCIAS (Recuperación de Configuración Maestra) ---
  preferences.begin("sonda", false); 
  estado.id_anterior = preferences.getUInt("run_id", 0); 
  estado.run_id = estado.id_anterior;
  fin_pend = preferences.getBool("fin_pend", false);  // <-- nuevo

  // Recuperamos umbrales y tiempos para autonomía sin internet
  T_ALERTA  = preferences.getFloat("t_alerta", 60.0);    // Default 60
  T_CRITICA = preferences.getFloat("t_critica", 45.0);   // Default 45
  T_REPORT_NORMAL_MS   = preferences.getUInt("t_norm_ms", 60000);
  T_REPORT_CRITICAL_MS = preferences.getUInt("t_crit_ms", 15000);
  T_REPORT_ALERTA_MS   = preferences.getUInt("t_aler_ms", 10000); // Recupera los 10s
  uint32_t t_recuperado = preferences.getUInt("t_reporte_ms", T_REPORT_NORMAL_MS); 
  estado.t_reporte_ms = t_recuperado;

  uint32_t persisted_seq = preferences.getUInt("last_seq", 0);
  if (persisted_seq && estado.last_seq < persisted_seq) estado.last_seq = persisted_seq;

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  bool cold = (cause != ESP_SLEEP_WAKEUP_TIMER);
  preferences.end();

// --- 2. MEDIDA Y TIEMPO (Orden Corregido) ---
// --- 2. MEDIDA Y TIEMPO (SIMPLIFICADO) ---
float t_medida = NAN;
float v_medida = NAN;
uint32_t ts_medida = 0;

if (estado.run_id > 0 || cold) {
  peripheralsPowerOn();
  measureOnce(t_medida, v_medida);
  if (!isnan(lastBatteryPct) && lastBatteryPct < 20.0f) ledBlink(5, 80, 80); // 5 rapidos = bateria baja
  
  // ✅ USAR LA NUEVA FUNCIÓN
  ensureValidRTC(ts_medida);
  
  peripheralsPowerOff();
  maybeAdjustIntervalByTemp(t_medida);
}

if (cold) {
  keepWifiDuringGrace = true;
  safeLog(">>> [BOOT] COLD boot: keepWifiDuringGrace=TRUE (pre-WiFi)\n");
}
// --- 3. CONECTAR Y ACTUALIZAR ---
connectWiFiIfNeeded();
if (WiFi.status() == WL_CONNECTED) {
  if (cold) checkAndApplyOTA(); // OTA solo en arranque frio (periodo de gracia inicial)
  if (cold) {
      keepWifiDuringGrace = true;
      safeLog(">>> [BOOT] COLD boot: keepWifiDuringGrace=TRUE\n");
    }
    // Si la medida falló por fecha vieja, intentamos NTP ahora que hay WiFi
    if (ts_medida == 0 && rtcPresente) {
        syncRTCFromNTP();
        // Opcional: Recapturar ts_medida si quieres que el reporte salga con la hora actual
        ts_medida = rtc.now().unixtime();
    }
    sincronizarConServidor(t_medida, false); 
        maybeAdjustIntervalByTemp(t_medida);

}
safeLog(">>> [DBG] cold=%d run_id=%u id_anterior=%u fin_pend=%d\n",
        cold ? 1 : 0, estado.run_id, estado.id_anterior, fin_pend ? 1 : 0);
        safeLog(">>> [DBG] Antes de enviar IDLE: cold=%d run_id=%u\n", cold?1:0, estado.run_id);
if (cold && estado.run_id == 0) {
keepWifiDuringGrace = true;

  safeLog(">>> [BOOT] Sin prueba activa: enviando estado IDLE (run=0)\n");
enviarReporteActual(NAN, NAN, 0, true);
}
  // --- 4. GESTIÓN DE REPORTES Y GRACIA (Cerebro del diagrama) ---
  bool esInicio  = (estado.id_anterior == 0 && estado.run_id > 0);
  bool esFin     = (estado.id_anterior > 0 && estado.run_id == 0);
  bool hayCambio = (estado.run_id != estado.id_anterior);
  uint32_t duracionGraciaMs = 0;

  // Determinar si entramos en modo gracia según el diagrama
  if (cold) duracionGraciaMs = T_GRACIA_INICIAL_MS;
  else if (fin_pend) duracionGraciaMs = T_GRACIA_FINAL_MS; // 1 minuto
  else if (hayCambio) duracionGraciaMs = T_GRACIA_FINAL_MS;

// PRE-ARMAR GRACE ANTES DE ENVIAR (evita WIFI disconnected antes del while)
if (duracionGraciaMs > 0) {
  keepWifiDuringGrace = true;
  // Si tu disconnectWiFiIfAllowed() también respeta enGracia, puedes activar esto:
  // enGracia = true;
  safeLog(">>> [GRACE-PRE] duracion=%u ms -> keepWifiDuringGrace=TRUE\n", duracionGraciaMs);
}

// DISPARO ÚNICO DE REPORTE
if (esInicio || esFin || (estado.run_id > 0) || cold) {

  if (esFin) {
    keepWifiDuringGrace = true;
    safeLog(">>> [FIN][HTTP] keepWifiDuringGrace=TRUE (no desconectar hasta terminar gracia)\n");

    uint32_t prev = estado.id_anterior;
    safeLog(">>> [FIN][HTTP] Detectado FIN por HTTP/fin_pend. Enviando cierre run=%u\n", prev);

    uint32_t backup = estado.run_id;
    estado.run_id = prev;

    enviarReporteActual(t_medida, v_medida, ts_medida, true);

    estado.run_id = backup;

    safeLog(">>> [FIN][HTTP] Enviando IDLE run=0\n");
    enviarReporteActual(lastTemp, NAN, 0, true);

    estado.id_anterior = estado.run_id;
  } else {
    enviarReporteActual(t_medida, v_medida, ts_medida, esInicio);
    estado.id_anterior = estado.run_id;
  }
}

  // Bucle de Gracia (MQTT y escucha de comandos)// Bucle de Gracia (MQTT y escucha de comandos)
if (duracionGraciaMs > 0 || modoOnlineForzado) {
  keepWifiDuringGrace = true;
  enGracia = true;
  updateLedByMode(false, true, (WiFi.status() == WL_CONNECTED));

  unsigned long finBucle = modoOnlineForzado ? 0UL : (millis() + duracionGraciaMs);
  // Timeout de seguridad para modoOnlineForzado: máximo 30 min para evitar loop infinito
  const unsigned long MAX_ONLINE_FORZADO_MS = 30UL * 60UL * 1000UL;
  unsigned long finOnlineForzado = millis() + MAX_ONLINE_FORZADO_MS;
  if (proxRep == 0) {
    proxRep = millis() + estado.t_reporte_ms;
    safeLog(">>> [GRACE] proxRep inicializado: %ums\n", estado.t_reporte_ms);
  }

  // Sin WiFi no tiene sentido esperar: no hay MQTT ni comandos posibles
  if (duracionGraciaMs > 0 && !modoOnlineForzado && WiFi.status() != WL_CONNECTED) {
    safeLog(">>> [GRACE] Sin WiFi → grace period cancelado (ahorra ~11mA x %u ms)\n", duracionGraciaMs);
    duracionGraciaMs = 0;
    finBucle = millis(); // fuerza salida inmediata del while
  }

  if (modoOnlineForzado) {
    safeLog(">>> [GRACE] ONLINE (ALERTA): esperando hasta que baje la temperatura\n");
  } else {
    safeLog(">>> [GRACE] Iniciando espera de %u ms\n", duracionGraciaMs);
  }

  while ((modoOnlineForzado && (long)(finOnlineForzado - millis()) > 0) || (long)(finBucle - millis()) > 0) {
    checkSoftWatchdog();
    if (abortarGracia) break;
    
    // ---- Procesamiento de acciones MQTT diferidas (NO perder iniciar tras finalizar) ----
if (pendingFinalize) {
  pendingFinalize = false;

  // Medir UNA vez para cierre + idle
  float t = NAN, v = NAN;
  uint32_t ts = 0;

  peripheralsPowerOn();
  measureOnce(t, v);
  if (rtcPresente) ts = rtc.now().unixtime();
  peripheralsPowerOff();

  // 1) Cierre de la prueba anterior
  uint32_t prev = pendingFinalizePrevRun;
  if (prev > 0) {
    uint32_t backup = estado.run_id;
    estado.run_id = prev;
    safeLog(">>> [MQTT][DEFER] cierre run=%u\n", prev);
    enviarReporteActual(t, v, ts, true);
    estado.run_id = backup;
  }

  // 2) IDLE run=0
  estado.run_id = 0;
  estado.sonda_colocada = false;

  preferences.begin("sonda", false);
  preferences.putUInt("run_id", 0);
  preferences.putBool("fin_pend", true);
  preferences.end();

  safeLog(">>> [MQTT][DEFER] idle run=0\n");
  enviarReporteActual(t, v, ts, true);

  // Reinicia ventana de gracia final
  finBucle = millis() + T_GRACIA_FINAL_MS;
  safeLog(">>> [GRACE] FIN detectado. Reiniciando gracia: %u ms\n", T_GRACIA_FINAL_MS);

  // Dar aire a MQTT inmediatamente
  mqttClient.loop();
  yield();

  // --- Re-sync inmediato por HTTP para capturar un iniciar muy rápido (si el MQTT se perdió) ---
  delay(300);        // margen chico (ajustable)
  mqttClient.loop();
  yield();

  uint32_t before = estado.run_id; // debería ser 0
  sincronizarConServidor(lastTemp, false); // GET /api/estado/<SONDA_ID> -> prueba_activa

  if (estado.run_id > 0 && estado.run_id != before) {
    safeLog(">>> [POST-FIN][HTTP] nueva prueba detectada run=%u\n", estado.run_id);
    pendingStartRun = estado.run_id;
    pendingStart = true;
  }

  // FIX: evitar "FIN detectado" doble por el otro bloque (run_id != id_anterior)
  estado.id_anterior = estado.run_id;

  // FIX: no caer al bloque de "cambio de ID" en esta misma vuelta del while
  continue;
}

if (pendingStart) {
  pendingStart = false;

  uint32_t id = pendingStartRun;
  if (id) {
    estado.run_id = id;
    estado.sonda_colocada = true;

    preferences.begin("sonda", false);
    preferences.putUInt("run_id", estado.run_id);
    preferences.putBool("fin_pend", false);
    preferences.end();

    // Medición FRESCA para que el primer reporte no sea viejo
    float t2 = NAN, v2 = NAN;
    uint32_t ts2 = 0;

    peripheralsPowerOn();
    measureOnce(t2, v2);
    if (rtcPresente) ts2 = rtc.now().unixtime();
    peripheralsPowerOff();

    safeLog(">>> [MQTT][DEFER] iniciar run=%u (fresh)\n", id);
    enviarReporteActual(t2, v2, ts2, true);

    estado.id_anterior = estado.run_id;
    proxRep = millis() + estado.t_reporte_ms;

    // opcional pero recomendable: evita que el bloque run_id!=id_anterior dispare algo en esta vuelta
    continue;
  }
}

if (pendingWakeupReport) {
  pendingWakeupReport = false;
  safeLog(">>> [MQTT][DEFER] wakeup_report -> Procesando actualización manual...\n");

  // 1. Declaramos variables locales para capturar los datos
  float t_manual = NAN;
  float v_manual = NAN;
  uint32_t ts_manual = 0;

  // 2. Realizamos la medición manualmente fuera de enviarReporteActual
  peripheralsPowerOn();
  measureOnce(t_manual, v_manual); 
  ensureValidRTC(ts_manual);

  peripheralsPowerOff();

  // 3. ¡AQUÍ ESTÁ LA CLAVE! Ahora sí tenemos la temperatura para evaluar
  if (!isnan(t_manual)) {
    safeLog(">>> [MQTT] Evaluando T=%.2f para ajuste de intervalo\n", t_manual);
    maybeAdjustIntervalByTemp(t_manual); 
  }

  // 4. Enviamos el reporte pasando los valores que ya medimos
  // Al pasarle t_manual y v_manual, la función ya no volverá a medir internamente
  enviarReporteActual(t_manual, v_manual, ts_manual, true);

  // 5. Actualizamos los tiempos del bucle para que respete el nuevo intervalo
  lastRuntimeReportMillis = millis();
  proxRep = millis() + estado.t_reporte_ms;

  safeLog(">>> [MQTT] Reporte enviado. Nuevo intervalo: %u ms\n", estado.t_reporte_ms);
    proxRep = millis() + estado.t_reporte_ms;
  safeLog(">>> [MQTT] proxRep actualizado por wakeup: %ums\n", estado.t_reporte_ms);
  
  continue;
}

    // Si el ID cambia por MQTT mientras estamos despiertos (por otros caminos)
    if (estado.run_id != estado.id_anterior) {
  bool mqttInicio = (estado.id_anterior == 0 && estado.run_id > 0);
  bool mqttFin    = (estado.id_anterior > 0 && estado.run_id == 0);

  // MEDIR FRESCO al detectar cambio de run
  float t = NAN, v = NAN;
  uint32_t ts = 0;
  peripheralsPowerOn();
  measureOnce(t, v);
  if (rtcPresente) ts = rtc.now().unixtime();
  peripheralsPowerOff();

  enviarReporteActual(t, v, ts, mqttInicio);

  estado.id_anterior = estado.run_id;
  proxRep = millis() + estado.t_reporte_ms;

  if (mqttFin) {
    finBucle = millis() + T_GRACIA_FINAL_MS;
    safeLog(">>> [GRACE] FIN detectado. Reiniciando gracia: %u ms\n", T_GRACIA_FINAL_MS);
  }
}

    if (millis() >= proxRep) {
      enviarReporteActual(NAN, NAN, 0, false);
      proxRep = millis() + estado.t_reporte_ms;

      if (modoOnlineForzado) {
        float t_now = NAN, v_now = NAN;

        peripheralsPowerOn();
        measureOnce(t_now, v_now);
        peripheralsPowerOff();

        if (!isnan(t_now)) {
          maybeAdjustIntervalByTemp(t_now);
          proxRep = millis() + estado.t_reporte_ms;

          if (!modoOnlineForzado) {
            safeLog(">>> [GRACE] Bajó de ALERTA (T=%.2f). Saliendo de ONLINE.\n", t_now);
            break;
          }
        } else {
          safeLog(">>> [GRACE] AVISO: no se pudo re-medir temperatura para salir de ONLINE\n");
        }
      }
    }if (WiFi.status() == WL_CONNECTED) {
    // Estas dos líneas le dicen al ESP32 qué son esas palabras
    unsigned long ahora = millis(); 
    static unsigned long ultimoIntentoMqtt = 0; // static hace que "recuerde" el tiempo

    if (!mqttClient.connected()) {
        // Solo entra aquí cada 5000ms (5 segundos)
        if (ahora - ultimoIntentoMqtt >= 5000) { 
            ultimoIntentoMqtt = ahora;
            
            safeLog(">>> [MQTT] Intentando reconexion rapida...\n");

            String willTopic = String("sonda/") + SONDA_ID + "/presence";
            if (mqttClient.connect(SONDA_ID, willTopic.c_str(), 1, true, "offline")) {
                mqtt_subscribe_topics();
                mqttClient.publish(willTopic.c_str(), "online", true);
                safeLog(">>> [MQTT] reconnected OK\n");
            }
        }
    } else {
        // SI ESTÁ CONECTADO: Esta es la clave para que reciba los comandos al instante
        mqttClient.loop(); 
    }
}

    delay(200);
    touchProgress();
    updateLedByMode(false, enGracia, WiFi.status() == WL_CONNECTED);
  }

  keepWifiDuringGrace = false;
  enGracia = false;
  updateLedByMode(false, false, (WiFi.status() == WL_CONNECTED));
}

  // --- 5. GUARDAR CAMBIOS FINALES ---
// --- 5. GUARDAR CAMBIOS FINALES ---
preferences.begin("sonda", false);
preferences.putUInt("run_id", estado.run_id);
preferences.putFloat("t_alerta", T_ALERTA);
preferences.putFloat("t_critica", T_CRITICA);

// AGREGAR ESTAS LÍNEAS PARA PERSISTIR LOS INTERVALOS
preferences.putUInt("t_norm_ms", T_REPORT_NORMAL_MS);
preferences.putUInt("t_crit_ms", T_REPORT_CRITICAL_MS);
preferences.putUInt("t_aler_ms", T_REPORT_ALERTA_MS);

preferences.putUInt("t_reporte_ms", estado.t_reporte_ms);
preferences.end();

  // --- 6. SUEÑO (Mecánica de precisión) ---

  // Apagado explícito: MQTT → WiFi → BT → Serie antes de dormir.
  // El driver WiFi/MQTT activo impide que esp_deep_sleep_start() complete.
  mqttClient.disconnect();
  if (WiFi.status() == WL_CONNECTED || WiFi.getMode() != WIFI_OFF) {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  }
  delay(500); // Tiempo para que las tareas FreeRTOS de WiFi/TCP terminen
  btStop();
  Serial.flush();

preferences.begin("sonda", true); // Modo solo lectura
  uint32_t id_real = preferences.getUInt("run_id", 0);
  bool fin_pend_real = preferences.getBool("fin_pend", false); // <-- NUEVO

  preferences.end();

  if (id_real > 0) {
    uint32_t despierto = millis();
    uint32_t t_para_dormir;

    // FIX: El tiempo a dormir es el intervalo de reporte MENOS el tiempo ya despierto.
    // Si ya pasamos el intervalo (boot lento), dormimos un mínimo de 5 segundos.
    if (estado.t_reporte_ms > despierto) {
      t_para_dormir = estado.t_reporte_ms - despierto;
    } else {
      t_para_dormir = 5000UL; // boot demasiado lento, dormir al menos 5s
      safeLog(">>> [SLEEP] AVISO: boot (%u ms) supero el intervalo (%u ms). Durmiendo minimo 5s.\n", despierto, estado.t_reporte_ms);
    }
    
    safeLog(">>> [SLEEP] Prueba ACTIVA (%u). Despierto: %u ms. Dormir: %u ms\n", id_real, despierto, t_para_dormir);
    esp_sleep_enable_timer_wakeup((uint64_t)t_para_dormir * 1000ULL);
    safeLog(">>> [SLEEP] Entrando en deep sleep NOW...\n");
    Serial.flush();
    esp_deep_sleep_start();
    safeLog(">>> [ERROR] esp_deep_sleep_start() retorno! Forzando restart.\n");
    Serial.flush();
    delay(100);
    esp_restart();
  } 
  else {
  if (fin_pend_real) {
    // Ya cumplimos la gracia final; limpiar flag para que no se repita en el próximo reset
    preferences.begin("sonda", false);
    preferences.putBool("fin_pend", false);
    preferences.end();
    safeLog(">>> [OFF] Fin de prueba: apagado definitivo (hasta RESET).\n");
  } else {
    safeLog(">>> [OFF] Sin prueba activa. Deep sleep hasta RESET.\n");
  }

  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL); // elimina timers / ext wakeups
  safeLog(">>> [SLEEP] Entrando en deep sleep NOW (sin timer)...\n");
  Serial.flush();
  esp_deep_sleep_start();
  safeLog(">>> [ERROR] esp_deep_sleep_start() retorno! Forzando restart.\n");
  Serial.flush();
  delay(100);
  esp_restart();
}
}
void loop() {
  // Nada aquí, todo ocurre en setup() + deep sleep
}