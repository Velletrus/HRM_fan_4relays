
// ======= ESP32 BLE HR + Relay Safe Switching (millis) + WebServer =======
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEClient.h>

// *************** WiFi (INSERISCI QUI) ***************
const char* WIFI_SSID = "FranzWifi";
const char* WIFI_PASS = "Velletri82.";

// *************** BLE UUID standard Heart Rate ***************
static BLEUUID HR_SERVICE_UUID((uint16_t)0x180D);
static BLEUUID HR_MEAS_CHAR_UUID((uint16_t)0x2A37);

// *************** BLE objects ***************
BLEScan* pBLEScan = nullptr;
BLEClient* pClient = nullptr;
BLERemoteCharacteristic* pHRChar = nullptr;
bool connected = false;

// *************** Target device ***************
BLEAddress targetAddress = BLEAddress((uint8_t*)"\0\0\0\0\0\0");
bool haveTarget = false;
bool connecting = false;

// *************** Target device name (opzionale) ***************
char targetName[32] = "";  // es. "Polar H10" (impostabile via web/seriale)

// *************** Uscite relè (avvolgimenti) ***************
const int V1_PIN     = 16;  // velocità 1
const int V2_PIN     = 17;  // velocità 2
const int V3_PIN     = 26;  // velocità 3
const int POWER_PIN  = 27;  // alimentazione ventilatore (master)
const int TOGGLE_PIN = 23;  // LED/uscita diagnostica blink

// *************** Temporizzazioni relè ***************
const unsigned long RELAY_GUARD_MS = 500;  // attesa tra fasi di commutazione

// *************** Soglie e isteresi ***************
int TH1  = 100;
int TH2  = 130;
int TH3  = 150;
int HYST = 3;

const int MIN_VALID_BPM = 40;
const int MAX_VALID_BPM = 220;

// *************** Stato HR/timeout ***************
int currentHrLevel = 0;  // livello calcolato da BPM: 0..3
int lastBpmValue   = -1; // ultimo BPM letto (per web)
unsigned long lastBpmMillis = 0;
const unsigned long DATA_TIMEOUT_MS = 7000; // se niente dati per 7s → OFF
unsigned long lastPoll = 0;
const unsigned long POLL_MS = 1000;         // lettura di fallback ogni 1s

// ======= Sblocco manuale automatico se non c'è fascia ======= // <<< NEW
const unsigned long HRM_CONNECT_TIMEOUT_MS_DEFAULT = 15000; // default 15s
unsigned long HRM_CONNECT_TIMEOUT_MS = HRM_CONNECT_TIMEOUT_MS_DEFAULT;
unsigned long bootMillis = 0;
bool autoManualEnabled = false;  // true quando abilitiamo MAN automatico

// *************** Stato relè (macchina a stati) ***************
int currentRelayLevel = 0;    // 0..3 (realmente attivo sui relè)
int desiredRelayLevel = 0;    // 0..3 (richiesto da HR o manuale)
bool manualMode = false;      // true = MAN, false = AUTO
bool powerOn = false;

enum RelayPhase {
  IDLE,
  PWR_ON,
  WAIT1,
  OFF_ALL,
  WAIT2,
  APPLY_COIL,
  WAIT3,
  FINISH_ON,
  PWR_OFF
};

RelayPhase relayPhase = IDLE;
unsigned long relayPhaseStart = 0;

// *************** NVS (Preferences) ***************
Preferences prefs;
const char* PREF_NS = "hrcfg";
const char* KEY_TH1 = "TH1";
const char* KEY_TH2 = "TH2";
const char* KEY_TH3 = "TH3";
const char* KEY_HYS = "HYST";
const char* KEY_NAM = "NAME";
// <<< NEW: chiave per timeout MAN unlock
const char* KEY_MTO = "MTO";

// *************** WebServer ***************
WebServer server(80);

// ============================================================
// Utility relè / uscite
// ============================================================
void setPower(bool on) {
  digitalWrite(POWER_PIN, on ? HIGH : LOW);
  powerOn = on;
}
void setAllCoilsLow() {
  digitalWrite(V1_PIN, LOW);
  digitalWrite(V2_PIN, LOW);
  digitalWrite(V3_PIN, LOW);
}
void applyCoilForLevelOnly(int level) {
  // Una sola bobina ON per volta (POWER gestito altrove)
  setAllCoilsLow();
  if      (level == 1) digitalWrite(V1_PIN, HIGH);
  else if (level == 2) digitalWrite(V2_PIN, HIGH);
  else if (level == 3) digitalWrite(V3_PIN, HIGH);
}
void initOutputs() {
  pinMode(V1_PIN, OUTPUT);
  pinMode(V2_PIN, OUTPUT);
  pinMode(V3_PIN, OUTPUT);
  pinMode(POWER_PIN, OUTPUT);
  pinMode(TOGGLE_PIN, OUTPUT);

  setAllCoilsLow();
  setPower(false);
  digitalWrite(TOGGLE_PIN, LOW);
}

// ============================================================
// BPM / Hysteresis
// ============================================================
void printThresholds() {
  Serial.printf("Soglie correnti: TH1=%d, TH2=%d, TH3=%d, HYST=%d\n", TH1, TH2, TH3, HYST);
  Serial.printf("Modalità: %s\n", manualMode ? "MANUALE" : "AUTO");
  Serial.printf("Timeout MAN sblocco (MTO)=%lu ms\n", HRM_CONNECT_TIMEOUT_MS); // <<< NEW
  if (strlen(targetName) > 0) {
    Serial.print("Fascia preferita (NAME): ");
    Serial.println(targetName);
  } else {
    Serial.println("Fascia preferita (NAME): <non impostata>");
  }
}

void loadPrefs() {
  prefs.begin(PREF_NS, true);
  TH1  = prefs.getInt(KEY_TH1, TH1);
  TH2  = prefs.getInt(KEY_TH2, TH2);
  TH3  = prefs.getInt(KEY_TH3, TH3);
  HYST = prefs.getInt(KEY_HYS, HYST);
  prefs.getString(KEY_NAM, targetName, sizeof(targetName));
  // <<< NEW: carica timeout (ms); default se non esiste
  HRM_CONNECT_TIMEOUT_MS = prefs.getULong(KEY_MTO, HRM_CONNECT_TIMEOUT_MS_DEFAULT);
  prefs.end();
}

void savePrefs() {
  prefs.begin(PREF_NS, false);
  prefs.putInt(KEY_TH1, TH1);
  prefs.putInt(KEY_TH2, TH2);
  prefs.putInt(KEY_TH3, TH3);
  prefs.putInt(KEY_HYS, HYST);
  prefs.putString(KEY_NAM, String(targetName));
  // <<< NEW: salva timeout (ms)
  prefs.putULong(KEY_MTO, HRM_CONNECT_TIMEOUT_MS);
  prefs.end();
  Serial.println("Soglie/NAME/MTO salvati in NVS.");
}

bool checkAndFixOrder() {
  bool ok = true;
  if (!(TH1 >= MIN_VALID_BPM && TH3 <= MAX_VALID_BPM)) ok = false;
  if (TH1 >= TH2) { TH2 = TH1 + 1; ok = false; }
  if (TH2 >= TH3) { TH3 = TH2 + 1; ok = false; }
  return ok;
}

// Decide il livello HR (0..3) su base BPM con isteresi
void decideLevelWithHysteresis(int bpm) {
  int newLevel = currentHrLevel;

  switch (currentHrLevel) {
    case 0:
      if      (bpm >= TH3) newLevel = 3;
      else if (bpm >= TH2) newLevel = 2;
      else if (bpm >= TH1) newLevel = 1;
      break;

    case 1:
      if      (bpm >= TH3) newLevel = 3;
      else if (bpm >= TH2) newLevel = 2;
      else if (bpm < (TH1 - HYST)) newLevel = 0;
      break;

    case 2:
      if      (bpm >= TH3) newLevel = 3;
      else if (bpm < (TH2 - HYST)) {
        if      (bpm >= TH1) newLevel = 1;
        else if (bpm < (TH1 - HYST)) newLevel = 0;
      }
      break;

    case 3:
      if (bpm < (TH3 - HYST)) {
        if      (bpm >= TH2) newLevel = 2;
        else if (bpm >= TH1) newLevel = 1;
        else if (bpm < (TH1 - HYST)) newLevel = 0;
      }
      break;
  }

  if (newLevel != currentHrLevel) {
    currentHrLevel = newLevel;
    Serial.printf("HR Livello → %d (BPM=%d)\n", currentHrLevel, bpm);
  }

  // In AUTO aggiorna il livello desiderato per la macchina a stati dei relè
  if (!manualMode) {
    desiredRelayLevel = currentHrLevel;
  }
}

// ============================================================
// Parser BLE HRM
// ============================================================
uint16_t parseBPM(const uint8_t* data, size_t len) {
  if (len < 2) return 0;
  uint8_t flags = data[0];
  bool hr16 = flags & 0x01;
  int index = 1;
  if (hr16) {
    if (index + 1 >= (int)len) return 0;
    return (uint16_t)(data[index] | (data[index + 1] << 8));
  } else {
    return (uint16_t)data[index];
  }
}

class MyNotifyCallback {
public:
  static void notifyCallback(BLERemoteCharacteristic* pChar, uint8_t* data, size_t length, bool isNotify) {
    uint16_t bpm = parseBPM(data, length);
    if (bpm >= MIN_VALID_BPM && bpm <= MAX_VALID_BPM) {
      lastBpmMillis = millis();
      lastBpmValue = (int)bpm;
      Serial.printf("BPM: %u\n", bpm);
      decideLevelWithHysteresis((int)bpm);
    }
  }
};

// ============================================================
// BLE: discover/subscribe/connect
// ============================================================
bool subscribeHRM() {
  if (!pClient || !pHRChar) return false;

  BLERemoteDescriptor* cccd = pHRChar->getDescriptor(BLEUUID((uint16_t)0x2902));
  if (cccd) {
    uint8_t notifyOn[2] = {0x01, 0x00};
    bool ok = cccd->writeValue(notifyOn, 2, true);
    Serial.printf("CCCD 0x2902 (notify): %s\n", ok ? "OK" : "FALLITO");
  } else {
    Serial.println("CCCD 0x2902 non trovato, procedo con registerForNotify.");
  }

  if (pHRChar->canNotify()) {
    pHRChar->registerForNotify(MyNotifyCallback::notifyCallback);
    Serial.println("Iscritto alle notifiche BPM.");
    return true;
  } else {
    Serial.println("La caratteristica non supporta notify. Userò la lettura periodica.");
    return true;
  }
}

bool discoverHRM() {
  Serial.println("Ricerca servizio HR...");
  BLERemoteService* pService = pClient->getService(HR_SERVICE_UUID);
  if (!pService) {
    Serial.println("Servizio Heart Rate (0x180D) non trovato.");
    return false;
  }
  Serial.println("Servizio HR trovato. Ricerca caratteristica 0x2A37...");
  pHRChar = pService->getCharacteristic(HR_MEAS_CHAR_UUID);
  if (!pHRChar) {
    Serial.println("Caratteristica 0x2A37 non trovata.");
    return false;
  }
  return true;
}

bool doConnect() {
  if (!haveTarget) return false;

  Serial.print("Connessione al dispositivo: ");
  Serial.println(targetAddress.toString().c_str());

  pClient = BLEDevice::createClient();
  bool ok = pClient->connect(targetAddress);
  if (!ok) {
    Serial.println("Connessione fallita.");
    return false;
  }
  Serial.println("Connesso.");

  if (!discoverHRM()) {
    pClient->disconnect();
    return false;
  }

  if (!subscribeHRM()) {
    pClient->disconnect();
    return false;
  }

  // Lettura immediata di fallback (se supportata)
  if (pHRChar->canRead()) {
    String val = pHRChar->readValue();
    int len = val.length();
    if (len > 0) {
      const uint8_t* data = (const uint8_t*)val.c_str();
      uint16_t bpm = parseBPM(data, (size_t)len);
      if (bpm >= MIN_VALID_BPM && bpm <= MAX_VALID_BPM) {
        lastBpmMillis = millis();
        lastBpmValue = (int)bpm;
        Serial.printf("BPM (read): %u\n", bpm);
        decideLevelWithHysteresis((int)bpm);
      }
    }
  }

  connected = true;
  return true;
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice device) override {
    // Filtra per NAME se impostato
    if (strlen(targetName) > 0) {
      if (!device.haveName()) return;
      if (!device.getName().equals(targetName)) return;
    }

    // Se non filtri per nome, richiedi HR service in advertising
    bool hasHRService = device.haveServiceUUID() && device.isAdvertisingService(HR_SERVICE_UUID);
    if (!hasHRService && strlen(targetName) == 0) return;

    Serial.print("Trovata fascia cardio: ");
    Serial.println(device.toString().c_str());

    BLEDevice::getScan()->stop();
    delay(100);
    BLEDevice::getScan()->clearResults();

    targetAddress = device.getAddress();
    haveTarget = true;
    connecting  = false;
  }
};

// ============================================================
// Serial commands (compatibile: 1,2,3 e 4=OFF)
// ============================================================
void handleSerialCommands() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  String cmd = line;
  String cmdUpper = cmd;
  cmdUpper.toUpperCase();

  if (cmdUpper == "GET") { printThresholds(); return; }
  if (cmdUpper == "SAVE") { savePrefs(); return; }
  if (cmdUpper == "RESET") {
    TH1 = 100; TH2 = 130; TH3 = 150; HYST = 3;
    HRM_CONNECT_TIMEOUT_MS = HRM_CONNECT_TIMEOUT_MS_DEFAULT; // <<< NEW
    targetName[0] = '\0';
    savePrefs();
    printThresholds();
    return;
  }
  if (cmdUpper == "HELP") {
    Serial.println("Comandi: GET | TH1=<val> | TH2=<val> | TH3=<val> | HYST=<val> | SAVE | RESET | NAME=<nome> | AUTO | MAN");
    Serial.println("Velocità manuale: 1 | 2 | 3 | 4 (OFF)");
    Serial.println("Vincoli: 40 <= TH1 < TH2 < TH3 <= 220, HYST >= 0");
    Serial.println("MTO via web: /api/save?MTO=<ms>  (timeout sblocco MAN)");
    return;
  }

  if (cmdUpper == "AUTO") { manualMode = false; desiredRelayLevel = currentHrLevel; Serial.println("Modalità AUTO (da BPM)"); return; }
  if (cmdUpper == "MAN")  { manualMode = true; Serial.println("Modalità MANUALE"); return; }

  if (cmdUpper.startsWith("NAME=")) {
    int eq = line.indexOf('=');
    if (eq >= 0) {
      String val = line.substring(eq + 1);
      val.trim();
      if (val.length() > 0 && val.length() < (int)sizeof(targetName)) {
        val.toCharArray(targetName, sizeof(targetName));
        Serial.print("NAME impostato: ");
        Serial.println(targetName);
        Serial.println("Usa SAVE per salvare in NVS.");
      } else {
        Serial.println("NAME non valido o troppo lungo.");
      }
    }
    return;
  }

  if (cmdUpper.startsWith("TH1=") || cmdUpper.startsWith("TH2=") || cmdUpper.startsWith("TH3=") || cmdUpper.startsWith("HYST=")) {
    int eq = line.indexOf('=');
    if (eq < 0) return;
    String key = line.substring(0, eq);
    String val = line.substring(eq + 1);
    key.toUpperCase();
    int n = val.toInt();

    if (key == "TH1") {
      if (n < MIN_VALID_BPM || n > MAX_VALID_BPM) { Serial.println("TH1 fuori range."); return; }
      TH1 = n;
    } else if (key == "TH2") {
      if (n < MIN_VALID_BPM || n > MAX_VALID_BPM) { Serial.println("TH2 fuori range."); return; }
      TH2 = n;
    } else if (key == "TH3") {
      if (n < MIN_VALID_BPM || n > MAX_VALID_BPM) { Serial.println("TH3 fuori range."); return; }
      TH3 = n;
    } else if (key == "HYST") {
      if (n < 0 || n > 20) { Serial.println("HYST fuori range (0..20)."); return; }
      HYST = n;
    }

    bool ok = checkAndFixOrder();
    printThresholds();
    if (!ok) Serial.println("Soglie corrette per mantenere l'ordine TH1 < TH2 < TH3.");
    Serial.println("Usa SAVE per rendere permanenti.");
    return;
  }

  // Comandi numerici: 1,2,3 e 4=OFF (come il tuo)
  if (cmdUpper == "1") { manualMode = true; desiredRelayLevel = 1; Serial.println("MAN: Velocità 1"); return; }
  if (cmdUpper == "2") { manualMode = true; desiredRelayLevel = 2; Serial.println("MAN: Velocità 2"); return; }
  if (cmdUpper == "3") { manualMode = true; desiredRelayLevel = 3; Serial.println("MAN: Velocità 3"); return; }
  if (cmdUpper == "4") { manualMode = true; desiredRelayLevel = 0; Serial.println("MAN: Tutto OFF"); return; }
}

// ============================================================
// Relay state machine (millis, POWER ON 0->X; OFF solo verso 0)
// ============================================================
void beginTransition() {
  unsigned long now = millis();

  if (desiredRelayLevel > 0) {
    // Da 0 -> velocità: POWER deve essere ON prima
    if (!powerOn) {
      setPower(true);
      relayPhase = WAIT1;         // aspetta prima di toccare le bobine
    } else {
      relayPhase = OFF_ALL;       // POWER già ON: vai a spegnere bobine
    }
  } else {
    // Verso OFF: prima bobine OFF, poi POWER OFF
    relayPhase = OFF_ALL;
  }
  relayPhaseStart = now;
}

void relayStateMachineTick() {
  static unsigned long lastBlink = 0;
  unsigned long now = millis();

  // Blink diagnostico
  if (now - lastBlink >= 500) {
    digitalWrite(TOGGLE_PIN, !digitalRead(TOGGLE_PIN));
    lastBlink = now;
  }

  if (relayPhase == IDLE) {
    if (desiredRelayLevel != currentRelayLevel) {
      beginTransition();
    }
    return;
  }

  switch (relayPhase) {
    case WAIT1:
      if (now - relayPhaseStart >= RELAY_GUARD_MS) {
        relayPhase = OFF_ALL;
        relayPhaseStart = now;
      }
      break;

    case OFF_ALL:
      // Spengo tutte le bobine; POWER resta come è
      setAllCoilsLow();
      relayPhase = WAIT2;
      relayPhaseStart = now;
      break;

    case WAIT2:
      if (now - relayPhaseStart >= RELAY_GUARD_MS) {
        if (desiredRelayLevel > 0) {
          relayPhase = APPLY_COIL; // verso una velocità
        } else {
          relayPhase = PWR_OFF;    // verso OFF: ora si può spegnere POWER
        }
        relayPhaseStart = now;
      }
      break;

    case APPLY_COIL:
      applyCoilForLevelOnly(desiredRelayLevel);
      relayPhase = WAIT3;
      relayPhaseStart = now;
      break;

    case WAIT3:
      if (now - relayPhaseStart >= RELAY_GUARD_MS) {
        relayPhase = FINISH_ON;
      }
      break;

    case FINISH_ON:
      currentRelayLevel = desiredRelayLevel;
      // POWER resta ON nei cambi 1<->2<->3
      Serial.printf("Relè: livello attivo %d (POWER ON)\n", currentRelayLevel);
      relayPhase = IDLE;
      break;

    case PWR_OFF:
      // Solo quando si va a 0: POWER OFF alla fine
      setPower(false);
      currentRelayLevel = 0;
      Serial.println("Relè: OFF totale (POWER OFF)");
      relayPhase = IDLE;
      break;

    default:
      relayPhase = IDLE;
      break;
  }
}

// ============================================================
// Web server (HTML semplice + API)
// ============================================================
String htmlPage() {
  String mode = manualMode ? "MANUALE" : "AUTO";
  String page = R"HTML(
<!DOCTYPE html>
<html lang="it">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>ESP32 Ventilatore HR</title>
<style>
body{font-family:sans-serif;max-width:800px;margin:20px auto;padding:0 10px;}
.card{border:1px solid #ddd;border-radius:8px;padding:12px;margin-bottom:12px;}
label{display:block;margin:4px 0;}
input[type=number],input[type=text]{width:160px;padding:6px;}
button{padding:6px 10px;margin:4px;}
.badge{display:inline-block;padding:4px 8px;border-radius:12px;background:#eee;}
</style>
</head>
<body>
<h2>ESP32 Ventilatore HR</h2>
<div class="card">
  <div>Modalità: <span class="badge" id="mode">)HTML";
  page += mode;
  page += R"HTML(</span></div>
  <div>Connessione HRM: <span id="conn">)HTML";
  page += connected ? "connesso" : "non connesso";
  page += R"HTML(</span></div>
  <div>POWER: <span id="pwr">)HTML";
  page += powerOn ? "ON" : "OFF";
  page += R"HTML(</span></div>
  <div>BPM attuale: <span id="bpm">)HTML";
  page += (lastBpmValue >= 0 && (millis()-lastBpmMillis)<=DATA_TIMEOUT_MS) ? String(lastBpmValue) : String("-");
  page += R"HTML(</span></div>
  <div>Livello HR: <span id="hrlvl">)HTML";
  page += String(currentHrLevel);
  page += R"HTML(</span></div>
  <div>Velocità attiva (relè): <span id="relvl">)HTML";
  page += String(currentRelayLevel);
  page += R"HTML(</span></div>
  <div>Manuale sbloccato: <span id="munl">)HTML"; // <<< NEW
  page += autoManualEnabled ? "SI" : "NO";
  page += R"HTML(</span></div>
</div>

<div class="card">
  <h3>Impostazioni soglie</h3>
  <form method="GET" action="/api/save">
    <label>TH1: <input type="number" name="TH1" min="40" max="220" required value=")HTML";
  page += String(TH1);
  page += R"HTML("></label>
    <label>TH2: <input type="number" name="TH2" min="40" max="220" required value=")HTML";
  page += String(TH2);
  page += R"HTML("></label>
    <label>TH3: <input type="number" name="TH3" min="40" max="220" required value=")HTML";
  page += String(TH3);
  page += R"HTML("></label>
    <label>Isteresi (HYST): <input type="number" name="HYST" min="0" max="20" required value=")HTML";
  page += String(HYST);
  page += R"HTML("></label>
    <label>Fascia preferita (NAME): <input type="text" name="NAME" maxlength="31" value=")HTML";
  page += String(targetName);
  page += R"HTML("></label>
    <label>Timeout MAN (MTO, ms): <input type="number" name="MTO" min="0" max="600000" required value=")HTML"; // <<< NEW
  page += String(HRM_CONNECT_TIMEOUT_MS);
  page += R"HTML("></label>
    <button type="submit">Salva</button>
  </form>
</div>

<div class="card">
  <h3>Controllo</h3>
  <button onclick="fetch('/mode?auto=1').then(()=>refresh())">AUTO</button>
  <button onclick="fetch('/mode?auto=0').then(()=>refresh())">MAN</button>
  <button onclick="fetch('/unlock').then(()=>refresh())">Sblocca MANUALE</button> <!-- <<< NEW -->
  <div style="margin-top:6px">
    <button onclick="fetch('/speed?level=1').then(()=>refresh())">Velocità 1</button>
    <button onclick="fetch('/speed?level=2').then(()=>refresh())">Velocità 2</button>
    <button onclick="fetch('/speed?level=3').then(()=>refresh())">Velocità 3</button>
    <button onclick="fetch('/speed?level=0').then(()=>refresh())">OFF</button>
  </div>
</div>

<script>
async function refresh(){
  const r = await fetch('/api/get');
  const j = await r.json();
  document.getElementById('mode').textContent = j.manualMode ? 'MANUALE' : 'AUTO';
  document.getElementById('conn').textContent = j.connected ? 'connesso' : 'non connesso';
  document.getElementById('pwr').textContent = j.powerOn ? 'ON' : 'OFF';
  document.getElementById('bpm').textContent = (j.bpm >= 0) ? j.bpm : '-';
  document.getElementById('hrlvl').textContent = j.hrLevel;
  document.getElementById('relvl').textContent = j.relayLevel;
  document.getElementById('munl').textContent = j.manualUnlocked ? 'SI' : 'NO'; // <<< NEW
}
setInterval(refresh, 1000);
refresh();
</script>
</body>
</html>
)HTML";
  return page;
}

void handleRoot() {
  server.send(200, "text/html", htmlPage());
}

void handleApiGet() {
  int bpmOut = (lastBpmValue >= 0 && (millis() - lastBpmMillis) <= DATA_TIMEOUT_MS) ? lastBpmValue : -1;
  String json = "{";
  json += "\"TH1\":" + String(TH1) + ",";
  json += "\"TH2\":" + String(TH2) + ",";
  json += "\"TH3\":" + String(TH3) + ",";
  json += "\"HYST\":" + String(HYST) + ",";
  json += "\"NAME\":\"" + String(targetName) + "\",";
  json += "\"manualMode\":" + String(manualMode ? "true" : "false") + ",";
  json += "\"connected\":" + String(connected ? "true" : "false") + ",";
  json += "\"powerOn\":" + String(powerOn ? "true" : "false") + ",";
  json += "\"hrLevel\":" + String(currentHrLevel) + ",";
  json += "\"relayLevel\":" + String(currentRelayLevel) + ",";
  json += "\"bpm\":" + String(bpmOut) + ",";
  // <<< NEW: stato unlock + timeout attuale
  json += "\"manualUnlocked\":" + String(autoManualEnabled ? "true" : "false") + ",";
  json += "\"MTO\":" + String(HRM_CONNECT_TIMEOUT_MS);
  json += "}";
  server.send(200, "application/json", json);
}

void handleApiSave() {
  // /api/save?TH1=..&TH2=..&TH3=..&HYST=..&NAME=..&MTO=..
  bool okParams = true;

  if (server.hasArg("TH1")) { TH1 = server.arg("TH1").toInt(); }
  if (server.hasArg("TH2")) { TH2 = server.arg("TH2").toInt(); }
  if (server.hasArg("TH3")) { TH3 = server.arg("TH3").toInt(); }
  if (server.hasArg("HYST")) { HYST = server.arg("HYST").toInt(); }
  if (server.hasArg("NAME")) {
    String nm = server.arg("NAME");
    nm.trim();
    if (nm.length() < (int)sizeof(targetName)) {
      nm.toCharArray(targetName, sizeof(targetName));
    } else {
      okParams = false;
    }
  }
  // <<< NEW: MTO (timeout sblocco MAN in ms)
  if (server.hasArg("MTO")) {
    unsigned long mto = server.arg("MTO").toInt();
    // validazione: 0..600000 ms (10 minuti)
    if (mto <= 600000) {
      HRM_CONNECT_TIMEOUT_MS = mto;
    } else {
      okParams = false;
    }
  }

  // Validazioni minime
  if (TH1 < MIN_VALID_BPM || TH1 > MAX_VALID_BPM) okParams = false;
  if (TH2 < MIN_VALID_BPM || TH2 > MAX_VALID_BPM) okParams = false;
  if (TH3 < MIN_VALID_BPM || TH3 > MAX_VALID_BPM) okParams = false;
  if (HYST < 0 || HYST > 20) okParams = false;

  bool orderOk = checkAndFixOrder();
  savePrefs();

  String msg = okParams ? "OK" : "Parametri fuori range / NAME troppo lungo.";
  if (!orderOk) msg += " Soglie corrette per mantenere TH1 < TH2 < TH3.";
  server.send(200, "text/plain", msg);
}

void handleMode() {
  // /mode?auto=1|0
  if (server.hasArg("auto")) {
    manualMode = (server.arg("auto") == "0");
    if (!manualMode) desiredRelayLevel = currentHrLevel;
  }
  server.send(200, "text/plain", manualMode ? "MANUALE" : "AUTO");
}

void handleSpeed() {
  // /speed?level=0..3
  if (server.hasArg("level")) {
    int lvl = server.arg("level").toInt();
    if (lvl >= 0 && lvl <= 3) {
      manualMode = true;
      desiredRelayLevel = lvl;
    }
  }
  server.send(200, "text/plain", "OK");
}

// ======= Endpoint: sblocca manuale via web ======= // <<< NEW
void handleUnlock() {
  manualMode = true;
  desiredRelayLevel = 0;     // tutto OFF finché non selezioni una velocità
  autoManualEnabled = true;  // marcatura sblocco attivo
  server.send(200, "text/plain", "MANUALE sbloccato");
  Serial.println("Sblocco manuale richiesto da web: MANUALE attivo");
}

// ============================================================
// Setup / Loop
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 BLE - Ventilatore HR con web server e cambio relè sicuro (millis)");

  bootMillis = millis();   // <<< NEW

  initOutputs();
  loadPrefs();
  printThresholds();

  // WiFi
  IPAddress local_ip(192, 168, 1, 84);     // IP desiderato
  IPAddress gateway(192, 168, 1, 254);     // Gateway
  IPAddress subnet(255, 255, 255, 0);      // Subnet mask
  IPAddress dns1(192, 168, 1, 254);        // DNS primario
  IPAddress dns2(8, 8, 8, 8);              // DNS secondario
  WiFi.config(local_ip, gateway, subnet, dns1, dns2);
  Serial.print("Connessione WiFi ");
  Serial.print(WIFI_SSID);
  Serial.println(" ...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("WiFi OK. IP: ");
  Serial.println(WiFi.localIP());

  // WebServer
  server.on("/", handleRoot);
  server.on("/api/get", handleApiGet);
  server.on("/api/save", handleApiSave);
  server.on("/mode", handleMode);
  server.on("/speed", handleSpeed);
  server.on("/unlock", handleUnlock); // <<< NEW
  server.begin();
  Serial.println("Web server avviato su porta 80.");

  // BLE
  BLEDevice::init("ESP32-HR-BPM");
  BLEDevice::setPower(ESP_PWR_LVL_P7); // opzionale
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(0); // scan continuo
}

void loop() {
  server.handleClient();
  handleSerialCommands();
  relayStateMachineTick();

  // Se ho trovato un target e non sto già provando, tenta la connessione
  if (haveTarget && !connected && !connecting) {
    connecting = true;
    if (!doConnect()) {
      connected = false;
      haveTarget = false;
      connecting = false;
      Serial.println("Riprovo: riavvio la scansione...");
      delay(300);
      pBLEScan->start(0);
    }
  }

  // Riconnessione BLE se cade
  if (connected && pClient && !pClient->isConnected()) {
    Serial.println("Disconnesso. Riparto con la scansione...");
    connected = false;
    haveTarget = false;
    connecting = false;
    pHRChar = nullptr;
    delay(200);
    pBLEScan->start(0);
  }

  // Fail-safe: se nessun BPM recente → OFF (solo in AUTO)
  if (millis() - lastBpmMillis > DATA_TIMEOUT_MS) {
    if (!manualMode && desiredRelayLevel != 0) {
      desiredRelayLevel = 0;  // Trigghera transizione verso OFF
      lastBpmValue = -1;
      Serial.println("Timeout dati BPM. Tutte le uscite OFF.");
    }
  }

  // Fallback: prova a leggere se non arrivano notify
  if (connected && pHRChar && (millis() - lastPoll >= POLL_MS)) {
    lastPoll = millis();
    if (pHRChar->canRead()) {
      String val = pHRChar->readValue();
      int len = val.length();
      if (len > 0) {
        const uint8_t* data = (const uint8_t*)val.c_str();
        uint16_t bpm = parseBPM(data, (size_t)len);
        if (bpm >= MIN_VALID_BPM && bpm <= MAX_VALID_BPM) {
          lastBpmMillis = millis();
          lastBpmValue = (int)bpm;
          Serial.printf("BPM (poll): %u\n", bpm);
          decideLevelWithHysteresis((int)bpm);
        }
      }
    }
  }

  // === Sblocco automatico MANUALE se non c'è HRM entro il timeout === // <<< NEW
  if (!autoManualEnabled && !connected && (millis() - bootMillis) >= HRM_CONNECT_TIMEOUT_MS) {
    manualMode = true;            // abilita controllo manuale
    desiredRelayLevel = 0;        // resta tutto OFF finché non scegli dal web
    autoManualEnabled = true;
    Serial.println("Nessuna fascia collegata entro il timeout: abilitata modalità MANUALE (web attivo).");
  }

  delay(10); // loop respirato
}
``
