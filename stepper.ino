#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <AccelStepper.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "protocol.h"

/*
 * =============================================================
 *  ESP32 Motor Sync (MASTER / SLAVE) – versione compatibile
 *  con nuova firma callback ESP-NOW (esp_now_recv_info *).
 *
 *  Se compili con un core più vecchio, l’#if di compatibilità
 *  userà automaticamente la firma legacy.
 * =============================================================
 */

//====================== CONFIGURAZIONE ======================
#define IS_MASTER          true   // true sul nodo master
#define NODE_ID            1       // ID >=1 per gli slave; 0 riservato a broadcast logico
#define ESPNOW_CHANNEL     6

constexpr int   STEPS_PER_REV = 200;
constexpr int   MICROSTEP_HW  = 1;    // Allinea ai pin MS1/MS2/MS3
#define INVERT_DIRECTION    false

constexpr float RPM_MIN = 0.0f;
constexpr float RPM_MAX = 200.0f;

constexpr float HOMING_RPM_FAST          = 5.0f;
constexpr float HOMING_RPM_SLOW          = 1.5f;
constexpr float HOMING_BACKOFF_FRACT     = 0.15f;
constexpr long  HOMING_TIMEOUT_MS        = 8000;
constexpr long  HOMING_ZERO_OFFSET_STEPS = 0;

constexpr unsigned long BROADCAST_PERIOD_MS = 100;

constexpr int OLED_WIDTH  = 128;
constexpr int OLED_HEIGHT = 64;

#define POT_FILTER_ALPHA 0.25f
#define ENABLE_DEBUG false
inline void dbg(const String& s){ if(ENABLE_DEBUG) Serial.println(s); }

//====================== PINOUT ======================
constexpr int PIN_STEP = 18;
constexpr int PIN_DIR  = 19;
constexpr int PIN_EN   = 23;

constexpr int PIN_POT  = 34;
constexpr int PIN_HALL = 32;
#define HALL_ACTIVE_LOW  true
#define HALL_DEBOUNCE_MS 3

constexpr int PIN_SYNC_BTN = 25; // master: homing broadcast
constexpr int PIN_LOCK_BTN = 33; // master: lock mode

constexpr int PIN_SDA = 21;
constexpr int PIN_SCL = 22;

//====================== OGGETTI ======================
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
AccelStepper      stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR);

//====================== STATO ======================
volatile bool  g_lockMode        = false;
volatile bool  g_sync0_trigger   = false;

float rpm_pot_raw   = 0.0f;
float rpm_pot_filt  = 0.0f;
float rpm_command   = 0.0f;
float rpm_active    = 0.0f;

volatile uint32_t g_txCounter    = 0;
unsigned long     lastBroadcast  = 0;

//====================== OLED BUFFER ======================
unsigned long g_lastOledUpdate = 0;
String g_oledL1, g_oledL2, g_oledL3, g_oledL4;

void setOledLines(const String& a,const String& b,const String& c,const String& d=""){
  g_oledL1=a; g_oledL2=b; g_oledL3=c; g_oledL4=d;
}
void serviceOled(){
  const unsigned long OLED_PERIOD_MS = 120;
  unsigned long now = millis();
  if (now - g_lastOledUpdate < OLED_PERIOD_MS) return;
  g_lastOledUpdate = now;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println(g_oledL1);
  display.println(g_oledL2);
  display.println(g_oledL3);
  if (g_oledL4.length()) display.println(g_oledL4);
  display.display();
}

//====================== TASK STEPPER ======================
volatile float g_targetStepsPerSec = 0.0f;
volatile bool  g_speedDirty        = false;

unsigned long g_lastActualSampleMs = 0;
long          g_lastActualPos      = 0;
float         g_actualStepsPerSec  = 0.0f;

void IRAM_ATTR updateActualStepsPerSec(){
  unsigned long now = millis();
  if (now - g_lastActualSampleMs >= 250){
    long pos   = stepper.currentPosition();
    long delta = pos - g_lastActualPos;
    float dt   = (now - g_lastActualSampleMs)/1000.0f;
    g_actualStepsPerSec = delta / dt;
    g_lastActualPos = pos;
    g_lastActualSampleMs = now;
  }
}

void stepperTask(void*){
  for(;;){
    if (g_speedDirty){
      stepper.setSpeed(g_targetStepsPerSec);
      g_speedDirty = false;
    }
    stepper.runSpeed();
    updateActualStepsPerSec();
  }
}

//====================== ESP-NOW INVIO ======================
bool sendMsg(const EspNowMsg& msg, const uint8_t dest[6]){
  return esp_now_send(dest, reinterpret_cast<const uint8_t*>(&msg), sizeof(EspNowMsg)) == ESP_OK;
}
bool sendBroadcast(const EspNowMsg& msg){
  return sendMsg(msg, ESPNOW_BROADCAST_ADDR);
}

//====================== CALLBACK ESP-NOW (compatibilità) ============
// Rileviamo la versione del core per decidere la firma.
#ifndef ESP_ARDUINO_VERSION
  // Se la macro non esiste, presumiamo core recente (>=3)
  #define ESP_ARDUINO_VERSION_VAL(major, minor, patch) 30000
  #define ESP_ARDUINO_VERSION 30000
#endif

// Funzione comune di parsing (riusata da entrambe le firme)
void handleEspNowPacket(const uint8_t * data, int len){
  if (len < (int)sizeof(EspNowMsg)) return;
  EspNowMsg m;
  memcpy(&m, data, sizeof(EspNowMsg));

  switch(static_cast<EspNowMsgType>(m.type)){
    case EspNowMsgType::SET:
      if (m.target_id == 0 || m.target_id == NODE_ID){
        rpm_command = constrain(m.rpm, RPM_MIN, RPM_MAX);
        if (g_lockMode){
          float sps = rpm_command * (STEPS_PER_REV * MICROSTEP_HW)/60.0f;
          if (INVERT_DIRECTION) sps = -sps;
          g_targetStepsPerSec = sps; g_speedDirty = true;
        }
      }
      break;
    case EspNowMsgType::MODE:
      g_lockMode = (m.mode != 0);
      if (g_lockMode){
        float sps = rpm_command * (STEPS_PER_REV * MICROSTEP_HW)/60.0f;
        if (INVERT_DIRECTION) sps = -sps;
        g_targetStepsPerSec = sps; g_speedDirty = true;
      }
      break;
    case EspNowMsgType::SYNC0:
      g_sync0_trigger = true;
      break;
    case EspNowMsgType::STAT:
    default:
      break;
  }
}

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3,0,0)
// Nuova firma (core 3.x / IDF5)
void onEspNowRecv(const esp_now_recv_info * info, const uint8_t * data, int len){
  (void)info;
  handleEspNowPacket(data, len);
}
#else
// Firma legacy (core < 3.0.0)
void onEspNowRecv(const uint8_t * mac, const uint8_t * data, int len){
  (void)mac;
  handleEspNowPacket(data, len);
}
#endif

void onEspNowSend(const uint8_t * mac, esp_now_send_status_t status){
  (void)mac; (void)status;
}

//====================== INIT ESP-NOW ======================
bool espNowInit(){
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true,true);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK){
    Serial.println("ERRORE esp_now_init");
    return false;
  }
  esp_now_register_send_cb(onEspNowSend);
  esp_now_register_recv_cb(onEspNowRecv);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, ESPNOW_BROADCAST_ADDR, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  esp_err_t st = esp_now_add_peer(&peer);
  if (st != ESP_OK && st != ESP_ERR_ESPNOW_EXIST){
    Serial.printf("ERRORE add_peer: %d\n", st);
    return false;
  }
  return true;
}

//====================== UTILS ======================
float clampf(float v,float a,float b){ if(v<a) return a; if(v>b) return b; return v; }
float potToRpm(int adc){
  float rpm = map(adc,0,4095,(int)(RPM_MIN*10),(int)(RPM_MAX*10))/10.0f;
  return clampf(rpm, RPM_MIN, RPM_MAX);
}
void setEnable(bool en){
  digitalWrite(PIN_EN, en?LOW:HIGH); // DRV8825: EN LOW = attivo
}

// Hall
bool hallActiveRaw(){
  bool v = digitalRead(PIN_HALL);
  return HALL_ACTIVE_LOW ? (v==LOW) : (v==HIGH);
}
bool hallActive(){
  if(!hallActiveRaw()) return false;
  delay(HALL_DEBOUNCE_MS);
  return hallActiveRaw();
}
bool isHomeActive(){ return hallActive(); }

// Homing helpers
void homingMoveConstant(float steps_per_s,unsigned long timeout_ms,bool (*cond)()){
  unsigned long t0 = millis();
  g_targetStepsPerSec = steps_per_s;
  g_speedDirty = true;
  while(!cond()){
    if (millis() - t0 > timeout_ms) break;
    delay(1);
  }
}
void homingMoveFixedSteps(float steps_per_s,long steps_count){
  g_targetStepsPerSec = steps_per_s;
  g_speedDirty = true;
  long start = stepper.currentPosition();
  while(labs(stepper.currentPosition()-start) < steps_count){
    delay(1);
  }
}
void doHomingBlocking(){
  dbg("Homing start");
  setEnable(true);
  float fast = HOMING_RPM_FAST * (STEPS_PER_REV * MICROSTEP_HW)/60.0f;
  float slow = HOMING_RPM_SLOW * (STEPS_PER_REV * MICROSTEP_HW)/60.0f;
  if (INVERT_DIRECTION){ fast=-fast; slow=-slow; }

  if (isHomeActive()){
    homingMoveConstant(fabs(fast), HOMING_TIMEOUT_MS/2, [](){return !isHomeActive();});
  }
  homingMoveConstant(-fabs(fast), HOMING_TIMEOUT_MS, [](){return isHomeActive();});
  long backSteps = (long)(HOMING_BACKOFF_FRACT * STEPS_PER_REV * MICROSTEP_HW);
  homingMoveFixedSteps(fabs(slow), backSteps);
  homingMoveConstant(-fabs(slow), HOMING_TIMEOUT_MS, [](){return isHomeActive();});
  stepper.setCurrentPosition(HOMING_ZERO_OFFSET_STEPS);
  g_targetStepsPerSec = 0; g_speedDirty = true;
  dbg("Homing done");
}

void applyRpm(float rpm){
  rpm = clampf(rpm, RPM_MIN, RPM_MAX);
  rpm_active = rpm;
  float sps = rpm * (STEPS_PER_REV * MICROSTEP_HW)/60.0f;
  if (INVERT_DIRECTION) sps = -sps;
  g_targetStepsPerSec = sps;
  g_speedDirty = true;
  if (fabs(sps) < 1.0f) setEnable(false);
  else                  setEnable(true);
}

//====================== MASTER ======================
void setupMaster(){
  pinMode(PIN_SYNC_BTN, INPUT_PULLUP);
  pinMode(PIN_LOCK_BTN, INPUT_PULLUP);
}
void loopMaster(){
  int adc = analogRead(PIN_POT);
  rpm_pot_raw  = potToRpm(adc);
  rpm_pot_filt = POT_FILTER_ALPHA*rpm_pot_raw + (1.0f-POT_FILTER_ALPHA)*rpm_pot_filt;

  g_lockMode = (digitalRead(PIN_LOCK_BTN)==LOW);

  static bool prevSync = HIGH;
  bool cur = digitalRead(PIN_SYNC_BTN);
  if (prevSync==HIGH && cur==LOW){
    EspNowMsg m{};
    m.type      = static_cast<uint8_t>(EspNowMsgType::SYNC0);
    m.target_id = 0;
    m.counter   = ++g_txCounter;
    sendBroadcast(m);
    g_sync0_trigger = true;
  }
  prevSync = cur;

  unsigned long now = millis();
  if (now - lastBroadcast > BROADCAST_PERIOD_MS){
    lastBroadcast = now;
    // MODE
    {
      EspNowMsg mm{};
      mm.type      = static_cast<uint8_t>(EspNowMsgType::MODE);
      mm.target_id = 0;
      mm.mode      = g_lockMode ? 1 : 0;
      mm.counter   = ++g_txCounter;
      sendBroadcast(mm);
    }
    // SET
    {
      EspNowMsg ms{};
      ms.type      = static_cast<uint8_t>(EspNowMsgType::SET);
      ms.target_id = 0;
      ms.rpm       = rpm_pot_filt;
      ms.counter   = ++g_txCounter;
      sendBroadcast(ms);
    }
  }

  applyRpm(rpm_pot_filt);

  if (g_sync0_trigger){
    g_sync0_trigger = false;
    doHomingBlocking();
  }

  float tgt = g_targetStepsPerSec;
  float act = g_actualStepsPerSec;
  char l1[32],l2[32],l3[32],l4[32];
  snprintf(l1,sizeof(l1),"MASTER RPM %.1f", rpm_pot_filt);
  snprintf(l2,sizeof(l2),"MODE:%s H:%d", g_lockMode?"LOCK":"FREE",(int)isHomeActive());
  snprintf(l3,sizeof(l3),"SPS T:%0.0f", fabs(tgt));
  snprintf(l4,sizeof(l4),"SPS A:%0.0f", fabs(act));
  setOledLines(l1,l2,l3,l4);
  serviceOled();
}

//====================== SLAVE ======================
void loopSlave(){
  int adc = analogRead(PIN_POT);
  rpm_pot_raw  = potToRpm(adc);
  rpm_pot_filt = POT_FILTER_ALPHA*rpm_pot_raw + (1.0f-POT_FILTER_ALPHA)*rpm_pot_filt;

  if (g_sync0_trigger){
    g_sync0_trigger = false;
    doHomingBlocking();
  }

  float targetRpm = g_lockMode ? rpm_command : rpm_pot_filt;
  applyRpm(targetRpm);

  float tgt = g_targetStepsPerSec;
  float act = g_actualStepsPerSec;
  char l1[32],l2[32],l3[32],l4[32];
  snprintf(l1,sizeof(l1),"NODE%d RPM %.1f", NODE_ID, targetRpm);
  snprintf(l2,sizeof(l2),"MODE:%s H:%d", g_lockMode?"LOCK":"FREE",(int)isHomeActive());
  snprintf(l3,sizeof(l3),"CMD:%.1f", rpm_command);
  snprintf(l4,sizeof(l4),"SPS T:%0.0f A:%0.0f", fabs(tgt), fabs(act));
  setOledLines(l1,l2,l3,l4);
  serviceOled();
}

//====================== SETUP ======================
void setup(){
  Serial.begin(115200);
  Serial.println("\n=== Motor Sync avvio ===");

  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  setEnable(true);
  pinMode(PIN_POT, INPUT);
  pinMode(PIN_HALL, INPUT_PULLUP);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)){
    Serial.println("OLED non trovato");
  } else {
    display.clearDisplay(); display.display();
  }

  stepper.setMaxSpeed(25000);

  if (!espNowInit()){
    setOledLines("ESP-NOW FAIL","Check CH","");
    serviceOled();
  }

  if (IS_MASTER){
    setupMaster();
    setOledLines("MASTER avvio","ESP-NOW OK",WiFi.macAddress());
  } else {
    char b1[32];
    snprintf(b1,sizeof(b1),"NODE %d avvio", NODE_ID);
    setOledLines(b1,"ESP-NOW OK",WiFi.macAddress());
  }
  serviceOled();

  rpm_pot_filt = 0;
  g_lastActualSampleMs = millis();
  g_lastActualPos      = stepper.currentPosition();

  // Task stepper su Core 0
  xTaskCreatePinnedToCore(stepperTask,"StepperTask",2048,nullptr,3,nullptr,0);
}

//====================== LOOP ======================
void loop(){
  if (IS_MASTER) loopMaster();
  else           loopSlave();
}
