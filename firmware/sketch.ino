#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <MFRC522.h>
#include <Keypad.h>
#include <Servo.h>

#config
#define PIN_RFID_RST 9
#define PIN_RFID_SS  10
#define PIN_SERVO    6
#define PIN_BUZZER   7
#define PIN_LED_BLUE 2
#define PIN_LED_RED  3

#define FW_VERSION "3.4.0"
#define MAX_CARDS 8
#define MAX_LOGS 20
#define MAX_PIN_LEN 6
#define MAX_ATTEMPTS 3
#define LOCKOUT_MS 30000UL
#define AUTOLOCK_MS 8000UL
#define INACTIVITY_MS 15000UL
#define SERVO_STEP_MS 12
#define SCAN_HOLD_MS 1200

#enums
enum Role : byte { ROLE_GUEST = 0, ROLE_ADMIN = 1, ROLE_MASTER = 2 };

enum EventType : byte {
    EV_LOGIN_OK = 0, EV_LOGIN_FAIL_CARD, EV_LOGIN_FAIL_PIN,
      EV_ALARM, EV_DOOR_OPEN, EV_DOOR_CLOSE,
        EV_CARD_ADDED, EV_CARD_DELETED, EV_PIN_CHANGED, EV_FACTORY_RESET
};

enum State {
    ST_BOOT, ST_IDLE, ST_AUTH_SCAN, ST_AUTH_PIN, ST_MENU,
      ST_STATUS, ST_SECURITY, ST_SEC_ADD_SCAN, ST_SEC_ADD_PIN, ST_SEC_DELETE, ST_SEC_CHANGE_SCAN, ST_SEC_CHANGE_PIN,
        ST_LOGS, ST_SYSTEM, ST_SETTINGS, ST_ABOUT,
          ST_ALARM, ST_DOOR_OPEN, ST_SCREENSAVER
};

enum MenuID : byte { M_LOGIN, M_STATUS, M_LOCK, M_SECURITY, M_SYSTEM, M_LOGS, M_SETTINGS, M_ABOUT };
enum SecID : byte { S_ADD, S_DELETE, S_CHANGE, S_FACTORY, S_SEC_BACK };
enum SetID : byte { SET_BRIGHT, SET_SOUND, SET_ANIM, SET_AUTOLOCK, SET_CLOCK, SET_PINLEN, SET_BACK };

#structs
struct Settings {
    byte soundOn;
      byte animOn;
        byte pinLength;
          byte brightSim;
            uint16_t autolockS;
              byte hour, minute, second;
                byte day, month;
                  int year;
};

struct CardRecord {
    byte uid[4];
      byte role;
        byte pin[MAX_PIN_LEN];
          byte pinLen;
            byte active;
};

struct LogEntry {
    byte eventType;
      byte cardIndex;
        uint32_t timestamp;
};

struct ToneStep { unsigned int freq; unsigned int durMs; };

#eeprom_map
#define EE_MAGIC_ADDR 0
#define EE_MAGIC 0xA5
#define EE_SETTINGS_ADDR 1
#define EE_CARDS_ADDR (EE_SETTINGS_ADDR + sizeof(Settings))
#define EE_LOGIDX_ADDR (EE_CARDS_ADDR + (MAX_CARDS * sizeof(CardRecord)))
#define EE_LOGS_ADDR (EE_LOGIDX_ADDR + 2)
#define cardAddr(i) (EE_CARDS_ADDR + (i) * sizeof(CardRecord))
#define logAddr(i) (EE_LOGS_ADDR + (i) * sizeof(LogEntry))

#globals
const byte ROWS = 4, COLS = 4;
byte rowPins[ROWS] = {A0, A1, A2, A3};
byte colPins[COLS] = {4, 5, 8, 1};
char keys[ROWS][COLS] = {
    {'1','2','3','A'},
      {'4','5','6','B'},
        {'7','8','9','C'},
          {'*','0','#','D'}
};

LiquidCrystal_I2C lcd(0x27, 16, 2);
MFRC522 rfid(PIN_RFID_SS, PIN_RFID_RST);
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);
Servo lockServo;

Settings settings;
State state = ST_BOOT;
State prevState = ST_IDLE;

byte pendingUid[4];
int pendingIdx = -1;
char pinBuf[MAX_PIN_LEN + 1];
byte pinPos = 0;
byte fails = 0;
unsigned long lockoutUntil = 0;

bool doorOpen = false;
unsigned long doorOpenedAt = 0;
int servoAngle = 0, servoTarget = 0;
unsigned long lastServoStep = 0;

unsigned long lastActivity = 0;
unsigned long stateEnteredAt = 0;
unsigned long lastSecondAt = 0;

int menuIdx = 0, secIdx = 0, setIdx = 0, logView = 0, delIdx = 0;

#melodies
const ToneStep M_BOOT[]    PROGMEM = {{523,80},{659,80},{784,120}};
const ToneStep M_CLICK[]   PROGMEM = {{1200,25}};
const ToneStep M_SUCCESS[] PROGMEM = {{880,70},{1174,70},{1568,140}};
const ToneStep M_FAIL[]    PROGMEM = {{220,150},{160,220}};
const ToneStep M_ALARM[]   PROGMEM = {{900,120},{500,120}};
const ToneStep M_UNLOCK[]  PROGMEM = {{659,60},{988,120}};
const ToneStep M_NAV[]     PROGMEM = {{900,20}};

const ToneStep* activeMel = nullptr;
byte melLen = 0, melPos = 0;
unsigned long melEndAt = 0;
bool melLoop = false;

#prototypes
void loadOrInitEEPROM();
void loadSettings();
void saveSettings();
void enterState(State s);
void updateLEDs();
void updateBuzzer();
void playMelody(const ToneStep* m, byte len, bool loop=false);
void stopMelody();
void updateServo();
void openDoor();
void closeDoor();
int findCard(byte* uid);
int freeSlot();
byte countCards();
void addLog(EventType ev, byte idx);
void tickClock();
bool scanCard();
void lcdCenter(byte r, const char* t);
void typeLine(byte r, const char* t, unsigned int ms);
void barLine(byte r, byte filled, byte total);
void bootSequence();
void idle(char key);
void authScan();
void authPin(char key);
void menu(char key);
void status(char key);
void security(char key);
void secAddScan(char key);
void secAddPin(char key);
void secDelete(char key);
void secChangeScan(char key);
void secChangePin(char key);
void logs(char key);
void systemScrn(char key);
void settings(char key);
void about(char key);
void alarm();
void doorOpenScrn();
void screensaver(char key);
const char* eventName(byte ev);

#setup
void setup() {
    pinMode(PIN_LED_BLUE, OUTPUT);
      pinMode(PIN_LED_RED, OUTPUT);
        pinMode(PIN_BUZZER, OUTPUT);

          Wire.begin();
            lcd.init();
              lcd.backlight();

                SPI.begin();
                  rfid.PCD_Init();

                    lockServo.attach(PIN_SERVO);
                      lockServo.write(0);
                        servoAngle = 0; servoTarget = 0;

                          loadOrInitEEPROM();
                            loadSettings();

                              lastSecondAt = millis();
                                enterState(ST_BOOT);
}

#loopvoid loop() {
    updateLEDs();
      updateBuzzer();
        updateServo();
          tickClock();

         char key = keypad.getKey();
           if (key) {
                lastActivity = millis();
                    if (settings.soundOn) playMelody(M_CLICK, 1);
                      
 
 
   if (state != ST_BOOT && state != ST_SCREENSAVER && state != ST_ALARM && state != ST_DOOR_OPEN) {
        if (millis() - lastActivity > INACTIVITY_MS) enterState(ST_SCREENSAVER);
          
 
 
   switch (state) {
        case ST_BOOT: break;
            case ST_IDLE: idle(key); break;
                case ST_AUTH_SCAN: authScan(); break;
                    case ST_AUTH_PIN: authPin(key); break;
                        case ST_MENU: menu(key); break;
                            case ST_STATUS: status(key); break;
                                case ST_SECURITY: security(key); break;
                                    case ST_SEC_ADD_SCAN: secAddScan(key); break;
                                        case ST_SEC_ADD_PIN: secAddPin(key); break;
                                            case ST_SEC_DELETE: secDelete(key); break;
                                                case ST_SEC_CHANGE_SCAN: secChangeScan(key); break;
                                                    case ST_SEC_CHANGE_PIN: secChangePin(key); break;
                                                        case ST_LOGS: logs(key); break;
                                                            case ST_SYSTEM: systemScrn(key); break;
                                                                case ST_SETTINGS: settings(key); break;
                                                                    case ST_ABOUT: about(key); break;
                                                                        case ST_ALARM: alarm(); break;
                                                                            case ST_DOOR_OPEN: doorOpenScrn(); break;
                                                                                case ST_SCREENSAVER: screensaver(key); break;
                                                                                  
   
   
   #state_machine
   void enterState(State s) {
      prevState = state;
        state = s;
          stateEnteredAt = millis();
            lcd.clear();

              switch (s) {
                    case ST_BOOT: bootSequence(); break;
                        case ST_AUTH_PIN: pinPos = 0; memset(pinBuf, 0, sizeof(pinBuf)); break;
                            case ST_MENU: menuIdx = 0; break;
                                case ST_LOGS: logView = 0; break;
                                    case ST_SEC_CHANGE_PIN: pinPos = 0; memset(pinBuf, 0, sizeof(pinBuf)); break;
                                        case ST_ALARM:
                                              digitalWrite(PIN_LED_RED, HIGH);
                                                    if (settings.soundOn) playMelody(M_ALARM, 2, true);
                                                          break;
                                                              case ST_DOOR_OPEN:
                                                                    openDoor();
                                                                          if (settings.soundOn) playMelody(M_UNLOCK, 2);
                                                                                break;
                                                                                    default: break;
                                                                                      
   
   
   #eeprom_mapvoid loadOrInitEEPROM() {
      byte m;
        EEPROM.get(EE_MAGIC_ADDR, m);
          if (m == EE_MAGIC) return;

            Settings s;
              memset(&s, 0, sizeof(s));
                s.soundOn = 1; s.animOn = 1; s.pinLength = 4; s.brightSim = 100;
                  s.autolockS = 8; s.hour = 0; s.minute = 0; s.second = 0;
                    s.day = 1; s.month = 1; s.year = 2025;
                      EEPROM.put(EE_SETTINGS_ADDR, s);

                        CardRecord empty;
                          memset(&empty, 0, sizeof(empty));
                            for (byte i = 0; i < MAX_CARDS; i++) EEPROM.put(cardAddr(i), empty);

                              uint16_t li = 0;
                                EEPROM.put(EE_LOGIDX_ADDR, li);

                                  LogEntry blank;
                                    blank.eventType = 0xFF; blank.cardIndex = 0xFF; blank.timestamp = 0;
                                      for (byte i = 0; i < MAX_LOGS; i++) EEPROM.put(logAddr(i), blank);

                                        EEPROM.put(EE_MAGIC_ADDR, (byte)EE_MAGIC);
 
 
 void loadSettings() {
    EEPROM.get(EE_SETTINGS_ADDR, settings);
      if (settings.pinLength == 0 || settings.pinLength > MAX_PIN_LEN) settings.pinLength = 4;
        if (settings.autolockS == 0) settings.autolockS = 8;
 
 
 void saveSettings() { EEPROM.put(EE_SETTINGS_ADDR, settings); }
 
 int findCard(byte* uid) {
    for (byte i = 0; i < MAX_CARDS; i++) {
          CardRecord c;
              EEPROM.get(cardAddr(i), c);
                  if (c.active && memcmp(c.uid, uid, 4) == 0) return i;
                    
 
   return -1;  
   
   int freeSlot() {
      for (byte i = 0; i < MAX_CARDS; i++) {
            CardRecord c;
                EEPROM.get(cardAddr(i), c);
                    if (!c.active) return i;
                      
 
   return -1;  
   
   byte countCards() {
      byte n = 0;
        for (byte i = 0; i < MAX_CARDS; i++) {
              CardRecord c;
                  EEPROM.get(cardAddr(i), c);
                      if (c.active) n++;
                        
 
   return n;  
   
   void addLog(EventType ev, byte idx) {
      uint16_t w;
        EEPROM.get(EE_LOGIDX_ADDR, w);
          LogEntry e;
            e.eventType = ev; e.cardIndex = idx;
              e.timestamp = (uint32_t)(millis() / 1000UL);
                EEPROM.put(logAddr(w % MAX_LOGS), e);
                  w = (w + 1) % MAX_LOGS;
                    EEPROM.put(EE_LOGIDX_ADDR, w);
 
 
 #tickClockvoid tickClock() {
    if (millis() - lastSecondAt < 1000) return;
      lastSecondAt += 1000;
        settings.second++;
          if (settings.second >= 60) { settings.second = 0; settings.minute++; }
            if (settings.minute >= 60) { settings.minute = 0; settings.hour++; }
              if (settings.hour >= 24) { settings.hour = 0; settings.day++; }
                if (settings.day > 31) { settings.day = 1; settings.month++; }
                  if (settings.month > 12) { settings.month = 1; settings.year++; }
 
 
 void printClock(char* buf, byte buflen) {
    snprintf(buf, buflen, "%02d:%02d:%02d", settings.hour, settings.minute, settings.second);
 
 
 #PIN_LED_BLUEvoid updateLEDs() {
    unsigned long now = millis();
      if (state == ST_ALARM) { digitalWrite(PIN_LED_BLUE, LOW); }
        else if (state == ST_AUTH_SCAN || state == ST_AUTH_PIN || state == ST_SEC_ADD_SCAN || state == ST_SEC_CHANGE_SCAN) {
              unsigned int p = now % 800;
                  unsigned int d = (p < 400) ? map(p,0,400,20,100) : map(p,400,800,100,20);
                      digitalWrite(PIN_LED_BLUE, (now % 20) < (d * 20UL / 100));
                        
  else {
        unsigned int p = now % 3000;
            unsigned int d = (p < 1500) ? map(p,0,1500,5,100) : map(p,1500,3000,100,5);
                digitalWrite(PIN_LED_BLUE, (now % 20) < (d * 20UL / 100));
                  
 
 
   if (state == ST_ALARM) digitalWrite(PIN_LED_RED, (now % 300) < 150);
     else if (millis() < lockoutUntil) digitalWrite(PIN_LED_RED, (now % 500) < 250);
       else digitalWrite(PIN_LED_RED, LOW); }
       
       #updateBuzzervoid playMelody(const ToneStep* m, byte len, bool loop) {
          activeMel = m; melLen = len; melPos = 0; melLoop = loop;
            ToneStep s;
              memcpy_P(&s, &m[0], sizeof(ToneStep));
                tone(PIN_BUZZER, s.freq);
                  melEndAt = millis() + s.durMs;
 
 
 void stopMelody() { activeMel = nullptr; noTone(PIN_BUZZER); }
 
 void updateBuzzer() {
    if (!activeMel) return;
      if (millis() < melEndAt) return;
        melPos++;
          if (melPos >= melLen) {
                if (melLoop) melPos = 0;
                    else { stopMelody(); return; }
                      
 
   ToneStep s;
     memcpy_P(&s, &activeMel[melPos], sizeof(ToneStep));
       tone(PIN_BUZZER, s.freq);
         melEndAt = millis() + s.durMs;  
         
         #servoAnglevoid updateServo() {
            if (servoAngle == servoTarget) return;
              if (millis() - lastServoStep < SERVO_STEP_MS) return;
                lastServoStep = millis();
                  servoAngle += (servoTarget > servoAngle) ? 1 : -1;
                    lockServo.write(servoAngle);
 
 
 void openDoor() {
    doorOpen = true;
      doorOpenedAt = millis();
        servoTarget = 90;
          addLog(EV_DOOR_OPEN, pendingIdx < 0 ? 0xFF : (byte)pendingIdx);
 
 
 void closeDoor() {
    doorOpen = false;
      servoTarget = 0;
        addLog(EV_DOOR_CLOSE, pendingIdx < 0 ? 0xFF : (byte)pendingIdx);
 
 
 #rfidbool scanCard() {
    if (!rfid.PICC_IsNewCardPresent()) return false;
      if (!rfid.PICC_ReadCardSerial()) return false;
        memcpy(pendingUid, rfid.uid.uidByte, 4);
          rfid.PICC_HaltA();
            return true;
 
 
 #lcd_helpers
 void lcdCenter(byte r, const char* t) {
    int len = strlen(t);
      int pad = (16 - len) / 2; if (pad < 0) pad = 0;
        lcd.setCursor(0, r); lcd.print(F("                "));
          lcd.setCursor(pad, r); lcd.print(t);
 
 
 void typeLine(byte r, const char* t, unsigned int ms) {
    lcd.setCursor(0, r);
      for (int i = 0; t[i]; i++) { lcd.print(t[i]); delay(ms); }
 
 
 void barLine(byte r, byte filled, byte total) {
    lcd.setCursor(0, r);
      for (byte i = 0; i < total; i++) lcd.print((char)(i < filled ? 255 : 176));
 
 
 #bootSequencevoid bootSequence() {
    lcd.clear();
      lcdCenter(0, F("NETIC BIOS"));
        char v[17]; snprintf(v, sizeof(v), "v%s", FW_VERSION);
          lcdCen v
;
  delay(700);
  
    lcd.clear();
      typeLine(0, "Memory Check...", 25);
        lcd.setCursor(0, 1);
          for (byte i = 0; i < 16; i++) { lcd.print((char)255); delay(45); }
            delay(300);
            
              const char* steps[] = {"Loading Sec Kernel","Init RFID...","Crypto Engine","Loading Drivers","Connecting..."};
                for (byte s = 0; s < 5; s++) {
                      lcd.clear();
                          typeLine(0, steps[s], 18);
                              if (settings.animOn) barLine(1, s + 1, 5);
                                  delay(350);
                                    
 
 
   lcd.clear();
     lcdCenter(0, F("ACCESS TERMINAL"));
       lcdCenter(1, F("READY"));
         delay(900);
           if (settings.soundOn) playMelody(M_BOOT, 3);
             enterState(ST_IDLE);  
             
             #idlevoid idle(char key) {
                static unsigned long last = 0;
                  if (millis() - last > 500) {
                        last = millis();
                            char t[9]; printClock(t, sizeof(t));
                                lcd.setCursor(0, 0);
                                    lcd.print(t);
                                        lcd.print(doorOpen ? F(" DOOR OPEN ") : F("            "));
                                            lcd.setCursor(0, 1);
                                                lcd.print(F("SCAN CARD #MENU"));
                                                  
 
 
   if (key == '#') { enterState(ST_MENU); return; }
   
     if (scanCard()) {
          if (millis() < lockoutUntil) { if (settings.soundOn) playMelody(M_FAIL, 2); enterState(ST_ALARM); return; }
              int idx = findCard(pendingUid);
                  if (idx < 0) {
                          fails++;
                                addLog(EV_LOGIN_FAIL_CARD, 0xFF);
                                      if (settings.soundOn) playMelody(M_FAIL, 2);
                                            lcd.clear(); lcdCenter(0, F("ACCESS DENIED"));
                                                  lcdCenter(1, F("UNKNOWN CARD"));
                                                        delay(SCAN_HOLD_MS);
                                                              if (fails >= MAX_ATTEMPTS) { lockoutUntil = millis() + LOCKOUT_MS; enterState(ST_ALARM); }
                                                                    else enterState(ST_IDLE);
                                                                          return;
                                                                              
 
     pendingIdx = idx;
         enterState(ST_AUTH_PIN);
               
               
               #ST_AUTH_SCANvoid authScan() {
                  static bool done = false;
                    if (stateEnteredAt == millis()) done = false;
                      lcdCenter(0, F("SCANNING..."));
                        lcdCenter(1, F("PLEASE WAIT"));
                          if (!done) {
                                done = true;
                                    delay(400);
                                        if (millis() < lockoutUntil) { enterState(ST_ALARM); return; }
                                            int idx = findCard(pendingUid);
                                                if (idx < 0) {
                                                        fails++; addLog(EV_LOGIN_FAIL_CARD, 0xFF);
                                                              if (settings.soundOn) playMelody(M_FAIL, 2);

      lcd.clear(); lcdCenter(0, F("ACCESS DENIED"));
            lcdCenter(1, F("UNKNOWN CARD"));
                  delay(SCAN_HOLD_MS);
                        if (fails >= MAX_ATTEMPTS) { lockoutUntil = millis() + LOCKOUT_MS; enterState(ST_ALARM); }
                              else enterState(ST_IDLE);
                                    return;
                                        
 
     pendingIdx = idx;
         enterState(ST_AUTH_PIN);
               
               
               #ST_AUTH_PINvoid authPin(char key) {
                  static unsigned long last = 0;
                    if (millis() - last > 200 || key) {
                          last = millis();
                              lcd.setCursor(0, 0); lcd.print(F("ENTER PIN:      "));
                                  lcd.setCursor(0, 1);
                                      for (byte i = 0; i < pinPos; i++) lcd.print('*');
                                          for (byte i = pinPos; i < 16; i++) lcd.print(' ');
                                            
 
   if (!key) return;
     if (key == 'D') { enterState(ST_IDLE); return; }
       if (key == '*') { if (pinPos > 0) pinPos--; return; }
         if (key == '#') {
              CardRecord c; EEPROM.get(cardAddr(pendingIdx), c);
                  bool ok = (pinPos == c.pinLen);
                      for (byte i = 0; ok && i < pinPos; i++) if (pinBuf[i] - '0' != c.pin[i]) ok = false;
                          lcd.clear();
                              if (ok) {
                                      fails = 0;
                                            addLog(EV_LOGIN_OK, pendingIdx);
                                                  if (settings.soundOn) playMelody(M_SUCCESS, 3);
                                                        lcdCenter(0, F("ACCESS GRANTED"));
                                                              enterState(ST_DOOR_OPEN);
                                                                  
  else {
          fails++;
                addLog(EV_LOGIN_FAIL_PIN, pendingIdx);
                      if (settings.soundOn) playMelody(M_FAIL, 2);
                            lcdCenter(0, F("ACCESS DENIED"));
                                  lcdCenter(1, F("WRONG PIN"));
                                        delay(SCAN_HOLD_MS);
                                              if (fails >= MAX_ATTEMPTS) { lockoutUntil = millis() + LOCKOUT_MS; enterState(ST_ALARM); }
                                                    else enterState(ST_IDLE);
                                                        
 
     return;
        }
          if (key >= '0' && key <= '9' && pinPos < settings.pinLength && pinPos < MAX_PIN_LEN) pinBuf[pinPos++] = key;  
          
          #menuvoid menu(char key) {
              static int lastIdx = -1;
                const char* items[] = {"LOGIN","STATUS","LOCK","SECURITY","SYSTEM","LOGS","SETTINGS","ABOUT"};
                  if (menuIdx != lastIdx) {
                        lastIdx = menuIdx;
                            lcd.setCursor(0, 0);
                                char b[17]; snprintf(b, sizeof(b), "> %-14s", items[menuIdx]);
                                    lcd.print(b);
                                        int nx = (menuIdx + 1) % 8;
                                            lcd.setCursor(0, 1);
                                                snprintf(b, sizeof(b), "  %-14s", items[nx]);
                                                    lcd.print(b);
                                                      
 
   if (!key) return;
     if (key == 'C') menuIdx = (menuIdx + 1) % 8;
       else if (key == 'D') enterState(ST_IDLE);
         else if (key == '#') {
              lastIdx = -1;
                  switch (menuIdx) {
                          case M_LOGIN: enterState(ST_IDLE); break;
                                case M_STATUS: enterState(ST_STATUS); break;
                                      case M_LOCK: if (!doorOpen) { openDoor(); if (settings.soundOn) playMelody(M_UNLOCK,2);} else { closeDoor(); } enterState(ST_IDLE); break;
                                            case M_SECURITY: secIdx = 0; enterState(ST_SECURITY); break;
                                                  case M_SYSTEM: enterState(ST_SYSTEM); break;
                                                        case M_LOGS: enterState(ST_LOGS); break;
                                                              case M_SETTINGS: setIdx = 0; enterState(ST_SETTINGS); break;
                                                                    case M_ABOUT: enterState(ST_ABOUT); break;
                                                                        
 
       
       
       #statusvoid status(char key) {
          if (key == 'D') { enterState(ST_MENU); return; }
            static unsigned long last = 0;
              if (millis() - last > 400) {
                    last = millis();
                        char t[9]; printClock(t, sizeof(t));
                            char l0[17], l1[17];
                                snprintf(l0, sizeof(l0), "%s D:%s", t, doorOpen ? "OPN" : "SHU");
                                    snprintf(l1, sizeof(l1), "ALM:%s F:%d", (millis() < lockoutUntil) ? "Y" : "N", fails);
                                        lcd.setCursor(0, 0); lcd.print(l0);
                                            lcd.setCursor(0, 1); lcd.print(l1);
                                              
   
   
   #securityvoid security(char key) {
      static const char* items[] = {"ADD CARD","DELETE CARD","CHANGE PIN","FACTORY","BACK"};
        if (millis() - stateEnteredAt < 30) { lcd.setCursor(0,0); lcd.print(F("SECURITY MENU   ")); }
          static byte lastSel = 255;
            if (secIdx != lastSel) {
                  lastSel = secIdx;
                      char b[17]; snprintf(b, sizeof(b), "> %-14s", items[secIdx]);
                          lcd.setCursor(0, 1); lcd.print(b);
                            
 
   if (!key) return;
     if (key == 'C') secIdx = (secIdx + 1) % 5;
       else if (key == 'D') enterState(ST_MENU);
         else if (key == '#') {
              lastSel = 255;
                  switch (secIdx) {
                          case S_ADD: enterState(ST_SEC_ADD_SCAN); break;
                                case S_DELETE: delIdx = 0; enterState(ST_SEC_DELETE); break;
                                      case S_CHANGE: enterState(ST_SEC_CHANGE_SCAN); break;
                                            case S_FACTORY:
                                                    for (byte i = 0; i < MAX_CARDS; i++) { CardRecord e; memset(&e,0,sizeof(e)); EEPROM.put(cardAddr(i), e); }
                                                            addLog(EV_FACTORY_RESET, 0xFF);
                                                                    lcd.clear(); lcdCenter(0, F("FACTORY RESET"));
                                                                            if (settings.soundOn) playMelody(M_SUCCESS, 3);
                                                                                    delay(900); enterState(ST_SECURITY); break;
                                                                                          case S_SEC_BACK: enterState(ST_MENU); break;
                                                                                              
 
       
       
       #ST_SEC_ADD_SCANvoid secAddScan(char key) {
          lcdCenter(0, F("SCAN NEW CARD"));
            lcdCenter(1, F("D=CANCEL"));
              if (key == 'D') { enterState(ST_SECURITY); return; }
                if (scanCard()) {
                      if (findCard(pendingUid) >= 0) { lcdCenter(0, F("CARD EXIST"))}S"
 ; delay(1000); enterState(ST_SECURITY); return;  
     int slot = freeSlot();
         if (slot < 0) { lcdCenter(0, F("STORAGE FULL")); delay(1000); enterState(ST_SECURITY); return; }
             pendingIdx = slot;
                 enterState(ST_SEC_ADD_PIN);
                       
                       
                       #ST_SEC_ADD_PINvoid secAddPin(char key) {
                          static unsigned long last = 0;
                            if (millis() - last > 200 || key) {
                                  last = millis();
                                      lcd.setCursor(0, 0); lcd.print(F("SET PIN:        "));
                                          lcd.setCursor(0, 1);
                                              for (byte i = 0; i < pinPos; i++) lcd.print('*');
                                                  for (byte i = pinPos; i < 16; i++) lcd.print(' ');
                                                    
 
   if (!key) return;
     if (key == 'D') { enterState(ST_SECURITY); return; }
       if (key == '*') { if (pinPos > 0) pinPos--; return; }
         if (key == '#') {
              if (pinPos == 0) return;
                  CardRecord c; memset(&c, 0, sizeof(c));
                      memcpy(c.uid, pendingUid, 4);
                          c.role = ROLE_GUEST; c.pinLen = pinPos;
                              for (byte i = 0; i < pinPos; i++) c.pin[i] = pinBuf[i] - '0';
                                  c.active = 1;
                                      EEPROM.put(cardAddr(pendingIdx), c);
                                          addLog(EV_CARD_ADDED, pendingIdx);
                                              if (settings.soundOn) playMelody(M_SUCCESS, 3);
                                                  lcd.clear(); lcdCenter(0, F("CARD SAVED"));
                                                      delay(900); enterState(ST_SECURITY);
                                                          return;
                                                            
 
   if (key >= '0' && key <= '9' && pinPos < settings.pinLength && pinPos < MAX_PIN_LEN) pinBuf[pinPos++] = key;  
   
   #ST_SEC_DELETEvoid secDelete(char key) {
      static unsigned long last = 0;
        while (delIdx < MAX_CARDS) {
              CardRecord c; EEPROM.get(cardAddr(delIdx), c);
                  if (c.active) break;
                      delIdx++;
                        
 
   if (millis() - last > 250) {
        last = millis();
            if (delIdx >= MAX_CARDS) { lcdCenter(0, F("NO CARDS")); lcdCenter(1, F("D=BACK")); }
                else {
                        char b[17]; snprintf(b, sizeof(b), "SLOT %d  #=DEL  ", delIdx);
                              lcd.setCursor(0, 0); lcd.print(b);
                                    lcd.setCursor(0, 1); lcd.print(F("C=NEXT D=BACK  "));
                                        
 
     
       if (!key) return;
         if (key == 'D') { enterState(ST_SECURITY); return; }
           if (delIdx >= MAX_CARDS) return;
             if (key == 'C') delIdx++;
               else if (key == '#') {
                    CardRecord c; memset(&c, 0, sizeof(c));
                        EEPROM.put(cardAddr(delIdx), c);
                            addLog(EV_CARD_DELETED, delIdx);
                                if (settings.soundOn) playMelody(M_SUCCESS, 2);
                                    lcd.clear(); lcdCenter(0, F("DELETED"));
                                        delay(800); last = 0;
                                          
   
   
   #ST_SEC_CHANGE_SCANvoid secChangeScan(char key) {
      lcdCenter(0, F("SCAN CARD"));
        lcdCenter(1, F("D=CANCEL"));
          if (key == 'D') { enterState(ST_SECURITY); return; }
            if (scanCard()) {
                  int idx = findCard(pendingUid);
                      if (idx < 0) { lcdCenter(0, F("NOT FOUND")); delay(1000); enterState(ST_SECURITY); return; }
                          pendingIdx = idx;
                              enterState(ST_SEC_CHANGE_PIN);
                                
   
   
   #ST_SEC_CHANGE_PINvoid secChangePin(char key) {
      static unsigned long last = 0;
        if (millis() - last > 200 || key) {
              last = millis();
                  lcd.setCursor(0, 0); lcd.print(F("NEW PIN:        "));
                      lcd.setCursor(0, 1);
                          for (byte i = 0; i < pinPos; i++) lcd.print('*');
                              for (byte i = pinPos; i < 16; i++) lcd.print(' ');
                                
 
   if (!key) return;
     if (key == 'D') { enterState(ST_SECURITY); return; }
       if (key == '*') { if (pinPos > 0) pinPos--; return; }
         if (key == '#') {
              if (pinPos == 0) return;
                  CardRecord c; EEPROM.get(cardAddr(pendingIdx), c);
                      c.pinLen = pinPos;
                          for (byte i = 0; i < pinPos; i++) c.pin[i] = pinBuf[i] - '0';
                              EEPROM.put(cardAddr(pendingIdx), c);
                                  addLog(EV_PIN_CHANGED, pendingIdx);
                                      if (settings.soundOn) playMelody(M_SUCCESS, 3);
                                          lcd.clear(); lcdCenter(0, F("PIN CHANGED"));
                                              delay(900); enterState(ST_SECURITY);
                                                  return;
                                                    
 
   if (key >= '0' && key <= '9' && pinPos < settings.pinLength && pinPos < MAX_PIN_LEN) pinBuf[pinPos++] = key;  
   
   #logsvoid logs(char key) {
      if (key == 'D') { enterState(ST_MENU); return; }
        if (key == 'C') logView = (logView + 1) % MAX_LOGS;
          static int lastDrawn = -1;
            if (logView != lastDrawn || millis() - stateEnteredAt < 30) {
                  lastDrawn = logView;
                      uint16_t w; EEPROM.get(EE_LOGIDX_ADDR, w);
                          int slot = (w - 1 - logView + MAX_LOGS * 2) % MAX_LOGS;
                              LogEntry e; EEPROM.get(logAddr(slot), e);
                                  char l0[17], l1[17];
                                      if (e.eventType == 0xFF) {
                                              snprintf(l0, sizeof(l0), "-- EMPTY --     ");
                                                    snprintf(l1, sizeof(l1), "C=NEXT D=BACK   ");
                                                        
  else {
          snprintf(l0, sizeof(l0), "%-16s", eventName(e.eventType));
                unsigned long t = e.timestamp;
                      snprintf(l1, sizeof(l1), "t+%02lu:%02lu:%02lu", t/3600, (t/60)%60, t%60);
                          
 
     lcd.setCursor(0, 0); lcd.print(l0);
         lcd.setCursor(0, 1); lcd.print(l1);
            }  
            
            const char* eventName(byte ev) {
                switch (ev) {
                      case EV_LOGIN_OK: return "LOGIN OK";
                          case EV_LOGIN_FAIL_CARD: return "BAD CARD";
                              case EV_LOGIN_FAIL_PIN: return "BAD PIN";
                                  case EV_ALARM: return "ALARM";
                                      case EV_DOOR_OPEN: return "DOOR OPEN";
                                          case EV_DOOR_CLOSE: return "DOOR SHUT";
                                              case EV_CARD_ADDED: return "CARD ADD";
                                                  case EV_CARD_DELETED: return "CARD DEL";
                                                      case EV_PIN_CHANGED: return "PIN CHG";
                                                          case EV_FACTORY_RESET: return "FACTORY";
                                                              default: return "---";
                                                                
   
   
   #systemScrnvoid systemScrn(char key) {
      if (key == 'D') { enterState(ST_MENU); return; }
        static unsigned long last = 0;
          if (millis() - last > 500) {
                last = millis();
                    char t[9]; printClock(t, sizeof(t));
                        char l0[17], l1[17];
                            snprintf(l0, sizeof(l0), "UP:%lus C:%d/%d", millis()/1000UL, countCards(), MAX_CARDS);
                                snprintf(l1, sizeof(l1), "v%s %s", FW_VERSION, t);
                                    lcd.setCursor(0, 0); lcd.print(l0);
                                        lcd.setCursor(0, 1); lcd.print(l1);
                                          
   
   
   #settingsvoid settings(char key) {
      static const char* labels[] = {"BRIGHT","SOUND","ANIM","AUTOLOCK","CLOCK","PINLEN","BACK"};
        if (key == 'D') { enterState(ST_MENU); return; }
          if (key == 'C') setIdx = (setIdx + 1) % 7;
            if (key == '#') {
                  switch (setIdx) {
                          case SET_BRIGHT: settings.brightSim = (settings.brightSim >= 100) ? 25 : settings.brightSim + 25; break;
                                case SET_SOUND: settings.soundOn = !settings.soundOn; break;
                                      case SET_ANIM: settings.animOn = !settings.animOn; break;
                                            case SET_AUTOLOCK: settings.autolockS = (settings.autolockS >= 30) ? 4 : settings.autolockS + 2; break;
                                                  case SET_CLOCK:
                                                          settings.minute += 5; if (settings.minute >= 60) { settings.minute = 0; settings.hour = (settings.hour + 1) % 24; }
                                                                  break;
                                                                        case SET_PINLEN: settings.pinLength = (settings.pinLength >= 6) ? 4 : settings.pinLength + 1; break;
                                                                              case SET_BACK: enterState(ST_MENU); return;
                                                                                  
 
     saveSettings();
         if (settings.soundOn) playMelody(M_NAV, 1);
             
               static unsigned long last = 0;
                 if (millis() - last > 200 || key) {
                      last = millis();
                          char l0[17], l1[17];
                              snprintf(l0, sizeof(l0), "> %-14s", labels[setIdx]);
                                  switch (setIdx) {
                                          case SET_BRIGHT: snprintf(l1, sizeof(l1), "  %d%%           ", settings.brightSim); break;
                                                case SET_SOUND: snprintf(l1, sizeof(l1), "  %-14s", settings.soundOn ? "ON" : "OFF"); break;
                                                      case SET_ANIM: snprintf(l1, sizeof(l1), "  %-14s", settings.animOn ? "ON" : "OFF"); break;
                                                            case SET_AUTOLOCK: snprintf(l1, sizeof(l1), "  %ds            ", settings.autolockS); break;
                                                                  case SET_CLOCK: { char t[9]; printClock(t,sizeof(t)); snprintf(l1, sizeof(l1), "  %-14s", t); } break;
                                                                        case SET_PINLEN: snprintf(l1, sizeof(l1), "  %d digits      ", settings.pinLength); break;
                                                                              case SET_BACK: snprintf(l1, sizeof(l1), "  %-14s", "EXIT"); break;
                                                                                  
 
     lcd.setCursor(0, 0); lcd.print(l0);
         lcd.setCursor(0, 1); lcd.print(l1);
               
               
               #ABOUTvoid about(char key) {
                  if (key == 'D') { enterState(ST_MENU); return; }
                    static byte page = 0;
                      if (key == 'C') page = (page + 1) % 2;
                        static unsigned long last = 0;
                          if (millis() - last > 300 || key) {
                                last = millis();
                                    if (page == 0) { lcdCenter(0, F("NETIC CYBER TERMINAL")); lcdCenter(1, F("v3.4.0")); }
                                        else { lcd.setCursor(0,0); lcd.print(__DATE__); lcd.setCursor(0,1); lcd.print(__TIME__); }
                                          
   
   
   #alarmvoid alarm() {
      static unsigned long last = 0;
        if (millis() - last > 300) {
              last = millis();
                  lcdCenter(0, F("!! LOCKED OUT !!"));
                      unsigned long rem = (lockoutUntil > millis()) ? (lockoutUntil - millis()) / 1000UL : 0;
                          char b[17]; snprintf(b, sizeof(b), "WAIT %lus       ", rem);
                              lcdCenter(1, b);
                                
 
   if (millis() >= lockoutUntil) { fails = 0; stopMelody(); enterState(ST_IDLE); }  
   
   #doorOpenvoid doorOpenScrn() {
      static unsigned long last = 0;
        if (millis() - last > 300) {
              last = millis();
                  lcdCenter(0, F("ACCESS GRANTED"));
                      unsigned long rem = (doorOpenedAt + (unsigned long)settings.autolockS * 1000UL > millis())
                                                ? (doorOpenedAt + (unsigned long)settings.autolockS * 1000UL - millis()) / 1000UL : 0;
                                                    char b[17]; snprintf(b, sizeof(b), "LOCK IN %lus     ", rem);
                                                        lcdCenter(1, b);
                                                          
 
   if (millis() - doorOpenedAt > (unsigned long)settings.autolockS * 1000UL) { closeDoor(); enterState(ST_IDLE); }  
   
   #screensavervoid screensaver(char key) {
      if (key) { lastActivity = millis(); enterState(ST_IDLE); return; }
        static unsigned long last = 0;
          if (millis() - last > 400) {
                last = millis();
                    lcd.clear();
                        int pos = (millis() / 400) % 16;
                            lcd.setCursor(pos, 0); lcd.print('*');
                                lcdCenter(1, F("NETIC STANDBY"));
                                  
 
   if (scanCard()) enterState(ST_AUTH_SCAN);  
          }
   }     }
   }     }
   }                       }
               }                             }
                 }               }
            }
   }       }
   }             }
            }                                   }
            }
   }      }       }
   }         }
   }            }             }
   }       }
   }      }                           }
                       }          }
       }             }
         }           }
   }           }
       }             }
         }                 }
          }                           }
         }                   }
               }                                           }
                          }
               }             }
     }                 }
             }             } }}}}}}}        }       }
 }      }       }
 }}}  }     }
   }   }
   } }
 }}  }           }
   }}  }          }   
}