#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <AccelStepper.h>
#include <EEPROM.h>  // Lightweight library for saving IP

// --- PIN DEFINITIONS ---
#define ONE_WIRE_BUS    A0
#define BUTTON_PIN      A1
#define ENCODER_A       A2
#define ENCODER_B       A3

#define PIN_COMMON_MIN  11 // Z- Z+
#define PIN_COMMON_MAX  9 // X- X+

// WARNING: Pin 4 is often used by Ethernet Shield for SD Card CS.
// If your shield has an SD slot, ensure it doesn't conflict.
#define M1_STEP_PIN     4
#define M1_DIR_PIN      7
#define M2_STEP_PIN     12
#define M2_DIR_PIN      13
#define EN_PIN          8

#define RUN_SPEED         1000
#define ACCEL             7000

// --- CONSTANTS ---
const unsigned long DEBOUNCE_DELAY = 300;
const int ENCODER_TURN_STEP = 100; // how many motor steps per one click
const int BACKOFF_STEPS = 100;
const long MAX_POSITION_LIMIT = 10000;
const long SAFETY_RUN_LIMIT = 20000;
const static char TRAILING_CHARS[] = "      ";

LiquidCrystal_I2C lcd(0x27, 16, 2);

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float temp1 = 0.0, temp2 = 0.0, temp3 = 0.0;

// MOTORS
AccelStepper motor1(AccelStepper::DRIVER, M1_STEP_PIN, M1_DIR_PIN);
AccelStepper motor2(AccelStepper::DRIVER, M2_STEP_PIN, M2_DIR_PIN);

// ETHERNET
byte mac[] = { 0x90, 0xA2, 0xDA, 0x10, 0xE2, 0x24 };
IPAddress staticIP(192, 168, 88, 20);
EthernetServer server(80);

// --- STATE VARIABLES ---
// Simplified State Machine
enum UIState { SCREEN_DASHBOARD, SCREEN_CONTROL_M1, SCREEN_CONTROL_M2 };
UIState currentScreen = SCREEN_DASHBOARD;

unsigned long lastButtonPress = 0;
int lastEncoderA = LOW;

bool motorsEnabled = false;
bool isBackingOff = false;

unsigned long lastTempUpdate = 0;
const long TEMP_UPDATE_INTERVAL = 5000;

// Heartbeat variables
unsigned long lastHeartbeat = 0;
char heartbeatChar = '*';

// --- FUNCTION PROTOTYPES ---
void handleNetwork();
void readSensors();
void handleInput();
void updateLCD();
void manageMotors();
void triggerBackoff(AccelStepper &motor, int direction);
void enableMotors(bool enable);
void loadIP(void);
void parseAndSaveIP(char* buffer);

void setup() {
    // Serial removed to save space

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);

    pinMode(PIN_COMMON_MIN, INPUT_PULLUP);
    pinMode(PIN_COMMON_MAX, INPUT_PULLUP);

    pinMode(EN_PIN, OUTPUT);
    enableMotors(false);

    lcd.init();
    lcd.backlight();
    lcd.setCursor(6,0);
    lcd.print(F("(ZK)"));
    lcd.setCursor(5,1);
    lcd.print(F("welcome"));
    delay(1000);

    sensors.begin();

    motor1.setMaxSpeed(RUN_SPEED); motor1.setAcceleration(ACCEL);
    motor2.setMaxSpeed(RUN_SPEED); motor2.setAcceleration(ACCEL);

    // Load IP from EEPROM
    loadIP();
    Ethernet.begin(mac, staticIP);
    server.begin();

    // 5. SHOW IP ON BOOT
    // It is critical to show the user the IP so they know where to connect
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("IP Address:"));
    lcd.setCursor(0,1);
    lcd.print(Ethernet.localIP());
    delay(4000); // Show IP for 3 seconds before going to dashboard

    lcd.clear();
}

void loop() {
    readSensors();
    handleInput();
    handleNetwork();
    manageMotors();
    updateLCD();
}

// --- EEPROM LOGIC ---

void loadIP() {
    // Read 4 bytes from EEPROM addresses 0,1,2,3
    byte ip0 = EEPROM.read(0);
    byte ip1 = EEPROM.read(1);
    byte ip2 = EEPROM.read(2);
    byte ip3 = EEPROM.read(3);

    // Basic validity check: if first byte is 0 or 255, assume empty/invalid
    if (ip0 == 0 || ip0 == 255) {
        // Fallback Default
        staticIP = IPAddress(192, 168, 88, 20);
    } else {
        staticIP = IPAddress(ip0, ip1, ip2, ip3);
    }
}

void parseAndSaveIP(char* buffer) {
    // Expected format: GET /setip?ip=192.168.1.55
    char* ipStart = strstr(buffer, "ip=");
    if (!ipStart) return;

    ipStart += 3; // Move past "ip="

    int bytes[4] = {0, 0, 0, 0};
    int byteIdx = 0;

    // Simple manual parsing to avoid String library
    // Scan until space or end of line
    while (*ipStart != ' ' && *ipStart != '\0' && byteIdx < 4) {
        if (*ipStart == '.') {
            byteIdx++;
        } else if (*ipStart >= '0' && *ipStart <= '9') {
            bytes[byteIdx] = (bytes[byteIdx] * 10) + (*ipStart - '0');
        }
        ipStart++;
    }

    // Save to EEPROM
    EEPROM.write(0, bytes[0]);
    EEPROM.write(1, bytes[1]);
    EEPROM.write(2, bytes[2]);
    EEPROM.write(3, bytes[3]);
}

void enableMotors(bool enable) {
    digitalWrite(EN_PIN, enable ? LOW : HIGH);
    motorsEnabled = enable;
}

void manageMotors() {
    // Auto-enable logic
    if ((motor1.distanceToGo() != 0 || motor2.distanceToGo() != 0) && !motorsEnabled) {
        enableMotors(true);
    } else if (motor1.distanceToGo() == 0 && motor2.distanceToGo() == 0 && motorsEnabled) {
        enableMotors(false);
    }

    bool minHit = (digitalRead(PIN_COMMON_MIN) == LOW);
    bool maxHit = (digitalRead(PIN_COMMON_MAX) == LOW);

    // Motor 1 Logic
    if (motor1.distanceToGo() != 0) {
        motor1.run();
        if (minHit && motor1.speed() < 0 && !isBackingOff) {
            motor1.stop();
            motor1.setCurrentPosition(0);
            triggerBackoff(motor1, 1);
        }
        else if (maxHit && motor1.speed() > 0 && !isBackingOff) {
            motor1.stop();
            motor1.setCurrentPosition(MAX_POSITION_LIMIT);
            triggerBackoff(motor1, -1);
        }
    }
    // Motor 2 Logic
    else if (motor2.distanceToGo() != 0) {
        motor2.run();
        if (minHit && motor2.speed() < 0 && !isBackingOff) {
            motor2.stop();
            motor2.setCurrentPosition(0);
            triggerBackoff(motor2, 1);
        }
        else if (maxHit && motor2.speed() > 0 && !isBackingOff) {
            motor2.stop();
            motor2.setCurrentPosition(MAX_POSITION_LIMIT);
            triggerBackoff(motor2, -1);
        }
    }
}

void triggerBackoff(AccelStepper &motor, int direction) {
    isBackingOff = true;
    long target = motor.currentPosition() + (direction * BACKOFF_STEPS);
    motor.moveTo(target);
    while (motor.distanceToGo() != 0) {
        motor.run();
    }
    isBackingOff = false;
}

void handleInput() {
    // 1. Encoder (Moves motors directly if screen is selected)
    int currentA = digitalRead(ENCODER_A);
    if (currentA != lastEncoderA && currentA == LOW) {
        int currentB = digitalRead(ENCODER_B);
        int direction = (currentB != currentA) ? 1 : -1;

        if (currentScreen == SCREEN_CONTROL_M1) {
            motor1.move(direction * ENCODER_TURN_STEP);
        }
        else if (currentScreen == SCREEN_CONTROL_M2) {
            motor2.move(direction * ENCODER_TURN_STEP);
        }
    }
    lastEncoderA = currentA;

    // 2. Button (Cycles Screens)
    if (digitalRead(BUTTON_PIN) == LOW) {
        if (millis() - lastButtonPress > DEBOUNCE_DELAY) {
            lastButtonPress = millis();

            // Cycle Logic: Dashboard -> M1 -> M2 -> Dashboard
            if (currentScreen == SCREEN_DASHBOARD) {
                currentScreen = SCREEN_CONTROL_M1;
            } else if (currentScreen == SCREEN_CONTROL_M1) {
                currentScreen = SCREEN_CONTROL_M2;
            } else {
                currentScreen = SCREEN_DASHBOARD;
            }

            lcd.clear();
        }
    }
}

void updateLCD() {
    // Toggle Heartbeat every 1000ms
    if (millis() - lastHeartbeat > 1000) {
        lastHeartbeat = millis();
        if (heartbeatChar == '*') heartbeatChar = '+';
        else heartbeatChar = '*';
    }

    static unsigned long lastLcdUpdate = 0;
    if (millis() - lastLcdUpdate < 200) return;
    lastLcdUpdate = millis();

    if (currentScreen == SCREEN_DASHBOARD) {
        lcd.setCursor(0, 0); lcd.print(F("1: ")); lcd.print((int)temp1); lcd.print(TRAILING_CHARS);
        lcd.setCursor(9, 0); lcd.print(F("2: ")); lcd.print((int)temp2); lcd.print(TRAILING_CHARS);
        lcd.setCursor(0, 1); lcd.print(F("room: ")); lcd.print((int)temp3); lcd.print(TRAILING_CHARS);
    }
    else if (currentScreen == SCREEN_CONTROL_M1) {
        lcd.setCursor(0, 0);
        lcd.print(F("Floor 1"));
        lcd.setCursor(0, 1);
        lcd.print(F("Pos: ")); lcd.print(motor1.currentPosition());
        lcd.print(TRAILING_CHARS); // clear trailing chars
    }
    else if (currentScreen == SCREEN_CONTROL_M2) {
        lcd.setCursor(0, 0);
        lcd.print(F("Floor 2"));
        lcd.setCursor(0, 1);
        lcd.print(F("Pos: ")); lcd.print(motor2.currentPosition());
        lcd.print(TRAILING_CHARS); // clear trailing chars
    }

    // Always draw heartbeat at 15,1 (Bottom Right)
    lcd.setCursor(15, 1);
    lcd.print(heartbeatChar);
}

void readSensors() {
    if (motorsEnabled) {
        return;
    }
    if (millis() - lastTempUpdate > TEMP_UPDATE_INTERVAL) {
        lastTempUpdate = millis();
        sensors.requestTemperatures();
        temp1 = sensors.getTempCByIndex(0);
        temp2 = sensors.getTempCByIndex(1);
        temp3 = sensors.getTempCByIndex(2);
    }
}

void handleNetwork() {
    EthernetClient client = server.available();
    if (client) {
        boolean currentLineIsBlank = true;
        char buffer[100];
        int bufIdx = 0;

        while (client.connected()) {
            if (client.available()) {
                char c = client.read();
                if (bufIdx < 99 && c != '\r' && c != '\n') {
                    buffer[bufIdx++] = c;
                    buffer[bufIdx] = '\0';
                }

                if (c == '\n' && currentLineIsBlank) {
                    client.println(F("HTTP/1.1 200 OK"));
                    client.println(F("Content-Type: application/json"));
                    client.println(F("Connection: close"));
                    client.println();

                    if (strstr(buffer, "GET /status") != 0) {
                        client.print(F("{"));
                        client.print(F("\"fl1\":")); client.print(temp1); client.print(F(","));
                        client.print(F("\"fl2\":")); client.print(temp2); client.print(F(","));
                        client.print(F("\"room\":")); client.print(temp3); client.print(F(","));
                        client.print(F("\"m1\":")); client.print(motor1.currentPosition()); client.print(F(","));
                        client.print(F("\"m2\":")); client.print(motor2.currentPosition());
                        client.print(F("}"));
                    }
                    else if (strstr(buffer, "GET /setip") != 0) {
                        // GET /setip?ip=192.168.1.55
                        parseAndSaveIP(buffer);
                        client.print(F("{\"status\":\"saved_reboot\"}"));
                    }
                    else if (strstr(buffer, "GET /set") != 0) {
                        char* mPtr = strstr(buffer, "motor=");
                        char* pPtr = strstr(buffer, "pos=");

                        if (mPtr && pPtr) {
                            char motorNum = *(mPtr + 6);
                            char pVal = *(pPtr + 4);
                            char pCheck = *(pPtr + 6);
                            char* vPtr = pPtr + 4; // Start of value string

                            AccelStepper* targetM = (motorNum == '1') ? &motor1 : &motor2;

                            // 1. Check Keywords (min/max)
                            if (pVal == 'm' && pCheck == 'n') { // min
                                targetM->moveTo(-SAFETY_RUN_LIMIT);
                            } else if (pVal == 'm' && pCheck == 'x') { // max
                                targetM->moveTo(SAFETY_RUN_LIMIT);
                            }
                            // 2. Parse Number for Relative Move
                            else {
                                long steps = 0;
                                bool neg = false;

                                // Check sign
                                if (*vPtr == '-') { neg = true; vPtr++; }
                                else if (*vPtr == '+') { vPtr++; } // Skip explicit +

                                // Parse digits manually to save space (no atoi)
                                while (*vPtr >= '0' && *vPtr <= '9') {
                                    steps = (steps * 10) + (*vPtr - '0');
                                    vPtr++;
                                }

                                if (neg) steps = -steps;
                                if (steps != 0) targetM->move(steps); // Relative move
                                else targetM->stop();
                            }
                            client.print(F("{\"status\":\"ok\"}"));
                        } else {
                            client.print(F("{\"error\":\"bad_req\"}"));
                        }
                    }
                    else {
                        client.print(F("/status\n/set?motor=1&pos=+200\n/set?motor=1&pos=min\n/setip?ip=192.168.1.55\n"));
                    }
                    break;
                }
                if (c == '\n') {
                    currentLineIsBlank = true;
                } else if (c != '\r') {
                    currentLineIsBlank = false;
                }
            }
        }
        delay(1);
        client.stop();
    }
}