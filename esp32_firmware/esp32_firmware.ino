#include <ESP32Servo.h>

// ─── 가변저항 ADC 핀 (S1~S5) ─────────────────────────────────────────
const int POT_PINS[5] = {34, 35, 36, 39, 32};

// ─── 조이스틱 핀 (S6 그리퍼) ─────────────────────────────────────────
#define JOY_PIN   33
#define JOY_HIGH  3200
#define JOY_LOW   1400

// ─── 서보 핀 (자율동작 모드) ──────────────────────────────────────────
const int SERVO_PINS[6] = {13, 12, 14, 27, 25, 26};

// ─── 동작 설정 ────────────────────────────────────────────────────────
const int ANGLE_INIT[6] = { 85, 94, 130,   4,  80,  72};
const int ANGLE_MIN[6]  = {  0,   0,   0,   0,   0,  20};
const int ANGLE_MAX[6]  = {180, 180, 180, 180, 180, 110};

const int ADC_LOW  = 200;
const int ADC_HIGH = 4075;

const unsigned long SERIAL_MS   = 100;
const int           LOOP_MS     = 10;
const unsigned long AUTO_TIMEOUT = 500;

// ─── 전역 변수 ────────────────────────────────────────────────────────
Servo         servos[6];
int           angles[6];
unsigned long lastSerialTime = 0;
unsigned long lastCmdTime    = 0;
bool          autoMode       = false;
String        cmdBuf         = "";

// ─── PC 명령 파싱: "a:90,90,90,90,90,80\n" ───────────────────────────
bool parseCommand(const String& line, int out[6]) {
    if (!line.startsWith("a:")) return false;
    String body = line.substring(2);
    int idx = 0, start = 0;
    for (int i = 0; i <= body.length() && idx < 6; i++) {
        if (i == body.length() || body[i] == ',') {
            out[idx++] = body.substring(start, i).toInt();
            start = i + 1;
        }
    }
    return idx == 6;
}

// ─── Setup ────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(300);

    for (int i = 0; i < 5; i++) pinMode(POT_PINS[i], INPUT);
    pinMode(JOY_PIN, INPUT);

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    for (int i = 0; i < 6; i++) {
        angles[i] = ANGLE_INIT[i];
        servos[i].setPeriodHertz(50);
        servos[i].attach(SERVO_PINS[i], 500, 2500);
        servos[i].write(angles[i]);
    }
    delay(1500);
    for (int i = 0; i < 5; i++) servos[i].detach();  // S1~S5만 detach (손으로 이동)

    Serial.println("[INIT] ready");
}

// ─── Loop ─────────────────────────────────────────────────────────────
void loop() {
    // ── PC 시리얼 명령 수신 (자율동작) ───────────────────────────────
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            cmdBuf.trim();
            int parsed[6];
            if (parseCommand(cmdBuf, parsed)) {
                if (!autoMode) {
                    for (int i = 0; i < 5; i++) {
                        servos[i].setPeriodHertz(50);
                        servos[i].attach(SERVO_PINS[i], 500, 2500);
                    }
                    autoMode = true;
                }
                for (int i = 0; i < 6; i++) {
                    angles[i] = constrain(parsed[i], ANGLE_MIN[i], ANGLE_MAX[i]);
                    servos[i].write(angles[i]);
                }
                lastCmdTime = millis();
            }
            cmdBuf = "";
        } else {
            cmdBuf += c;
        }
    }

    if (autoMode && (millis() - lastCmdTime > AUTO_TIMEOUT)) {
        autoMode = false;
    }

    // ── 가변저항 S1~S5 + 조이스틱 S6 (수집 모드) ─────────────────────
if (!autoMode) {
    for (int i = 0; i < 5; i++) {
        int raw = analogRead(POT_PINS[i]);
        int deg = map(raw, ADC_LOW, ADC_HIGH, 0, 180);
        angles[i] = constrain(deg, ANGLE_MIN[i], ANGLE_MAX[i]);
    }

    int joy = analogRead(JOY_PIN);
    if      (joy > JOY_HIGH) angles[5] = constrain(angles[5] + 1, ANGLE_MIN[5], ANGLE_MAX[5]);
    else if (joy < JOY_LOW)  angles[5] = constrain(angles[5] - 1, ANGLE_MIN[5], ANGLE_MAX[5]);
    servos[5].write(angles[5]);
}

    // ── 10Hz 각도 전송 ────────────────────────────────────────────────
    unsigned long now = millis();
    if (now - lastSerialTime >= SERIAL_MS) {
        lastSerialTime = now;
        Serial.printf("s1:%d,s2:%d,s3:%d,s4:%d,s5:%d,s6:%d\n",
                      angles[0], angles[1], angles[2],
                      angles[3], angles[4], angles[5]);
    }

    delay(LOOP_MS);
}
