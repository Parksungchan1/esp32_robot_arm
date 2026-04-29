#include <ESP32Servo.h>

// ─── 서보 핀 ──────────────────────────────────────────────────────────
const int SERVO_PINS[6] = {13, 12, 14, 27, 25, 26};

// ─── 조이스틱 ADC 핀 ─────────────────────────────────────────────────
const int JOY_PINS[6] = {34, 35, 36, 39, 32, 33};

// ─── 동작 설정 ────────────────────────────────────────────────────────
const int ANGLE_INIT[6] = { 77, 136, 180,   0,  90,  69};
const int ANGLE_MIN[6]  = {  0,   0,   0,   0,   0,  40};
const int ANGLE_MAX[6]  = {180, 180, 180, 180, 180, 110};
const int ANGLE_STEP    = 1;

const int ADC_HIGH = 2700;
const int ADC_LOW  = 1400;

const unsigned long SERIAL_MS  = 100;   // 각도 전송 주기 (10Hz)
const int           LOOP_MS    = 10;    // 루프 주기 (~100Hz)
const unsigned long AUTO_TIMEOUT = 500; // 이 ms 동안 PC 명령 없으면 조이스틱으로 복귀

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
    int idx = 0;
    int start = 0;
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
    delay(200);

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

    Serial.println("[INIT] ready");
}

// ─── Loop ─────────────────────────────────────────────────────────────
void loop() {
    // 시리얼 수신 처리
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            cmdBuf.trim();
            int parsed[6];
            if (parseCommand(cmdBuf, parsed)) {
                for (int i = 0; i < 6; i++) {
                    angles[i] = constrain(parsed[i], ANGLE_MIN[i], ANGLE_MAX[i]);
                    servos[i].write(angles[i]);
                }
                lastCmdTime = millis();
                autoMode = true;
            }
            cmdBuf = "";
        } else {
            cmdBuf += c;
        }
    }

    // AUTO_TIMEOUT 동안 명령 없으면 조이스틱 모드로 복귀
    if (autoMode && (millis() - lastCmdTime > AUTO_TIMEOUT)) {
        autoMode = false;
    }

    // 조이스틱 모드
    if (!autoMode) {
        for (int i = 0; i < 6; i++) {
            int val = analogRead(JOY_PINS[i]);
            if (val > ADC_HIGH) {
                angles[i] = constrain(angles[i] + ANGLE_STEP, ANGLE_MIN[i], ANGLE_MAX[i]);
            } else if (val < ADC_LOW) {
                angles[i] = constrain(angles[i] - ANGLE_STEP, ANGLE_MIN[i], ANGLE_MAX[i]);
            }
            servos[i].write(angles[i]);
        }
    }

    // 100ms마다 PC로 현재 각도 전송
    unsigned long now = millis();
    if (now - lastSerialTime >= SERIAL_MS) {
        lastSerialTime = now;
        Serial.printf("s1:%d,s2:%d,s3:%d,s4:%d,s5:%d,s6:%d\n",
                      angles[0], angles[1], angles[2],
                      angles[3], angles[4], angles[5]);
    }

    delay(LOOP_MS);
}
