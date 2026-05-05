#include <Arduino.h>

// ── Motor PWM Pins ────────────────────────────────────────────────────────────
#define LEFT_RPWM 27
#define LEFT_LPWM 14
#define LEFT_REN 23
#define LEFT_LEN 26
#define RIGHT_RPWM 32
#define RIGHT_LPWM 33
#define RIGHT_REN 25
#define RIGHT_LEN 21

// ── Encoder Pins (Phase A / Phase B) ──────────────────────────────────────────
#define ENC_LF_A 4
#define ENC_LF_B 5
#define ENC_LB_A 13
#define ENC_LB_B 15
#define ENC_RF_A 18
#define ENC_RF_B 19
#define ENC_RB_A 22
#define ENC_RB_B 16

// ── Kinematics & Limits ───────────────────────────────────────────────────────
static constexpr float MAX_RAD_S = 5.0f;
static constexpr int MIN_PWM = 80;
// 11 PPR * 596 Gear Ratio = 6556 ticks/rev. (2*PI / 6556)
static constexpr float TICKS_TO_RAD = 0.0009584f;

// ── Volatile Odometry Counters (Must be volatile for interrupts) ──────────────
volatile long ticks_lf = 0;
volatile long ticks_lb = 0;
volatile long ticks_rf = 0;
volatile long ticks_rb = 0;

unsigned long last_telem_time = 0;

// ── Interrupt Service Routines (IRAM_ATTR keeps them fast in RAM) ─────────────
// Note: If a wheel reports negative velocity when driving forward, swap the ++ and --
// Change the left side to subtract on HIGH and add on LOW
void IRAM_ATTR isr_lf() { if (digitalRead(ENC_LF_B)) ticks_lf = ticks_lf - 1; else ticks_lf = ticks_lf + 1; }
void IRAM_ATTR isr_lb() { if (digitalRead(ENC_LB_B)) ticks_lb = ticks_lb - 1; else ticks_lb = ticks_lb + 1; }

// Leave the right side alone for now until you test it on the new pins
void IRAM_ATTR isr_rf() { if (digitalRead(ENC_RF_B)) ticks_rf = ticks_rf + 1; else ticks_rf = ticks_rf - 1; }
void IRAM_ATTR isr_rb() { if (digitalRead(ENC_RB_B)) ticks_rb = ticks_rb + 1; else ticks_rb = ticks_rb - 1; }


// ── Motor Control Helpers ─────────────────────────────────────────────────────
static void drive(int pwm_l, bool fwd_l, int pwm_r, bool fwd_r)
{
    ledcWrite(LEFT_RPWM, fwd_l ? pwm_l : 0);
    ledcWrite(LEFT_LPWM, fwd_l ? 0 : pwm_l);
    ledcWrite(RIGHT_RPWM, fwd_r ? pwm_r : 0);
    ledcWrite(RIGHT_LPWM, fwd_r ? 0 : pwm_r);
}

static int vel_to_pwm(float vel_rad_s)
{
    int pwm = (int)(fabsf(vel_rad_s) / MAX_RAD_S * 255.0f);
    pwm = constrain(pwm, 0, 255);
    if (pwm > 0 && pwm < MIN_PWM)
        pwm = MIN_PWM;
    return pwm;
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);

    // Motor EN Pins
    pinMode(LEFT_REN, OUTPUT);
    digitalWrite(LEFT_REN, HIGH);
    pinMode(LEFT_LEN, OUTPUT);
    digitalWrite(LEFT_LEN, HIGH);
    pinMode(RIGHT_REN, OUTPUT);
    digitalWrite(RIGHT_REN, HIGH);
    pinMode(RIGHT_LEN, OUTPUT);
    digitalWrite(RIGHT_LEN, HIGH);

    // PWM Setup
    ledcAttach(LEFT_RPWM, 5000, 8);
    ledcAttach(LEFT_LPWM, 5000, 8);
    ledcAttach(RIGHT_RPWM, 5000, 8);
    ledcAttach(RIGHT_LPWM, 5000, 8);
    drive(0, true, 0, true);

    // Encoder Pins
    pinMode(ENC_LF_A, INPUT_PULLUP);
    pinMode(ENC_LF_B, INPUT_PULLUP);
    pinMode(ENC_LB_A, INPUT_PULLUP);
    pinMode(ENC_LB_B, INPUT_PULLUP);
    // 34-39 do not have internal pullups. They are set as INPUT natively.
    pinMode(ENC_RF_A, INPUT_PULLUP);
    pinMode(ENC_RF_B, INPUT_PULLUP);
    pinMode(ENC_RB_A, INPUT_PULLUP);
    pinMode(ENC_RB_B, INPUT_PULLUP);

    // Attach Hardware Interrupts to Phase A
    attachInterrupt(digitalPinToInterrupt(ENC_LF_A), isr_lf, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_LB_A), isr_lb, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_RF_A), isr_rf, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_RB_A), isr_rb, RISING);
}

// ── Main Loop ─────────────────────────────────────────────────────────────────
void loop()
{
    // 1. Read Commands from Raspberry Pi
    if (Serial.available() > 0)
    {
        String incoming = Serial.readStringUntil('\n');
        incoming.trim();

        if (incoming.startsWith("<") && incoming.endsWith(">"))
        {
            incoming = incoming.substring(1, incoming.length() - 1);
            float cmds[4];
            int commaIndex = 0;
            for (int i = 0; i < 3; i++)
            {
                commaIndex = incoming.indexOf(',');
                cmds[i] = incoming.substring(0, commaIndex).toFloat();
                incoming = incoming.substring(commaIndex + 1);
            }
            cmds[3] = incoming.toFloat();

            int pwm_l = vel_to_pwm(cmds[0]);
            int pwm_r = vel_to_pwm(cmds[2]);
            drive(pwm_l, cmds[0] >= 0.0f, pwm_r, cmds[2] >= 0.0f);
        }
    }

    // 2. Send Odometry to Raspberry Pi (50Hz / 20ms)
    if (millis() - last_telem_time >= 20)
    {
        last_telem_time = millis();

        // Atomically grab ticks to prevent reading during an interrupt fire
        noInterrupts();
        long lf = ticks_lf, lb = ticks_lb, rf = ticks_rf, rb = ticks_rb;
        interrupts();

        float pos_lf = lf * TICKS_TO_RAD;
        float pos_lb = lb * TICKS_TO_RAD;
        float pos_rf = rf * TICKS_TO_RAD;
        float pos_rb = rb * TICKS_TO_RAD;

        Serial.printf("E:%.3f,%.3f,%.3f,%.3f\n", pos_lf, pos_lb, pos_rf, pos_rb);
    }
}