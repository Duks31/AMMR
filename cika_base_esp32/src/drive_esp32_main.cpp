#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h> // by Electronic Cats - install via Arduino Library Manager

// ── Motor PWM Pins ────────────────────────────────────────────────────────────
#define LEFT_RPWM 27
#define LEFT_LPWM 14
#define LEFT_REN 23
#define LEFT_LEN 26
#define RIGHT_RPWM 32
#define RIGHT_LPWM 33
#define RIGHT_REN 25
#define RIGHT_LEN 2

// ── Encoder Pins ──────────────────────────────────────────────────────────────
#define ENC_LF_A 4
#define ENC_LF_B 5
#define ENC_LB_A 13
#define ENC_LB_B 15
#define ENC_RF_A 18
#define ENC_RF_B 19
#define ENC_RB_A 34
#define ENC_RB_B 16

// ── Kinematics ────────────────────────────────────────────────────────────────
static constexpr float MAX_RAD_S = 1.36f; // Updated for 13rpm ~ 0.12m/s
static constexpr int MIN_PWM = 30;        // TODO: To be Tested
static constexpr float TICKS_TO_RAD = 0.0009584f;

// ── Calibration Offsets ───────────────────────────────────────────────────────
const float accel_x_offset = 0.0103f;
const float accel_y_offset = -0.0217f;
const float accel_z_offset = 0.5211f;
const float gyro_x_offset = -0.0510f;
const float gyro_y_offset = -0.0259f;
const float gyro_z_offset = 0.0080f;

// Mag calibration
const float mag_x_offset = -91.50f;
const float mag_y_offset = 181.00f;
const float mag_z_offset = 307.50f;
const float MAG_SCALE = 0.92f; // µT per LSB at ±1.3Ga

// ── Volatile Encoder Counters ─────────────────────────────────────────────────
volatile long ticks_lf = 0, ticks_lb = 0, ticks_rf = 0, ticks_rb = 0;

// ── Timing ────────────────────────────────────────────────────────────────────
unsigned long last_enc_time = 0; // 50Hz
unsigned long last_imu_time = 0; // 20Hz

// ── IMU ───────────────────────────────────────────────────────────────────────
MPU6050 mpu;

// ── ISRs ─────────────────────────────────────────────────────────────────────
void IRAM_ATTR isr_lf()
{
    if (digitalRead(ENC_LF_B))
        ticks_lf++;
    else
        ticks_lf--;
}
void IRAM_ATTR isr_lb()
{
    if (digitalRead(ENC_LB_B))
        ticks_lb++;
    else
        ticks_lb--;
}
void IRAM_ATTR isr_rf()
{
    if (digitalRead(ENC_RF_B))
        ticks_rf--;
    else
        ticks_rf++;
}
void IRAM_ATTR isr_rb()
{
    if (digitalRead(ENC_RB_B))
        ticks_rb--;
    else
        ticks_rb++;
}

// ── Motor Helpers ─────────────────────────────────────────────────────────────
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

void enableMPUBypass()
{
    // Write 0x02 to register 0x37 to enable I2C bypass
    Wire.beginTransmission(0x68);
    Wire.write(0x37);
    Wire.write(0x02);
    Wire.endTransmission();
}

bool readMag(float &mx, float &my, float &mz)
{
    Wire.beginTransmission(0x2C); // QMC5883P Address
    Wire.write(0x01);             // Start reading at X LSB
    Wire.endTransmission(false);
    Wire.requestFrom((uint16_t)0x2C, (uint8_t)6);

    if (Wire.available() < 6)
        return false;

    // QMC5883P byte order is X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB
    uint8_t x_lsb = Wire.read();
    uint8_t x_msb = Wire.read();
    uint8_t y_lsb = Wire.read();
    uint8_t y_msb = Wire.read();
    uint8_t z_lsb = Wire.read();
    uint8_t z_msb = Wire.read();

    int16_t rx = (x_msb << 8) | x_lsb;
    int16_t ry = (y_msb << 8) | y_lsb;
    int16_t rz = (z_msb << 8) | z_lsb;

    // Subtract raw offset FIRST, then scale to micro-Teslas (µT)
    mx = (rx - mag_x_offset) * MAG_SCALE;
    my = (ry - mag_y_offset) * MAG_SCALE;
    mz = (rz - mag_z_offset) * MAG_SCALE;

    return true;
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(460800);

    // Motor EN pins
    pinMode(LEFT_REN, OUTPUT);
    digitalWrite(LEFT_REN, HIGH);
    pinMode(LEFT_LEN, OUTPUT);
    digitalWrite(LEFT_LEN, HIGH);
    pinMode(RIGHT_REN, OUTPUT);
    digitalWrite(RIGHT_REN, HIGH);
    pinMode(RIGHT_LEN, OUTPUT);
    digitalWrite(RIGHT_LEN, HIGH);

    // PWM
    ledcAttach(LEFT_RPWM, 5000, 8);
    ledcAttach(LEFT_LPWM, 5000, 8);
    ledcAttach(RIGHT_RPWM, 5000, 8);
    ledcAttach(RIGHT_LPWM, 5000, 8);
    drive(0, true, 0, true);

    // Encoders
    pinMode(ENC_LF_A, INPUT_PULLUP);
    pinMode(ENC_LF_B, INPUT_PULLUP);
    pinMode(ENC_LB_A, INPUT_PULLUP);
    pinMode(ENC_LB_B, INPUT_PULLUP);
    pinMode(ENC_RF_A, INPUT_PULLUP);
    pinMode(ENC_RF_B, INPUT_PULLUP);
    pinMode(ENC_RB_A, INPUT_PULLUP);
    pinMode(ENC_RB_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENC_LF_A), isr_lf, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_LB_A), isr_lb, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_RF_A), isr_rf, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_RB_A), isr_rb, RISING);

    // IMU — I2C on GPIO 21/22
    Wire.begin(21, 22);
    mpu.initialize();
    enableMPUBypass();
    Wire.beginTransmission(0x2C);
    Wire.write(0x0A); // Control Register 1
    Wire.write(0xCF); // Continuous mode, 200Hz
    Wire.endTransmission();

    if (!mpu.testConnection())
    {
        Serial.println("ERR:MPU6050 not found");
    }
    // MPU6050 library uses raw int16 internally; we read raw and scale manually
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4); // ±4g  → 8192 LSB/g
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500); // ±500°/s → 65.5 LSB/°/s
}

// ── Main Loop ─────────────────────────────────────────────────────────────────
void loop()
{
    unsigned long now = millis();

    // 1. Read commands from Pi
    if (Serial.available() > 0)
    {
        String incoming = Serial.readStringUntil('\n');
        incoming.trim();
        if (incoming == "<RESET>")
        {
            noInterrupts();
            ticks_lf = ticks_lb = ticks_rf = ticks_rb = 0;
            interrupts();
        }
        else if (incoming.startsWith("<") && incoming.endsWith(">"))
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

    if (now - last_enc_time >= 100)
    {
        last_enc_time = now;
        noInterrupts();
        long lf = ticks_lf, lb = ticks_lb, rf = ticks_rf, rb = ticks_rb;
        interrupts();

        Serial.printf("E:%.3f,%.3f,%.3f,%.3f\n",
                      lf * TICKS_TO_RAD, lb * TICKS_TO_RAD,
                      rf * TICKS_TO_RAD, rb * TICKS_TO_RAD);
    }

    unsigned long now2 = millis();
    if (now2 - last_imu_time >= 200)
    {
        last_imu_time = now2;

        int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
        mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

        // Scale to SI units and apply calibration offsets
        // ±4g range  → 8192 LSB/g → multiply by 9.80665/8192
        const float A_SCALE = 9.80665f / 8192.0f;
        // ±500°/s range → 65.5 LSB/°/s → multiply by (PI/180)/65.5
        const float G_SCALE = (3.14159265f / 180.0f) / 65.5f;

        float ax = (ax_raw * A_SCALE) - accel_x_offset;
        float ay = (ay_raw * A_SCALE) - accel_y_offset;
        float az = (az_raw * A_SCALE) - accel_z_offset;
        float gx = (gx_raw * G_SCALE) - gyro_x_offset;
        float gy = (gy_raw * G_SCALE) - gyro_y_offset;
        float gz = (gz_raw * G_SCALE) - gyro_z_offset;

        float mx = 0, my = 0, mz = 0;
        bool mag_ok = readMag(mx, my, mz);

        if (mag_ok)
        {
            // Format: I:ax,ay,az,gx,gy,gz,mx,my,mz
            Serial.printf("I:%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                          ax, ay, az, gx, gy, gz, mx, my, mz);
        }
        else
        {
            // Send without mag so pipeline doesn't stall
            Serial.printf("I:%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,0.0,0.0,0.0\n",
                          ax, ay, az, gx, gy, gz);
        }
    }
}