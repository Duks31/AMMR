#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <MPU6050.h>

// ── WiFi Credentials ─────────────────────────────────────────────────────────
const char* SSID     = "ncep";
const char* PASSWORD = "googlepixel";
const uint16_t TCP_PORT = 8888;

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
static constexpr float MAX_RAD_S    = 1.36f;
static constexpr int   MIN_PWM      = 30;
static constexpr float TICKS_TO_RAD = 0.0009584f;

// ── Calibration Offsets ───────────────────────────────────────────────────────
const float accel_x_offset =  0.0103f;
const float accel_y_offset = -0.0217f;
const float accel_z_offset =  0.5211f;
const float gyro_x_offset  = -0.0510f;
const float gyro_y_offset  = -0.0259f;
const float gyro_z_offset  =  0.0080f;

const float mag_x_offset =  -91.50f;
const float mag_y_offset =  181.00f;
const float mag_z_offset =  307.50f;
const float MAG_SCALE    =    0.92f;

// ── State ─────────────────────────────────────────────────────────────────────
volatile long ticks_lf = 0, ticks_lb = 0, ticks_rf = 0, ticks_rb = 0;
unsigned long last_enc_time = 0;
unsigned long last_imu_time = 0;

WiFiServer server(TCP_PORT);
WiFiClient client;
// String tcp_rx_buf;
char tcp_rx_buf[128]; // Fast static buffer
int rx_index = 0;

MPU6050 mpu;

// ── ISRs ──────────────────────────────────────────────────────────────────────
void IRAM_ATTR isr_lf() { if (digitalRead(ENC_LF_B)) ticks_lf++; else ticks_lf--; }
void IRAM_ATTR isr_lb() { if (digitalRead(ENC_LB_B)) ticks_lb++; else ticks_lb--; }
void IRAM_ATTR isr_rf() { if (digitalRead(ENC_RF_B)) ticks_rf--; else ticks_rf++; }
void IRAM_ATTR isr_rb() { if (digitalRead(ENC_RB_B)) ticks_rb--; else ticks_rb++; }

// ── Motor Helpers ─────────────────────────────────────────────────────────────
static void drive(int pwm_l, bool fwd_l, int pwm_r, bool fwd_r)
{
    ledcWrite(LEFT_RPWM,  fwd_l ? pwm_l : 0);
    ledcWrite(LEFT_LPWM,  fwd_l ? 0 : pwm_l);
    ledcWrite(RIGHT_RPWM, fwd_r ? pwm_r : 0);
    ledcWrite(RIGHT_LPWM, fwd_r ? 0 : pwm_r);
}

static int vel_to_pwm(float vel_rad_s)
{
    int pwm = (int)(fabsf(vel_rad_s) / MAX_RAD_S * 255.0f);
    pwm = constrain(pwm, 0, 255);
    if (pwm > 0 && pwm < MIN_PWM) pwm = MIN_PWM;
    return pwm;
}

// ── Mag ───────────────────────────────────────────────────────────────────────
void enableMPUBypass()
{
    Wire.beginTransmission(0x68);
    Wire.write(0x37);
    Wire.write(0x02);
    Wire.endTransmission();
}

bool readMag(float &mx, float &my, float &mz)
{
    Wire.beginTransmission(0x2C);
    Wire.write(0x01);
    Wire.endTransmission(false);
    Wire.requestFrom((uint16_t)0x2C, (uint8_t)6);
    if (Wire.available() < 6) return false;

    uint8_t x_lsb = Wire.read(), x_msb = Wire.read();
    uint8_t y_lsb = Wire.read(), y_msb = Wire.read();
    uint8_t z_lsb = Wire.read(), z_msb = Wire.read();

    mx = ((int16_t)((x_msb << 8) | x_lsb) - mag_x_offset) * MAG_SCALE;
    my = ((int16_t)((y_msb << 8) | y_lsb) - mag_y_offset) * MAG_SCALE;
    mz = ((int16_t)((z_msb << 8) | z_lsb) - mag_z_offset) * MAG_SCALE;
    return true;
}

// ── Parse and execute one command line ───────────────────────────────────────
void handle_command(const char* line)
{
    Serial.println(line);   

    if (strncmp(line, "<RESET>", 7) == 0) {
        noInterrupts();
        ticks_lf = ticks_lb = ticks_rf = ticks_rb = 0;
        interrupts();
    } 
    else if (line[0] == '<') {
        float lf, lb, rf, rb;
        // sscanf is significantly faster than String.toFloat()
        if (sscanf(line, "<%f,%f,%f,%f>", &lf, &lb, &rf, &rb) == 4) {
            drive(vel_to_pwm(lf), lf >= 0.0f,
                  vel_to_pwm(rf), rf >= 0.0f);
        }
    }
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);

    // Motors
    pinMode(LEFT_REN,  OUTPUT); digitalWrite(LEFT_REN,  HIGH);
    pinMode(LEFT_LEN,  OUTPUT); digitalWrite(LEFT_LEN,  HIGH);
    pinMode(RIGHT_REN, OUTPUT); digitalWrite(RIGHT_REN, HIGH);
    pinMode(RIGHT_LEN, OUTPUT); digitalWrite(RIGHT_LEN, HIGH);

    ledcAttach(LEFT_RPWM,  5000, 8);
    ledcAttach(LEFT_LPWM,  5000, 8);
    ledcAttach(RIGHT_RPWM, 5000, 8);
    ledcAttach(RIGHT_LPWM, 5000, 8);
    drive(0, true, 0, true);

    // Encoders
    pinMode(ENC_LF_A, INPUT_PULLUP); pinMode(ENC_LF_B, INPUT_PULLUP);
    pinMode(ENC_LB_A, INPUT_PULLUP); pinMode(ENC_LB_B, INPUT_PULLUP);
    pinMode(ENC_RF_A, INPUT_PULLUP); pinMode(ENC_RF_B, INPUT_PULLUP);
    pinMode(ENC_RB_A, INPUT_PULLUP); pinMode(ENC_RB_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENC_LF_A), isr_lf, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_LB_A), isr_lb, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_RF_A), isr_rf, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_RB_A), isr_rb, RISING);

    // IMU
    Wire.begin(21, 22);
    mpu.initialize();
    enableMPUBypass();
    Wire.beginTransmission(0x2C);
    Wire.write(0x0A);
    Wire.write(0xCF);
    Wire.endTransmission();
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);

    // WiFi
    Serial.printf("Connecting to %s\n", SSID);
    WiFi.begin(SSID, PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.printf("\nConnected! IP: %s\n", WiFi.localIP().toString().c_str());

    server.begin();
    server.setNoDelay(true);
    Serial.printf("TCP server on port %d\n", TCP_PORT);
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop()
{
    // Accept new client if none connected
    if (!client || !client.connected()) {
        client = server.accept();
        if (client) {
            Serial.println("Client connected");
            rx_index = 0; // Reset buffer
        }
    }

    // Read incoming commands efficiently
    if (client && client.connected() && client.available()) {
        while (client.available()) {
            char c = client.read();
            if (c == '\n') {
                tcp_rx_buf[rx_index] = '\0'; // Null-terminate the string
                if (rx_index > 0) handle_command(tcp_rx_buf);
                rx_index = 0; // Reset for the next packet
            } 
            else if (rx_index < sizeof(tcp_rx_buf) - 1) {
                // Ignore carriage returns, only add valid chars
                if (c != '\r') {
                    tcp_rx_buf[rx_index++] = c;
                }
            }
        }
    }

    unsigned long now = millis();

    // Encoder telemetry at 10 Hz
    if (now - last_enc_time >= 100) {
        last_enc_time = now;
        noInterrupts();
        long lf = ticks_lf, lb = ticks_lb, rf = ticks_rf, rb = ticks_rb;
        interrupts();

        if (client && client.connected()) {
            client.printf("E:%.3f,%.3f,%.3f,%.3f\n",
                          lf * TICKS_TO_RAD, lb * TICKS_TO_RAD,
                          rf * TICKS_TO_RAD, rb * TICKS_TO_RAD);
        }
    }

    // IMU telemetry at 5 Hz
    if (now - last_imu_time >= 200) {
        last_imu_time = now;

        int16_t ax_r, ay_r, az_r, gx_r, gy_r, gz_r;
        mpu.getMotion6(&ax_r, &ay_r, &az_r, &gx_r, &gy_r, &gz_r);

        const float A_SCALE = 9.80665f / 8192.0f;
        const float G_SCALE = (3.14159265f / 180.0f) / 65.5f;

        float ax = (ax_r * A_SCALE) - accel_x_offset;
        float ay = (ay_r * A_SCALE) - accel_y_offset;
        float az = (az_r * A_SCALE) - accel_z_offset;
        float gx = (gx_r * G_SCALE) - gyro_x_offset;
        float gy = (gy_r * G_SCALE) - gyro_y_offset;
        float gz = (gz_r * G_SCALE) - gyro_z_offset;

        float mx = 0, my = 0, mz = 0;
        bool mag_ok = readMag(mx, my, mz);

        if (client && client.connected()) {
            if (mag_ok)
                client.printf("I:%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                               ax, ay, az, gx, gy, gz, mx, my, mz);
            else
                client.printf("I:%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,0.0,0.0,0.0\n",
                               ax, ay, az, gx, gy, gz);
        }
    }
}