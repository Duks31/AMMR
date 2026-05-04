// ─────────────────────────────────────────────────────────────────────────────
// drive_esp32_main.cpp — Final Production Firmware for CIKA AMR
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>
#include <nav_msgs/msg/odometry.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This firmware requires Arduino framework with serial transport.
#endif

// ─────────────────────────────────────────────────────────────────────────────
// Error Handling & Safety
// ─────────────────────────────────────────────────────────────────────────────
#define RCCHECK(fn)           \
    {                         \
        rcl_ret_t rc = (fn);  \
        if (rc != RCL_RET_OK) \
            error_loop();     \
    }
#define RCSOFTCHECK(fn)      \
    {                        \
        rcl_ret_t rc = (fn); \
        (void)rc;            \
    }

void error_loop()
{
    while (1)
    {
        digitalWrite(2, !digitalRead(2)); // Blink onboard LED for error
        delay(100);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Robot Geometry (Must match URDF)
// ─────────────────────────────────────────────────────────────────────────────
static constexpr float WHEEL_SEPARATION = 0.363f;
static constexpr float WHEEL_RADIUS = 0.0885f;
static constexpr float ENCODER_CPR = 330.0f; // Update per motor label
static constexpr float TICKS_TO_RAD = (2.0f * 3.14159265f) / ENCODER_CPR;

// ─────────────────────────────────────────────────────────────────────────────
// Hardware Pins (BTS7960 & Encoders)
// ─────────────────────────────────────────────────────────────────────────────
#define LEFT_RPWM 27
#define LEFT_LPWM 14
#define LEFT_REN 23
#define LEFT_LEN 26

#define RIGHT_RPWM 32
#define RIGHT_LPWM 33
#define RIGHT_REN 25
#define RIGHT_LEN 21

#define ENCODER_LEFT_PIN 22
#define ENCODER_RIGHT_PIN 4

#define PWM_FREQ_HZ 5000
#define PWM_RESOLUTION 8
static constexpr float MAX_WHEEL_SPEED_RAD_S = 5.0f;

// ─────────────────────────────────────────────────────────────────────────────
// Global State
// ─────────────────────────────────────────────────────────────────────────────
volatile int32_t enc_ticks_left = 0;
volatile int32_t enc_ticks_right = 0;
float current_vel_left = 0.0f;
float current_vel_right = 0.0f;
unsigned long last_cmd_time = 0;

// Odometry state
int32_t last_ticks_left = 0;
int32_t last_ticks_right = 0;
float pose_x = 0.0f;
float pose_y = 0.0f;
float heading = 0.0f;

// ─────────────────────────────────────────────────────────────────────────────
// ISRs — Direction inferred from velocity command
// ─────────────────────────────────────────────────────────────────────────────
void IRAM_ATTR isr_encoder_left() { (current_vel_left >= 0) ? enc_ticks_left++ : enc_ticks_left--; }
void IRAM_ATTR isr_encoder_right() { (current_vel_right >= 0) ? enc_ticks_right++ : enc_ticks_right--; }

// ─────────────────────────────────────────────────────────────────────────────
// Micro-ROS Objects
// ─────────────────────────────────────────────────────────────────────────────
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

rcl_subscription_t wheel_cmd_sub;
rcl_publisher_t odom_pub;
rcl_timer_t odom_timer;

std_msgs__msg__Float32MultiArray wheel_cmd_msg;
nav_msgs__msg__Odometry odom_msg;
float wheel_cmd_data[4];

// ─────────────────────────────────────────────────────────────────────────────
// Motor Control
// ─────────────────────────────────────────────────────────────────────────────
void set_motor_left(float vel_rad_s)
{
    const float DEADZONE = 0.15f;
    if (fabsf(vel_rad_s) < DEADZONE)
        vel_rad_s = 0.0f;
    int pwm = (int)(fabsf(vel_rad_s) / MAX_WHEEL_SPEED_RAD_S * 255.0f);
    pwm = constrain(pwm, 0, 255);
    if (vel_rad_s >= 0.0f)
    {
        ledcWrite(0, pwm);
        ledcWrite(1, 0);
    }
    else
    {
        ledcWrite(0, 0);
        ledcWrite(1, pwm);
    }
}

void set_motor_right(float vel_rad_s)
{
    const float DEADZONE = 0.15f;
    if (fabsf(vel_rad_s) < DEADZONE)
        vel_rad_s = 0.0f;
    int pwm = (int)(fabsf(vel_rad_s) / MAX_WHEEL_SPEED_RAD_S * 255.0f);
    pwm = constrain(pwm, 0, 255);
    if (vel_rad_s >= 0.0f)
    {
        ledcWrite(2, pwm);
        ledcWrite(3, 0);
    }
    else
    {
        ledcWrite(2, 0);
        ledcWrite(3, pwm);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Callbacks
// ─────────────────────────────────────────────────────────────────────────────
void wheel_cmd_callback(const void *msg_in)
{
    const auto *cmd = reinterpret_cast<const std_msgs__msg__Float32MultiArray *>(msg_in);
    if (cmd->data.size < 4)
        return;

    current_vel_left = (cmd->data.data[0] + cmd->data.data[1]) * 0.5f;
    current_vel_right = (cmd->data.data[2] + cmd->data.data[3]) * 0.5f;

    set_motor_left(current_vel_left);
    set_motor_right(current_vel_right);
    last_cmd_time = millis();
}

void odom_timer_callback(rcl_timer_t *timer, int64_t)
{
    if (timer == nullptr)
        return;

    int32_t cur_left = enc_ticks_left;
    int32_t cur_right = enc_ticks_right;

    int32_t d_left = cur_left - last_ticks_left;
    int32_t d_right = cur_right - last_ticks_right;
    last_ticks_left = cur_left;
    last_ticks_right = cur_right;

    float d_left_m = (float)d_left * TICKS_TO_RAD * WHEEL_RADIUS;
    float d_right_m = (float)d_right * TICKS_TO_RAD * WHEEL_RADIUS;

    float d_center = (d_left_m + d_right_m) * 0.5f;
    float d_theta = (d_right_m - d_left_m) / WHEEL_SEPARATION;

    pose_x += d_center * cosf(heading + d_theta * 0.5f);
    pose_y += d_center * sinf(heading + d_theta * 0.5f);
    heading += d_theta;

    odom_msg.pose.pose.position.x = pose_x;
    odom_msg.pose.pose.position.y = pose_y;
    odom_msg.pose.pose.orientation.z = sinf(heading * 0.5f);
    odom_msg.pose.pose.orientation.w = cosf(heading * 0.5f);

    odom_msg.twist.twist.linear.x = d_center / 0.1f;
    odom_msg.twist.twist.angular.z = d_theta / 0.1f;

    RCSOFTCHECK(rcl_publish(&odom_pub, &odom_msg, nullptr));
}

// ─────────────────────────────────────────────────────────────────────────────
// Initialization
// ─────────────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    delay(2000);

    // Motor PWM & Enable
    ledcSetup(0, PWM_FREQ_HZ, PWM_RESOLUTION);
    ledcSetup(1, PWM_FREQ_HZ, PWM_RESOLUTION);
    ledcSetup(2, PWM_FREQ_HZ, PWM_RESOLUTION);
    ledcSetup(3, PWM_FREQ_HZ, PWM_RESOLUTION);
    ledcAttachPin(LEFT_RPWM, 0);
    ledcAttachPin(LEFT_LPWM, 1);
    ledcAttachPin(RIGHT_RPWM, 2);
    ledcAttachPin(RIGHT_LPWM, 3);

    pinMode(LEFT_REN, OUTPUT);
    pinMode(LEFT_LEN, OUTPUT);
    digitalWrite(LEFT_REN, HIGH);
    digitalWrite(LEFT_LEN, HIGH);
    pinMode(RIGHT_REN, OUTPUT);
    pinMode(RIGHT_LEN, OUTPUT);
    digitalWrite(RIGHT_REN, HIGH);
    digitalWrite(RIGHT_LEN, HIGH);

    // Encoders
    pinMode(ENCODER_LEFT_PIN, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), isr_encoder_left, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), isr_encoder_right, RISING);

    // ROS Init (Domain 0)
    allocator = rcl_get_default_allocator();
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, 0));
    RCCHECK(rclc_support_init_with_options(&support, 0, nullptr, &init_options, &allocator));

    RCCHECK(rclc_node_init_default(&node, "cika_drive_esp32", "cika", &support));

    // Comm objects
    wheel_cmd_msg.data.data = wheel_cmd_data;
    wheel_cmd_msg.data.size = 4;
    RCCHECK(rclc_subscription_init_default(&wheel_cmd_sub, &node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/cika/drive/wheel_vel_cmd"));

    static char f_odom[] = "odom";
    static char f_base[] = "base_link";
    odom_msg.header.frame_id.data = f_odom;
    odom_msg.header.frame_id.size = strlen(f_odom);
    odom_msg.child_frame_id.data = f_base;
    odom_msg.child_frame_id.size = strlen(f_base);

    RCCHECK(rclc_publisher_init_default(&odom_pub, &node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/cika/odom"));

    RCCHECK(rclc_timer_init_default(&odom_timer, &support, RCL_MS_TO_NS(100), odom_timer_callback));

    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &wheel_cmd_sub, &wheel_cmd_msg, &wheel_cmd_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &odom_timer));
}

void loop()
{
    // Safety Watchdog
    if (millis() - last_cmd_time > 500)
    {
        set_motor_left(0.0f);
        set_motor_right(0.0f);
    }
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
    delay(10);
}