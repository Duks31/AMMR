// cika_base_esp32/src/main.cpp
// ─────────────────────────────────────────────────────────────────────────────
// Drive ESP32 firmware for cika AMR
//
// Subscribes : /cika/drive/wheel_vel_cmd  std_msgs/Float32MultiArray
//              [vel_lf, vel_lb, vel_rf, vel_rb]  rad/s
//
// Publishes  : /cika/odom  nav_msgs/Odometry
//              Pose + twist computed from encoder ticks
//
// Transport  : Serial UART at 115200 baud  (micro-ROS agent on RPi/laptop)
//
// ─────────────────────────────────────────────────────────────────────────────
// STATUS FLAGS — search for TODO before flashing on real hardware:
//
//   TODO_PINS   : Set BTS7960 and encoder GPIO numbers to match your wiring
//   TODO_CPR    : Confirm encoder counts-per-revolution for your motor variant
//   TODO_ISR    : Uncomment the encoder ISRs and attachInterrupt calls
//   TODO_PWM    : Tune the rad/s → PWM scaling against actual motor speed
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
// Error handling — same pattern as your test sketch
// ─────────────────────────────────────────────────────────────────────────────
#define RCCHECK(fn)           \
    {                         \
        rcl_ret_t rc = (fn);  \
        if (rc != RCL_RET_OK) \
        {                     \
            error_loop();     \
        }                     \
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
        delay(100);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Robot geometry — must match controllers.yaml and the URDF <param> tags
// ─────────────────────────────────────────────────────────────────────────────
static constexpr float WHEEL_SEPARATION = 0.363f; // metres
static constexpr float WHEEL_RADIUS = 0.0885f;    // metres

// ─────────────────────────────────────────────────────────────────────────────
// TODO_CPR: Encoder resolution
// For the 32-31ZY 100kg planetary motor with hall encoder:
//   Hall CPR on motor shaft : ~11 pulses/rev (verify with your datasheet)
//   Typical gearbox ratio   : 30:1 (check label on your motor — may be different)
//   Effective CPR at wheel  : 11 × 30 = 330
// Update this value once you have confirmed the gearbox ratio.
// ─────────────────────────────────────────────────────────────────────────────
static constexpr float ENCODER_CPR = 330.0f;
static constexpr float TICKS_TO_RAD = (2.0f * 3.14159265f) / ENCODER_CPR;

// ─────────────────────────────────────────────────────────────────────────────
// TODO_PINS: BTS7960 pin assignments — update to match your actual wiring
// Each side of a skid-steer has one BTS7960 driver:
//   EN    = enable pin (HIGH to enable driver)
//   RPWM  = forward PWM
//   LPWM  = reverse PWM
// ─────────────────────────────────────────────────────────────────────────────
#define LEFT_EN 25
#define LEFT_RPWM 26
#define LEFT_LPWM 27
#define RIGHT_EN 14
#define RIGHT_RPWM 12
#define RIGHT_LPWM 13

// TODO_PINS: Hall encoder input pins — use input-only GPIOs (34, 35, 36, 39)
// for noise immunity; connect encoder A channel only (single-channel counting)
#define ENCODER_LEFT_PIN 34
#define ENCODER_RIGHT_PIN 35

// PWM config (ESP32 LEDC peripheral)
#define PWM_FREQ_HZ 5000
#define PWM_RESOLUTION 8 // 8-bit: 0–255

// TODO_PWM: Maximum wheel speed in rad/s at full PWM (255).
// Measure this once motors are wired: command full speed and read encoder velocity.
// Placeholder: 13 RPM × (2π/60) × gearbox_output ≈ depends on your gearbox
static constexpr float MAX_WHEEL_SPEED_RAD_S = 5.0f; // TUNE THIS

// ─────────────────────────────────────────────────────────────────────────────
// micro-ROS objects
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

// Storage for the Float32MultiArray data buffer (4 floats)
float wheel_cmd_data[4] = {0.0f, 0.0f, 0.0f, 0.0f};

// ─────────────────────────────────────────────────────────────────────────────
// Encoder state (written by ISRs, read by odometry timer)
// volatile: prevents compiler from optimising away ISR-written values
// ─────────────────────────────────────────────────────────────────────────────
volatile int32_t enc_ticks_left = 0;
volatile int32_t enc_ticks_right = 0;
int32_t last_ticks_left = 0;
int32_t last_ticks_right = 0;

// Integrated pose
float pose_x = 0.0f;
float pose_y = 0.0f;
float heading = 0.0f;

// ─────────────────────────────────────────────────────────────────────────────
// TODO_ISR: Encoder interrupt service routines
// Uncomment these AND the attachInterrupt() calls in setup() once wired.
// For a single-channel (no direction) hall encoder, count every RISING edge.
// Direction is inferred from the sign of the velocity command.
// ─────────────────────────────────────────────────────────────────────────────
// void IRAM_ATTR isr_encoder_left()  { enc_ticks_left++;  }
// void IRAM_ATTR isr_encoder_right() { enc_ticks_right++; }

// ─────────────────────────────────────────────────────────────────────────────
// Motor helpers
// ─────────────────────────────────────────────────────────────────────────────
void set_motor_left(float vel_rad_s)
{
    // TODO_PWM: replace the linear scale with your calibrated mapping
    int pwm = (int)(fabsf(vel_rad_s) / MAX_WHEEL_SPEED_RAD_S * 255.0f);
    pwm = constrain(pwm, 0, 255);
    if (vel_rad_s >= 0.0f)
    {
        ledcWrite(0, pwm); // forward
        ledcWrite(1, 0);
    }
    else
    {
        ledcWrite(0, 0);
        ledcWrite(1, pwm); // reverse
    }
}

void set_motor_right(float vel_rad_s)
{
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
// Subscriber callback: /cika/drive/wheel_vel_cmd
// Receives [vel_lf, vel_lb, vel_rf, vel_rb] in rad/s from CikaDriveInterface
// ─────────────────────────────────────────────────────────────────────────────
void wheel_cmd_callback(const void *msg_in)
{
    const auto *cmd =
        reinterpret_cast<const std_msgs__msg__Float32MultiArray *>(msg_in);

    if (cmd->data.size < 4)
    {
        return;
    }

    // Average front and back — both wheels on each side get the same command
    float vel_left = (cmd->data.data[0] + cmd->data.data[1]) * 0.5f;
    float vel_right = (cmd->data.data[2] + cmd->data.data[3]) * 0.5f;

    set_motor_left(vel_left);
    set_motor_right(vel_right);
}

// ─────────────────────────────────────────────────────────────────────────────
// Timer callback: publishes /cika/odom at 10 Hz
// ─────────────────────────────────────────────────────────────────────────────
void odom_timer_callback(rcl_timer_t *timer, int64_t)
{
    if (timer == nullptr)
    {
        return;
    }

    // ── Read encoder deltas ──────────────────────────────────────────────────
    // TODO_ISR: Once encoders are wired and ISRs are enabled, replace the
    // stub zeros with the real encoder values below:
    //   int32_t cur_left  = enc_ticks_left;
    //   int32_t cur_right = enc_ticks_right;
    int32_t cur_left = 0;  // STUB — remove once encoders are wired
    int32_t cur_right = 0; // STUB — remove once encoders are wired

    int32_t d_left = cur_left - last_ticks_left;
    int32_t d_right = cur_right - last_ticks_right;
    last_ticks_left = cur_left;
    last_ticks_right = cur_right;

    // ── Skid-steer odometry ──────────────────────────────────────────────────
    float d_left_m = (float)d_left * TICKS_TO_RAD * WHEEL_RADIUS;
    float d_right_m = (float)d_right * TICKS_TO_RAD * WHEEL_RADIUS;

    float d_center = (d_left_m + d_right_m) * 0.5f;
    float d_theta = (d_right_m - d_left_m) / WHEEL_SEPARATION;

    pose_x += d_center * cosf(heading + d_theta * 0.5f);
    pose_y += d_center * sinf(heading + d_theta * 0.5f);
    heading += d_theta;

    static constexpr float DT = 0.1f; // 10 Hz

    // ── Fill odometry message ────────────────────────────────────────────────
    odom_msg.header.stamp.sec = 0;
    odom_msg.header.stamp.nanosec = 0;

    odom_msg.pose.pose.position.x = pose_x;
    odom_msg.pose.pose.position.y = pose_y;
    odom_msg.pose.pose.position.z = 0.0;

    // Yaw-only quaternion
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sinf(heading * 0.5f);
    odom_msg.pose.pose.orientation.w = cosf(heading * 0.5f);

    odom_msg.twist.twist.linear.x = d_center / DT;
    odom_msg.twist.twist.angular.z = d_theta / DT;

    RCSOFTCHECK(rcl_publish(&odom_pub, &odom_msg, nullptr));
}

// ─────────────────────────────────────────────────────────────────────────────
// setup
// ─────────────────────────────────────────────────────────────────────────────
void setup()
{
    // ── Serial / micro-ROS transport ────────────────────────────────────────
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    delay(2000);

    // ── BTS7960 PWM channels (ESP32 LEDC) ───────────────────────────────────
    ledcSetup(0, PWM_FREQ_HZ, PWM_RESOLUTION); // LEFT  forward
    ledcSetup(1, PWM_FREQ_HZ, PWM_RESOLUTION); // LEFT  reverse
    ledcSetup(2, PWM_FREQ_HZ, PWM_RESOLUTION); // RIGHT forward
    ledcSetup(3, PWM_FREQ_HZ, PWM_RESOLUTION); // RIGHT reverse

    ledcAttachPin(LEFT_RPWM, 0);
    ledcAttachPin(LEFT_LPWM, 1);
    ledcAttachPin(RIGHT_RPWM, 2);
    ledcAttachPin(RIGHT_LPWM, 3);

    pinMode(LEFT_EN, OUTPUT);
    digitalWrite(LEFT_EN, HIGH);
    pinMode(RIGHT_EN, OUTPUT);
    digitalWrite(RIGHT_EN, HIGH);

    // TODO_ISR: Uncomment once encoder wires are connected
    // pinMode(ENCODER_LEFT_PIN,  INPUT_PULLUP);
    // pinMode(ENCODER_RIGHT_PIN, INPUT_PULLUP);
    // attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN),  isr_encoder_left,  RISING);
    // attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), isr_encoder_right, RISING);

    // ── micro-ROS init — same pattern as your test sketch ────────────────────
    allocator = rcl_get_default_allocator();

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, 0));
    RCCHECK(rclc_support_init_with_options(&support, 0, nullptr, &init_options, &allocator));

    // Node name "cika_drive_esp32" in namespace "cika"
    // → appears as /cika/cika_drive_esp32 in ros2 node list
    RCCHECK(rclc_node_init_default(&node, "cika_drive_esp32", "cika", &support));

    // ── Subscriber: wheel velocity commands ─────────────────────────────────
    wheel_cmd_msg.data.data = wheel_cmd_data;
    wheel_cmd_msg.data.size = 4;
    wheel_cmd_msg.data.capacity = 4;

    RCCHECK(rclc_subscription_init_default(
        &wheel_cmd_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/cika/drive/wheel_vel_cmd"));

    // ── Publisher: odometry ─────────────────────────────────────────────────
    static char frame_odom[] = "odom";
    static char frame_base[] = "base_link";

    odom_msg.header.frame_id.data = frame_odom;
    odom_msg.header.frame_id.size = strlen(frame_odom);
    odom_msg.header.frame_id.capacity = strlen(frame_odom) + 1;

    odom_msg.child_frame_id.data = frame_base;
    odom_msg.child_frame_id.size = strlen(frame_base);
    odom_msg.child_frame_id.capacity = strlen(frame_base) + 1;

    RCCHECK(rclc_publisher_init_default(
        &odom_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/cika/odom"));

    // ── Timer: odometry at 10 Hz ─────────────────────────────────────────────
    RCCHECK(rclc_timer_init_default(
        &odom_timer,
        &support,
        RCL_MS_TO_NS(100),
        odom_timer_callback));

    // ── Executor: 2 handles (1 subscriber + 1 timer) ─────────────────────────
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &wheel_cmd_sub, &wheel_cmd_msg,
        &wheel_cmd_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &odom_timer));
}

// ─────────────────────────────────────────────────────────────────────────────
// loop — same pattern as your test sketch
// ─────────────────────────────────────────────────────────────────────────────
void loop()
{
    delay(10);
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}