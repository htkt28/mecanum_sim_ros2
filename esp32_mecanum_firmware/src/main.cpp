#include <Arduino.h>
#include <ESP32Encoder.h>
#include <Wire.h>
#include <MPU6050_light.h>

// ============================================================================
// FIRMWARE ESP32 ROBOT MECANUM TỰ HÀNH - ĐỒNG BỘ 100% VỚI URDF ROS 2 JAZZY
// Thứ tự 4 bánh phần cứng:
//   - Bánh 1 (idx 0): Trái Trước (FL - Front Left)
//   - Bánh 2 (idx 1): Phải Trước (FR - Front Right)
//   - Bánh 3 (idx 2): Phải Sau   (RR - Rear Right)
//   - Bánh 4 (idx 3): Trái Sau   (RL - Rear Left)
// ============================================================================

MPU6050 mpu(Wire);

#define NUM_WHEELS 4

enum WheelIndex {
  IDX_FL = 0, // Bánh 1: Trái Trước
  IDX_FR = 1, // Bánh 2: Phải Trước
  IDX_RR = 2, // Bánh 3: Phải Sau
  IDX_RL = 3  // Bánh 4: Trái Sau
};

// 1. CẤU HÌNH CHÂN PHẦN CỨNG THEO THỨ TỰ [0:FL, 1:FR, 2:RR, 3:RL]
const int encPinA[NUM_WHEELS] = {32, 26, 16, 19};
const int encPinB[NUM_WHEELS] = {33, 25, 17, 18};

const int rpwmPin[NUM_WHEELS] = {2, 13, 22, 4};
const int lpwmPin[NUM_WHEELS] = {15, 14, 23, 5};

const int MOTOR_DIR[NUM_WHEELS] = {1, -1, 1, -1};

const int I2C_SDA_PIN = 21;
const int I2C_SCL_PIN = 27;

const int PWM_FREQ = 20000;
const int PWM_RESOLUTION = 8;

// 2. THÔNG SỐ CƠ KHÍ THEO ĐÚNG FILE URDF (robot_core.xacro)
ESP32Encoder encoders[NUM_WHEELS];

const float GEAR_RATIO = 30.0f;
const float PULSES_PER_REV = 11.0f * 4.0f * GEAR_RATIO; // 1320 xung/vòng bánh
const float WHEEL_RADIUS = 0.0485f;                     // Bán kính bánh trong URDF: 0.0485m
const float WHEEL_DIAMETER = 2.0f * WHEEL_RADIUS;       // Đường kính bánh: 0.097m (97mm)
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * PI;
// Khoảng cách tâm bánh trong URDF: wheel_offset_x (0.125) + wheel_offset_y (0.150) = 0.275m
const float LX_PLUS_LY = 0.125f + 0.150f;
const float GRAVITY_MSS = 9.80665f;

// 3. THÔNG SỐ PID TỐC ĐỘ DÀI BÁNH XE (m/s)
float Kp = 220.0f, Ki = 850.0f, Kd = 8.0f;

int64_t prev_count[NUM_WHEELS] = {0, 0, 0, 0};
float velocity[NUM_WHEELS]     = {0.0f, 0.0f, 0.0f, 0.0f};
float target_vel[NUM_WHEELS]   = {0.0f, 0.0f, 0.0f, 0.0f};

float integral[NUM_WHEELS]     = {0.0f, 0.0f, 0.0f, 0.0f};
float prev_error[NUM_WHEELS]   = {0.0f, 0.0f, 0.0f, 0.0f};

bool mpu_ok = false;
unsigned long prev_pid_time = 0;
unsigned long last_cmd_time = 0;
bool ros_cmd_active = false;

// Chu kỳ vòng lặp PID & truyền Telemetry lên Raspberry Pi: 25ms (40 Hz)
const unsigned long PID_INTERVAL_MS = 25;
// Watchdog an toàn: tự phanh nếu mất gói lệnh từ Raspberry Pi quá 500ms
const unsigned long CMD_WATCHDOG_MS = 500;

char serial_buf[96];
uint8_t serial_idx = 0;

// ============================================================================
// HÀM ĐIỀU KHIỂN ĐỘNG CƠ BTS7960 & PID
// ============================================================================
void writePWM(int wheel_idx, int r_val, int l_val) {
  #if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(rpwmPin[wheel_idx], r_val);
    ledcWrite(lpwmPin[wheel_idx], l_val);
  #else
    ledcWrite(wheel_idx * 2, r_val);
    ledcWrite(wheel_idx * 2 + 1, l_val);
  #endif
}

void setMotorPWM(int wheel_idx, int speed_pwm) {
  speed_pwm = constrain(speed_pwm * MOTOR_DIR[wheel_idx], -255, 255);
  if (speed_pwm > 0) writePWM(wheel_idx, speed_pwm, 0);
  else if (speed_pwm < 0) writePWM(wheel_idx, 0, -speed_pwm);
  else writePWM(wheel_idx, 0, 0);
}

int computePID(int i, float setpoint, float measured, float dt_sec) {
  if (fabsf(setpoint) < 0.001f) {
    integral[i] = 0.0f;
    prev_error[i] = 0.0f;
    return 0;
  }
  if ((setpoint > 0.0f && integral[i] < 0.0f) || (setpoint < 0.0f && integral[i] > 0.0f)) {
    integral[i] = 0.0f;
  }
  float error = setpoint - measured;
  integral[i] = constrain(integral[i] + error * dt_sec, -255.0f / Ki, 255.0f / Ki);
  float derivative = (error - prev_error[i]) / dt_sec;
  prev_error[i] = error;
  return constrain((int)roundf((Kp * error) + (Ki * integral[i]) + (Kd * derivative)), -255, 255);
}

void stopRobot() {
  for (int i = 0; i < NUM_WHEELS; i++) {
    target_vel[i] = 0.0f;
    integral[i] = 0.0f;
    prev_error[i] = 0.0f;
    setMotorPWM(i, 0);
  }
}

// ============================================================================
// ĐỘNG HỌC NGƯỢC MECANUM CHUẨN ROS REP-103 (+vx Tiến, +vy Trái, +wz Xoay Trái CCW)
// Ánh xạ đúng vào [0:FL, 1:FR, 2:RR, 3:RL]
// ============================================================================
void applyBodyTwist(float vx, float vy, float wz) {
  if (fabsf(vx) < 0.001f && fabsf(vy) < 0.001f && fabsf(wz) < 0.001f) {
    stopRobot();
    return;
  }
  float rot = LX_PLUS_LY * wz;
  target_vel[IDX_FL] = vx - vy - rot; // Bánh 1: Trái Trước
  target_vel[IDX_FR] = vx + vy + rot; // Bánh 2: Phải Trước
  target_vel[IDX_RR] = vx - vy + rot; // Bánh 3: Phải Sau
  target_vel[IDX_RL] = vx + vy - rot; // Bánh 4: Trái Sau
}

// ============================================================================
// XỬ LÝ GÓI LỆNH TỪ RASPBERRY PI 5 QUA USB SERIAL
//   - "V,vx,vy,wz" : Đặt vận tốc thân xe (m/s, m/s, rad/s) từ /cmd_vel
//   - "R"          : Reset bộ đếm Encoder về 0
// ============================================================================
void processSerialLine(char *line) {
  if (line[0] == 'V' && line[1] == ',') {
    float vx = 0.0f, vy = 0.0f, wz = 0.0f;
    if (sscanf(line + 2, "%f,%f,%f", &vx, &vy, &wz) == 3) {
      applyBodyTwist(vx, vy, wz);
      last_cmd_time = millis();
      ros_cmd_active = true;
    }
  } else if (line[0] == 'R') {
    stopRobot();
    for (int i = 0; i < NUM_WHEELS; i++) {
      encoders[i].clearCount();
      prev_count[i] = 0;
      velocity[i] = 0.0f;
    }
    ros_cmd_active = false;
  }
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serial_idx > 0) {
        serial_buf[serial_idx] = '\0';
        processSerialLine(serial_buf);
        serial_idx = 0;
      }
    } else if (serial_idx < sizeof(serial_buf) - 1) {
      serial_buf[serial_idx++] = c;
    } else {
      serial_idx = 0;
    }
  }
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  for (int i = 0; i < NUM_WHEELS; i++) {
    encoders[i].attachFullQuad(encPinA[i], encPinB[i]);
    encoders[i].clearCount();
    #if ESP_ARDUINO_VERSION_MAJOR >= 3
      ledcAttach(rpwmPin[i], PWM_FREQ, PWM_RESOLUTION);
      ledcAttach(lpwmPin[i], PWM_FREQ, PWM_RESOLUTION);
    #else
      ledcSetup(i * 2, PWM_FREQ, PWM_RESOLUTION);
      ledcAttachPin(rpwmPin[i], i * 2);
      ledcSetup(i * 2 + 1, PWM_FREQ, PWM_RESOLUTION);
      ledcAttachPin(lpwmPin[i], i * 2 + 1);
    #endif
  }
  stopRobot();
  // Chờ 1.5s cho thân xe tắt hẳn dao động nếu chân Strapping (GPIO 2, 5, 14, 15) vừa nháy lúc Boot
  delay(1500);

  // Khởi tạo cảm biến IMU MPU6050 trên chân SDA=21, SCL=27
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);
  if (mpu.begin() == 0) {
    mpu.calcOffsets(true, true);
    mpu_ok = true;
    Serial.println("#INFO,MPU6050_READY");
  } else {
    mpu_ok = false;
    Serial.println("#WARN,MPU6050_FAIL");
  }

  for (int i = 0; i < NUM_WHEELS; i++) {
    encoders[i].clearCount();
    prev_count[i] = 0;
  }
  prev_pid_time = millis();
  last_cmd_time = millis();
}

// ============================================================================
// LOOP CHÍNH
// ============================================================================
void loop() {
  if (mpu_ok) {
    mpu.update();
  }

  readSerialCommands();

  unsigned long now = millis();

  // Bảo vệ Watchdog: quá 500ms không nhận gói V,... mới từ ROS 2 thì tự phanh dừng xe
  if (ros_cmd_active && (now - last_cmd_time > CMD_WATCHDOG_MS)) {
    stopRobot();
    ros_cmd_active = false;
  }

  unsigned long dt = now - prev_pid_time;
  if (dt >= PID_INTERVAL_MS) {
    float dt_sec = (float)dt / 1000.0f;
    prev_pid_time = now;

    float wheel_pos_rad[NUM_WHEELS];

    for (int i = 0; i < NUM_WHEELS; i++) {
      int64_t current_count = encoders[i].getCount();
      int64_t delta_count = current_count - prev_count[i];
      prev_count[i] = current_count;

      float raw_vel = (((float)delta_count / PULSES_PER_REV) * WHEEL_CIRCUMFERENCE) / dt_sec;
      velocity[i] = 0.7f * raw_vel + 0.3f * velocity[i];

      int pwm = computePID(i, target_vel[i], velocity[i], dt_sec);
      setMotorPWM(i, pwm);

      wheel_pos_rad[i] = ((float)current_count / PULSES_PER_REV) * TWO_PI;
    }

    // Động học thuận Mecanum từ [IDX_FL, IDX_FR, IDX_RR, IDX_RL]
    float v_fl = velocity[IDX_FL];
    float v_fr = velocity[IDX_FR];
    float v_rr = velocity[IDX_RR];
    float v_rl = velocity[IDX_RL];

    float vx_meas = 0.25f * (v_fl + v_fr + v_rl + v_rr);
    float vy_meas = 0.25f * (-v_fl + v_fr + v_rl - v_rr);
    float wz_meas = (-v_fl + v_fr - v_rl + v_rr) / (4.0f * LX_PLUS_LY);

    // Đọc dữ liệu IMU MPU6050 chuyển sang chuẩn đơn vị SI của ROS 2 (rad, rad/s, m/s^2)
    float yaw_rad = 0.0f, gz_rad_s = 0.0f;
    float ax_ms2 = 0.0f, ay_ms2 = 0.0f, az_ms2 = GRAVITY_MSS;
    if (mpu_ok) {
      yaw_rad  = mpu.getAngleZ() * DEG_TO_RAD;
      gz_rad_s = mpu.getGyroZ() * DEG_TO_RAD;
      ax_ms2   = mpu.getAccX() * GRAVITY_MSS;
      ay_ms2   = mpu.getAccY() * GRAVITY_MSS;
      az_ms2   = mpu.getAccZ() * GRAVITY_MSS;
    }

    // Gửi khung dữ liệu Telemetry lên Raspberry Pi 5 (40 Hz) theo thứ tự chuẩn:
    // D,vx,vy,wz,yaw,gz,ax,ay,az,pos_fl,pos_fr,pos_rl,pos_rr
    char out_buf[192];
    snprintf(
      out_buf, sizeof(out_buf),
      "D,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
      vx_meas, vy_meas, wz_meas,
      yaw_rad, gz_rad_s,
      ax_ms2, ay_ms2, az_ms2,
      wheel_pos_rad[IDX_FL], wheel_pos_rad[IDX_FR], wheel_pos_rad[IDX_RL], wheel_pos_rad[IDX_RR]
    );
    Serial.print(out_buf);
  }
}
