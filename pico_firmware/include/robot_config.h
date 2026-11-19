#pragma once
// Pin map = spec section 3.1. Items marked TUNE must be measured on the real robot.

// ---- Phase switches (spec: Phase 1 = emulation, Phase 2 = hardware commissioning) ----
#define PHASE2_CLOSED_LOOP       0   // 0: open-loop duty = target/max (Phase 1). 1: discrete PID on encoder ticks
#define PHASE2_STALL_PROTECTION  0   // 0 in simulation (no encoder feedback -> would false-trip)
#define PID_USE_FEEDFORWARD      0   // spec PID is pure PID; set 1 if you want ff + PID

// ---- Pins ----
#define PIN_UART_TX        0
#define PIN_UART_RX        1
#define PIN_I2C_SDA        4          // I2C0, MPU-6050 (external 4.7k pull-ups)
#define PIN_I2C_SCL        5
#define PIN_L_PWM          6          // TB6612 PWMA
#define PIN_L_IN1          7          // AIN1
#define PIN_L_IN2          8          // AIN2
#define PIN_R_PWM          9          // PWMB
#define PIN_R_IN1          10         // BIN1
#define PIN_R_IN2          11         // BIN2
#define PIN_STBY           12
#define PIN_ENC_L_A        14         // B = 15, PIO0
#define PIN_ENC_R_A        16         // B = 17, PIO1
#define PIN_SONAR_TRIG     18         // shared trigger for both HC-SR04
#define PIN_SONAR_L_ECHO   19         // via 1k/2k divider!
#define PIN_SONAR_R_ECHO   20         // via 1k/2k divider!
#define PIN_BATT_ADC       26         // via 10k/2.7k divider

// ---- Geometry / motor ----
#define WHEELBASE_CM          24.0f
#define WHEEL_DIAMETER_CM     6.5f      // TUNE: not in the spec. 100 cm/s needs >= ~10.7 cm wheels at 178 RPM
#define ENC_TICKS_PER_REV     1320.0f   // 11 PPR * 30 * 4
#define MOTOR_MAX_RPM         178.0f
#define MAX_WHEEL_SPEED_CMS   (3.14159265f * WHEEL_DIAMETER_CM * MOTOR_MAX_RPM / 60.0f)
#define ENC_L_INVERT          0
#define ENC_R_INVERT          0
#define MOTOR_L_INVERT        0
#define MOTOR_R_INVERT        0

// ---- Timing ----
#define PWM_FREQ_HZ           10000
#define SLEW_FULL_SCALE_MS    150       // 100% / 150 ms = 6.67% per 10 ms tick
#define WATCHDOG_CHECK_MS     5
#define MOTOR_TICK_MS         10
#define SONAR_WINDOW_MS       30
#define PID_PERIOD_MS         50        // 20 Hz
#define SAFETY_PERIOD_MS      100       // stall check, battery, telemetry (10 Hz)

// ---- PID (ticks per 50 ms -> duty). Spec: start Ki = Kd = 0, raise Kp first ----
#define PID_KP                0.004f    // TUNE
#define PID_KI                0.0f      // TUNE
#define PID_KD                0.0f      // TUNE
#define PID_I_CLAMP           0.25f     // anti-windup: +-25% of max PWM

// ---- Sonar override (spec 3.3.4) ----
#define OBSTACLE_STOP_CM      20

// ---- Stall protection (spec 3.3.2) ----
#define STALL_DUTY_MIN        0.35f
#define STALL_TICKS_MAX       3         // |delta ticks| < 3 per 100 ms
#define STALL_CHECKS          2

// ---- Battery (spec 3.3.1): raw ADC codes after 64-sample average ----
#define BATT_CODE_EMPTY       2470      // 9.6 V
#define BATT_CODE_FULL        3240      // 12.6 V
