#pragma once

/*
 * ESP-IDF configuration for the SCARA firmware.
 * Motion commands are locked until the physical wiring has been checked.
 */
#define HARDWARE_WIRING_VERIFIED 0

/* Inherited scale from the existing firmware. Verify CPR semantics experimentally. */
#define MOTOR_ENCODER_CPR       64.0f
#define GEARBOX_RATIO           50.0f
#define BELT_RATIO              (100.0f / 20.0f)
#define COUNTS_PER_JOINT_REV    (MOTOR_ENCODER_CPR * GEARBOX_RATIO * BELT_RATIO)
#define DEG_PER_COUNT           (360.0f / COUNTS_PER_JOINT_REV)

/* 0 degrees = center of travel; positive = right limit switch. */
#define J1_ENCODER_SIGN         (+1)
#define J2_ENCODER_SIGN         (+1)
#define J1_MOTOR_SIGN           (+1)
#define J2_MOTOR_SIGN           (+1)

/* MC33926: INV direction, D2 PWM. */
#define J1_DIR_GPIO             26
#define J1_PWM_GPIO             25
#define J2_DIR_GPIO             33
#define J2_PWM_GPIO             32

/* Encoder channels. GPIO 34/35/36/39 are input-only and have no internal pull-ups. */
#define J1_ENC_A_GPIO           35
#define J1_ENC_B_GPIO           34
#define J2_ENC_A_GPIO           39
#define J2_ENC_B_GPIO           36

/* NO switches from GPIO to GND; active LOW with pull-ups. */
#define J1_LIMIT_RIGHT_GPIO     27
#define J1_LIMIT_LEFT_GPIO      13
#define J2_LIMIT_RIGHT_GPIO     19
#define J2_LIMIT_LEFT_GPIO      18

/* Measured by hand in the original notes; verify before use. */
#define J1_SWITCH_RIGHT_DEG     95.6f
#define J2_SWITCH_RIGHT_DEG     72.2f
#define J1_SOFT_LIMIT_DEG       88.0f
#define J2_SOFT_LIMIT_DEG       67.0f

/* Fixed-rate controller and velocity filter. */
#define CONTROL_PERIOD_MS       5
#define SPEED_FILTER_ALPHA      0.30f
#define POSITION_DEADBAND_DEG   0.15f

/* Cascade control starter gains: tune on the real mechanism, not assumed final values. */
#define KP_POSITION_DEFAULT     2.0f
#define KP_VELOCITY_DEFAULT     0.008f
#define KI_VELOCITY_DEFAULT     0.025f
#define MAX_SPEED_DEFAULT       25.0f
#define MAX_DUTY_DEFAULT        0.55f

#define HOME_DUTY               0.55f
#define HOME_BACKOFF_DEG        8.0f
#define HOME_TIMEOUT_MS         30000
#define HOME_STALL_WINDOW_MS    400
#define HOME_STALL_MIN_COUNTS   8

#define OPEN_LOOP_TEST_MAX      0.70f
#define OPEN_LOOP_TEST_MS       200

/* Local Wi-Fi access point. Change the sample password before motion is enabled. */
#define WIFI_AP_SSID            "SCARA-Control"
#define WIFI_AP_PASSWORD        "cambia_esta_clave"
#define TCP_COMMAND_PORT        3333

#define PWM_FREQUENCY_HZ        20000
#define PWM_DUTY_BITS           LEDC_TIMER_10_BIT
#define PWM_DUTY_MAX            ((1U << 10) - 1U)
