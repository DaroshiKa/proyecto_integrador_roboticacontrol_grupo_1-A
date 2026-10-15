#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "lwip/inet.h"
#include "lwip/sockets.h"

#include "robot_config.h"

static const char *TAG = "scara";
static SemaphoreHandle_t g_state_mutex;
static portMUX_TYPE g_encoder_mux = portMUX_INITIALIZER_UNLOCKED;

typedef struct {
    int pin_a;
    int pin_b;
    volatile int32_t count;
    volatile uint8_t previous_ab;
} Encoder;

typedef enum {
    HOME_IDLE = 0,
    HOME_SEEK_RIGHT,
    HOME_BACKOFF
} HomeState;

typedef struct {
    Encoder encoder;
    int dir_gpio;
    int pwm_gpio;
    ledc_channel_t pwm_channel;
    int encoder_sign;
    int motor_sign;
    int limit_right_gpio;
    int limit_left_gpio;
    float switch_right_deg;
    float soft_limit_deg;

    bool enabled;
    bool homed;
    bool limit_right_active;
    bool limit_left_active;
    bool tripped;
    bool home_done;
    uint8_t home_fail;
    HomeState homing;
    uint32_t home_start_ms;
    uint32_t home_check_ms;
    int32_t home_check_count;

    float target_deg;
    float kp_position;
    float kp_velocity;
    float ki_velocity;
    float max_speed_deg_s;
    float max_duty;
    float velocity_integral;
    float position_deg;
    float velocity_deg_s;
    float duty;

    float open_loop_duty;
    uint32_t open_loop_until_ms;
    int32_t last_count;
} Joint;

static Joint g_joints[2];
static int g_selected_joint = 0;

/* Quadrature x4 transition table. */
static DRAM_ATTR const int8_t g_qem[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static float clamp_float(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static bool time_before(uint32_t current, uint32_t deadline)
{
    return (int32_t)(current - deadline) < 0;
}

static int32_t encoder_get_count(const Encoder *encoder)
{
    int32_t result;
    portENTER_CRITICAL(&g_encoder_mux);
    result = encoder->count;
    portEXIT_CRITICAL(&g_encoder_mux);
    return result;
}

static void encoder_set_count(Encoder *encoder, int32_t value)
{
    portENTER_CRITICAL(&g_encoder_mux);
    encoder->count = value;
    portEXIT_CRITICAL(&g_encoder_mux);
}

static void IRAM_ATTR encoder_isr(void *argument)
{
    Encoder *encoder = (Encoder *)argument;
    const uint8_t current_ab =
        (uint8_t)((gpio_get_level((gpio_num_t)encoder->pin_a) << 1) |
                  gpio_get_level((gpio_num_t)encoder->pin_b));

    portENTER_CRITICAL_ISR(&g_encoder_mux);
    const uint8_t index = (uint8_t)((encoder->previous_ab << 2) | current_ab);
    encoder->count += g_qem[index];
    encoder->previous_ab = current_ab;
    portEXIT_CRITICAL_ISR(&g_encoder_mux);
}

static void motor_write(Joint *joint, float command)
{
    float u = command * (float)joint->motor_sign;
    u = clamp_float(u, -1.0f, 1.0f);

    if (fabsf(u) > 0.0001f) {
        gpio_set_level((gpio_num_t)joint->dir_gpio, (u < 0.0f) ? 1 : 0);
    }

    uint32_t duty = (uint32_t)lroundf(fabsf(u) * (float)PWM_DUTY_MAX);
    if (duty > PWM_DUTY_MAX) duty = PWM_DUTY_MAX;
    (void)ledc_set_duty(LEDC_LOW_SPEED_MODE, joint->pwm_channel, duty);
    (void)ledc_update_duty(LEDC_LOW_SPEED_MODE, joint->pwm_channel);
    joint->duty = command;
}

static void reset_controller(Joint *joint)
{
    joint->velocity_integral = 0.0f;
    joint->open_loop_duty = 0.0f;
    joint->open_loop_until_ms = 0;
}

static void init_joint_data(void)
{
    g_joints[0] = (Joint) {
        .encoder = {.pin_a = J1_ENC_A_GPIO, .pin_b = J1_ENC_B_GPIO},
        .dir_gpio = J1_DIR_GPIO,
        .pwm_gpio = J1_PWM_GPIO,
        .pwm_channel = LEDC_CHANNEL_0,
        .encoder_sign = J1_ENCODER_SIGN,
        .motor_sign = J1_MOTOR_SIGN,
        .limit_right_gpio = J1_LIMIT_RIGHT_GPIO,
        .limit_left_gpio = J1_LIMIT_LEFT_GPIO,
        .switch_right_deg = J1_SWITCH_RIGHT_DEG,
        .soft_limit_deg = J1_SOFT_LIMIT_DEG,
        .kp_position = KP_POSITION_DEFAULT,
        .kp_velocity = KP_VELOCITY_DEFAULT,
        .ki_velocity = KI_VELOCITY_DEFAULT,
        .max_speed_deg_s = MAX_SPEED_DEFAULT,
        .max_duty = MAX_DUTY_DEFAULT
    };

    g_joints[1] = (Joint) {
        .encoder = {.pin_a = J2_ENC_A_GPIO, .pin_b = J2_ENC_B_GPIO},
        .dir_gpio = J2_DIR_GPIO,
        .pwm_gpio = J2_PWM_GPIO,
        .pwm_channel = LEDC_CHANNEL_1,
        .encoder_sign = J2_ENCODER_SIGN,
        .motor_sign = J2_MOTOR_SIGN,
        .limit_right_gpio = J2_LIMIT_RIGHT_GPIO,
        .limit_left_gpio = J2_LIMIT_LEFT_GPIO,
        .switch_right_deg = J2_SWITCH_RIGHT_DEG,
        .soft_limit_deg = J2_SOFT_LIMIT_DEG,
        .kp_position = KP_POSITION_DEFAULT,
        .kp_velocity = KP_VELOCITY_DEFAULT,
        .ki_velocity = KI_VELOCITY_DEFAULT,
        .max_speed_deg_s = MAX_SPEED_DEFAULT,
        .max_duty = MAX_DUTY_DEFAULT
    };
}

static void configure_hardware(void)
{
    gpio_config_t direction_pins = {
        .pin_bit_mask = (1ULL << J1_DIR_GPIO) | (1ULL << J2_DIR_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&direction_pins));
    gpio_set_level((gpio_num_t)J1_DIR_GPIO, 0);
    gpio_set_level((gpio_num_t)J2_DIR_GPIO, 0);

    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = PWM_DUTY_BITS,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    for (int i = 0; i < 2; ++i) {
        Joint *joint = &g_joints[i];
        ledc_channel_config_t channel_config = {
            .gpio_num = joint->pwm_gpio,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = joint->pwm_channel,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
    }

    gpio_config_t inputs = {
        .pin_bit_mask =
            (1ULL << J1_ENC_A_GPIO) | (1ULL << J1_ENC_B_GPIO) |
            (1ULL << J2_ENC_A_GPIO) | (1ULL << J2_ENC_B_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    ESP_ERROR_CHECK(gpio_config(&inputs));

    gpio_config_t limit_inputs = {
        .pin_bit_mask =
            (1ULL << J1_LIMIT_RIGHT_GPIO) | (1ULL << J1_LIMIT_LEFT_GPIO) |
            (1ULL << J2_LIMIT_RIGHT_GPIO) | (1ULL << J2_LIMIT_LEFT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&limit_inputs));

    for (int i = 0; i < 2; ++i) {
        Joint *joint = &g_joints[i];
        joint->encoder.previous_ab =
            (uint8_t)((gpio_get_level((gpio_num_t)joint->encoder.pin_a) << 1) |
                      gpio_get_level((gpio_num_t)joint->encoder.pin_b));
        joint->last_count = encoder_get_count(&joint->encoder);
    }

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    for (int i = 0; i < 2; ++i) {
        Joint *joint = &g_joints[i];
        ESP_ERROR_CHECK(gpio_isr_handler_add(
            (gpio_num_t)joint->encoder.pin_a, encoder_isr, &joint->encoder));
        ESP_ERROR_CHECK(gpio_isr_handler_add(
            (gpio_num_t)joint->encoder.pin_b, encoder_isr, &joint->encoder));
    }

    for (int i = 0; i < 2; ++i) {
        motor_write(&g_joints[i], 0.0f);
    }
}

static void fail_homing(Joint *joint, uint8_t reason)
{
    motor_write(joint, 0.0f);
    joint->homing = HOME_IDLE;
    joint->homed = false;
    joint->home_fail = reason;
    joint->velocity_integral = 0.0f;
}

static bool homing_step(Joint *joint, uint32_t now, int32_t count)
{
    if (joint->homing == HOME_IDLE) return false;

    if (joint->limit_left_active) {
        fail_homing(joint, 1);
        return true;
    }

    if ((uint32_t)(now - joint->home_start_ms) > HOME_TIMEOUT_MS) {
        fail_homing(joint, 2);
        return true;
    }

    if (joint->homing == HOME_SEEK_RIGHT && joint->limit_right_active) {
        motor_write(joint, 0.0f);
        const int32_t reference_count =
            (int32_t)lroundf((float)joint->encoder_sign *
                            joint->switch_right_deg / DEG_PER_COUNT);
        encoder_set_count(&joint->encoder, reference_count);
        joint->last_count = reference_count;
        joint->position_deg = joint->switch_right_deg;
        joint->velocity_deg_s = 0.0f;
        joint->homed = true;
        joint->homing = HOME_BACKOFF;
        joint->home_check_ms = now;
        joint->home_check_count = reference_count;
        joint->velocity_integral = 0.0f;
        return true;
    }

    if (joint->homing == HOME_BACKOFF &&
        joint->position_deg <= joint->switch_right_deg - HOME_BACKOFF_DEG &&
        !joint->limit_right_active) {
        motor_write(joint, 0.0f);
        joint->homing = HOME_IDLE;
        joint->home_done = true;
        joint->target_deg = joint->position_deg;
        joint->velocity_integral = 0.0f;
        return true;
    }

    if ((uint32_t)(now - joint->home_check_ms) >= HOME_STALL_WINDOW_MS) {
        if (abs((int)(count - joint->home_check_count)) < HOME_STALL_MIN_COUNTS) {
            fail_homing(joint, 3);
            return true;
        }
        joint->home_check_count = count;
        joint->home_check_ms = now;
    }

    motor_write(joint, (joint->homing == HOME_SEEK_RIGHT) ? HOME_DUTY : -HOME_DUTY);
    return true;
}

static void stop_all_locked(void)
{
    for (int i = 0; i < 2; ++i) {
        Joint *joint = &g_joints[i];
        joint->enabled = false;
        joint->homing = HOME_IDLE;
        reset_controller(joint);
        motor_write(joint, 0.0f);
    }
}

static void control_task(void *argument)
{
    (void)argument;
    TickType_t wake_time = xTaskGetTickCount();
    const float dt = (float)CONTROL_PERIOD_MS / 1000.0f;

    for (;;) {
        const uint32_t now = now_ms();
        bool finished_home[2] = {false, false};
        uint8_t failed_home[2] = {0, 0};

        if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            for (int i = 0; i < 2; ++i) {
                Joint *joint = &g_joints[i];
                const int32_t count = encoder_get_count(&joint->encoder);
                const int32_t delta_count = count - joint->last_count;
                joint->last_count = count;

                joint->position_deg =
                    (float)joint->encoder_sign * (float)count * DEG_PER_COUNT;
                const float raw_speed =
                    (float)joint->encoder_sign * (float)delta_count * DEG_PER_COUNT / dt;
                joint->velocity_deg_s +=
                    SPEED_FILTER_ALPHA * (raw_speed - joint->velocity_deg_s);

                joint->limit_right_active =
                    gpio_get_level((gpio_num_t)joint->limit_right_gpio) == 0;
                joint->limit_left_active =
                    gpio_get_level((gpio_num_t)joint->limit_left_gpio) == 0;

                const HomeState old_home_state = joint->homing;
                if (homing_step(joint, now, count)) {
                    finished_home[i] = (old_home_state != HOME_IDLE &&
                                       joint->homing == HOME_IDLE &&
                                       joint->home_done);
                    failed_home[i] = joint->home_fail;
                    joint->home_fail = 0;
                    continue;
                }

                if (joint->enabled &&
                    (joint->limit_right_active || joint->limit_left_active ||
                     fabsf(joint->position_deg) > joint->soft_limit_deg + 1.0f)) {
                    joint->enabled = false;
                    joint->tripped = true;
                    reset_controller(joint);
                    motor_write(joint, 0.0f);
                }

                if (!joint->enabled) {
                    const bool no_limit_active =
                        !joint->limit_right_active && !joint->limit_left_active;
                    const float test_duty =
                        (no_limit_active && time_before(now, joint->open_loop_until_ms))
                        ? joint->open_loop_duty : 0.0f;
                    motor_write(joint, test_duty);
                    if (test_duty == 0.0f) joint->open_loop_duty = 0.0f;
                    continue;
                }

                const float position_error = joint->target_deg - joint->position_deg;
                if (fabsf(position_error) <= POSITION_DEADBAND_DEG) {
                    joint->velocity_integral = 0.0f;
                    motor_write(joint, 0.0f);
                    continue;
                }

                /* Outer loop: P(position) -> requested angular velocity. */
                const float velocity_reference = clamp_float(
                    joint->kp_position * position_error,
                    -joint->max_speed_deg_s, joint->max_speed_deg_s);

                /* Inner loop: PI(velocity) -> signed motor duty. */
                const float velocity_error = velocity_reference - joint->velocity_deg_s;
                const float raw_duty =
                    joint->kp_velocity * velocity_error + joint->velocity_integral;
                const float duty = clamp_float(raw_duty, -joint->max_duty, joint->max_duty);

                /* Conditional integration prevents windup during saturation. */
                const bool saturated_in_error_direction =
                    (raw_duty > joint->max_duty && velocity_error > 0.0f) ||
                    (raw_duty < -joint->max_duty && velocity_error < 0.0f);
                if (!saturated_in_error_direction) {
                    joint->velocity_integral = clamp_float(
                        joint->velocity_integral +
                            joint->ki_velocity * velocity_error * dt,
                        -joint->max_duty, joint->max_duty);
                }

                motor_write(joint, duty);
            }
            xSemaphoreGive(g_state_mutex);
        }

        for (int i = 0; i < 2; ++i) {
            if (finished_home[i]) {
                ESP_LOGI(TAG, "J%d homing OK; position %.2f deg",
                         i + 1, g_joints[i].position_deg);
            }
            if (failed_home[i] != 0) {
                const char *reason = failed_home[i] == 1 ? "left limit activated" :
                                     failed_home[i] == 2 ? "timeout" :
                                     "not enough encoder movement";
                ESP_LOGE(TAG, "J%d homing aborted: %s", i + 1, reason);
            }
            if (g_joints[i].tripped) {
                ESP_LOGE(TAG, "J%d safety limit reached: motor disabled", i + 1);
                g_joints[i].tripped = false;
            }
        }

        vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

static void append_status(char *out, size_t out_size)
{
    Joint *joint = &g_joints[g_selected_joint];
    snprintf(out, out_size,
             "J%d en=%d homed=%d homing=%d target=%.2f pos=%.2f vel=%.2f deg/s "
             "duty=%.3f limR=%d limL=%d | Kp_pos=%.3f Kp_vel=%.5f Ki_vel=%.5f "
             "vmax=%.1f umax=%.2f",
             g_selected_joint + 1, joint->enabled ? 1 : 0, joint->homed ? 1 : 0,
             (int)joint->homing, joint->target_deg, joint->position_deg,
             joint->velocity_deg_s, joint->duty,
             joint->limit_right_active ? 1 : 0, joint->limit_left_active ? 1 : 0,
             joint->kp_position, joint->kp_velocity, joint->ki_velocity,
             joint->max_speed_deg_s, joint->max_duty);
}

static void command_execute(const char *line, char *out, size_t out_size)
{
    char command[24] = {0};
    char arg1[32] = {0};
    char arg2[32] = {0};
    const int fields = sscanf(line, "%23s %31s %31s", command, arg1, arg2);

    if (fields <= 0) {
        snprintf(out, out_size, "Empty command. Type 'help'.");
        return;
    }
    for (char *p = command; *p; ++p) *p = (char)tolower((unsigned char)*p);

    if (!strcmp(command, "help") || !strcmp(command, "?")) {
        snprintf(out, out_size,
            "Commands: j1 | j2 | home | on | off | stop | p <deg> | kp <value> | "
            "pi <kp> <ki> | vmax <deg/s> | u <0..0.35> | test <duty> | s\n"
            "Motion commands are locked until HARDWARE_WIRING_VERIFIED is set to 1.");
        return;
    }

    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        snprintf(out, out_size, "Busy; retry.");
        return;
    }

    Joint *joint = &g_joints[g_selected_joint];
    char *end = NULL;
    const float value1 = (fields >= 2) ? strtof(arg1, &end) : 0.0f;
    const bool arg1_is_number = (fields >= 2 && end != arg1 && *end == '\0');
    char *end2 = NULL;
    const float value2 = (fields >= 3) ? strtof(arg2, &end2) : 0.0f;
    const bool arg2_is_number = (fields >= 3 && end2 != arg2 && *end2 == '\0');

    if (!strcmp(command, "j1") || !strcmp(command, "j2")) {
        g_selected_joint = (command[1] == '1') ? 0 : 1;
        append_status(out, out_size);
    } else if (!strcmp(command, "s") || !strcmp(command, "status")) {
        append_status(out, out_size);
    } else if (!strcmp(command, "stop") || !strcmp(command, "x")) {
        stop_all_locked();
        snprintf(out, out_size, "STOP: both joints disabled.");
    } else if (!strcmp(command, "off")) {
        joint->enabled = false;
        joint->homing = HOME_IDLE;
        reset_controller(joint);
        motor_write(joint, 0.0f);
        snprintf(out, out_size, "J%d disabled.", g_selected_joint + 1);
    } else if (!strcmp(command, "on")) {
#if !HARDWARE_WIRING_VERIFIED
        snprintf(out, out_size, "Motion locked: verify wiring then set HARDWARE_WIRING_VERIFIED 1.");
#else
        if (!joint->homed) {
            snprintf(out, out_size, "Not homed. Run 'home' first.");
        } else if (joint->homing != HOME_IDLE) {
            snprintf(out, out_size, "Homing is still active.");
        } else if (joint->limit_right_active || joint->limit_left_active) {
            snprintf(out, out_size, "A limit switch is active. Release it before enabling.");
        } else {
            joint->target_deg = joint->position_deg;
            joint->velocity_integral = 0.0f;
            joint->enabled = true;
            snprintf(out, out_size, "J%d enabled; holding current position.",
                     g_selected_joint + 1);
        }
#endif
    } else if (!strcmp(command, "home") || !strcmp(command, "h")) {
#if !HARDWARE_WIRING_VERIFIED
        snprintf(out, out_size, "Motion locked: verify wiring then set HARDWARE_WIRING_VERIFIED 1.");
#else
        stop_all_locked();
        joint = &g_joints[g_selected_joint];
        joint->homed = false;
        joint->home_done = false;
        joint->home_fail = 0;
        joint->home_start_ms = now_ms();
        joint->home_check_ms = joint->home_start_ms;
        joint->home_check_count = encoder_get_count(&joint->encoder);
        joint->homing = HOME_SEEK_RIGHT;
        snprintf(out, out_size,
                 "Homing J%d toward RIGHT switch at duty %.2f. Keep the physical power disconnect ready.",
                 g_selected_joint + 1, HOME_DUTY);
#endif
    } else if (!strcmp(command, "p") || !strcmp(command, "pos")) {
        if (!arg1_is_number) {
            snprintf(out, out_size, "Usage: p <degrees>");
        } else {
            joint->target_deg = clamp_float(value1, -joint->soft_limit_deg,
                                             joint->soft_limit_deg);
            snprintf(out, out_size, "J%d target=%.2f deg%s",
                     g_selected_joint + 1, joint->target_deg,
                     fabsf(value1 - joint->target_deg) > 0.001f ? " (soft-limited)" : "");
        }
    } else if (!strcmp(command, "kp")) {
        if (!arg1_is_number || value1 < 0.1f || value1 > 10.0f) {
            snprintf(out, out_size, "Usage: kp <0.1..10>");
        } else {
            joint->kp_position = value1;
            snprintf(out, out_size, "Kp_position=%.3f", joint->kp_position);
        }
    } else if (!strcmp(command, "pi")) {
        if (!arg1_is_number || !arg2_is_number ||
            value1 < 0.0f || value1 > 0.1f || value2 < 0.0f || value2 > 1.0f) {
            snprintf(out, out_size, "Usage: pi <Kp_velocity 0..0.1> <Ki_velocity 0..1>");
        } else {
            joint->kp_velocity = value1;
            joint->ki_velocity = value2;
            joint->velocity_integral = 0.0f;
            snprintf(out, out_size, "PI_velocity Kp=%.5f Ki=%.5f",
                     joint->kp_velocity, joint->ki_velocity);
        }
    } else if (!strcmp(command, "vmax")) {
        if (!arg1_is_number || value1 < 1.0f || value1 > 90.0f) {
            snprintf(out, out_size, "Usage: vmax <1..90 deg/s>");
        } else {
            joint->max_speed_deg_s = value1;
            snprintf(out, out_size, "Vmax=%.1f deg/s", joint->max_speed_deg_s);
        }
    } else if (!strcmp(command, "u")) {
        if (!arg1_is_number || value1 < 0.05f || value1 > 0.35f) {
            snprintf(out, out_size, "Usage: u <0.05..0.35>");
        } else {
            joint->max_duty = value1;
            joint->velocity_integral =
                clamp_float(joint->velocity_integral, -value1, value1);
            snprintf(out, out_size, "Maximum duty=%.2f", joint->max_duty);
        }
    } else if (!strcmp(command, "test")) {
#if !HARDWARE_WIRING_VERIFIED
        snprintf(out, out_size, "Motion locked: verify wiring then set HARDWARE_WIRING_VERIFIED 1.");
#else
        if (!arg1_is_number || fabsf(value1) > OPEN_LOOP_TEST_MAX) {
            snprintf(out, out_size, "Usage: test <-0.35..0.35>");
        } else if (joint->enabled || joint->homing != HOME_IDLE) {
            snprintf(out, out_size, "Disable the joint and wait for homing to stop first.");
        } else if (joint->limit_left_active || joint->limit_right_active) {
            snprintf(out, out_size, "A limit switch is active; release it before testing.");
        } else {
            joint->open_loop_duty = value1;
            joint->open_loop_until_ms = now_ms() + OPEN_LOOP_TEST_MS;
            snprintf(out, out_size,
                     "Open-loop pulse %.2f for %d ms. Keep the mechanism clear.",
                     value1, OPEN_LOOP_TEST_MS);
        }
#endif
    } else {
        snprintf(out, out_size, "Unknown command. Type 'help'.");
    }

    xSemaphoreGive(g_state_mutex);
}

static void serial_console_task(void *argument)
{
    (void)argument;
    char line[128];
    char response[768];

    printf("\nSCARA ESP-IDF controller ready; motors disabled.\n");
    printf("Wi-Fi AP: %s | TCP port: %d\n", WIFI_AP_SSID, TCP_COMMAND_PORT);
    printf("Change WIFI_AP_PASSWORD and verify wiring before enabling motion.\n");

    while (fgets(line, sizeof(line), stdin) != NULL) {
        line[strcspn(line, "\r\n")] = '\0';
        command_execute(line, response, sizeof(response));
        printf("%s\n", response);
    }
    vTaskDelete(NULL);
}

static void wifi_start_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    (void)esp_netif_create_default_wifi_ap();

    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_config));

    wifi_config_t ap_config = {0};
    memcpy(ap_config.ap.ssid, WIFI_AP_SSID, strlen(WIFI_AP_SSID));
    ap_config.ap.ssid_len = strlen(WIFI_AP_SSID);
    memcpy(ap_config.ap.password, WIFI_AP_PASSWORD, strlen(WIFI_AP_PASSWORD));
    ap_config.ap.channel = 6;
    ap_config.ap.max_connection = 2;
    ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi AP '%s' started. Connect then TCP to 192.168.4.1:%d",
             WIFI_AP_SSID, TCP_COMMAND_PORT);
}

static int send_all(int sock, const char *data, size_t length)
{
    size_t sent = 0;
    while (sent < length) {
        const int result = send(sock, data + sent, length - sent, 0);
        if (result < 0) return -1;
        if (result == 0) break;
        sent += (size_t)result;
    }
    return (int)sent;
}

static void stop_all_safely(void)
{
    if (g_state_mutex != NULL &&
        xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(250)) == pdTRUE) {
        stop_all_locked();
        xSemaphoreGive(g_state_mutex);
    }
}

static void tcp_command_task(void *argument)
{
    (void)argument;
    struct sockaddr_in server_address = {0};
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(TCP_COMMAND_PORT);
    server_address.sin_addr.s_addr = htonl(INADDR_ANY);

    int listen_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_socket < 0) {
        ESP_LOGE(TAG, "Unable to create TCP socket: errno=%d", errno);
        vTaskDelete(NULL);
        return;
    }

    int reuse = 1;
    (void)setsockopt(listen_socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    if (bind(listen_socket, (struct sockaddr *)&server_address, sizeof(server_address)) < 0) {
        ESP_LOGE(TAG, "TCP bind failed: errno=%d", errno);
        close(listen_socket);
        vTaskDelete(NULL);
        return;
    }
    if (listen(listen_socket, 1) < 0) {
        ESP_LOGE(TAG, "TCP listen failed: errno=%d", errno);
        close(listen_socket);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "TCP command server listening on port %d", TCP_COMMAND_PORT);

    for (;;) {
        struct sockaddr_in client_address;
        socklen_t client_length = sizeof(client_address);
        int client_socket = accept(listen_socket,
                                   (struct sockaddr *)&client_address,
                                   &client_length);
        if (client_socket < 0) {
            ESP_LOGW(TAG, "accept failed: errno=%d", errno);
            vTaskDelay(pdMS_TO_TICKS(250));
            continue;
        }

        struct timeval timeout = {.tv_sec = 1, .tv_usec = 0};
        (void)setsockopt(client_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        const char *banner =
            "SCARA TCP console. One command per line; type help.\r\n";
        (void)send_all(client_socket, banner, strlen(banner));

        char line[128];
        size_t used = 0;
        char response[768];
        bool connected = true;
        uint32_t last_command_ms = now_ms();

        while (connected) {
            char buffer[96];
            const int received = recv(client_socket, buffer, sizeof(buffer), 0);
            if (received == 0) {
                connected = false;
                break;
            }
            if (received < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                    if ((uint32_t)(now_ms() - last_command_ms) > 30000U) {
                        ESP_LOGW(TAG, "TCP client idle timeout");
                        connected = false;
                    }
                    continue;
                }
                connected = false;
                break;
            }

            for (int i = 0; i < received; ++i) {
                const char c = buffer[i];
                if (c == '\r') continue;
                if (c == '\n') {
                    line[used] = '\0';
                    command_execute(line, response, sizeof(response));
                    used = 0;
                    last_command_ms = now_ms();
                    if (send_all(client_socket, response, strlen(response)) < 0 ||
                        send_all(client_socket, "\r\n", 2) < 0) {
                        connected = false;
                        break;
                    }
                } else if (used < sizeof(line) - 1) {
                    line[used++] = c;
                } else {
                    used = 0;
                    const char *error = "Command too long; try again.\r\n";
                    (void)send_all(client_socket, error, strlen(error));
                }
            }
        }

        shutdown(client_socket, SHUT_RDWR);
        close(client_socket);
        /* A lost Wi-Fi console must not leave the remote controller enabled. */
        stop_all_safely();
        ESP_LOGW(TAG, "TCP client disconnected; both joints disabled");
    }
}

void app_main(void)
{
    esp_err_t nvs_result = nvs_flash_init();
    if (nvs_result == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_result = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_result);

    g_state_mutex = xSemaphoreCreateMutex();
    if (g_state_mutex == NULL) {
        ESP_LOGE(TAG, "Could not create controller mutex");
        return;
    }

    init_joint_data();
    configure_hardware();

    xTaskCreate(control_task, "control_task", 4096, NULL, 10, NULL);
    xTaskCreate(serial_console_task, "serial_console", 4096, NULL, 5, NULL);

    wifi_start_softap();
    xTaskCreate(tcp_command_task, "tcp_command", 6144, NULL, 4, NULL);

    ESP_LOGI(TAG, "Started. Motors remain disabled until wiring is verified, homed and enabled.");
}
