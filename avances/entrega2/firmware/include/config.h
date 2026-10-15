#pragma once
#include <Arduino.h>

// =====================================================================
//  MECANICA  (BOM / informe)
// =====================================================================
constexpr float MOTOR_ENC_CPR   = 64.0f;          // cuentas por vuelta del MOTOR, decodificacion x4
constexpr float GEARBOX_RATIO   = 50.0f;          // confirmar la relacion exacta en la ficha de Pololu
constexpr float PULLEY_RATIO    = 100.0f / 20.0f; // polea 20T (motor) -> 100T (articulacion) = 5
constexpr float COUNTS_PER_JOINT_REV = MOTOR_ENC_CPR * GEARBOX_RATIO * PULLEY_RATIO; // 16000
constexpr float DEG_PER_COUNT   = 360.0f / COUNTS_PER_JOINT_REV;                     // 0.0225 deg

// Convencion de angulos: 0 deg = brazo estirado y alineado (centro del recorrido),
// DERECHA = POSITIVO. Medido a mano con ENC_SIGN = +1: NO cambiar ENC_SIGN.
// Si la prueba 'o' dice "signo invertido", cambia MOTOR_SIGN (sentido del motor).
constexpr int ENC_SIGN_J1   = +1, ENC_SIGN_J2   = +1;
constexpr int MOTOR_SIGN_J1 = +1, MOTOR_SIGN_J2 = +1;

// Posicion del switch DERECHO medida desde el centro (medida a mano: afinar con el homing real).
// El izquierdo queda simetrico (-valor).
constexpr float J1_SWITCH_R_DEG = 95.6f;   // recorrido total medido: 191.2 deg
constexpr float J2_SWITCH_R_DEG = 72.2f;   // recorrido total medido: 144.3 deg
// Limite de software: unos 5-7 deg antes de cada switch.
constexpr float J1_LIMIT_DEG = 88.0f;
constexpr float J2_LIMIT_DEG = 67.0f;

// Homing: va al switch derecho a duty bajo, fija la posicion y se retira BACKOFF_DEG.
constexpr float    HOME_DUTY         = 0.20f;
constexpr float    HOME_BACKOFF_DEG  = 8.0f;
constexpr uint32_t HOME_TIMEOUT_MS   = 30000;
constexpr int      HOME_STALL_COUNTS = 8;      // minimo de cuentas cada 400 ms; si no, aborta

// =====================================================================
//  PINES (mapeados a mano sobre la placa)
//  MC33926: direccion por INV, velocidad por PWM en el pin D2 del driver.
//  IN1=3V3, IN2=GND, EN=3V3, D1=GND en la placa: no los maneja el ESP32.
//  Ojo: con D2 en bajo el driver queda en tri-state (el motor gira libre).
//  GPIO 34/35/36(VP)/39(VN) son solo entrada y NO tolerantes a 5 V.
// =====================================================================
#define PINS_VERIFIED true

constexpr int J1_DIR = 26, J1_PWM = 25;      // driver 1: INV, D2
constexpr int J2_DIR = 33, J2_PWM = 32;      // driver 2: INV, D2

constexpr int J1_ENC_A = 35, J1_ENC_B = 34;  // blanco, amarillo
constexpr int J2_ENC_A = 39, J2_ENC_B = 36;  // blanco (VN), amarillo (VP)

// Finales de carrera: contacto NO entre el pin y GND -> activo en BAJO (INPUT_PULLUP)
constexpr int J1_LIM_R = 27, J1_LIM_L = 13;
constexpr int J2_LIM_R = 19, J2_LIM_L = 18;

// =====================================================================
//  CONTROL
// =====================================================================
constexpr int   CONTROL_HZ  = 1000;
constexpr int   PWM_FREQ_HZ = 20000;   // el MC33926 acepta hasta 20 kHz
constexpr int   PWM_BITS    = 10;
constexpr float DERIV_ALPHA = 0.1f;    // filtro pasa-bajas de la velocidad medida

// Valores de arranque (duty por grado, por (grado*s), por (grado/s)). Solo un punto de partida.
constexpr float DEF_KP = 0.05f, DEF_KI = 0.0f, DEF_KD = 0.001f;
constexpr float DEF_UMAX = 0.25f;      // duty maximo al arrancar: bajo a proposito
