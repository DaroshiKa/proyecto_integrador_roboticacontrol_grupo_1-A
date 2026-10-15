// SCARA Estacion 2 -- J1 y J2: encoders + MC33926 + PID de posicion + homing
// Sin ROS todavia. Convencion: 0 deg = centro del recorrido, derecha = positivo.
//
// Comandos serial (115200, terminados en Enter):
//   j <1|2>            selecciona articulacion
//   o <duty>           PRUEBA de signo y zona muerta en lazo abierto: duty (max 0.7) durante 300 ms (solo deshabilitada)
//   h                  homing: va al switch DERECHO (duty bajo), fija la posicion y se retira
//   z                  declara la posicion actual como 0 deg (brazo en el centro) y la da por referenciada
//   e <0|1>            deshabilita / habilita (arranca DESHABILITADA; habilitar exige referencia: h o z)
//   t <deg>            angulo objetivo, limitado al limite de software de cada articulacion
//   g <kp> <ki> <kd>   ganancias        u <0..1>   duty maximo
//   v                  alterna el stream: ms,objetivo,posicion,duty
//   p                  estado           x          PARO: deshabilita todo y cancela el homing
#include <Arduino.h>
#include "config.h"

// ------------------------------------------------------------------ Encoder
struct Encoder {
  int pinA = -1, pinB = -1;
  volatile int32_t count = 0;
  volatile uint8_t prev = 0;
};
static const int8_t QEM[16] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};

void IRAM_ATTR encISR(void* arg) {
  Encoder* e = static_cast<Encoder*>(arg);
  uint8_t curr = (digitalRead(e->pinA) << 1) | digitalRead(e->pinB);
  e->count += QEM[(e->prev << 2) | curr];
  e->prev = curr;
}

// ------------------------------------------------------------------ Motor (MC33926)
// Direccion: pin INV. Velocidad: PWM en el pin D2 (D2 en bajo = tri-state, sin freno).
// 'sign' corrige el sentido del motor para que duty positivo = angulo creciente.
struct Motor { int dir, pwm, ch, sign; };

void motorBegin(const Motor& m) {
  pinMode(m.dir, OUTPUT); digitalWrite(m.dir, LOW);
  double f = ledcSetup(m.ch, PWM_FREQ_HZ, PWM_BITS);
  Serial.printf("LEDC canal %d: frecuencia real = %.0f Hz\n", m.ch, f);   // 0 = configuracion invalida
  ledcAttachPin(m.pwm, m.ch);
  ledcWrite(m.ch, 0);
}
void motorWrite(const Motor& m, float u) {          // u en [-1, 1]
  u = constrain(u, -1.0f, 1.0f) * (float)m.sign;
  if (u != 0.0f) digitalWrite(m.dir, u < 0 ? HIGH : LOW);
  ledcWrite(m.ch, (uint32_t)(fabsf(u) * ((1 << PWM_BITS) - 1)));
}

// ------------------------------------------------------------------ Joint
struct Joint {
  Encoder enc;
  Motor   mot;
  int     encSign;
  int     limR, limL;
  float   limitDeg, switchRDeg;
  float   uMin;                                      // zona muerta de esta articulacion
  volatile bool  enabled = false, homed = false, tripped = false;
  volatile bool  limRHit = false, limLHit = false, limitActive = false;
  volatile uint8_t homing = 0;                       // 0 no, 1 hacia el switch, 2 retirandose
  volatile uint8_t homeFail = 0;                     // 1 switch izquierdo, 2 tiempo, 3 sin movimiento
  volatile bool  homeDone = false;
  uint32_t homeStart = 0, homeCheckMs = 0;
  int32_t  homeCheckCount = 0;
  volatile float targetDeg = 0, kp = DEF_KP, ki = DEF_KI, kd = DEF_KD, umax = DEF_UMAX;
  volatile float openDuty = 0;
  volatile uint32_t openUntil = 0;
  float integ = 0, prevPos = 0, dFilt = 0;
  volatile float posDeg = 0, duty = 0;
};
Joint joints[2];
int sel = 0;
bool streaming = false;

void jointBegin(Joint& j, int a, int b, int dir, int pwm, int ch, int encSign, int motSign,
                int limR, int limL, float limitDeg, float switchRDeg, float uMin) {
  j.enc.pinA = a; j.enc.pinB = b; j.encSign = encSign;
  j.limR = limR; j.limL = limL; j.limitDeg = limitDeg; j.switchRDeg = switchRDeg;
  j.uMin = uMin;
  j.mot = {dir, pwm, ch, motSign};
  motorBegin(j.mot);                                  // primero el motor: salida en 0
  pinMode(limR, INPUT_PULLUP); pinMode(limL, INPUT_PULLUP);
  pinMode(a, INPUT); pinMode(b, INPUT);               // 34/35/36/39 no tienen pull-up interno
  j.enc.prev = (digitalRead(a) << 1) | digitalRead(b);
  attachInterruptArg(a, encISR, &j.enc, CHANGE);
  attachInterruptArg(b, encISR, &j.enc, CHANGE);
}
inline float jointPosDeg(const Joint& j) { return j.encSign * (float)j.enc.count * DEG_PER_COUNT; }

// ------------------------------------------------------------------ Homing (dentro de la tarea de control)
// Devuelve true si este ciclo lo manejo el homing.
static bool homingStep(Joint& j, float pos, uint32_t now) {
  if (!j.homing) return false;
  uint8_t fail = 0;

  if (j.homing == 1 && j.limRHit) {                   // llego al switch: la posicion queda definida
    motorWrite(j.mot, 0);
    j.enc.count = (int32_t)lroundf(j.encSign * j.switchRDeg / DEG_PER_COUNT);
    j.homed = true; j.homing = 2;
    j.homeCheckMs = now; j.homeCheckCount = j.enc.count;
    j.duty = 0; j.integ = 0; j.prevPos = j.switchRDeg;
    return true;
  }
  if (j.homing == 2 && pos <= j.switchRDeg - HOME_BACKOFF_DEG && !j.limRHit) {   // retirado
    motorWrite(j.mot, 0); j.duty = 0;
    j.homing = 0; j.homeDone = true; j.targetDeg = pos; j.prevPos = pos; j.integ = 0;
    return true;
  }

  if (j.limLHit) fail = 1;
  else if (now - j.homeStart > HOME_TIMEOUT_MS) fail = 2;
  else if (now - j.homeCheckMs >= 400) {
    int32_t c = j.enc.count;
    if (abs(c - j.homeCheckCount) < HOME_STALL_COUNTS) fail = 3;
    j.homeCheckCount = c; j.homeCheckMs = now;
  }
  if (fail) { motorWrite(j.mot, 0); j.duty = 0; j.homing = 0; j.homeFail = fail; return true; }

  float u = (j.homing == 1) ? HOME_DUTY : -HOME_DUTY;
  motorWrite(j.mot, u); j.duty = u; j.prevPos = pos;
  return true;
}

// ------------------------------------------------------------------ Tarea de control (nucleo 1)
void controlTask(void*) {
  TickType_t last = xTaskGetTickCount();
  const float dt = 1.0f / CONTROL_HZ;
  for (;;) {
    uint32_t now = millis();
    for (auto& j : joints) {
      float pos = jointPosDeg(j);
      j.posDeg = pos;

      j.limRHit = (digitalRead(j.limR) == LOW);
      j.limLHit = (digitalRead(j.limL) == LOW);
      j.limitActive = j.limRHit || j.limLHit;

      if (homingStep(j, pos, now)) continue;

      if (j.enabled && j.limitActive) { j.enabled = false; j.tripped = true; }

      if (!j.enabled) {
        float u = (now < j.openUntil) ? j.openDuty : 0.0f;   // solo la prueba 'o' mueve en deshabilitado
        motorWrite(j.mot, u); j.duty = u; j.integ = 0; j.prevPos = pos; continue;
      }

      float tgt  = constrain((float)j.targetDeg, -j.limitDeg, j.limitDeg);
      float err  = tgt - pos;
      float vel  = (pos - j.prevPos) / dt;            // derivada sobre la MEDICION
      j.prevPos  = pos;
      j.dFilt   += DERIV_ALPHA * (vel - j.dFilt);

      float uRaw = j.kp * err + j.integ - j.kd * j.dFilt;
      float u    = constrain(uRaw, -j.umax, j.umax);
      if (u == uRaw || (uRaw * err) < 0) j.integ += j.ki * err * dt;   // anti-windup

      // Compensacion de zona muerta: mapea la salida del PID [0,1] al rango util [uMin,1].
      // Fuera de la banda muerta de error; dentro de ella el duty queda como lo dio el PID.
      if (fabsf(err) > DEADBAND_DEG && u != 0.0f) {
        u = copysignf(j.uMin + (1.0f - j.uMin) * fabsf(u), u);
        u = constrain(u, -j.umax, j.umax);
      }

      motorWrite(j.mot, u);
      j.duty = u;
    }
    vTaskDelayUntil(&last, pdMS_TO_TICKS(1000 / CONTROL_HZ));
  }
}

// ------------------------------------------------------------------ Comandos serial
void printStatus() {
  Joint& j = joints[sel];
  const char* lim = (j.limRHit && j.limLHit) ? "RL" : j.limRHit ? "R" : j.limLHit ? "L" : "-";
  Serial.printf("J%d en=%d homed=%d lim=%s target=%.2f pos=%.2f deg duty=%.3f | kp=%.4f ki=%.4f kd=%.4f umax=%.2f | counts=%ld\n",
                sel + 1, (int)j.enabled, (int)j.homed, lim, j.targetDeg, jointPosDeg(j), j.duty,
                j.kp, j.ki, j.kd, j.umax, (long)j.enc.count);
}

void handleLine(char* s) {
  Joint& j = joints[sel];
  float a, b, c; int k;
  switch (s[0]) {
    case 'j': if (sscanf(s + 1, "%d", &k) == 1 && k >= 1 && k <= 2) sel = k - 1; break;
    case 'x':
      for (auto& q : joints) { q.enabled = false; q.homing = 0; q.openUntil = 0; }
      Serial.println("PARO: todo deshabilitado.");
      break;
    case 'e': if (sscanf(s + 1, "%d", &k) == 1) {
                if (k) {
                  if (!j.homed) { Serial.println("Sin referencia: usa 'h' (homing) o, con el brazo en el centro, 'z'."); return; }
                  if (j.homing) { Serial.println("Espera a que termine el homing."); return; }
                  if (j.limitActive) { Serial.println("Un final de carrera esta activo: mueve la articulacion a mano para liberarlo."); return; }
                  j.targetDeg = jointPosDeg(j); j.prevPos = j.targetDeg; j.integ = 0; j.enabled = true;
                } else { j.enabled = false; }
              } break;
    case 't': if (sscanf(s + 1, "%f", &a) == 1) j.targetDeg = constrain(a, -j.limitDeg, j.limitDeg); break;
    case 'g': if (sscanf(s + 1, "%f %f %f", &a, &b, &c) == 3) { j.kp = a; j.ki = b; j.kd = c; j.integ = 0; } break;
    case 'u': if (sscanf(s + 1, "%f", &a) == 1) j.umax = constrain(a, 0.0f, 1.0f); break;
    case 'z':
      if (j.homing) { Serial.println("Espera a que termine el homing."); return; }
      j.enc.count = 0; j.targetDeg = 0; j.prevPos = 0; j.integ = 0; j.homed = true; break;
    case 'v': streaming = !streaming; break;
    case 'h':
      if (j.enabled) { Serial.println("Deshabilita primero (e 0)."); return; }
      if (j.homing)  { Serial.println("Ya esta en homing."); return; }
      Serial.printf("Homing J%d hacia el switch DERECHO a duty %.2f. Mano lista para cortar la fuente ('x' = paro).\n", sel + 1, HOME_DUTY);
      j.homeFail = 0; j.homeDone = false;
      j.homeStart = millis(); j.homeCheckMs = j.homeStart; j.homeCheckCount = j.enc.count;
      j.homing = 1;
      return;
    case 'o':
      if (sscanf(s + 1, "%f", &a) == 1) {
        if (j.enabled || j.homing) { Serial.println("Deshabilita primero (e 0) y espera a que termine el homing."); return; }
        a = constrain(a, -0.7f, 0.7f);                // antes 0.3: no alcanzaba la zona muerta
        float p0 = jointPosDeg(j);
        j.openDuty = a; j.openUntil = millis() + 300;
        uint32_t t0 = millis();
        while (millis() - t0 < 450) {                 // diagnostico: lo que realmente aplica el hilo de control
          Serial.printf("t=%lu duty=%.3f counts=%ld\n", (unsigned long)(millis() - t0), j.duty, (long)j.enc.count);
          delay(50);
        }
        float dp = jointPosDeg(j) - p0;
        Serial.printf("Prueba J%d duty=%+.2f -> cambio %+.2f deg\n", sel + 1, a, dp);
        if (fabsf(dp) < 0.5f) Serial.println("Casi no se movio: sube el duty (zona muerta) o revisa 12 V, D2 y encoder.");
        else if ((a > 0) != (dp > 0)) Serial.printf("Signo invertido: cambia MOTOR_SIGN_J%d a -1 en config.h (NO toques ENC_SIGN).\n", sel + 1);
        else Serial.println("Signo correcto: duty positivo sube el angulo (hacia el switch derecho).");
      } break;
    case 'p': break;
    default:  return;
  }
  printStatus();
}

void setup() {
  Serial.begin(115200);
  delay(300);
  if (!PINS_VERIFIED) {
    Serial.println("PINS_VERIFIED en false en config.h: los motores quedan apagados.");
    for (;;) delay(1000);
  }
  jointBegin(joints[0], J1_ENC_A, J1_ENC_B, J1_DIR, J1_PWM, 0, ENC_SIGN_J1, MOTOR_SIGN_J1, J1_LIM_R, J1_LIM_L, J1_LIMIT_DEG, J1_SWITCH_R_DEG, U_MIN_J1);
  jointBegin(joints[1], J2_ENC_A, J2_ENC_B, J2_DIR, J2_PWM, 1, ENC_SIGN_J2, MOTOR_SIGN_J2, J2_LIM_R, J2_LIM_L, J2_LIMIT_DEG, J2_SWITCH_R_DEG, U_MIN_J2);
  xTaskCreatePinnedToCore(controlTask, "control", 4096, nullptr, 5, nullptr, 1);
  Serial.println("Listo. Todo DESHABILITADO. 'j 1' y 'o 0.5' para zona muerta/signo, luego 'h' para el homing. 'x' = paro.");
}

void loop() {
  static char buf[64]; static uint8_t n = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { if (n) { buf[n] = 0; handleLine(buf); n = 0; } }
    else if (n < sizeof(buf) - 1) buf[n++] = ch;
  }
  for (int i = 0; i < 2; i++) {
    Joint& q = joints[i];
    if (q.tripped) { q.tripped = false; Serial.printf("J%d: final de carrera activado, deshabilitada.\n", i + 1); }
    if (q.homeFail) {
      uint8_t f = q.homeFail; q.homeFail = 0;
      if (f == 1)      Serial.printf("J%d: homing abortado: llego al switch IZQUIERDO (sentido invertido: revisa MOTOR_SIGN_J%d).\n", i + 1, i + 1);
      else if (f == 2) Serial.printf("J%d: homing abortado: tiempo agotado.\n", i + 1);
      else             Serial.printf("J%d: homing abortado: el motor no se mueve (revisa 12 V, D2 y el encoder).\n", i + 1);
    }
    if (q.homeDone) { q.homeDone = false; Serial.printf("J%d: homing listo, posicion %.1f deg. Ya puedes usar 'e 1' y 't 0'.\n", i + 1, jointPosDeg(q)); }
  }
  static uint32_t lastPrint = 0;
  if (streaming && millis() - lastPrint >= 20) {
    lastPrint = millis();
    Joint& j = joints[sel];
    Serial.printf("%lu,%.2f,%.2f,%.3f\n", (unsigned long)millis(), j.targetDeg, j.posDeg, j.duty);
  }
  delay(1);
}
