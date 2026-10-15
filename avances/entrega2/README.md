# Entrega 2 — Manipulador SCARA con ESP-IDF y control en cascada

Firmware nativo de **ESP-IDF** para el ESP32-WROOM-32. Esta versión reemplaza el proyecto Arduino/PlatformIO anterior por CMake y componentes de ESP-IDF.

## Funcionalidad

- Encoders incrementales para las articulaciones J1 y J2.
- PWM para los drivers MC33926.
- Control en cascada: **P de posición** produce la referencia de velocidad; **PI de velocidad** produce el duty del motor.
- Límites angulares por software y finales de carrera físicos.
- Homing con timeout y detección de ausencia de movimiento.
- La misma consola de comandos por puerto serie y por Wi-Fi (SoftAP + TCP).
- Motores deshabilitados al arrancar.
- Los comandos de movimiento están bloqueados por defecto hasta revisar el cableado.

## Estructura

```text
firmware/
├── CMakeLists.txt
├── sdkconfig.defaults
└── main/
    ├── CMakeLists.txt
    ├── main.c
    └── robot_config.h
```

## Antes de mover los motores

1. Comprueba alimentación, GND común, cableado del ESP32, dos drivers MC33926, encoders y finales de carrera.
2. Cambia `WIFI_AP_PASSWORD` en `main/robot_config.h` por una clave propia de al menos 8 caracteres.
3. La escala heredada es `64 × 50 × 5 = 16000` cuentas/vuelta articular. Como el decodificador usa transiciones en cuadratura x4, confirma qué significa exactamente “64 CPR” en el encoder instalado. Mide manualmente una vuelta y calibra la escala antes de confiar en los ángulos.
4. Solo después de verificar el cableado, cambia `HARDWARE_WIRING_VERIFIED` de `0` a `1` en `main/robot_config.h` y recompila. Es una confirmación explícita, no una medición automática.

No hemos compilado este firmware con tu versión local de ESP-IDF ni lo hemos probado sobre el robot físico. El sentido de motor, el conteo del encoder, los switches y las ganancias deben verificarse con cuidado. Mantén un interruptor físico de alimentación al alcance; `stop` es una parada por software, no un paro de emergencia certificado.

## Instalar y compilar

Instala ESP-IDF 5.3 o posterior siguiendo la guía oficial de Espressif: [Get Started — ESP32](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html).

Abre una terminal con el entorno de ESP-IDF activado y ejecuta desde esta carpeta:

```bash
cd avances/entrega2/firmware
idf.py set-target esp32
idf.py build
```

Si `idf.py` no se reconoce, primero activa el entorno con `export.sh` (Linux/macOS) o `export.bat` (Windows) de tu instalación de ESP-IDF.

Identifica el puerto serie en Linux con:

```bash
ls /dev/ttyUSB* /dev/ttyACM*
```

Usa el puerto real de tu placa:

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

El monitor suele funcionar a 115200 baudios. Mantén la fuente de motores desconectada durante la primera compilación y carga.

## Comandos simples

Escribe un comando por línea y presiona Enter. La consola serie y Wi-Fi comparten el mismo parser.

| Comando | Acción |
|---|---|
| `help` | Ayuda |
| `j1`, `j2` | Selecciona articulación |
| `s` | Muestra posición, velocidad, límites y ganancias |
| `test 0.15` | Pulso corto para revisar dirección, solo con el sistema deshabilitado |
| `home` | Busca el switch derecho y se retira |
| `on` / `off` | Habilita / deshabilita la articulación seleccionada |
| `p 5` | Ordena una posición en grados |
| `kp 2` | Cambia la P de posición |
| `pi 0.008 0.025` | Cambia Kp y Ki del PI de velocidad |
| `vmax 20` | Limita la velocidad en grados/s |
| `u 0.25` | Limita el duty máximo |
| `stop` | Deshabilita ambas articulaciones |

Secuencia inicial sugerida, después de revisar cableado y activar la confirmación en configuración:

1. Selecciona un eje con `j1` o `j2`.
2. Haz primero una prueba breve con `test 0.15`, con el mecanismo despejado. Aumenta con precaución solo si el motor no vence la zona muerta, sin superar `0.70` en esta configuración.
3. Confirma que el encoder cambia en el sentido esperado. Si el motor gira al revés, corrige `J1_MOTOR_SIGN` o `J2_MOTOR_SIGN`; si el encoder lee al revés, revisa su signo por separado.
4. Solo entonces ejecuta `home`, siempre con supervisión física y posibilidad de cortar alimentación inmediatamente.
5. Cuando termine el homing, prueba `on` y un movimiento corto como `p 3`. Consulta `s` y usa `stop` cuando termines.

No ejecutes homing si el sentido de giro no está confirmado: el firmware no puede detectar todos los errores de cableado ni sustituir la supervisión física.

## Control en cascada

El lazo de posición calcula una referencia de velocidad:

```text
e_pos = theta_ref - theta
omega_ref = clamp(Kp_pos * e_pos, -vmax, +vmax)
```

El lazo de velocidad genera el mando del motor:

```text
e_vel = omega_ref - omega_measured
u = clamp(Kp_vel * e_vel + integral, -umax, +umax)
integral += Ki_vel * e_vel * Ts
```

La integración se congela cuando la salida está saturada en la dirección del error para reducir el windup. El lazo corre cada 5 ms (200 Hz) y filtra la velocidad estimada.

Para sintonizar, comienza con duty bajo y límites físicos: primero estabiliza la velocidad con el PI, luego sube gradualmente Kp de posición. Los parámetros incluidos son valores iniciales prudentes, no ganancias validadas. Cambiar las ganancias con `kp` o `pi` afecta la sesión actual; al reiniciar, se cargan los valores definidos en `robot_config.h`.

## Control por Wi-Fi

La placa inicia un punto de acceso local:

- SSID: `SCARA-Control`
- Clave: `WIFI_AP_PASSWORD` de `main/robot_config.h`
- IP: `192.168.4.1`
- Puerto TCP: `3333`

Conecta tu portátil o teléfono a la red de la ESP32. En Linux, ejecuta:

```bash
nc 192.168.4.1 3333
```

Después usa los mismos comandos (`s`, `j1`, `home`, `p 3`, etc.). Cada comando debe terminar en Enter. Puedes usar cualquier cliente TCP que se conecte a `192.168.4.1:3333` si no tienes `nc`.

El servidor permite un cliente TCP a la vez. Una desconexión o 30 segundos sin comandos desde el cliente TCP provocan la deshabilitación de ambos ejes. La consola serial puede seguir conectada. El canal TCP no cifra los comandos: cambia la clave y evita conectar dispositivos no confiables.

## Pines

Consulta [docs/mapa_pines.md](docs/mapa_pines.md). Los GPIO 34, 35, 36 y 39 son solo entrada y no tienen pull-up interno. Asegúrate de que las señales del encoder no excedan 3.3 V.

## Pendiente de validación real

- Confirmar que compila con la versión de ESP-IDF instalada.
- Calibrar CPR efectivo y grados por cuenta.
- Verificar signos de encoder y motor, además de límites físicos.
- Ajustar PI de velocidad y luego la P de posición en pruebas de baja energía.
- Añadir un paro de emergencia físico independiente para toda operación real.
