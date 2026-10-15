# Entrega 2 — Firmware del manipulador SCARA

Firmware para un ESP32-WROOM-32 que controla las articulaciones rotacionales J1 y J2 de un manipulador SCARA. El proyecto original usa PlatformIO con Arduino-ESP32 2.0.x, dos drivers MC33926 y encoders en cuadratura.

## Contenido

```text
entrega2/
├── README.md
├── docs/
│   └── mapa_pines.md
└── firmware/
    ├── .gitignore
    ├── platformio.ini
    ├── include/
    │   └── config.h
    └── src/
        └── main.cpp
```

## Funcionalidad documentada en el firmware

- Lectura de los encoders incrementales de J1 y J2 con decodificación en cuadratura.
- Accionamiento de los motores mediante drivers MC33926.
- Control PID de posición por articulación y ajuste de ganancias por puerto serial.
- Homing hacia el final de carrera derecho, con tiempo máximo y detección de falta de movimiento.
- Límites de software para las articulaciones y deshabilitación al detectar un final de carrera.
- Comandos seriales para seleccionar articulación, referenciar, habilitar/deshabilitar, establecer objetivo, ajustar ganancias y revisar el estado.

El firmware arranca con los motores deshabilitados. Antes de operar el mecanismo, revisa el cableado, la dirección de giro y los finales de carrera con el sistema asegurado.

## Requisitos

- PlatformIO Core o la extensión PlatformIO para VS Code.
- Placa ESP32-WROOM-32 compatible con el entorno `esp32dev`.
- Drivers MC33926 y encoders según el mapa de pines documentado.
- Fuente y cableado dimensionados para los motores del montaje.

## Compilar, cargar y abrir el monitor serial

Desde una terminal, entra a la carpeta `firmware`:

```bash
cd avances/entrega2/firmware
pio run
pio run -t upload
pio device monitor -b 115200
```

La compilación y la carga son operaciones distintas. Ejecuta primero `pio run`; conecta la placa y ejecuta la carga solo después de revisar la configuración.

## Comandos seriales

Monitor a 115200 baudios; envía cada comando terminado en Enter.

| Comando | Función |
|---|---|
| `j 1` / `j 2` | Selecciona la articulación |
| `o <duty>` | Prueba breve de sentido de giro, con duty limitado a ±0.3 y articulación deshabilitada |
| `h` | Ejecuta el homing hacia el switch derecho |
| `z` | Define la posición actual como 0°; úsalo solo con el brazo en el centro |
| `e 0` / `e 1` | Deshabilita / habilita después de referenciar |
| `t <grados>` | Fija la referencia angular, limitada por software |
| `g <kp> <ki> <kd>` | Cambia ganancias PID |
| `u <0..1>` | Define el duty máximo |
| `v` | Activa/desactiva la salida de datos para Serial Plotter |
| `p` | Imprime el estado |
| `x` | Parada: deshabilita motores y cancela el homing |

Mantén acceso a un medio físico para cortar la alimentación durante las pruebas. No pruebes el homing sin poder detener el mecanismo de inmediato.

## Documentación técnica

- [Mapa de pines y recorrido](docs/mapa_pines.md)
- [Configuración de control y pines](firmware/include/config.h)
- [Configuración PlatformIO](firmware/platformio.ini)
- [Código principal](firmware/src/main.cpp)

## Estado y verificaciones pendientes

El código contiene control de posición, encoders, homing y comandos por serial. Según el README del repositorio original, quedan pendientes o por completar la verificación del mapa de pines con el hardware, el ajuste final del homing, el PID en cascada posición-velocidad, la secuencia del eje Z y los adaptadores de micro-ROS/serial con arbitraje de modos y watchdog.

**Importante:** el README original indica que debe verificarse el mapa de pines, pero el archivo `config.h` copiado actualmente define `PINS_VERIFIED true`. Esta diferencia debe resolverse revisando el montaje real antes de usar los motores; no se debe tomar la constante como prueba de que el cableado fue verificado.

## Fuente del material

Copia de trabajo del firmware y documentación existentes en el [repositorio original, carpeta Robótica](https://github.com/DaroshiKa/proyecto_integrador_robotica_control_grupo_1-A/tree/main/Rob%C3%B3tica). El repositorio original no se modifica por esta copia.