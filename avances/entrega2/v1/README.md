# V1 — Arduino-ESP32 + PlatformIO

Esta carpeta conserva la versión anterior del firmware para que puedas seguir usándola mientras pruebas la migración ESP-IDF.

## Entorno

- ESP32-WROOM-32.
- PlatformIO con framework Arduino.
- Plataforma `espressif32@6.9.0`.
- Drivers MC33926, encoders en cuadratura y finales de carrera.
- PID de posición por articulación con ajustes por serial, según el firmware original.

## Estructura

```text
v1/
├── platformio.ini
├── .gitignore
├── include/
│   └── config.h
└── src/
    └── main.cpp
```

## Compilar y cargar

Abre una terminal en esta carpeta con PlatformIO disponible:

```bash
cd avances/entrega2/v1
pio run
pio run -t upload
pio device monitor -b 115200
```

## Comandos de la versión anterior

Envía cada comando desde el monitor serie y presiona Enter:

| Comando | Acción |
|---|---|
| `j 1` / `j 2` | Selecciona articulación |
| `o 0.5` | Pulso breve para comprobar el sentido, con la articulación deshabilitada |
| `h` | Homing al switch derecho |
| `z` | Define la posición actual como cero; solo si el brazo está centrado |
| `e 0` / `e 1` | Deshabilita / habilita una articulación ya referenciada |
| `t 5` | Objetivo angular en grados |
| `g 0.05 0 0.001` | Ajusta Kp, Ki y Kd del PID de posición |
| `u 0.5` | Límite de duty |
| `v` | Activa/desactiva el stream para Serial Plotter |
| `p` | Estado |
| `x` | Parada de software: deshabilita ambos ejes y cancela homing |

## Advertencias

Esta es la versión anterior, conservada sin modificar su implementación. Verifica cableado, convención de los encoders y sentido de los motores antes de mover el manipulador. El comando `x` no reemplaza un interruptor físico de emergencia.

La versión nueva está en [V2 Wi-Fi / ESP-IDF](../v2_wifi/README.md). El mapa compartido de pines está en [docs/mapa_pines.md](../docs/mapa_pines.md).
