# Mapa de pines y recorrido — SCARA (ESP32 WROOM-32 + 2 × MC33926)

Mapeado a mano sobre la placa (no hay esquemático).

## Encoders (alimentados con Vin y GND)
| Articulación | Canal A (blanco) | Canal B (amarillo) |
|---|---|---|
| J1 | D35 | D34 |
| J2 | VN (GPIO39) | VP (GPIO36) |

## Drivers MC33926
| Pin del driver | Driver 1 (J1) | Driver 2 (J2) |
|---|---|---|
| INV (dirección) | D26 | D33 |
| D2 / PWM (velocidad) | D25 | D32 |
| IN1 | 3V3 | 3V3 |
| IN2 | GND | GND |
| EN, SLEW | 3V3 | 3V3 |
| D1 | a GND en la placa del driver (verificar) | GND |
| OUT1 / OUT2 | cable negro / rojo del motor 1 | cable negro / rojo del motor 2 |
| VIN / GND | +V / −V de la fuente | +V / −V de la fuente |

Con IN1 en alto e IN2 en bajo, el pin INV invierte el sentido y el PWM en D2 fija la velocidad.
Con D2 en bajo el driver queda en tri-state: el motor gira libre, sin freno.
FB (corriente) y SF (falla) no están conectados.

## Finales de carrera (contacto NO entre el pin y GND, activos en bajo)
| Articulación | Derecho | Izquierdo |
|---|---|---|
| J1 | D27 | D13 |
| J2 | D19 | D18 |

## Recorrido (medido a mano entre switches)
Convención: 0° = brazo estirado y alineado con un eje (centro del recorrido); derecha = positivo.

| Articulación | Recorrido total | Switch derecho | Límite de software |
|---|---|---|---|
| J1 | 191.2° | +95.6° | ±88° |
| J2 | 144.3° | +72.2° | ±67° |

La posición de los switches se midió presionándolos a mano; conviene afinarla con el homing real.

## Pendiente
- Z (articulación prismática, solenoide): no disponible todavía, sin pin asignado.
- Verificar con el homing real la posición de los switches y que el izquierdo quede simétrico.
