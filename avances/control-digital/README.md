# Avances de Control Digital

Espacio para organizar el trabajo de Control Digital sin mezclarlo con el firmware de Robótica.

## Estructura propuesta

```text
avances/control-digital/
├── README.md
├── matlab/       # Scripts y funciones MATLAB
├── modelos/      # Modelos de planta y discretización
├── simulaciones/ # Resultados y gráficas
└── informes/     # Documentación de prácticas y entregas
```

Las carpetas se irán incorporando a medida que se agreguen archivos reales.

## Para cada entrega

- Describir el sistema, los parámetros y las unidades.
- Mantener los scripts reproducibles y evitar rutas absolutas de la máquina local.
- Registrar la metodología de diseño del controlador y las restricciones.
- Guardar gráficas con ejes, unidades y leyendas legibles.
- Separar resultados simulados de resultados medidos.

## Historial

Usa mensajes de commit específicos, por ejemplo:

- `feat(control): add discrete plant model`
- `feat(control): implement controller tuning`
- `docs(control): document validation results`

No se deben presentar simulaciones como validaciones experimentales si no se ejecutaron en el hardware.