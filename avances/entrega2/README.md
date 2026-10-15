# Entrega 2 — Firmware SCARA

Se conservan dos versiones del firmware para poder volver a la anterior mientras se valida la migración.

## Versiones

- **[V1 — Arduino / PlatformIO](v1/README.md)**: firmware anterior para uso temporal.
- **[V2 Wi-Fi — ESP-IDF](v2_wifi/README.md)**: versión nueva en C con control en cascada P de posición + PI de velocidad, consola serial y Wi-Fi TCP.

## Recomendación

Usa **v1** si necesitas recuperar exactamente el firmware anterior. La versión nueva debe compilarse en tu entorno y validarse de forma cuidadosa en el hardware antes de mover el mecanismo.

El mapa de conexiones compartido está en [docs/mapa_pines.md](docs/mapa_pines.md). Las versiones mantienen el código y la configuración por separado.
