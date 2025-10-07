# Laboratorio Nº 1 - Microcontroladores.
## Integrantes del grupo Transferitos:
  - Beneyto, Mateo.
  - Castro, Mauricio Nicolás.
  - Kalchichen, Lucas Gernán.
  - Lopez Soto, Martin.
  - Peralta Ruiz, Nadine Andrea.


## Detalles de la solución:
La solución propuesta se implementará en un microcontrolador que coordina la lectura de sensores y el accionamiento de actuadores para cumplir los objetivos definidos. A continuación, se detalla cómo se materializa cada uno:

### Monitoreo de Control de Propulsión
#### Sensores utilizados:
    - Sensor de temperatura → permite detectar variaciones térmicas que condicionan la propulsión.
    - Sensor de turbulencia → mide la estabilidad del vuelo, indicando si existen perturbaciones que obligan a correcciones.
 #### Actuador involucrado:
    - Motor de paso → simula el motor de la cápsula. Su velocidad de giro se ajusta en tiempo real según las lecturas de los sensores, aumentando o disminuyendo la potencia de ascenso hasta alcanzar los 40.000 metros.
     
### Monitoreo de Condición de Eyección/Aterrizaje
  #### Sensores utilizados:
    - Sensor de temperatura → indica sobrecalentamiento cuando se superan los límites de seguridad.
    - Sensor de turbulencia → detecta inestabilidad que impide la eyección.
  #### Actuadores involucrados:
    - Luces LED RGB → representan el estado de seguridad del vuelo:
    - Verde: condiciones seguras para aterrizaje/eyección.
    - Azul parpadeante: turbulencia detectada (no seguro).
    - Rojo parpadeante: sobrecalentamiento (no seguro).
    - LED adicional: simboliza el despliegue del paracaídas cuando se cumplen las condiciones seguras en el ascenso y   simboliza el aterrizaje de la cápsula cuando se cumplen las condiciones seguras en el descenso.
    
### Integración de los Monitoreos
Ambos lazos de control funcionan de manera simultánea y complementaria: mientras el motor de paso regula el ascenso según temperatura y turbulencia, el sistema de indicadores luminosos comunica si la cápsula se encuentra en un estado apto para eyección.

![WhatsApp Image 2025-10-02 at 13 57 07](https://github.com/user-attachments/assets/c756afd3-715d-4daf-af81-1603462fb952)


## Detalles del desarrollo:
El código fué desarrollado en Python, con la IDE TONY.
### Las librerías utilizadas fueron:
  - /lib/adafruit_minimqtt: Copiar la carpeta completa.
  - /lib/adafruit_ticks.mpy: Módulo que necesita minimqtt.
  - /lib/adafruit_connection_manager.mpy: Módulo que necesita minimqtt.
  - /lib/adafruit_esp32spi_socketpool.mpy: Módulo para conectarnos a la red.
