# Proyecto de Graduación

Desarrollo y evaluación de estrategias de control para el seguimiento de trayectorias y la coordinación de robots móviles **Pololu 3Pi**. El repositorio reúne simulaciones en **MATLAB**, implementaciones en **Webots** y pruebas con robots físicos mediante **Python y Robotat**.

## Objetivo

Estudiar el comportamiento de varios agentes durante el seguimiento de trayectorias y de otros robots, utilizando configuraciones con líder y esquemas de persecución cíclica sin líder fijo.

## Organización del repositorio

| Carpeta o conjunto | Descripción |
| --- | --- |
| MATLAB: versiones iniciales | Simulaciones de seguimiento circular con PID, acercamiento exponencial, persecución de puntos y LQI, además de pruebas de generación de trayectorias. |
| MATLAB: versiones optimizadas | Versiones refactorizadas para configurar varios agentes y una implementación de persecución cíclica. |
| `wbtlab11_Matlab` | Proyecto de Webots con controladores MATLAB para el seguimiento de un líder y alternativas de control. |
| `wbtlab11_Py` | Proyecto de Webots con controladores Python para una formación con líder y seguidores. |
| `wbtlab11_Py_Cycl` | Variante de Webots en Python con persecución cíclica, donde los agentes forman un anillo sin líder fijo. |
| Seguimiento de Agentes | Implementación para robots físicos: recepción de poses desde Robotat, control de ruedas y visualización en vivo. |

Las versiones iniciales y optimizadas permiten revisar la evolución de las implementaciones. Los proyectos de Webots y el seguimiento con Robotat trasladan estas estrategias a entornos de simulación física y experimentación.

## Estrategias implementadas

- **Seguimiento de trayectorias:** desplazamiento hacia puntos de referencia y recorridos circulares.
- **Seguimiento de agentes:** cada robot utiliza la posición de otro agente como referencia.
- **Persecución cíclica:** los agentes se conectan en un ciclo cerrado y comparten la misma lógica de seguimiento.
- **Control de movimiento:** variantes con PID, velocidad de acercamiento exponencial y LQI, según la implementación.

## Herramientas utilizadas

| Herramienta | Uso |
| --- | --- |
| MATLAB | Simulación, diseño de control y análisis de trayectorias. |
| Webots | Simulación de robots diferenciales con sensores y motores. |
| Python | Controladores, comunicación y visualización. |
| Robotat | Obtención de la posición y orientación de los agentes durante las pruebas físicas. |
| MQTT y TCP | Recepción de poses y envío de comandos a los robots. |

## Cómo utilizar el repositorio

1. Seleccionar la carpeta correspondiente al entorno de trabajo: MATLAB, Webots o robots físicos.
2. Consultar su README para conocer los requisitos, archivos principales y parámetros de esa variante.
3. Preparar el entorno y revisar la configuración antes de ejecutar los controladores.
4. Ejecutar la implementación siguiendo las instrucciones de su carpeta.

En los proyectos de Webots se abre el mundo incluido desde el simulador. En las simulaciones de MATLAB se ejecuta el script elegido. Para el seguimiento físico se utiliza el programa principal de Python con acceso a la red Robotat y a los robots configurados.

## Documentación

Cada implementación cuenta con documentación específica sobre su funcionamiento, configuración y ejecución. Este README presenta la organización general; los detalles de control y comunicación se describen en los documentos de cada variante.
