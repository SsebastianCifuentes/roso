# RoboSoccer LabSens-PUCV

![Alt text](images/labsens.png)

## Propósito del algoritmo
El algoritmo tiene como propósito ser la visión artificial de una cámara ubicada en altura, la cual debe ser capaz de detectar a los diferentes robots dentro del campo de juego. Estos robots poseen dos círculos en su cabecera, uno grande el cual sirve para diferenciar a los equipos, y uno pequeño, el cual sirve para diferenciar a los jugadores. Adicionalmente también debe ser capaz de detectar una pelota en el campo de juego.

![Alt text](images/cancha.png)

## Librerías utilizadas
Se utilizaron un par de librerías, las cuales son la base del proyecto actual, las cuales son las siguientes:
- OpenCV
- Protobuf
- YAML
- BOOST
- Iostream
- Fstream

## Funcionamiento del algoritmo
Existen dos algoritmos desarrollados en C++ los cuales son similares, pero poseen diferentes funcionalidades.
- calibration.cpp
    - Este algoritmo permite al usuario visualizar mediante una interfaz en Ubuntu lo que está observando la cámara en tiempo real, con el motivo de realizar una calibración de los colores, verificar los parámetros de Hough o lo que estime conveniente, ya que podrá ver la detección en tiempo real.
- roso.cpp
    - Este algoritmo permite al usuario enviar datos a una dirección IP y puerto establecidos en el archivo de configuración, en donde se hace un envío de las posiciones en el campo de juego de todos los robots en adición con sus ángulos respectivos y las coordenadas de la pelota.
- receiver.cpp
  - Este algoritmo recibe las coordenadas de los robots empaquetadas y las lee en tiempo real. Este es el codigo base para empezar los movimientos con los robots.

> [!IMPORTANT]
> El archivo de configuración para todos los códigos es el de parameters.yaml.

## Cámara
Este algoritmo está desarrollado con la cámara C922 PRO de Logitech, la cual entrega una resolución máxima de 1920x1080 a 60 FPS en el entorno de Ubuntu. En un principio esta camara dada las actualizaciones estaba limitada a ser utilizada con el software de Logitech, pero con el tiempo se pudo integrar al archivo de configuracion.

> [!NOTE]
> Al ejecutar el algoritmo muchas veces, la cámara tiende a colocarse más oscura, por lo que es necesario desconectarla y volverla a conectar al entorno de Ubuntu.

## Paginas utilizadas para realizar la calibración de colores
Para realizar la calibración, el procedimiento consiste en capturar diferentes fotogramas en tiempo real de la cámara y almacenarlos. Luego mediante el [Selector de color en imagen](https://imagecolorpicker.com/es) se carga la imagen y se toma el valor en HEX de los diferentes pixeles para cada color a calibrar. Posteriormente se utiliza un [Convertidor de color (HEX a HSV)](https://www.peko-step.com/es/tool/hsvrgb.html) en donde se transforma de HEX a HSV, el cual utiliza el algoritmo en cuestión.

> [!NOTE]
> En la página los valores en HSV deben estar de H: 0-360, S: 0-255, V: 0-255 ya que así es más fácil trabajarlos. Adicionalmente la escala que usa OpenCV para los valores de HSV es distinta, donde recomiendo revisar el siguiente enlace [OpenCV: Changing Colorspaces](https://docs.opencv.org/3.4/df/d9d/tutorial_py_colorspaces.html).

## Instalación
```
git clone https://github.com/SsebastianCifuentes/roso.git
cd repo/build
make
./calibration
```

## Ejecución del algoritmo
1. Conectar la cámara web al computador a utilizar.
2. Iniciar la máquina virtual con el entorno Ubuntu.
3. Verificar que todas las librerías se encuentren instaladas.
4. Habilitar la cámara web en el entorno Ubuntu.
5. Compilar el algoritmo (Opcional).
6. Configurar archivo de configuracion dependiendo del entorno (Opcional)
7. Ejecutar el algoritmo.
8. Seleccionar el ROI (Región de interés) y presionar Enter. (En el caso de calibration)
9. Verificar si la detección o envío de datos es el correcto.

## Posibles mejoras del algoritmo actual
1. Adquirir una cámara dedicada con alta tasa de fotogramas y compatible con Ubuntu.
2. Separar el cálculo interno de la función HoughCircles para la detección de círculos pequeños.
4. Para los robots, se recomienda el uso de RF en vez de BT debido a que no puede fijarse en canales de frecuencia. Para más información revisar el siguiente [enlace](https://robocup-ssl.github.io/ssl-rules/sslrules.html#_vision).
5. Buscar una forma de que se calculen ROIs para cada robot en base a su coordenada anterior para ahorrar memoria.
6. Implementar YOLO o IA para mejorar detección.
7. Tratar de cambiar el formato de los círculos en base a los colores de la RoboCup SSL.
