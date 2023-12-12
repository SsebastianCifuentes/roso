# RoboSoccer LabSens-PUCV

![Alt text](images/labsens.png)

## Proposito del algoritmo
El algoritmo tiene como proposito ser la vision artificial de una camara ubicada en altura, la cual debe ser capaz de detectar a los diferentes robots dentro del campo de juego. Estos robots poseen dos circulos en su cabecera, uno grande el cual sirve para diferenciar a los equipos, y uno pequeño, el cual sirve para diferenciar a los jugadores. Adicionalmente tambien debe ser capaz de detectar una pelota en el campo de juego.

![Alt text](images/cancha.png)

## Librerías utilizadas
Se utilizaron un par de librerías, las cuales son la base del proyecto actual, las cuales son las siguientes:
- OpenCV
- Chrono
- YAML
- BOOST
- Iostream
- Fstream

## Funcionamiento del algoritmo
Existen dos algoritmos desarollados en C++ los cuales son similares pero poseen diferentes funcionalidades.
- calibration.cpp
    - Este algoritmo permite al usuario visualizar mediante una interfaz en Ubuntu lo que está observando la camara en tiempo real, con el motivo de realizar una calibracion de los colores, verificar los parametros de Hough o lo que estime conveniente, ya que podrá ver la detección en tiempo real.
- roso.cpp
    - Este algoritmo permite al usuario envíar datos a una direccion IP y puerto establecidos en el archivo de configuracion, en donde se hace un envío de las posiciones en el campo de juego de todos los robots en adición con sus angulos respectivos y las coordenadas de la pelota. 

> [!IMPORTANT]
> El archivo de configuración para ambos codigos es el de parameters.yaml. Para el algoritmo del servidor se utiliza config.json.

> [!WARNING]
> El algoritmo calibration.cpp posee un problema cuando un robot sale o entra al ROI, deteniendo el codigo. Como recomendación no se deben salir de la cancha, y si se quiere ingresar, que sea con la mano encima.

## Camara
Este algoritmo está desarrollado con la camara C922 PRO de Logitech, la cual entrega una resolución maxima de 1920x1080 a 30 FPS en el entorno de Ubuntu. Adicionalmente, esta camara no es configurable en ese entorno (brillo, saturación, etc) por lo que se opta a utilizar el software Logitech G HUB y configurar la camara con estos parametros:

![Alt text](images/camara.jpg)

> [!NOTE]
> Al ejecutar el algoritmo muchas veces, la camara tiende a colocarse mas oscura, por lo que es necesario desconectarla y volverla a conectar al entorno de Ubuntu.

## Paginas utilizadas para realizar la calibración de colores
Para realizar la calibración, el procedimiento consiste en capturar diferentes fotogramas en tiempo real de la camara y almacenarlos. Luego mediante el [Selector de color en imagen](https://imagecolorpicker.com/es) se carga la imagen y se toma el valor en HEX de los diferentes pixeles para cada color a calibrar. Posteriormente se utiliza un [Convertidor de color (HEX a HSV)](https://www.peko-step.com/es/tool/hsvrgb.html) en donde se transforma de HEX a HSV, el cual utiliza el algoritmo en cuestion.

> [!NOTE]
> En la pagina los valores en HSV deben estar de H: 0-360, S: 0-255, V: 0-255 ya que así es más facil trabajarlos. Adicionalmente la escala que usa OpenCV para los valores de HSV es distinta, donde recomiendo revisar el siguiente enlace [OpenCV: Changing Colorspaces](https://docs.opencv.org/3.4/df/d9d/tutorial_py_colorspaces.html).

## Compilación del algoritmo utilizando g++
Para el archivo roso.cpp
```
g++ -std=c++11 -o roso roso.cpp `pkg-config --cflags --libs opencv4` -lboost_system -lboost_thread -pthread -lyaml-cpp -fopenmp
```
Para el archivo calibration.cpp
```
g++ -o calibration calibration.cpp `pkg-config --cflags --libs opencv4` -lyaml-cpp -fopenmp
```

## Posibles mejoras del algoritmo actual
1. Adquirir una camara dedicada con alta tasa de fotogramas y compatible con Ubuntu.
2. Separar el calculo interno de la función HoughCircles para la detección de circulos pequeños.
3. Mejorar el codigo de calibración para que evite que se cierre al momento de que un circulo salga del ROI.
4. Para los robots, se recomienda el uso de RF en vez de BT debido a que no puede fijarse en canales de frecuencia. Para más información revisar el siguiente [enlace](https://robocup-ssl.github.io/ssl-rules/sslrules.html#_vision).
5. Buscar una forma de que se calculen ROIs para cada robot en base a su coordenada anterior para ahorrar memoria.
6. Implementar YOLO o IA para mejorar detección.
7. Modularizar el codigo.
8. Tratar de cambiar el formato de los circulos en base a los colores de la RoboCup SSL.



