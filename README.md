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
> El archivo de configuración para ambos codigos es el de parameters.yaml. Para el algoritmo del servidor se utiliza config.json

## Funcionamiento del algoritmo

## Compilación del algoritmo utilizando g++

Para el archivo roso.cpp
```
g++ -std=c++11 -o roso roso.cpp `pkg-config --cflags --libs opencv4` -lboost_system -lboost_thread -pthread -lyaml-cpp -fopenmp
```
Para el archivo calibration.cpp

```
g++ -o calibration calibration.cpp `pkg-config --cflags --libs opencv4` -lyaml-cpp -fopenmp
```


> Text that is a quote

> [!NOTE]
> Highlights information that users should take into account, even when skimming.

> [!TIP]
> Optional information to help a user be more successful.

> [!IMPORTANT]
> Crucial information necessary for users to succeed.

> [!WARNING]
> Critical content demanding immediate user attention due to potential risks.

> [!CAUTION]
> Negative potential consequences of an action.







