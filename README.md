# RoboSoccer EIE-PUCV
Proyecto de RoboSoccer para la asignatura de Seminario de Proyectos del año 2023

Para el archivo roso.cpp

g++ -std=c++11 -o roso roso.cpp `pkg-config --cflags --libs opencv4` -lboost_system -lboost_thread -pthread -lyaml-cpp -fopenmp

Para el archivo calibration.cpp

g++ -o calibration calibration.cpp `pkg-config --cflags --libs opencv4` -lyaml-cpp -fopenmp
