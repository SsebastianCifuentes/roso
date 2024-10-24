cmake_minimum_required(VERSION 3.10)
project(RoboSoccer)

# Establece el estándar de C++
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Encontrar las librerías necesarias
find_package(OpenCV 4.10 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenMP REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Threads REQUIRED)

# Añadir directorios de inclusión
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${yaml-cpp_INCLUDE_DIRS}
    ${Protobuf_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/src
)

# Añadir los archivos .proto
set(PROTO_FILES src/detection.proto)

# Generar los archivos Protobuf
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

# Añadir ejecutables desde el directorio src
add_executable(roso src/roso.cpp ${PROTO_SRCS})
add_executable(calibration src/calibration.cpp)
add_executable(receiver src/receiver.cpp ${PROTO_SRCS})

# Enlazar las librerías necesarias para cada ejecutable
target_link_libraries(roso PUBLIC
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    yaml-cpp
    Threads::Threads
    ${Protobuf_LIBRARIES}
)

target_link_libraries(calibration PUBLIC
    ${OpenCV_LIBS}
    yaml-cpp
    Threads::Threads
)

target_link_libraries(receiver PUBLIC
    ${Boost_LIBRARIES}
    Threads::Threads
    ${Protobuf_LIBRARIES}
)

# Enlazar OpenMP si está disponible
if (OpenMP_CXX_FOUND)
    target_link_libraries(roso PUBLIC OpenMP::OpenMP_CXX)
    target_link_libraries(calibration PUBLIC OpenMP::OpenMP_CXX)
    target_link_libraries(receiver PUBLIC OpenMP::OpenMP_CXX)
endif()
