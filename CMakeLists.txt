cmake_minimum_required(VERSION 3.10)
project(RoboSoccer)

# Establece el estándar de C++
set(CMAKE_CXX_STANDARD 11)

# Encontrar las librerías necesarias
find_package(OpenCV 4.10 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(OpenMP REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Protobuf REQUIRED)

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

# Generar los archivos Protobuf en el directorio src
foreach(_proto_file ${PROTO_FILES})
    # Obtener el nombre del archivo sin extensión
    get_filename_component(_proto_filename ${_proto_file} NAME_WE)
    # Establecer las rutas de los archivos generados
    set(_generated_src "${CMAKE_SOURCE_DIR}/src/${_proto_filename}.pb.cc")
    set(_generated_hdr "${CMAKE_SOURCE_DIR}/src/${_proto_filename}.pb.h")
    # Añadir el comando personalizado para generar los archivos
    add_custom_command(
        OUTPUT ${_generated_src} ${_generated_hdr}
        COMMAND ${Protobuf_PROTOC_EXECUTABLE}
        ARGS --cpp_out=${CMAKE_SOURCE_DIR}/src -I ${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/${_proto_file}
        DEPENDS ${CMAKE_SOURCE_DIR}/${_proto_file}
        COMMENT "Generando archivos Protobuf para ${_proto_file}"
    )
    # Añadir los archivos generados a las listas
    list(APPEND PROTO_SRCS ${_generated_src})
    list(APPEND PROTO_HDRS ${_generated_hdr})
endforeach()

# Marcar los archivos generados como generados
set_source_files_properties(${PROTO_SRCS} ${PROTO_HDRS} PROPERTIES GENERATED TRUE)

# Añadir ejecutables desde el directorio src
add_executable(roso src/roso.cpp ${PROTO_SRCS})
add_executable(calibration src/calibration.cpp)
add_executable(receiver src/receiver.cpp ${PROTO_SRCS})

# Enlazar las librerías necesarias para cada ejecutable
target_link_libraries(roso PUBLIC
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    yaml-cpp
    pthread
    ${Protobuf_LIBRARIES}
)

target_link_libraries(calibration PUBLIC
    ${OpenCV_LIBS}
    yaml-cpp
    pthread
)

target_link_libraries(receiver PUBLIC
    ${Boost_LIBRARIES}
    pthread
    ${Protobuf_LIBRARIES}
)

# Enlazar OpenMP si está disponible
if (OpenMP_CXX_FOUND)
    target_link_libraries(roso PUBLIC OpenMP::OpenMP_CXX)
    target_link_libraries(calibration PUBLIC OpenMP::OpenMP_CXX)
    target_link_libraries(receiver PUBLIC OpenMP::OpenMP_CXX)
endif()
