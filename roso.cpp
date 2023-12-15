/*  PROYECTO DE VISION ARTIFICIAL PARA LA ASIGNATURA DE ROBOSOCCER 2023

    AUTOR: SEBASTIAN CIFUENTES PEREZ
    PROFESOR GUIA: DANIEL YUNGE
    CORREFERENTE: GUILLERMO CID

    Este codigo se encarga de la deteccion de robots en tiempo real
    y el envio de datos hacia un servidor para posterior ejecucion
    de movimientos de los robots.

    Los parametros de HoughCircles estan adaptados a las condiciones
    del laboratorio LabSens, con iluminacion blanca constante, con
    la camara Logitech C922 ubicada a 2[m] de altura de la cancha y 
    con diametros de circulos:
    -Grandes: 110 [mm]
    -Pequeños: 30 [mm]
    -Pelota: 44 [mm] (Ping pong tradicional) */

#include <opencv2/opencv.hpp>
#include <chrono>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <boost/asio.hpp>

// Estructura para almacenar los parámetros
struct DetectionParameters {
    int CAMERA_WIDTH;
    int CAMERA_HEIGHT;
    std::string IP;
    std::string PORT;
    cv::Scalar LOWER_RED, UPPER_RED, LOWER_RED2, UPPER_RED2;
    cv::Scalar LOWER_BLUE, UPPER_BLUE;
    cv::Scalar LOWER_YELLOW, UPPER_YELLOW;
    cv::Scalar LOWER_GREEN, UPPER_GREEN;
    cv::Scalar LOWER_SKYBLUE, UPPER_SKYBLUE;
    cv::Scalar LOWER_ORANGE, UPPER_ORANGE;
    cv::Scalar LOWER_WHITE, UPPER_WHITE;
    double DP, MIN_DIST;
    int PARAM1_BIG, PARAM2_BIG, MIN_RADIUS_BIG, MAX_RADIUS_BIG;
    int PARAM1_SMALL, PARAM2_SMALL, MIN_RADIUS_SMALL, MAX_RADIUS_SMALL;
    int PARAM1_BALL, PARAM2_BALL, MIN_RADIUS_BALL, MAX_RADIUS_BALL;
    int KERNELSIZEBIG, KERNELSIZEMEDIUM, KERNELSIZESMALL;
    double SIGMA;
    float FONT_SCALE, TEXT_THICKNESS;
};

// Función para cargar los parámetros desde un archivo YAML
DetectionParameters loadParameters(const std::string& filename) {
    DetectionParameters params;

    try {
        YAML::Node config = YAML::LoadFile(filename);

        params.CAMERA_WIDTH = config["camera"]["width"].as<int>();
        params.CAMERA_HEIGHT = config["camera"]["height"].as<int>();

        params.IP = config["server"]["ip"].as<std::string>(); 
        params.PORT = config["server"]["port"].as<std::string>();  

        params.LOWER_RED = cv::Scalar(config["color"]["red"]["lower"][0].as<int>(),config["color"]["red"]["lower"][1].as<int>(),config["color"]["red"]["lower"][2].as<int>());
        params.UPPER_RED = cv::Scalar(config["color"]["red"]["upper"][0].as<int>(),config["color"]["red"]["upper"][1].as<int>(),config["color"]["red"]["upper"][2].as<int>());
        params.LOWER_RED2 = cv::Scalar(config["color"]["red"]["lower2"][0].as<int>(),config["color"]["red"]["lower2"][1].as<int>(),config["color"]["red"]["lower2"][2].as<int>());
        params.UPPER_RED2 = cv::Scalar(config["color"]["red"]["upper2"][0].as<int>(),config["color"]["red"]["upper2"][1].as<int>(),config["color"]["red"]["upper2"][2].as<int>());
        
        params.LOWER_BLUE = cv::Scalar(config["color"]["blue"]["lower"][0].as<int>(),config["color"]["blue"]["lower"][1].as<int>(),config["color"]["blue"]["lower"][2].as<int>());
        params.UPPER_BLUE = cv::Scalar(config["color"]["blue"]["upper"][0].as<int>(),config["color"]["blue"]["upper"][1].as<int>(),config["color"]["blue"]["upper"][2].as<int>());
        
        params.LOWER_YELLOW = cv::Scalar(config["color"]["yellow"]["lower"][0].as<int>(),config["color"]["yellow"]["lower"][1].as<int>(),config["color"]["yellow"]["lower"][2].as<int>());
        params.UPPER_YELLOW = cv::Scalar(config["color"]["yellow"]["upper"][0].as<int>(),config["color"]["yellow"]["upper"][1].as<int>(),config["color"]["yellow"]["upper"][2].as<int>());
        
        params.LOWER_GREEN = cv::Scalar(config["color"]["green"]["lower"][0].as<int>(),config["color"]["green"]["lower"][1].as<int>(),config["color"]["green"]["lower"][2].as<int>());
        params.UPPER_GREEN = cv::Scalar(config["color"]["green"]["upper"][0].as<int>(),config["color"]["green"]["upper"][1].as<int>(),config["color"]["green"]["upper"][2].as<int>());
        
        params.LOWER_SKYBLUE = cv::Scalar(config["color"]["skyblue"]["lower"][0].as<int>(),config["color"]["skyblue"]["lower"][1].as<int>(),config["color"]["skyblue"]["lower"][2].as<int>());
        params.UPPER_SKYBLUE = cv::Scalar(config["color"]["skyblue"]["upper"][0].as<int>(),config["color"]["skyblue"]["upper"][1].as<int>(),config["color"]["skyblue"]["upper"][2].as<int>());
        
        params.LOWER_ORANGE = cv::Scalar(config["color"]["orange"]["lower"][0].as<int>(),config["color"]["orange"]["lower"][1].as<int>(),config["color"]["orange"]["lower"][2].as<int>());
        params.UPPER_ORANGE = cv::Scalar(config["color"]["orange"]["upper"][0].as<int>(),config["color"]["orange"]["upper"][1].as<int>(),config["color"]["orange"]["upper"][2].as<int>());
        
        params.LOWER_WHITE = cv::Scalar(config["color"]["white"]["lower"][0].as<int>(),config["color"]["white"]["lower"][1].as<int>(),config["color"]["white"]["lower"][2].as<int>());
        params.UPPER_WHITE = cv::Scalar(config["color"]["white"]["upper"][0].as<int>(),config["color"]["white"]["upper"][1].as<int>(),config["color"]["white"]["upper"][2].as<int>());
        
        params.DP = config["hough"]["dp"].as<double>();
        params.MIN_DIST = config["hough"]["min_dist"].as<double>();

        params.PARAM1_BIG = config["hough"]["param1_big"].as<int>();
        params.PARAM2_BIG = config["hough"]["param2_big"].as<int>();
        params.MIN_RADIUS_BIG = config["hough"]["min_radius_big"].as<int>();
        params.MAX_RADIUS_BIG = config["hough"]["max_radius_big"].as<int>();

        params.PARAM1_SMALL = config["hough"]["param1_small"].as<int>();
        params.PARAM2_SMALL = config["hough"]["param2_small"].as<int>();
        params.MIN_RADIUS_SMALL = config["hough"]["min_radius_small"].as<int>();
        params.MAX_RADIUS_SMALL = config["hough"]["max_radius_small"].as<int>();

        params.PARAM1_BALL = config["hough"]["param1_ball"].as<int>();
        params.PARAM2_BALL = config["hough"]["param2_ball"].as<int>();
        params.MIN_RADIUS_BALL = config["hough"]["min_radius_ball"].as<int>();
        params.MAX_RADIUS_BALL = config["hough"]["max_radius_ball"].as<int>();

        params.KERNELSIZEBIG = config["gaussian"]["kernel_size_big"].as<int>();
        params.KERNELSIZEMEDIUM = config["gaussian"]["kernel_size_medium"].as<int>();
        params.KERNELSIZESMALL = config["gaussian"]["kernel_size_small"].as<int>();

        params.SIGMA = config["gaussian"]["sigma"].as<double>();

        params.FONT_SCALE = config["font_size"]["scale"].as<float>();
        params.TEXT_THICKNESS = config["font_size"]["thickness"].as<float>();

    } catch (const std::exception& e) {
        std::cerr << "Error al cargar los parámetros desde el archivo YAML: " << e.what() << std::endl;
    }

    return params;
}

using boost::asio::ip::udp;

// Función para enviar un mensaje al servidor usando UDP.
bool send_message_to_server(const std::string& host, const std::string& port, const std::string& message) {
    try {
        boost::asio::io_context io_context;
        udp::resolver resolver(io_context);
        udp::socket socket(io_context);

        auto endpoints = resolver.resolve(udp::v4(), host, port);
        
        // Utiliza UDP en lugar de TCP
        socket.open(udp::v4());

        // Envia el mensaje al servidor
        socket.send_to(boost::asio::buffer(message), *endpoints.begin());
        
        std::cout << "Mensaje enviado por UDP:" << "\n\n" << message << std::endl;
        return true;
    }
    catch (std::exception& e) {
        std::cerr << "Excepción: " << e.what() << std::endl;
        return false;
    }
}

// Función para inicializar la camara
bool initializeCamera(cv::VideoCapture& cap, int width, int height) {
    cap.open(0, cv::CAP_V4L); // Abre la cámara predeterminada
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);

    if (!cap.isOpened()) {
        std::cerr << "Error al abrir la cámara." << std::endl;
        return false;
    }
    return true;
}

// Función para capturar un frame
bool captureFrame(cv::VideoCapture& cap, cv::Mat& frame) {
    cap >> frame; // Captura un nuevo fotograma

    if (frame.empty()) {
        std::cerr << "Error al capturar un fotograma." << std::endl;
        return false;
    }
    return true;
}

int main() {
    // Cargar los parámetros desde el archivo YAML
    DetectionParameters params = loadParameters("parameters.yaml");

    cv::VideoCapture cap;
    if (!initializeCamera(cap, params.CAMERA_WIDTH, params.CAMERA_HEIGHT)) {
        return 1; // Si la cámara no pudo ser inicializada, termina el programa
    }

    // Inicializacion de variables de frame, mascaras separadas y combinadas
    cv::Mat frame, mask_red, mask_red2, mask_blue, mask_yellow, mask_green, mask_skyblue, mask_orange, mask_white, combinedSmallMask, combinedBigMask;

    if (!captureFrame(cap, frame)) {
        return 1; // Si no se pudo capturar un fotograma, termina el programa
    }

    // Permite al usuario seleccionar un área de interés (ROI)
    cv::Rect roi = cv::selectROI("Detección de Círculos", frame);
    cv::destroyWindow("Detección de Círculos");

    // Permite realizar el conteo de FPS
    int frames = 0;
    auto start = std::chrono::high_resolution_clock::now();
    
    while (true) {
        if (!captureFrame(cap, frame)) {
            break; // Si no se pudo capturar un fotograma, sale del bucle
        }

        cv::Mat hsv, roiFrame = frame(roi);
        cv::cvtColor(roiFrame, hsv, cv::COLOR_BGR2HSV);

        // Define los rangos de colores y sus mascaras en paralelo
        #pragma omp parallel sections
        {
            #pragma omp section
            {
                cv::inRange(hsv, params.LOWER_RED, params.UPPER_RED, mask_red);
                cv::inRange(hsv, params.LOWER_RED2, params.UPPER_RED2, mask_red2);
            }
            #pragma omp section
            {
                cv::inRange(hsv, params.LOWER_BLUE, params.UPPER_BLUE, mask_blue);
                cv::inRange(hsv, params.LOWER_YELLOW, params.UPPER_YELLOW, mask_yellow);
            }
            #pragma omp section
            {
                cv::inRange(hsv, params.LOWER_GREEN, params.UPPER_GREEN, mask_green);
                cv::inRange(hsv, params.LOWER_SKYBLUE, params.UPPER_SKYBLUE, mask_skyblue);
            }
            #pragma omp section
            {
                cv::inRange(hsv, params.LOWER_ORANGE, params.UPPER_ORANGE, mask_orange);
                cv::inRange(hsv, params.LOWER_WHITE, params.UPPER_WHITE, mask_white);
            }
        }

        cv::bitwise_or(mask_red, mask_red2, mask_red);

        //Define las mascaras grandes, pequeñas y para la pelota en paralelo
        #pragma omp parallel sections
        {
            #pragma omp section
            {   
                combinedSmallMask = mask_yellow + mask_green + mask_skyblue + mask_orange;
                cv::GaussianBlur(combinedSmallMask, combinedSmallMask, cv::Size(params.KERNELSIZESMALL, params.KERNELSIZESMALL), params.SIGMA);
            }
            #pragma omp section
            {
                combinedBigMask = mask_red + mask_blue;
                cv::GaussianBlur(combinedBigMask, combinedBigMask, cv::Size(params.KERNELSIZEBIG, params.KERNELSIZEBIG), params.SIGMA);
            }
            #pragma omp section
            {
                cv::GaussianBlur(mask_white, mask_white, cv::Size(params.KERNELSIZEMEDIUM, params.KERNELSIZEMEDIUM), params.SIGMA);
            }
        }

        // Inicializacion de variables para almacenar posiciones de los robots
        std::vector<cv::Vec3f> bigCircles,redTeam1, redTeam2, redTeam3, blueTeam1, blueTeam2, blueTeam3;
        cv::HoughCircles(combinedBigMask, bigCircles, cv::HOUGH_GRADIENT, params.DP, params.MIN_DIST, params.PARAM1_BIG, params.PARAM2_BIG, params.MIN_RADIUS_BIG, params.MAX_RADIUS_BIG);
        
        // Dibuja y etiqueta los círculos grandes y busca solo un círculo pequeño dentro de ellos
        for (size_t i = 0; i < bigCircles.size(); i++) {
            cv::Point bigCenter(cvRound(bigCircles[i][0]), cvRound(bigCircles[i][1]));
            int bigRadius = cvRound(bigCircles[i][2]);

            // Detecta círculos pequeños dentro del círculo grande
            cv::Rect roiRect(bigCenter.x - bigRadius, bigCenter.y - bigRadius, bigRadius * 2, bigRadius * 2);
            cv::Mat roiMask = combinedSmallMask(roiRect);
            std::vector<cv::Vec3f> smallCircles;
            cv::HoughCircles(roiMask, smallCircles, cv::HOUGH_GRADIENT, params.DP, params.MIN_DIST, params.PARAM1_SMALL, params.PARAM2_SMALL, params.MIN_RADIUS_SMALL, params.MAX_RADIUS_SMALL);

            // Si se encuentra un círculo pequeño dentro del círculo grande, deja de buscar más círculos pequeños
            if (!smallCircles.empty()) {
                cv::Point smallCenter(cvRound(smallCircles[0][0])+ roiRect.x, cvRound(smallCircles[0][1]) + roiRect.y);
                int smallRadius = cvRound(smallCircles[0][2]);

                // Etiqueta el círculo grande y pequeño
                uchar redPixel = mask_red.at<uchar>(bigCenter.y, bigCenter.x);
                uchar bluePixel = mask_blue.at<uchar>(bigCenter.y, bigCenter.x);
                uchar yellowPixel = mask_yellow.at<uchar>(smallCenter.y, smallCenter.x);
                uchar greenPixel = mask_green.at<uchar>(smallCenter.y, smallCenter.x);
                
                // Angulo entre C. grande y C pequeño
                float angle = atan2(smallCenter.y - bigCenter.y, smallCenter.x - bigCenter.x) * 180 / CV_PI;
                if (angle < 0) {angle += 360;}

                if (redPixel > 0) {
                    uchar skybluePixel = mask_skyblue.at<uchar>(smallCenter.y, smallCenter.x);

                    if (yellowPixel > 0) {
                        redTeam1.push_back(cv::Vec3f(bigCenter.x, bigCenter.y, angle));
                    } else if (greenPixel > 0) {
                        redTeam2.push_back(cv::Vec3f(bigCenter.x, bigCenter.y, angle));
                    } else if (skybluePixel > 0) {
                        redTeam3.push_back(cv::Vec3f(bigCenter.x, bigCenter.y, angle));
                    }
                } else if (bluePixel > 0) {
                    uchar orangePixel = mask_orange.at<uchar>(smallCenter.y, smallCenter.x);

                    if (yellowPixel > 0) {
                        blueTeam1.push_back(cv::Vec3f(bigCenter.x, bigCenter.y, angle));
                    } else if (greenPixel > 0) {
                        blueTeam2.push_back(cv::Vec3f(bigCenter.x, bigCenter.y, angle));
                    } else if (orangePixel > 0) {
                        blueTeam3.push_back(cv::Vec3f(bigCenter.x, bigCenter.y, angle));
                    }
                }
            }
        }

        // Detecta y almacena la posición de la pelota
        std::vector<cv::Vec3f> whiteCircles;
        std::vector<cv::Point> ballPositions;
        cv::HoughCircles(mask_white, whiteCircles, cv::HOUGH_GRADIENT, params.DP, params.MIN_DIST, params.PARAM1_BALL, params.PARAM2_BALL, params.MIN_RADIUS_BALL, params.MAX_RADIUS_BALL);

        if (!whiteCircles.empty()) {
            cv::Point mediumCenter(cvRound(whiteCircles[0][0]), cvRound(whiteCircles[0][1]));
            int mediumRadius = cvRound(whiteCircles[0][2]);
            ballPositions.push_back(cv::Point(mediumCenter.x, mediumCenter.y));
        }

        // Imprime las coordenadas del jugador y la pelota
        std::stringstream buffer;
        buffer << "-----Equipo Rojo-----\n";
        buffer << " Jugador 1: (" << (redTeam1.size() > 0 ? std::to_string(redTeam1.back()[0]) + ", " + std::to_string(redTeam1.back()[1]) + ", " + std::to_string(redTeam1.back()[2]) : "Desconocido") << ")\n";
        buffer << " Jugador 2: (" << (redTeam2.size() > 0 ? std::to_string(redTeam2.back()[0]) + ", " + std::to_string(redTeam2.back()[1]) + ", " + std::to_string(redTeam2.back()[2]) : "Desconocido") << ")\n";
        buffer << " Jugador 3: (" << (redTeam3.size() > 0 ? std::to_string(redTeam3.back()[0]) + ", " + std::to_string(redTeam3.back()[1]) + ", " + std::to_string(redTeam3.back()[2]) : "Desconocido") << ")\n";
        buffer << "\n-----Equipo Azul-----\n";
        buffer << " Jugador 1: (" << (blueTeam1.size() > 0 ? std::to_string(blueTeam1.back()[0]) + ", " + std::to_string(blueTeam1.back()[1]) + ", " + std::to_string(blueTeam1.back()[2]) : "Desconocido") << ")\n";
        buffer << " Jugador 2: (" << (blueTeam2.size() > 0 ? std::to_string(blueTeam2.back()[0]) + ", " + std::to_string(blueTeam2.back()[1]) + ", " + std::to_string(blueTeam2.back()[2]) : "Desconocido") << ")\n";
        buffer << " Jugador 3: (" << (blueTeam3.size() > 0 ? std::to_string(blueTeam3.back()[0]) + ", " + std::to_string(blueTeam3.back()[1]) + ", " + std::to_string(blueTeam3.back()[2]) : "Desconocido") << ")\n";
        buffer << "\n-----Pelota-----\n";
        buffer << " Pelota: (" << (ballPositions.size() > 0 ? std::to_string(ballPositions.back().x) + ", " + std::to_string(ballPositions.back().y) : "Desconocido") << ")\n";
        std::cout << buffer.str() << std::endl;

        // Enviar mensaje al servidor usando UDP
        if (send_message_to_server(params.IP, params.PORT, buffer.str())) {
            std::cout << "Coordenadas enviadas exitosamente" << std::endl;
        } else {
            std::cout << "No se pudo enviar las coordenadas al servidor" << std::endl;
        }

        // Calcula el tiempo actual y el tiempo de ejecucion
        frames++;
        auto end = std::chrono::high_resolution_clock::now(); 
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();

        // Calcula y muestra el FPS
        if (duration >= 1) { 
            int fps = frames / duration;
            std::cout << "FPS: " << fps << std::endl;
            frames = 0; // Reinicia el contador de fotogramas
            start = std::chrono::high_resolution_clock::now(); // Reinicia el tiempo de inicio
        }
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
