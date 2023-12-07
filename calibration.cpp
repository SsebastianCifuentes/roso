/*  PROYECTO DE VISION ARTIFICIAL PARA LA ASIGNATURA DE ROBOSOCCER 2023

    AUTOR: SEBASTIAN CIFUENTES PEREZ
    PROFESOR GUIA: DANIEL YUNGE
    CORREFERENTE: GUILLERMO CID

    Este codigo se encarga de la deteccion de robots en tiempo real
    y la visualizacion de la deteccion realizada para posterior
    calibracion de colores o tamaños.

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

// Estructura para almacenar los parámetros
struct DetectionParameters {
    int CAMERA_WIDTH;
    int CAMERA_HEIGHT;
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

int main() {
    cv::VideoCapture cap(0, cv::CAP_V4L);
    
    // Cargar los parámetros desde el archivo YAML
    DetectionParameters params = loadParameters("parameters.yaml");

    cap.set(cv::CAP_PROP_FRAME_WIDTH, params.CAMERA_WIDTH);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, params.CAMERA_HEIGHT);

    if (!cap.isOpened()) {
        std::cerr << "Error al abrir la cámara." << std::endl;
        return 1;
    }

    cv::namedWindow("Detección de Círculos", cv::WINDOW_NORMAL);

    // Inicializacion de variables de frame, mascaras separadas y combinadas
    cv::Mat mask_red, mask_red2, mask_blue, mask_yellow, mask_green, mask_skyblue, mask_orange, mask_white, combinedSmallMask, combinedBigMask;
    cv::Mat frame; cap >> frame; 

    cv::Rect roi = cv::selectROI("Detección de Círculos", frame);

    int frames = 0;
    auto start = std::chrono::high_resolution_clock::now();

    while (true) {
        cap >> frame; // Captura un fotograma de la cámara

        if (frame.empty()) {
            std::cerr << "Error al capturar un fotograma." << std::endl;
            break;
        }

        // Obtén el área de interés (ROI) de la imagen
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

        // Permite la visualizacion de las mascaras en tiempo real
        //cv::imshow("Máscara Roja", mask_red);
        //cv::imshow("Máscara Azul", mask_blue);
        //cv::imshow("Máscara Amarilla", mask_yellow);
        //cv::imshow("Máscara Verde", mask_green);
        //cv::imshow("Máscara Celeste", mask_skyblue);
        //cv::imshow("Máscara Naranja", mask_orange);
        //cv::imshow("Mascara Blanca", mask_white);
        //cv::imshow("Mascaras", combined_mask);
        //cv::imshow("Mascaras Grandes", combinedBigMask);
        //cv::imshow("Mascaras Pequeñas", combinedSmallMask);

        // Detecta círculos grandes (rojo o azul)
        std::vector<cv::Vec3f> bigCircles;
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
                cv::Point smallCenter(cvRound(smallCircles[0][0]), cvRound(smallCircles[0][1]));
                int smallRadius = cvRound(smallCircles[0][2]);

                // Etiqueta el círculo grande y pequeño
                std::string bigColor, smallColor;
                cv::Scalar teamColor, playerColor;

                uchar redPixel = mask_red.at<uchar>(bigCenter.y, bigCenter.x);
                uchar bluePixel = mask_blue.at<uchar>(bigCenter.y, bigCenter.x);
                uchar yellowPixel = mask_yellow.at<uchar>(roiRect.y + smallCenter.y, roiRect.x + smallCenter.x);
                uchar greenPixel = mask_green.at<uchar>(roiRect.y + smallCenter.y, roiRect.x + smallCenter.x);

                if (redPixel > 0) {
                    bigColor = "Equipo Rojo";
                    teamColor = cv::Scalar(0, 0, 255);
                    uchar skybluePixel = mask_skyblue.at<uchar>(roiRect.y + smallCenter.y, roiRect.x + smallCenter.x);

                    if (yellowPixel > 0) {
                        smallColor = "Jugador 1";
                        playerColor = cv::Scalar(0, 255, 255);
                        }
                    else if (greenPixel > 0) {
                        smallColor = "Jugador 2";
                        playerColor = cv::Scalar(0, 255, 0);
                        }
                    else if (skybluePixel > 0) {
                        smallColor = "Jugador 3";
                        playerColor = cv::Scalar(255, 255, 0);
                        }
                    else {
                        smallColor = "Jugador desconocido";
                        playerColor = cv::Scalar(255, 255, 255);
                        }

                } else if (bluePixel > 0) {
                    bigColor = "Equipo Azul";
                    teamColor = cv::Scalar(255, 0, 0);
                    uchar orangePixel = mask_orange.at<uchar>(roiRect.y + smallCenter.y, roiRect.x + smallCenter.x);

                    if (yellowPixel > 0) {
                        smallColor = "Jugador 1";
                        playerColor = cv::Scalar(0, 255, 255);
                        }
                    else if (greenPixel > 0) {
                        smallColor = "Jugador 2";
                        playerColor = cv::Scalar(0, 255, 0);
                        }
                    else if (orangePixel > 0) {
                        smallColor = "Jugador 3";
                        playerColor = cv::Scalar(0, 128, 255);
                        }
                    else {
                        smallColor = "Jugador desconocido";
                        playerColor = cv::Scalar(255, 255, 255);
                        }

                } else {
                    bigColor = "Equipo desconocido";
                    teamColor = cv::Scalar(255, 255, 255);
                }

                std::string label = bigColor + ", " + smallColor;
                cv::circle(roiFrame, bigCenter, bigRadius, cv::Scalar(0, 255, 0), 2);
                cv::circle(roiFrame, cv::Point(bigCenter.x + smallCenter.x - bigRadius, bigCenter.y + smallCenter.y - bigRadius), smallRadius, cv::Scalar(0, 255, 0), 2);

                size_t commaPos = label.find(", ");
                cv::putText(roiFrame, label.substr(0, commaPos), bigCenter, cv::FONT_HERSHEY_DUPLEX, params.FONT_SCALE, teamColor, params.TEXT_THICKNESS);
                cv::putText(roiFrame, label.substr(commaPos + 2), cv::Point(bigCenter.x, bigCenter.y + 20), cv::FONT_HERSHEY_DUPLEX, params.FONT_SCALE, playerColor, params.TEXT_THICKNESS);
            }
        }

        // Detecta y etiqueta la pelota naranja
        std::vector<cv::Vec3f> whiteCircles;
        cv::HoughCircles(mask_white, whiteCircles, cv::HOUGH_GRADIENT, params.DP, params.MIN_DIST, params.PARAM1_BALL, params.PARAM2_BALL, params.MIN_RADIUS_BALL, params.MAX_RADIUS_BALL);
        if (!whiteCircles.empty()) {
            cv::Point mediumCenter(cvRound(whiteCircles[0][0]), cvRound(whiteCircles[0][1]));
            int mediumRadius = cvRound(whiteCircles[0][2]);
            std::string label = "Pelota";
            cv::circle(roiFrame, mediumCenter, mediumRadius, cv::Scalar(255, 255, 255), 2);
            cv::putText(roiFrame, label, cv::Point(mediumCenter.x - mediumRadius, mediumCenter.y - mediumRadius - 10), cv::FONT_HERSHEY_DUPLEX, params.FONT_SCALE, cv::Scalar(255, 255, 255), params.TEXT_THICKNESS);
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

        // Muestra la imagen con las etiquetas
        cv::imshow("Detección de Círculos", roiFrame);

        char key = cv::waitKey(1); 
        if (key == 27)
            break;
    }

    // Libera la captura y cierra las ventanas
    cap.release();
    cv::destroyAllWindows();
    return 0;
}
