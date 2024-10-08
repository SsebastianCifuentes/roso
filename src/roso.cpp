/*  PROYECTO DE VISIÓN ARTIFICIAL PARA LA ASIGNATURA DE ROBOSOCCER 2023

    AUTOR: SEBASTIÁN CIFUENTES PÉREZ
    PROFESOR GUÍA: DANIEL YUNGE
    CORREFERENTE: GUILLERMO CID

    Este código se encarga de la detección de robots en tiempo real
    y el envío de datos mediante multicast UDP utilizando Protobuf.
    El ROI se define en el archivo parameters.yaml.

    Los parámetros de HoughCircles están adaptados a las condiciones
    del laboratorio LabSens, con iluminación blanca constante, con
    la cámara Logitech C922 ubicada a 2[m] de altura de la cancha y 
    con diámetros de círculos:
    - Grandes: 110 [mm]
    - Pequeños: 30 [mm]
    - Pelota: 44 [mm] (Ping pong tradicional)
*/

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <boost/asio.hpp>
#include <omp.h> // Incluir OpenMP
#include "detection.pb.h" // Incluir el archivo generado por Protobuf

// Estructura para almacenar los parámetros
struct DetectionParameters {
    // Parámetros de la cámara
    int CAMERA_WIDTH;
    int CAMERA_HEIGHT;
    int BRIGHTNESS;
    int CONTRAST;
    int SATURATION;
    int SHARPNESS;
    int GAIN;
    int BACKLIGHT_COMPENSATION;
    bool AUTO_WHITE_BALANCE;
    int WHITE_BALANCE;
    int AUTO_EXPOSURE;  
    int EXPOSURE;
    bool AUTO_FOCUS;
    int FOCUS;
    int FPS;
    std::string FORMAT;

    // Parámetros de red
    std::string MULTICAST_ADDRESS;
    std::string PORT;

    // Definiciones de color
    cv::Scalar LOWER_RED, UPPER_RED, LOWER_RED2, UPPER_RED2;
    cv::Scalar LOWER_BLUE, UPPER_BLUE;
    cv::Scalar LOWER_YELLOW, UPPER_YELLOW;
    cv::Scalar LOWER_GREEN, UPPER_GREEN;
    cv::Scalar LOWER_SKYBLUE, UPPER_SKYBLUE;
    cv::Scalar LOWER_ORANGE, UPPER_ORANGE;
    cv::Scalar LOWER_WHITE, UPPER_WHITE;

    // Parámetros de HoughCircles
    double DP, MIN_DIST;
    int PARAM1_BIG, PARAM2_BIG, MIN_RADIUS_BIG, MAX_RADIUS_BIG;
    int PARAM1_SMALL, PARAM2_SMALL, MIN_RADIUS_SMALL, MAX_RADIUS_SMALL;
    int PARAM1_BALL, PARAM2_BALL, MIN_RADIUS_BALL, MAX_RADIUS_BALL;
    int KERNELSIZEBIG, KERNELSIZEMEDIUM, KERNELSIZESMALL;
    double SIGMA;

    // Parámetros del ROI
    int ROI_X;
    int ROI_Y;
    int ROI_WIDTH;
    int ROI_HEIGHT;
};

// Estructura para los rangos de color
struct ColorRange {
    std::string name;
    cv::Scalar lower;
    cv::Scalar upper;
    cv::Mat mask;
};

// Función para cargar los parámetros desde un archivo YAML
DetectionParameters loadParameters(const std::string& filename) {
    DetectionParameters params;

    try {
        YAML::Node config = YAML::LoadFile(filename);

        // Parámetros de la cámara
        params.CAMERA_WIDTH = config["camera"]["width"].as<int>();
        params.CAMERA_HEIGHT = config["camera"]["height"].as<int>();
        params.BRIGHTNESS = config["camera"]["brightness"].as<int>();
        params.CONTRAST = config["camera"]["contrast"].as<int>();
        params.SATURATION = config["camera"]["saturation"].as<int>();
        params.SHARPNESS = config["camera"]["sharpness"].as<int>();
        params.GAIN = config["camera"]["gain"].as<int>();
        params.BACKLIGHT_COMPENSATION = config["camera"]["backlight_compensation"].as<int>();
        params.AUTO_WHITE_BALANCE = config["camera"]["auto_white_balance"].as<bool>();
        params.WHITE_BALANCE = config["camera"]["white_balance"].as<int>();
        params.AUTO_EXPOSURE = config["camera"]["auto_exposure"].as<int>();  
        params.EXPOSURE = config["camera"]["exposure"].as<int>();
        params.AUTO_FOCUS = config["camera"]["auto_focus"].as<bool>();
        params.FOCUS = config["camera"]["focus"].as<int>();
        params.FPS = config["camera"]["fps"].as<int>();
        params.FORMAT = config["camera"]["format"].as<std::string>();

        // Parámetros de red
        params.MULTICAST_ADDRESS = config["network"]["multicast_address"].as<std::string>();
        params.PORT = config["network"]["port"].as<std::string>();

        // Definiciones de color
        params.LOWER_RED = cv::Scalar(config["color"]["red"]["lower"][0].as<int>(), config["color"]["red"]["lower"][1].as<int>(), config["color"]["red"]["lower"][2].as<int>());
        params.UPPER_RED = cv::Scalar(config["color"]["red"]["upper"][0].as<int>(), config["color"]["red"]["upper"][1].as<int>(), config["color"]["red"]["upper"][2].as<int>());
        params.LOWER_RED2 = cv::Scalar(config["color"]["red"]["lower2"][0].as<int>(), config["color"]["red"]["lower2"][1].as<int>(), config["color"]["red"]["lower2"][2].as<int>());
        params.UPPER_RED2 = cv::Scalar(config["color"]["red"]["upper2"][0].as<int>(), config["color"]["red"]["upper2"][1].as<int>(), config["color"]["red"]["upper2"][2].as<int>());

        params.LOWER_BLUE = cv::Scalar(config["color"]["blue"]["lower"][0].as<int>(), config["color"]["blue"]["lower"][1].as<int>(), config["color"]["blue"]["lower"][2].as<int>());
        params.UPPER_BLUE = cv::Scalar(config["color"]["blue"]["upper"][0].as<int>(), config["color"]["blue"]["upper"][1].as<int>(), config["color"]["blue"]["upper"][2].as<int>());

        params.LOWER_YELLOW = cv::Scalar(config["color"]["yellow"]["lower"][0].as<int>(), config["color"]["yellow"]["lower"][1].as<int>(), config["color"]["yellow"]["lower"][2].as<int>());
        params.UPPER_YELLOW = cv::Scalar(config["color"]["yellow"]["upper"][0].as<int>(), config["color"]["yellow"]["upper"][1].as<int>(), config["color"]["yellow"]["upper"][2].as<int>());

        params.LOWER_GREEN = cv::Scalar(config["color"]["green"]["lower"][0].as<int>(), config["color"]["green"]["lower"][1].as<int>(), config["color"]["green"]["lower"][2].as<int>());
        params.UPPER_GREEN = cv::Scalar(config["color"]["green"]["upper"][0].as<int>(), config["color"]["green"]["upper"][1].as<int>(), config["color"]["green"]["upper"][2].as<int>());

        params.LOWER_SKYBLUE = cv::Scalar(config["color"]["skyblue"]["lower"][0].as<int>(), config["color"]["skyblue"]["lower"][1].as<int>(), config["color"]["skyblue"]["lower"][2].as<int>());
        params.UPPER_SKYBLUE = cv::Scalar(config["color"]["skyblue"]["upper"][0].as<int>(), config["color"]["skyblue"]["upper"][1].as<int>(), config["color"]["skyblue"]["upper"][2].as<int>());

        params.LOWER_ORANGE = cv::Scalar(config["color"]["orange"]["lower"][0].as<int>(), config["color"]["orange"]["lower"][1].as<int>(), config["color"]["orange"]["lower"][2].as<int>());
        params.UPPER_ORANGE = cv::Scalar(config["color"]["orange"]["upper"][0].as<int>(), config["color"]["orange"]["upper"][1].as<int>(), config["color"]["orange"]["upper"][2].as<int>());

        params.LOWER_WHITE = cv::Scalar(config["color"]["white"]["lower"][0].as<int>(), config["color"]["white"]["lower"][1].as<int>(), config["color"]["white"]["lower"][2].as<int>());
        params.UPPER_WHITE = cv::Scalar(config["color"]["white"]["upper"][0].as<int>(), config["color"]["white"]["upper"][1].as<int>(), config["color"]["white"]["upper"][2].as<int>());

        // Parámetros de HoughCircles
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

        // Cargar los parámetros del ROI
        params.ROI_X = config["roi"]["x"].as<int>();
        params.ROI_Y = config["roi"]["y"].as<int>();
        params.ROI_WIDTH = config["roi"]["width"].as<int>();
        params.ROI_HEIGHT = config["roi"]["height"].as<int>();

    } catch (const std::exception& e) {
        std::cerr << "Error al cargar los parámetros desde el archivo YAML: " << e.what() << std::endl;
    }

    return params;
}

// Función para inicializar los rangos de color
void initializeColorRanges(const DetectionParameters& params, std::vector<ColorRange>& colorRanges) {
    colorRanges.push_back({"RED1", params.LOWER_RED, params.UPPER_RED, cv::Mat()});
    colorRanges.push_back({"RED2", params.LOWER_RED2, params.UPPER_RED2, cv::Mat()});
    colorRanges.push_back({"BLUE", params.LOWER_BLUE, params.UPPER_BLUE, cv::Mat()});
    colorRanges.push_back({"YELLOW", params.LOWER_YELLOW, params.UPPER_YELLOW, cv::Mat()});
    colorRanges.push_back({"GREEN", params.LOWER_GREEN, params.UPPER_GREEN, cv::Mat()});
    colorRanges.push_back({"SKYBLUE", params.LOWER_SKYBLUE, params.UPPER_SKYBLUE, cv::Mat()});
    colorRanges.push_back({"ORANGE", params.LOWER_ORANGE, params.UPPER_ORANGE, cv::Mat()});
    colorRanges.push_back({"WHITE", params.LOWER_WHITE, params.UPPER_WHITE, cv::Mat()});
}

// Configurar la cámara con los parámetros leídos
void configureCamera(cv::VideoCapture& cap, const DetectionParameters& params) {
    cap.set(cv::CAP_PROP_FRAME_WIDTH, params.CAMERA_WIDTH);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, params.CAMERA_HEIGHT);
    cap.set(cv::CAP_PROP_BRIGHTNESS, params.BRIGHTNESS);
    cap.set(cv::CAP_PROP_CONTRAST, params.CONTRAST);
    cap.set(cv::CAP_PROP_SATURATION, params.SATURATION);
    cap.set(cv::CAP_PROP_SHARPNESS, params.SHARPNESS);
    cap.set(cv::CAP_PROP_GAIN, params.GAIN);
    cap.set(cv::CAP_PROP_BACKLIGHT, params.BACKLIGHT_COMPENSATION);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc(params.FORMAT[0], params.FORMAT[1], params.FORMAT[2], params.FORMAT[3]));
    cap.set(cv::CAP_PROP_FPS, params.FPS);
    if (params.AUTO_WHITE_BALANCE) {
        cap.set(cv::CAP_PROP_AUTO_WB, 1);
    } else {
        cap.set(cv::CAP_PROP_WB_TEMPERATURE, params.WHITE_BALANCE);
        cap.set(cv::CAP_PROP_AUTO_WB, 0);
    }
    if (params.AUTO_EXPOSURE == 1) {
        cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
        cap.set(cv::CAP_PROP_EXPOSURE, params.EXPOSURE);
    } else if (params.AUTO_EXPOSURE == 3) {
        cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 3);
    }
    cap.set(cv::CAP_PROP_AUTOFOCUS, params.AUTO_FOCUS);
    cap.set(cv::CAP_PROP_FOCUS, params.FOCUS);
}

// Inicializar la cámara
bool initializeCamera(cv::VideoCapture& cap, const DetectionParameters& params) {
    cap.open(0, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::cerr << "Error al abrir la cámara." << std::endl;
        return false;
    }
    configureCamera(cap, params);
    return true;
}

// Capturar un fotograma de la cámara
bool captureFrame(cv::VideoCapture& cap, cv::Mat& frame) {
    cap >> frame;
    if (frame.empty()) {
        std::cerr << "Error al capturar un fotograma." << std::endl;
        return false;
    }
    return true;
}

// Procesar la imagen para detectar colores y realizar transformaciones
void processFrame(const cv::Mat& roiFrame, const DetectionParameters& params,
                  std::vector<ColorRange>& colorRanges,
                  cv::Mat& combinedSmallMask, cv::Mat& combinedBigMask, cv::Mat& mask_white) {

    // Conversión a HSV
    cv::Mat hsv;
    cv::cvtColor(roiFrame, hsv, cv::COLOR_BGR2HSV);

    // Procesar cada color en paralelo
    #pragma omp parallel for
    for (int i = 0; i < static_cast<int>(colorRanges.size()); ++i) {
        cv::inRange(hsv, colorRanges[i].lower, colorRanges[i].upper, colorRanges[i].mask);
    }

    // Combinar máscaras rojas
    cv::Mat mask_red;
    cv::bitwise_or(colorRanges[0].mask, colorRanges[1].mask, mask_red);

    // Combinar máscaras para objetos pequeños
    combinedSmallMask = cv::Mat::zeros(roiFrame.size(), CV_8UC1);
    combinedSmallMask += colorRanges[3].mask; // YELLOW
    combinedSmallMask += colorRanges[4].mask; // GREEN
    combinedSmallMask += colorRanges[5].mask; // SKYBLUE
    combinedSmallMask += colorRanges[6].mask; // ORANGE
    cv::GaussianBlur(combinedSmallMask, combinedSmallMask, cv::Size(params.KERNELSIZESMALL, params.KERNELSIZESMALL), params.SIGMA);

    // Combinar máscaras para objetos grandes
    combinedBigMask = cv::Mat::zeros(roiFrame.size(), CV_8UC1);
    combinedBigMask += mask_red;              // RED
    combinedBigMask += colorRanges[2].mask;   // BLUE
    cv::GaussianBlur(combinedBigMask, combinedBigMask, cv::Size(params.KERNELSIZEBIG, params.KERNELSIZEBIG), params.SIGMA);

    // Procesar máscara de la pelota
    mask_white = colorRanges[7].mask; // WHITE
    cv::GaussianBlur(mask_white, mask_white, cv::Size(params.KERNELSIZEMEDIUM, params.KERNELSIZEMEDIUM), params.SIGMA);
}

// Detectar círculos grandes (equipos)
std::vector<cv::Vec3f> detectBigCircles(const cv::Mat& combinedBigMask, const DetectionParameters& params) {
    std::vector<cv::Vec3f> bigCircles;
    cv::HoughCircles(combinedBigMask, bigCircles, cv::HOUGH_GRADIENT, params.DP, params.MIN_DIST, params.PARAM1_BIG, params.PARAM2_BIG, params.MIN_RADIUS_BIG, params.MAX_RADIUS_BIG);
    return bigCircles;
}

// Detectar círculos pequeños (jugadores) dentro de un círculo grande
std::vector<cv::Vec3f> detectSmallCircles(const cv::Mat& roiMask, const DetectionParameters& params) {
    std::vector<cv::Vec3f> smallCircles;
    cv::HoughCircles(roiMask, smallCircles, cv::HOUGH_GRADIENT, params.DP, params.MIN_DIST, params.PARAM1_SMALL, params.PARAM2_SMALL, params.MIN_RADIUS_SMALL, params.MAX_RADIUS_SMALL);
    return smallCircles;
}

// Detectar y preparar datos de la pelota
void detectBall(const cv::Mat& mask_white, std::vector<detection::DetectedObject>& detectedObjects, const DetectionParameters& params) {
    std::vector<cv::Vec3f> whiteCircles;
    cv::HoughCircles(mask_white, whiteCircles, cv::HOUGH_GRADIENT, params.DP, params.MIN_DIST, params.PARAM1_BALL, params.PARAM2_BALL, params.MIN_RADIUS_BALL, params.MAX_RADIUS_BALL);
    if (!whiteCircles.empty()) {
        cv::Point center(cvRound(whiteCircles[0][0]), cvRound(whiteCircles[0][1]));

        // Crear un mensaje DetectedObject para la pelota
        detection::DetectedObject ballMsg;
        ballMsg.set_type(detection::DetectedObject::BALL);
        ballMsg.set_x(center.x + params.ROI_X);
        ballMsg.set_y(center.y + params.ROI_Y);
        detectedObjects.push_back(ballMsg);
    }
}

// Etiquetar los círculos grandes y pequeños y preparar mensajes Protobuf
void labelObjects(const std::vector<cv::Vec3f>& bigCircles, const cv::Mat& combinedSmallMask,
                  const cv::Mat& mask_red, const cv::Mat& mask_blue, const std::vector<ColorRange>& colorRanges,
                  std::vector<detection::DetectedObject>& detectedObjects, const DetectionParameters& params) {

    // Paralelizar el bucle
    #pragma omp parallel for
    for (int i = 0; i < static_cast<int>(bigCircles.size()); i++) {
        cv::Point bigCenter(cvRound(bigCircles[i][0]), cvRound(bigCircles[i][1]));
        int bigRadius = cvRound(bigCircles[i][2]);

        // Detecta círculos pequeños dentro del círculo grande
        cv::Rect roiRect(bigCenter.x - bigRadius, bigCenter.y - bigRadius, bigRadius * 2, bigRadius * 2);

        // Validar que el ROI esté dentro de los límites de la imagen
        if (roiRect.x < 0 || roiRect.y < 0 || roiRect.x + roiRect.width > combinedSmallMask.cols || roiRect.y + roiRect.height > combinedSmallMask.rows)
            continue;

        cv::Mat roiMask = combinedSmallMask(roiRect);
        std::vector<cv::Vec3f> smallCircles = detectSmallCircles(roiMask, params);

        // Si se encuentra un círculo pequeño dentro del círculo grande
        if (!smallCircles.empty()) {
            cv::Point smallCenter(cvRound(smallCircles[0][0]), cvRound(smallCircles[0][1]));
            int smallRadius = cvRound(smallCircles[0][2]);

            // Etiqueta el círculo grande y pequeño
            int team = 0; // 1: Rojo, 2: Azul
            int playerID = 0; // 1, 2, 3
            float angle = atan2(smallCenter.y - bigRadius, smallCenter.x - bigRadius) * 180 / CV_PI;
            if (angle < 0) angle += 360;

            uchar redPixel = mask_red.at<uchar>(bigCenter.y, bigCenter.x);
            uchar bluePixel = mask_blue.at<uchar>(bigCenter.y, bigCenter.x);

            uchar yellowPixel = colorRanges[3].mask.at<uchar>(roiRect.y + smallCenter.y, roiRect.x + smallCenter.x);
            uchar greenPixel = colorRanges[4].mask.at<uchar>(roiRect.y + smallCenter.y, roiRect.x + smallCenter.x);
            uchar skybluePixel = colorRanges[5].mask.at<uchar>(roiRect.y + smallCenter.y, roiRect.x + smallCenter.x);
            uchar orangePixel = colorRanges[6].mask.at<uchar>(roiRect.y + smallCenter.y, roiRect.x + smallCenter.x);

            if (redPixel > 0) {
                team = 1; // Equipo Rojo
                if (yellowPixel > 0) {
                    playerID = 1;
                } else if (greenPixel > 0) {
                    playerID = 2;
                } else if (skybluePixel > 0) {
                    playerID = 3;
                }
            } else if (bluePixel > 0) {
                team = 2; // Equipo Azul
                if (yellowPixel > 0) {
                    playerID = 1;
                } else if (greenPixel > 0) {
                    playerID = 2;
                } else if (orangePixel > 0) {
                    playerID = 3;
                }
            }

            // Crear un mensaje DetectedObject para el robot
            detection::DetectedObject robotMsg;
            robotMsg.set_type(detection::DetectedObject::ROBOT);
            robotMsg.set_x(bigCenter.x + params.ROI_X);
            robotMsg.set_y(bigCenter.y + params.ROI_Y);
            robotMsg.set_angle(angle);
            robotMsg.set_team(team);
            robotMsg.set_id(playerID);

            // Agregar a la lista de objetos detectados en sección crítica
            #pragma omp critical
            {
                detectedObjects.push_back(robotMsg);
            }
        }
    }
}

// Función para enviar mensajes Protobuf mediante multicast UDP
void sendDetectedObjects(const std::vector<detection::DetectedObject>& detectedObjects, const DetectionParameters& params) {
    // Crear un socket UDP
    boost::asio::io_context io_context;
    boost::asio::ip::udp::socket socket(io_context);

    boost::asio::ip::udp::endpoint endpoint(boost::asio::ip::address::from_string(params.MULTICAST_ADDRESS), std::stoi(params.PORT));

    socket.open(boost::asio::ip::udp::v4());

    // Serializar y enviar los mensajes
    for (const auto& obj : detectedObjects) {
        std::string serializedData;
        obj.SerializeToString(&serializedData);
        socket.send_to(boost::asio::buffer(serializedData), endpoint);
    }
}

// Función principal
int main() {
    GOOGLE_PROTOBUF_VERIFY_VERSION;

    DetectionParameters params = loadParameters("parameters.yaml");

    cv::VideoCapture cap;
    if (!initializeCamera(cap, params)) {
        return 1;
    }

    cv::Mat frame;
    cv::Rect roi;
    bool roiDefined = false;

    // Inicializar los rangos de color
    std::vector<ColorRange> colorRanges;
    initializeColorRanges(params, colorRanges);

    while (true) {
        if (!captureFrame(cap, frame)) break;

        // Si el ROI no está definido, utilizar los valores del archivo YAML
        if (!roiDefined) {
            roi = cv::Rect(params.ROI_X, params.ROI_Y, params.ROI_WIDTH, params.ROI_HEIGHT);

            // Validar que el ROI esté dentro de los límites de la imagen
            if (roi.width <= 0 || roi.height <= 0 ||
                roi.x < 0 || roi.y < 0 ||
                (roi.x + roi.width) > frame.cols || (roi.y + roi.height) > frame.rows) {
                std::cerr << "ROI no válido. Por favor, verifica los valores en parameters.yaml." << std::endl;
                return 1;
            } else {
                roiDefined = true;
            }
        }

        cv::Mat roiFrame = frame(roi);

        // Procesar el frame
        cv::Mat combinedSmallMask, combinedBigMask, mask_white;
        processFrame(roiFrame, params, colorRanges, combinedSmallMask, combinedBigMask, mask_white);

        // Detectar círculos grandes
        std::vector<cv::Vec3f> bigCircles = detectBigCircles(combinedBigMask, params);

        // Preparar objetos detectados
        std::vector<detection::DetectedObject> detectedObjects;

        // Detectar y preparar datos de la pelota
        detectBall(mask_white, detectedObjects, params);

        // Etiquetar los círculos grandes y pequeños y preparar datos de robots
        labelObjects(bigCircles, combinedSmallMask, colorRanges[0].mask, colorRanges[2].mask, colorRanges, detectedObjects, params);

        // Enviar los objetos detectados vía multicast UDP
        sendDetectedObjects(detectedObjects, params);
    }

    // Liberar la captura
    cap.release();

    google::protobuf::ShutdownProtobufLibrary();

    return 0;
}
