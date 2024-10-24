/*  PROYECTO DE VISIÓN POR COMPUTADOR PARA ROBOCUP SOCCER 2024

    AUTOR: SEBASTIÁN CIFUENTES PÉREZ
    PROFESOR GUÍA: DANIEL YUNGE
    PROFESOR CO-GUÍA: GUILLERMO CID

    Este código se encarga de la detección en tiempo real de robots
    y la visualización de la detección realizada para posterior
    calibración de colores o tamaños.

    Los parámetros de HoughCircles están adaptados a las condiciones
    del laboratorio LabSens, con iluminación blanca constante, utilizando
    la cámara Logitech C922 ubicada a 2 metros sobre el campo y
    con diámetros de círculos:
    - Grandes: 110 mm
    - Pequeños: 30 mm
    - Pelota: 44 mm (Pelota de ping pong tradicional)
*/

#include <opencv2/opencv.hpp>
#include <chrono>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <omp.h>

// Estructura para almacenar los parámetros
struct DetectionParameters {
    // Parámetros de control de imagen
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

    cv::Scalar LOWER_RED, UPPER_RED, LOWER_RED2, UPPER_RED2;
    cv::Scalar LOWER_BLUE, UPPER_BLUE;
    cv::Scalar LOWER_YELLOW, UPPER_YELLOW;
    cv::Scalar LOWER_GREEN, UPPER_GREEN;
    cv::Scalar LOWER_SKYBLUE, UPPER_SKYBLUE;
    cv::Scalar LOWER_ORANGE, UPPER_ORANGE;
    cv::Scalar LOWER_WHITE, UPPER_WHITE;

    double DP;
    // Parámetros min_dist separados para cada tipo de círculo
    double MIN_DIST_BIG;
    double MIN_DIST_SMALL;
    double MIN_DIST_BALL;

    int PARAM1_BIG, PARAM2_BIG, MIN_RADIUS_BIG, MAX_RADIUS_BIG;
    int PARAM1_SMALL, PARAM2_SMALL, MIN_RADIUS_SMALL, MAX_RADIUS_SMALL;
    int PARAM1_BALL, PARAM2_BALL, MIN_RADIUS_BALL, MAX_RADIUS_BALL;
    int KERNELSIZEBIG, KERNELSIZEMEDIUM, KERNELSIZESMALL;
    double SIGMA;
    float FONT_SCALE, TEXT_THICKNESS;
};

// Estructura para rangos de color
struct ColorRange {
    std::string name;
    cv::Scalar lower;
    cv::Scalar upper;
    cv::Mat mask;
};

// Función para cargar parámetros desde un archivo YAML
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

        // Definiciones de colores
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

        // Nuevos parámetros min_dist para cada tipo de círculo
        params.MIN_DIST_BIG = config["hough"]["min_dist_big"].as<double>();
        params.MIN_DIST_SMALL = config["hough"]["min_dist_small"].as<double>();
        params.MIN_DIST_BALL = config["hough"]["min_dist_ball"].as<double>();

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
        std::cerr << "Error loading parameters from YAML file: " << e.what() << std::endl;
    }

    return params;
}

// Función para inicializar rangos de color
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

// Configurar la cámara con los parámetros cargados
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
        std::cerr << "Error opening the camera." << std::endl;
        return false;
    }
    configureCamera(cap, params);
    return true;
}

// Capturar un frame de la cámara
bool captureFrame(cv::VideoCapture& cap, cv::Mat& frame) {
    cap >> frame;
    if (frame.empty()) {
        std::cerr << "Error capturing a frame." << std::endl;
        return false;
    }
    return true;
}

// Seleccionar la región de interés (ROI)
cv::Rect selectROI(const cv::Mat& frame) {
    cv::Rect roi = cv::selectROI("Select ROI", frame);
    cv::destroyWindow("Select ROI");
    return roi;
}

// Procesar la imagen para detectar colores y realizar transformaciones
void processFrame(const cv::Mat& roiFrame, const DetectionParameters& params,
                  std::vector<ColorRange>& colorRanges,
                  cv::Mat& combinedSmallMask, cv::Mat& combinedBigMask, cv::Mat& mask_white) {

    // Convertir a HSV
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

    // Procesar máscara para la pelota
    mask_white = colorRanges[7].mask; // WHITE
    cv::GaussianBlur(mask_white, mask_white, cv::Size(params.KERNELSIZEMEDIUM, params.KERNELSIZEMEDIUM), params.SIGMA);
}

// Detectar círculos grandes (equipos)
std::vector<cv::Vec3f> detectBigCircles(const cv::Mat& combinedBigMask, const DetectionParameters& params) {
    std::vector<cv::Vec3f> bigCircles;
    cv::HoughCircles(combinedBigMask, bigCircles, cv::HOUGH_GRADIENT, params.DP, params.MIN_DIST_BIG, params.PARAM1_BIG, params.PARAM2_BIG, params.MIN_RADIUS_BIG, params.MAX_RADIUS_BIG);
    return bigCircles;
}

// Detectar círculos pequeños (jugadores) dentro de un círculo grande
std::vector<cv::Vec3f> detectSmallCircles(const cv::Mat& roiMask, const DetectionParameters& params) {
    std::vector<cv::Vec3f> smallCircles;
    cv::HoughCircles(roiMask, smallCircles, cv::HOUGH_GRADIENT, params.DP, params.MIN_DIST_SMALL, params.PARAM1_SMALL, params.PARAM2_SMALL, params.MIN_RADIUS_SMALL, params.MAX_RADIUS_SMALL);
    return smallCircles;
}

// Detectar y etiquetar la pelota
void detectAndLabelBall(const cv::Mat& mask_white, cv::Mat& roiFrame, const DetectionParameters& params) {
    std::vector<cv::Vec3f> whiteCircles;
    cv::HoughCircles(mask_white, whiteCircles, cv::HOUGH_GRADIENT, params.DP, params.MIN_DIST_BALL, params.PARAM1_BALL, params.PARAM2_BALL, params.MIN_RADIUS_BALL, params.MAX_RADIUS_BALL);
    if (!whiteCircles.empty()) {
        cv::Point mediumCenter(cvRound(whiteCircles[0][0]), cvRound(whiteCircles[0][1]));
        int mediumRadius = cvRound(whiteCircles[0][2]);
        std::string label = "Ball";
        cv::circle(roiFrame, mediumCenter, mediumRadius, cv::Scalar(255, 255, 255), 2);
        cv::putText(roiFrame, label, cv::Point(mediumCenter.x - mediumRadius, mediumCenter.y - mediumRadius - 10), cv::FONT_HERSHEY_DUPLEX, params.FONT_SCALE, cv::Scalar(255, 255, 255), params.TEXT_THICKNESS);
    }
}

// Etiquetar los círculos grandes y pequeños
void labelObjects(const std::vector<cv::Vec3f>& bigCircles, const cv::Mat& combinedSmallMask, cv::Mat& roiFrame, const DetectionParameters& params,
                  const cv::Mat& mask_red, const cv::Mat& mask_blue, const std::vector<ColorRange>& colorRanges) {

    // Crear una copia de roiFrame para dibujar de forma segura en paralelo
    cv::Mat roiFrameCopy = roiFrame.clone();

    #pragma omp parallel for
    for (int i = 0; i < static_cast<int>(bigCircles.size()); i++) {
        cv::Point bigCenter(cvRound(bigCircles[i][0]), cvRound(bigCircles[i][1]));
        int bigRadius = cvRound(bigCircles[i][2]);

        // Detectar círculos pequeños dentro del círculo grande
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

            // Etiquetar el círculo grande y el pequeño
            std::string bigColor, smallColor;
            cv::Scalar teamColor, playerColor;

            uchar redPixel = mask_red.at<uchar>(bigCenter.y, bigCenter.x);
            uchar bluePixel = mask_blue.at<uchar>(bigCenter.y, bigCenter.x);

            uchar yellowPixel = colorRanges[3].mask.at<uchar>(roiRect.y + smallCenter.y, roiRect.x + smallCenter.x);
            uchar greenPixel = colorRanges[4].mask.at<uchar>(roiRect.y + smallCenter.y, roiRect.x + smallCenter.x);
            uchar skybluePixel = colorRanges[5].mask.at<uchar>(roiRect.y + smallCenter.y, roiRect.x + smallCenter.x);
            uchar orangePixel = colorRanges[6].mask.at<uchar>(roiRect.y + smallCenter.y, roiRect.x + smallCenter.x);

            if (redPixel > 0) {
                bigColor = "Red Team";
                teamColor = cv::Scalar(0, 0, 255);

                if (yellowPixel > 0) {
                    smallColor = "Player 1";
                    playerColor = cv::Scalar(0, 255, 255);
                }
                else if (greenPixel > 0) {
                    smallColor = "Player 2";
                    playerColor = cv::Scalar(0, 255, 0);
                }
                else if (skybluePixel > 0) {
                    smallColor = "Player 3";
                    playerColor = cv::Scalar(255, 255, 0);
                }
                else {
                    smallColor = "Unknown Player";
                    playerColor = cv::Scalar(255, 255, 255);
                }

            } else if (bluePixel > 0) {
                bigColor = "Blue Team";
                teamColor = cv::Scalar(255, 0, 0);

                if (yellowPixel > 0) {
                    smallColor = "Player 1";
                    playerColor = cv::Scalar(0, 255, 255);
                }
                else if (greenPixel > 0) {
                    smallColor = "Player 2";
                    playerColor = cv::Scalar(0, 255, 0);
                }
                else if (orangePixel > 0) {
                    smallColor = "Player 3";
                    playerColor = cv::Scalar(0, 128, 255);
                }
                else {
                    smallColor = "Unknown Player";
                    playerColor = cv::Scalar(255, 255, 255);
                }

            } else {
                bigColor = "Unknown Team";
                teamColor = cv::Scalar(255, 255, 255);
                smallColor = "Unknown Player";
                playerColor = cv::Scalar(255, 255, 255);
            }

            std::string label = bigColor + ", " + smallColor;

            // Dibujo en sección crítica
            #pragma omp critical
            {
                cv::circle(roiFrameCopy, bigCenter, bigRadius, cv::Scalar(0, 255, 0), 2);
                cv::circle(roiFrameCopy, cv::Point(bigCenter.x + smallCenter.x - bigRadius, bigCenter.y + smallCenter.y - bigRadius), smallRadius, cv::Scalar(0, 255, 0), 2);

                size_t commaPos = label.find(", ");
                if (commaPos != std::string::npos) {
                    cv::putText(roiFrameCopy, label.substr(0, commaPos), bigCenter, cv::FONT_HERSHEY_DUPLEX, params.FONT_SCALE, teamColor, params.TEXT_THICKNESS);
                    cv::putText(roiFrameCopy, label.substr(commaPos + 2), cv::Point(bigCenter.x, bigCenter.y + 20), cv::FONT_HERSHEY_DUPLEX, params.FONT_SCALE, playerColor, params.TEXT_THICKNESS);
                }
            }
        }
    }

    // Actualizar roiFrame con la copia modificada
    roiFrame = roiFrameCopy;
}

// Mostrar la imagen en pantalla
void displayFrame(const cv::Mat& frame) {
    cv::imshow("Circle Detection", frame);
    char key = cv::waitKey(1);
    if (key == 27) { // Tecla ESC
        exit(0);  // Salir si se presiona 'ESC'
    }
}

// Función para calcular y mostrar FPS en la consola
void calculateAndDisplayFPS(int& frames, std::chrono::steady_clock::time_point& start) {
    frames++;
    auto now = std::chrono::steady_clock::now();
    double elapsed_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(now - start).count();
    if (elapsed_seconds >= 1.0) {
        double fps = frames / elapsed_seconds;
        std::cout << "FPS: " << fps << std::endl;
        frames = 0;
        start = now;
    }
}

int main() {
    // Cargar parámetros desde el archivo YAML
    DetectionParameters params = loadParameters("parameters.yaml");

    // Inicializar la cámara
    cv::VideoCapture cap;
    if (!initializeCamera(cap, params)) {
        return 1;
    }

    cv::Mat frame;
    cv::Rect roi;
    bool roiDefined = false;
    bool windowCreated = false;

    // Inicializar rangos de color
    std::vector<ColorRange> colorRanges;
    initializeColorRanges(params, colorRanges);

    // Variables para el cálculo de FPS
    int frames = 0;
    auto start = std::chrono::steady_clock::now();

    while (true) {
        if (!captureFrame(cap, frame)) break;

        // Si el ROI no está definido, seleccionarlo
        if (!roiDefined) {
            roi = selectROI(frame);
            // Verificar si el ROI es válido
            if (roi.width <= 0 || roi.height <= 0 ||
                roi.x < 0 || roi.y < 0 ||
                (roi.x + roi.width) > frame.cols || (roi.y + roi.height) > frame.rows) {
                std::cerr << "Invalid ROI. Please select a valid ROI." << std::endl;
                roiDefined = false;
                continue;
            } else {
                roiDefined = true;
                // Crear la ventana después de definir el ROI
                if (!windowCreated) {
                    cv::namedWindow("Circle Detection", cv::WINDOW_AUTOSIZE);
                    windowCreated = true;
                }
            }
        }

        // Extraer el ROI del frame
        cv::Mat roiFrame = frame(roi);

        // Procesar el frame
        cv::Mat combinedSmallMask, combinedBigMask, mask_white;
        processFrame(roiFrame, params, colorRanges, combinedSmallMask, combinedBigMask, mask_white);

        // Detectar círculos grandes (equipos)
        std::vector<cv::Vec3f> bigCircles = detectBigCircles(combinedBigMask, params);

        // Detectar y etiquetar la pelota
        detectAndLabelBall(mask_white, roiFrame, params);

        // Etiquetar los círculos grandes y pequeños (jugadores)
        labelObjects(bigCircles, combinedSmallMask, roiFrame, params, colorRanges[0].mask, colorRanges[2].mask, colorRanges);

        // Calcular y mostrar FPS en la consola
        calculateAndDisplayFPS(frames, start);

        // Mostrar el frame procesado
        displayFrame(roiFrame);
    }

    // Liberar recursos
    cap.release();
    cv::destroyAllWindows();
    return 0;
}
