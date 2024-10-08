#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <chrono>
#include <sstream>

// Estructura para almacenar los tiempos de CPU
struct CpuTimes {
    unsigned long long user;
    unsigned long long nice;
    unsigned long long system;
    unsigned long long idle;
    unsigned long long iowait;
    unsigned long long irq;
    unsigned long long softirq;
    unsigned long long steal;
    unsigned long long guest;
    unsigned long long guest_nice;
};

// Función para leer los tiempos de CPU desde /proc/stat
bool readCpuTimes(CpuTimes& times) {
    std::ifstream stat_file("/proc/stat");
    if (!stat_file.is_open()) {
        std::cerr << "No se pudo abrir /proc/stat" << std::endl;
        return false;
    }

    std::string line;
    std::getline(stat_file, line);

    std::istringstream ss(line);
    std::string cpu_label;
    ss >> cpu_label;

    if (cpu_label.substr(0, 3) != "cpu") {
        std::cerr << "Formato inesperado en /proc/stat" << std::endl;
        return false;
    }

    ss >> times.user >> times.nice >> times.system >> times.idle >> times.iowait
       >> times.irq >> times.softirq >> times.steal >> times.guest >> times.guest_nice;

    return true;
}

// Función para calcular el porcentaje de uso de CPU entre dos lecturas
double calculateCpuUsage(const CpuTimes& prev, const CpuTimes& curr) {
    unsigned long long prev_idle = prev.idle + prev.iowait;
    unsigned long long curr_idle = curr.idle + curr.iowait;

    unsigned long long prev_non_idle = prev.user + prev.nice + prev.system + prev.irq + prev.softirq + prev.steal;
    unsigned long long curr_non_idle = curr.user + curr.nice + curr.system + curr.irq + curr.softirq + curr.steal;

    unsigned long long prev_total = prev_idle + prev_non_idle;
    unsigned long long curr_total = curr_idle + curr_non_idle;

    double total_diff = static_cast<double>(curr_total - prev_total);
    double idle_diff = static_cast<double>(curr_idle - prev_idle);

    double cpu_percentage = (total_diff - idle_diff) / total_diff * 100.0;

    return cpu_percentage;
}

int main() {
    // Abre la cámara
    cv::VideoCapture cap(0,cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::cerr << "Error al abrir la cámara." << std::endl;
        return -1;
    }

    // Establece la resolución y el formato de captura
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);  // Ancho (cambia a 640 para 480p)
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720); // Alto (cambia a 480 para 480p)
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G')); // Formato MJPG
    cap.set(cv::CAP_PROP_FPS, 30);


    // Variables para cálculo de FPS
    int frames = 0;
    auto start = std::chrono::steady_clock::now();

    // Variables para medición de CPU
    CpuTimes prev_cpu_times;
    CpuTimes curr_cpu_times;
    readCpuTimes(prev_cpu_times);

    cv::Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Error al capturar un fotograma." << std::endl;
            break;
        }

        // Calcular y mostrar el FPS en la consola
        frames++;
        auto now = std::chrono::steady_clock::now();
        double elapsed_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(now - start).count();
        if (elapsed_seconds >= 1.0) {
            double fps = frames / elapsed_seconds;
            frames = 0;
            start = now;

            // Leer los tiempos actuales de CPU y calcular el uso
            readCpuTimes(curr_cpu_times);
            double cpu_usage = calculateCpuUsage(prev_cpu_times, curr_cpu_times);
            prev_cpu_times = curr_cpu_times;

            // Mostrar FPS y uso de CPU
            std::cout << "FPS: " << fps << " - Uso de CPU: " << cpu_usage << "%" << std::endl;
        }

        // Muestra el video
        cv::imshow("Video", frame);
        if (cv::waitKey(1) == 27) break; // Salir con la tecla ESC
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
