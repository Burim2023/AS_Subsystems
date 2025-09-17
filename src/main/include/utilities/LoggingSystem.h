#pragma once

#include <iostream>
#include <string>
#include <chrono>
#include <sstream>
#include <ctime>
#include <iomanip>

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define YELLOW  "\033[33m"
#define GREEN   "\033[32m"
#define CYAN    "\033[36m"
#define BLUE    "\033[34m"
#define PURPLE  "\033[35m"
#define WHITE   "\033[37m"

#define BOLD        "\033[1m"
#define bold_res    "\033[22m"
#define ITALICS     "\033[3m"
#define UNDERLINE   "\033[4m"

struct ModeInfo {
    std::string color;
    std::string name;
};

extern ModeInfo last_mode;

void SetupLogging();

inline std::string current_time() {
    using namespace std::chrono;
    auto now = system_clock::now();
    auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

    std::time_t now_c = system_clock::to_time_t(now);
    std::tm* now_tm = std::localtime(&now_c);

    std::ostringstream oss;
    oss << std::put_time(now_tm, "%Y-%m-%d %H:%M:%S");
    oss << '.' << std::setfill('0') << std::setw(3) << ms.count();
    return oss.str();
}

// default log infos
#define LOG_INFO(value)    { std::cout << "[" << current_time() << "] " << BOLD << GREEN   << "[INFO] "  << RESET << value << std::endl; }
#define LOG_WARN(value)    { std::cout << "[" << current_time() << "] " << YELLOW  << "[WARN] "  << RESET << value << std::endl; }
#define LOG_ERROR(value)   { std::cerr << "[" << current_time() << "] " << RED     << "[ERROR] " << RESET << value << std::endl; }

// mode log infos
#define LOG_AUTONOMOUS(value)   { std::cout << "[" << current_time() << "] " << PURPLE << "[AUTONOMOUS] "    << RESET << value << std::endl; }
#define LOG_TELEOP(value)       { std::cout << "[" << current_time() << "] " << CYAN   << "[TELEOP] "        << RESET << value << std::endl; }
#define LOG_TEST(value)         { std::cout << "[" << current_time() << "] " << YELLOW << "[TEST] "          << RESET << value << std::endl; }
#define LOG_DISABLED(value)     { std::cout << "[" << current_time() << "] " << last_mode.color << last_mode.name << RESET << value << std::endl; }