#include "utilities/LoggingSystem.h"
#include <fstream>

ModeInfo last_mode = {RESET, "[INIT]"};
std::ofstream logFile;

void SetupLogging() {
    logFile.open("/home/pi/robot.log", std::ios::out | std::ios::app);
    std::cout.rdbuf(logFile.rdbuf());
    std::cerr.rdbuf(logFile.rdbuf());
}