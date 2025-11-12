#include "SignalHandler.h"
#include <signal.h>
#include <execinfo.h>
#include <unistd.h>
#include <iostream>

static void CrashHandler(int sig) {
    void *bt[50];
    int bt_size = backtrace(bt, 50);
    // Print to stderr (visible in robot console)
    std::cerr << "*** Crash detected: signal " << sig << " ***\n";
    backtrace_symbols_fd(bt, bt_size, STDERR_FILENO);
    // flush and abort
    std::cerr << "*** End crash backtrace ***\n";
    _exit(128 + sig);
}

void InstallCrashHandler() {
    struct sigaction sa;
    sa.sa_handler = CrashHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART;
    sigaction(SIGSEGV, &sa, nullptr);
    sigaction(SIGABRT, &sa, nullptr);
    sigaction(SIGFPE, &sa, nullptr);
}
