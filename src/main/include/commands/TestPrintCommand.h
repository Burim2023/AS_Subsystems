#pragma once

#include <frc2/command/InstantCommand.h>

/**
 * Simple instant command to test if commands are working.
 * This command runs once and immediately finishes.
 */
class TestPrintCommand : public frc2::InstantCommand {
public:
    TestPrintCommand() : frc2::InstantCommand([]() {
        std::cout << "🎯 SUCCESS: Command-based framework is working!" << std::endl;
        std::cout << "Commands can be scheduled and executed properly." << std::endl;
    }) {}
};