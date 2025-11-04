#include "RobotContainer.h"
#include <frc2/command/button/JoystickButton.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Constants.h"

RobotContainer::RobotContainer() 
    : m_amcu(nullptr),
      m_ultrasonic(Constants::kLeftTriggerPort, Constants::kLeftEchoPort, Constants::kRightTriggerPort, Constants::kRightEchoPort),
      //m_lidar(studica::Lidar::kUSB0),
      
      m_wallAlignDriveCommand(nullptr, &m_ultrasonic, &m_lidar, 15.0, 15, 30),
      m_driveUntilWallCommand(nullptr, &m_ultrasonic, &m_lidar, 15.0, 28.0, 15),
      m_cobraLineFollowCommand(&m_lineFollower, 40.0, static_cast<uint8_t>(15), 0.30),
      m_autoPickSequence(&m_arm, &m_gripper, &m_gripperJoint, &m_elevator),
      m_autoRetractAndLift(&m_arm, &m_extender),
      m_calibrateExtenderOnly(&m_extender),
      m_demoExtender(&m_extender, 2.0),
      m_simpleDrive(nullptr, 0.3, 0.0, 0.0),
      //m_testSequence(m_amcu, &m_arm, &m_extender),
      m_testSequence(m_amcu),
      m_gripperOperateUp(&m_gripperJoint, &m_gripper, GripperOperate::Position::UP, true, 2.0),
      m_gripperOperateDown(&m_gripperJoint, &m_gripper, GripperOperate::Position::DOWN, false, 2.0),
      m_gripperPickup(&m_gripperJoint, &m_gripper, GripperOperate::Position::MID, false, 2.0),
      m_gripperPickupSequence(&m_gripperJoint, &m_gripper),
      m_calibrateElevator(&m_elevator, 15.0),
      m_elevatorGround(&m_elevator, ElevatorPresets::Position::GROUND),
      m_elevatorLow(&m_elevator, ElevatorPresets::Position::LOW),
      m_elevatorHigh(&m_elevator, ElevatorPresets::Position::HIGH),
      m_elevatorCustom(&m_elevator, 60.0f, 2.0f),
      m_elevatorTestSequence(&m_elevator),
      //picksequence with camera apple detection
      m_smartPickSequence(&m_arm, &m_gripper, &m_gripperJoint, &m_camera, &m_elevator),
      m_driveSmartPickupGround(&m_arm, &m_gripper, &m_gripperJoint, &m_camera, &m_elevator, m_amcu),
      // Apple detection commands - simplified
      m_checkAppleGrip(&m_camera, AppleGripperCheckCommand::CheckMode::QUICK_CHECK, 1.0),
      m_waitForGrip(&m_camera, AppleGripperCheckCommand::CheckMode::CONTINUOUS_MONITOR, 5.0),
      m_monitorGrip(&m_camera, AppleGripperCheckCommand::CheckMode::CONTINUOUS_MONITOR, 10.0)
      // m_pickupAndDerliverSequence(&m_arm, &m_gripper, &m_gripperJoint, &m_camera, &m_elevator, m_amcu, 0.5)
       {
  
  // Initialize all subsystems
  m_arm.Init();
  m_extender.Init();
  m_gripper.Init();
  m_gripperJoint.Init();
  m_camera.InitDashboard();
  m_camera.Start();
  // Note: Elevator will be initialized in SetAMCU() method
  m_ultrasonic.Init();
  m_lidar.Init();      // create studica::Lidar instance
  m_lidar.StartScan(); // start scanning (non-blocking)
  // m_lineFollower = std::make_unique<LineFollower>(0, 1, 2, 3);
  m_lineFollower.setMinSignal(0.65);

  // Configure the button bindings
  ConfigureButtonBindings();
  
  // Setup autonomous chooser
  m_chooser.SetDefaultOption("Test Command Sequence", &m_testSequence);
  m_chooser.AddOption("Simple Drive Forward", &m_simpleDrive);
  m_chooser.AddOption("Gripper Up & Open", &m_gripperOperateUp);
  m_chooser.AddOption("Gripper Down & Close", &m_gripperOperateDown);
  m_chooser.AddOption("Gripper Pickup (Mid & Close)", &m_gripperPickup);
  m_chooser.AddOption("Gripper Pickup Sequence", &m_gripperPickupSequence);
  m_chooser.AddOption("Full Pick Sequence", &m_autoPickSequence);
  //m_chooser.AddOption("Smart Pick Sequence", new SmartPickSequence(&m_arm, &m_gripper, &m_gripperJoint, &m_camera, &m_elevator));
  m_chooser.AddOption("Smart Pick Sequence", &m_smartPickSequence);
  //m_chooser.AddOption("Drive to Apple And Pick off Ground!", &m_driveSmartPickupGround);
  m_chooser.AddOption("Drive Smart Pickup", new frc2::InstantCommand([this] {
        if (!m_amcu) {
            std::cout << "RobotContainer: AMCU is null - cannot start DriveSmartPickupGround" << std::endl;
            return;
        }
        // construct and schedule a fresh command instance
        frc2::CommandScheduler::GetInstance().Schedule(
            new DriveSmartPickupGround(&m_arm, &m_gripper, &m_gripperJoint, &m_camera, &m_elevator, m_amcu)
        );
    }));
  // m_chooser.AddOption("Pickup&Deliver", &m_pickupAndDerliverSequence);

  
  // Elevator commands
  m_chooser.AddOption("Calibrate Elevator", &m_calibrateElevator);
  m_chooser.AddOption("Elevator to Ground", &m_elevatorGround);
  m_chooser.AddOption("Elevator to Low", &m_elevatorLow);
  m_chooser.AddOption("Elevator to High", &m_elevatorHigh);
  m_chooser.AddOption("Elevator Custom 60mm", &m_elevatorCustom);
  m_chooser.AddOption("Elevator Test Sequence", &m_elevatorTestSequence);

  // Extender commands
  m_chooser.AddOption("Extender DEMO", &m_demoExtender);
  m_chooser.AddOption("Calibrate Extender", &m_calibrateExtenderOnly);

  // Apple grip check commands
  m_chooser.AddOption("Quick Apple Check", &m_checkAppleGrip);        // Quick check (1s)
  m_chooser.AddOption("Monitor Apple (5s)", &m_waitForGrip);        // Wait for successful grip (5s)
  m_chooser.AddOption("Monitor Apple (10s)", &m_monitorGrip);

  //Drive with Sensors
  m_chooser.AddOption("Drive with Sensor", &m_wallAlignDriveCommand);
  m_chooser.AddOption("Drive Until Wall", &m_driveUntilWallCommand);
  m_chooser.AddOption("Line Follow", &m_cobraLineFollowCommand);
  
  
  //m_chooser.AddOption("Retract and Lift", &m_autoRetractAndLift);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

void RobotContainer::ConfigureButtonBindings() {
  // Since your OI class uses direct button methods instead of a controller object,
  // we'll skip button bindings for now and rely on teleop periodic control
  // 
  // To add button bindings, you would need to either:
  // 1. Add a GetDriveController() method to your OI class that returns frc::Joystick&
  // 2. Or use a different approach with triggers based on your existing OI methods
  
  // For now, commands can be triggered manually or through autonomous mode
}

void RobotContainer::SetAMCU(AMCU* amcu) {
  m_amcu = amcu;
  
  // Initialize elevator with AMCU
  m_elevator.Init(amcu);
  
  m_testSequence.SetAMCU(amcu);

  m_wallAlignDriveCommand.SetAMCU(amcu);

  m_driveUntilWallCommand.SetAMCU(amcu);

  m_cobraLineFollowCommand.SetAMCU(amcu);

  //m_simpleDrive.SetAMCU(amcu);
  // Update SimpleDrive command with the actual AMCU instance
  //m_simpleDrive = SimpleDrive(amcu, 0.3, 0.0, 0.0); // Forward at 30% speed
  // Reinitialize the test sequence with the actual AMCU instance
  // Note: This is a bit of a hack - ideally we'd pass AMCU in the constructor

  // // Example: Use bumpers to change speed
  // frc2::JoystickButton(&m_oi.GetDriveJoystick(), OI::LEFT_SHOULDER)
  //     .WhenPressed(frc2::InstantCommand([this] { m_teleopDrive.SetSpeedMultiplier(0.4); }, {})); // 40% speed

  // frc2::JoystickButton(&m_oi.GetDriveJoystick(), OI::RIGHT_SHOULDER)
  //     .WhenPressed(frc2::InstantCommand([this] { m_teleopDrive.SetSpeedMultiplier(0.6); }, {})); // 80% speed
}

RobotContainer::~RobotContainer() {
  // Stop LiDAR scanning cleanly when RobotContainer is destroyed
  std::cout << "RobotContainer: Stopping LiDAR scan on shutdown..." << std::endl;
  m_lidar.StopScan();
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // Return the selected command from the chooser
  return m_chooser.GetSelected();
}
