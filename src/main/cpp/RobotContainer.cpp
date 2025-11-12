#include "RobotContainer.h"
#include <frc2/command/button/JoystickButton.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Constants.h"

RobotContainer::RobotContainer()
    : m_amcu(nullptr),
      m_sensorManager(nullptr),
      m_wallAlignDriveCommand(nullptr, nullptr, 15.0, 15, 30),
      m_driveUntilWallCommand(nullptr, nullptr, 15.0, 28.0, 15),
      m_cobraLineFollowCommand(nullptr, 40.0, static_cast<uint8_t>(15), 0.30),
      m_autoPickSequence(&m_arm, &m_gripper, &m_gripperJoint, &m_elevator),
      m_autoRetractAndLift(&m_arm, &m_extender),
      m_calibrateExtenderOnly(&m_extender),
      m_demoExtender(&m_extender, 2.0),
      m_simpleDrive(nullptr, 0.3, 0.0, 0.0),
      // m_testSequence(m_amcu, &m_arm, &m_extender),
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
      // picksequence with camera apple detection
      m_smartPickSequence(&m_arm, &m_gripper, &m_gripperJoint, &m_camera, &m_elevator),
      m_smartPickSequenceMid(&m_arm, &m_gripper, &m_gripperJoint, &m_camera, &m_elevator),
      m_smartPickSequenceHigh(&m_arm, &m_gripper, &m_gripperJoint, &m_camera, &m_elevator),
      m_qrCodeReaderCommandSingle(&m_camera, QRCodeReaderCommand::ReadMode::SINGLE_READ),
      m_qrCodeReaderCommandTimed(&m_camera, 10.0), // 10 second timeout
      m_qrCodeReaderCommandContinuous(&m_camera, QRCodeReaderCommand::ReadMode::CONTINUOUS_READ),
      m_driveSmartPickupGround(&m_arm, &m_gripper, &m_gripperJoint, &m_camera, &m_elevator, m_amcu),
      // Apple detection commands - simplified
      m_checkAppleGrip(&m_camera, AppleGripperCheckCommand::CheckMode::QUICK_CHECK, 1.0),
      m_waitForGrip(&m_camera, AppleGripperCheckCommand::CheckMode::CONTINUOUS_MONITOR, 5.0),
      m_monitorGrip(&m_camera, AppleGripperCheckCommand::CheckMode::CONTINUOUS_MONITOR, 10.0),
        
      // Initialize StoreAppleCommand instances (stores all 3 apples automatically)
      // 
      m_competitionAuto(nullptr, &m_arm, &m_gripper, &m_gripperJoint, &m_camera, &m_elevator, &m_extender)
// m_pickupAndDerliverSequence(&m_arm, &m_gripper, &m_gripperJoint, &m_camera, &m_elevator, m_amcu, 0.5)
{

  // Initialize all subsystems
  m_arm.Init();
  m_extender.Init();
  m_gripper.Init();
  m_gripperJoint.Init();
  try
  {
    m_camera.InitDashboard();
    m_camera.Start();
  }
  catch (const std::exception &e)
  {
    std::cout << "WARNING: Camera initialization failed: " << e.what() << std::endl;
    std::cout << "Robot will continue without camera functionality" << std::endl;
  }

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
  // m_chooser.AddOption("Smart Pick Sequence", new SmartPickSequence(&m_arm, &m_gripper, &m_gripperJoint, &m_camera, &m_elevator));
  m_chooser.AddOption("Smart Pick Ground", &m_smartPickSequence);
  m_chooser.AddOption("Smart Pick Mid", &m_smartPickSequenceMid);
  m_chooser.AddOption("Smart Pick High", &m_smartPickSequenceHigh);
  m_chooser.AddOption("Drive Smart Pickup", &m_driveSmartPickupGround); // FIXED: Use member variable instead of leaked new
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
  m_chooser.AddOption("Quick Apple Check", &m_checkAppleGrip); // Quick check (1s)
  m_chooser.AddOption("Monitor Apple (5s)", &m_waitForGrip);   // Wait for successful grip (5s)
  m_chooser.AddOption("Monitor Apple (10s)", &m_monitorGrip);

  // Drive with Sensors
  m_chooser.AddOption("Drive with Sensor", &m_wallAlignDriveCommand);
  m_chooser.AddOption("Drive Until Wall", &m_driveUntilWallCommand);
  m_chooser.AddOption("Line Follow", &m_cobraLineFollowCommand);

  //QR Code Reader
  m_chooser.AddOption("QR READ Single", &m_qrCodeReaderCommandSingle);
  m_chooser.AddOption("QR READ Timed", &m_qrCodeReaderCommandTimed);
  m_chooser.AddOption("QR READ Continuous", &m_qrCodeReaderCommandContinuous);
  // m_chooser.AddOption("Retract and Lift", &m_autoRetractAndLift);

  // ADD APPLE STORAGE OPTIONS TO CHOOSER:
  // m_chooser.AddOption("Store Apple (Auto)", &m_storeAppleAuto);           // Camera detection
  // m_chooser.AddOption("Store Apple (Red)", &m_storeAppleRed);             // Manual red
  // m_chooser.AddOption("Store Apple (Yellow)", &m_storeAppleYellow);       // Manual yellow
  // m_chooser.AddOption("Store Apple (Green)", &m_storeAppleGreen); 
  m_chooser.AddOption("Competition Auto", &m_competitionAuto);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

void RobotContainer::ConfigureButtonBindings() {}

void RobotContainer::SetAMCU(AMCU *amcu_ptr)
{
  m_amcu = amcu_ptr;

  m_elevator.Init(amcu_ptr);
  m_testSequence.SetAMCU(amcu_ptr);
  m_wallAlignDriveCommand.SetAMCU(amcu_ptr);
  m_driveUntilWallCommand.SetAMCU(amcu_ptr);
  m_cobraLineFollowCommand.SetAMCU(amcu_ptr);
  m_competitionAuto.SetAMCU(amcu_ptr);

}

void RobotContainer::SetSensorManager(SensorManager *sensor_ptr)
{
  m_sensorManager = sensor_ptr;

  if (m_sensorManager)
  {
    m_wallAlignDriveCommand.SetSensorManager(m_sensorManager);
    m_driveUntilWallCommand.SetSensorManager(m_sensorManager);

    if (m_sensorManager->GetLineFollower())
    {
      m_sensorManager->GetLineFollower()->setMinSignal(0.65);
      m_cobraLineFollowCommand.SetLineFollower(m_sensorManager->GetLineFollower());
    }

    std::cout << "RobotContainer: Updated commands with SensorManager\n";
  }
  else
  {
    std::cout << "RobotContainer: WARNING - SensorManager is null!\n";
  }
}

RobotContainer::~RobotContainer()
{
  if (m_camera.IsRunning()) {
        m_camera.Stop();
    }
}

frc2::Command *RobotContainer::GetAutonomousCommand()
{
  return m_chooser.GetSelected();
}
