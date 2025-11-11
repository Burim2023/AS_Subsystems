
#pragma once

#include <hal/CAN.h>
#include <stdlib.h>
#include <atomic>
#include <mutex>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/SubsystemBase.h>
#include <list>

#define ID_11_BIT 0x40000000
#define CAN_ID 0x446

using namespace std::literals::chrono_literals;

#define SMALL_HALF_OF_UINT16T 0b0000000011111111
#define BIG_HALF_OF_UINT16T 0b1111111100000000

#define MAX_MSG_STORE 10

enum Motor
{
  MOTOR_0,
  MOTOR_1,
  MOTOR_2,
  MOTOR_3
};

// allows up to 12 bytes for a TLV frame (sended as two and
// reunioned after receive)
enum ExtendedTLV
{
  NO,
  FIRST_PART,
  SECOND_PART
};

struct TLV_Frame
{
  uint8_t tag;
  uint8_t len;
  enum ExtendedTLV ext;
  std::list<uint8_t> value;
};

class AMCU : public frc2::SubsystemBase
{
public:
  // ----------------------------------------------------------------------
  // description:     constructor
  // parameters:      -
  // return value:    -
  // ----------------------------------------------------------------------
  AMCU();

  // ----------------------------------------------------------------------
  // description:     destructor
  // parameters:      -
  // return value:    -
  // ----------------------------------------------------------------------
  ~AMCU();

  // ----------------------------------------------------------------------
  // description:     initializes the omnidirectional drive base
  // parameters:      wheelRadius_mm ... wheel radius in mm
  //                  robotRadius_mm ... robot radius in mm
  //                  motorLeft ... number of the left motor
  //                  motorRight ... number of the right motor
  //                  motorBack ... number of the back motor
  // return value:    -
  // ----------------------------------------------------------------------
  void initOmniDriveBase(uint8_t wheelRadius_mm, uint16_t robotRadius_mm,
                         enum Motor motorLeft, enum Motor motorRight,
                         enum Motor motorBack);

  // ----------------------------------------------------------------------
  // description:     initializes the mecanum drive base
  // parameters:      wheelRadius_mm ... wheel radius in mm
  //                  robotX_mm ... robot length in mm
  //                  robotY_mm ... robot width in mm
  //                  motorFrontLeft ... number of the front left motor
  //                  motorFrontRight ... number of the front right motor
  //                  motorBackLeft ... number of the back left motor
  //                  motorBackRight ... number of the back right motor
  // return value:    -
  // ----------------------------------------------------------------------
  void initMecanumDriveBase(uint8_t wheelRadius_mm, uint16_t robotX_mm,
                            uint16_t robotY_mm, enum Motor motorFrontLeft,
                            enum Motor motorFrontRight,
                            enum Motor motorBackLeft,
                            enum Motor motorBackRight);

  // ----------------------------------------------------------------------
  // description:     initializes the 2 wheeled differential drive
  // parameters:      wheelRadius_mm ... wheel radius in mm
  //                  wheelDistance_mm ... distance between wheels in mm
  //                  motorLeft ... number of the left motor
  //                  motorRight ... number of the right motor
  // return value:    -
  // ----------------------------------------------------------------------
  void initDifferentialDriveBase2Wheel(uint8_t wheelRadius_mm,
                                       uint16_t wheelDistance_mm,
                                       enum Motor motorLeft,
                                       enum Motor motorRight);

  // ----------------------------------------------------------------------
  // description:     initializes the 4 wheeled differential drive
  // parameters:      wheelRadius_mm ... wheel radius in mm
  //                  wheelDistance_mm ... wheel distance in mm
  //                  motorFrontLeft ... number of the front left motor
  //                  motorFrontRight ... number of the front right motor
  //                  motorBackLeft ... number of the back left motor
  //                  motorBackRight ... number of the back right motor
  // return value:    -
  // ----------------------------------------------------------------------
  void initDifferentialDriveBase4Wheel(uint8_t wheelRadius_mm,
                                       uint16_t wheelDistance_mm,
                                       enum Motor motorFrontLeft,
                                       enum Motor motorFrontRight,
                                       enum Motor motorBackLeft,
                                       enum Motor motorBackRight);

  // ----------------------------------------------------------------------
  // description:     sets the values of the PID controller used for the
  //                  motors
  // parameters:      kp ... the Kp value
  //                  ki ... the Ki value
  //                  kd ... the Kd value
  // return value:    -
  // ----------------------------------------------------------------------
  void setPID(float kp, float ki, float kd);

  // ----------------------------------------------------------------------
  // description:     activates a limit switch
  // parameters:      motor ... the number of the motor (0,2 or 3, motor 1
  //                            does not have interrupts)
  //                  high ... switch low (0) or switch high (1)
  //                  enable ... enable (1) or disable (0) the switch
  //                  mode ... normally open (0) or normally closed (1)
  //                  bounce ... bounceback activated(1) or deactivated(0)
  // return value:    -
  // ----------------------------------------------------------------------
  void setLimitSwitches(enum Motor motor, uint8_t high, uint8_t enable,
                        uint8_t mode, uint8_t bounce);

  // ----------------------------------------------------------------------
  // description:     set the RPM of a single motor
  // parameters:      motor ... the number of the motor
  //                  rpm ... the desired RPM (-100 <= rpm <= 100)
  // return value:    -
  // ----------------------------------------------------------------------
  void setRPM(enum Motor motor, int8_t rpm);

  // ----------------------------------------------------------------------
  // description:     sets the speed in percent of a single motor
  // parameters:      motor ... the number of the motor
  //                  percent ... the speed in percent +/- 100
  // return value:    -
  // ----------------------------------------------------------------------
  void setSpeed(enum Motor motor, int8_t percent);

  // ----------------------------------------------------------------------
  // description:     reset the encoder value
  // parameters:      motor ... the number of the motor
  // return value:    -
  // ----------------------------------------------------------------------
  void resetEncoder(enum Motor motor);

  // ----------------------------------------------------------------------
  // description:     stops all motors of the drive base
  // parameters:      -
  // return value:    -
  // ----------------------------------------------------------------------
  void stop();

  // ----------------------------------------------------------------------
  // description:     returns the value from an encoder
  // parameters:      motor ... the number of the motor
  // return value:    the encoder value
  // ----------------------------------------------------------------------
  int16_t getEncoder(enum Motor motor);

  // ----------------------------------------------------------------------
  // description:     returns the RPM of a motor
  // parameters:      motor ... the number of the motor
  // return value:    the rpm
  // ----------------------------------------------------------------------
  int8_t getRPM(enum Motor motor);

  // ----------------------------------------------------------------------
  // description:     registers a function as callback function that gets
  //                  triggered when a limit switch triggers an interrupt
  // parameters:      pCallback ... the callback function
  // return value:    -
  // ----------------------------------------------------------------------
  void registerLimitSwitchCallback(void (*pCallback)(uint8_t motorNr,
                                                     uint8_t high));

  // ----------------------------------------------------------------------
  // description:     registers a fucntion as a callback function that gets
  //                  triggered when a drive (timeDrive, distanceDrive)
  //                  triggers an interrupt
  // parameters:      -
  // return value:    -
  // ----------------------------------------------------------------------
  void registerDriveActionCallback(void (*pCallback)());

  // ----------------------------------------------------------------------
  // description:     sets a speed for the base in x-, y- and w-direction
  // parameters:      xSpeed_cms ... speed in x direction in cm/s
  //                  ySpeed_cms ... speed in y direction in cm/s
  //                  wSpeed_degs ... speed in w direction in degrees/s
  // return value:    -
  // ----------------------------------------------------------------------
  void speedDrive(uint8_t xSpeed_cms, uint8_t ySpeed_cms,
                  uint8_t wSpeed_degs);

  // ----------------------------------------------------------------------
  // description:     sets a speed for the base in x-, y- and w-direction
  //                  for a specified time
  // parameters:      xSpeed_cms ... speed in x direction in cm/s
  //                  ySpeed_cms ... speed in y direction in cm/s
  //                  wSpeed_degs ... speed in w direction in degrees/s
  // return value:    -
  // ----------------------------------------------------------------------
  void timeDrive(uint8_t xSpeed_cms, uint8_t ySpeed_cms,
                 uint8_t wSpeed_degs, uint8_t time_s);

  // ----------------------------------------------------------------------
  // description:     drives the base for a specified distance
  // parameters:      xMeter ... distance to drive in x direction in m
  //                  yMeter ... distance to drive in y direction in m
  //                  omega_degree ... distance to drive in w direction in
  //                                   degrees
  // return value:    -
  // ----------------------------------------------------------------------
  void driveDistance(uint8_t xMeter, uint8_t yMeter,
                     uint16_t omega_degree);

private:
  // CAN requests -> sended over CAN to the Titan via TLV frames
  const uint8_t TAG_OK = 0x00;          // 0x00 = ok
  const uint8_t TAG_FAIL = 0x01;        // 0x01 = fail
  const uint8_t TAG_SET_RPM = 0x10;     // 0x10 = setRPM
  const uint8_t TAG_GET_RPM = 0x11;     // 0x11 = getRPM
  const uint8_t TAG_GET_ENC = 0x12;     // 0x12 = getEncoder
  const uint8_t TAG_RESET_ENC = 0x13;   // 0x13 = resetEncoder
  const uint8_t TAG_SET_SPEED = 0x14;   // 0x14 = setSpeed
  const uint8_t TAG_SET_PID = 0x15;     // 0x15 = setPID
  const uint8_t TAG_SET_LIMITSW = 0x16; // 0x16 = setLimitSwitches
  const uint8_t TAG_DR_SPEED = 0x20;    // 0x20 = driveSpeed
  const uint8_t TAG_DR_TIME = 0x21;     // 0x21 = driveTime
  const uint8_t TAG_DR_DIST = 0x22;     // 0x22 = driveDistance
  const uint8_t TAG_DR_STOP = 0x23;     // 0x23 = stop
  const uint8_t TAG_BASE_OMNI = 0x30;   // 0x30 = initOmni
  const uint8_t TAG_BASE_MEC = 0x31;    // 0x31 = initMecanum
  const uint8_t TAG_BASE_DIFF2 = 0x32;  // 0x32 = initDiff_2
  const uint8_t TAG_BASE_DIFF4 = 0x33;  // 0x33 = initDiff_4

  // CAN responses -> received over CAN from via TLV frames
  const uint8_t TAG_R_OK = 0x40;   // 0x40 = ok
  const uint8_t TAG_R_FAIL = 0x41; // 0x41 = fail
  const uint8_t TAG_R_RPM = 0x42;  // 0x42 = getRPM
  const uint8_t TAG_R_ENC = 0x43;  // 0x43 = getEncoder
  const uint8_t TAG_R_DONE = 0x44; // 0x44 = done

  uint16_t id;
  int32_t status;
  uint32_t handleCAN;
  std::atomic<uint8_t> cnt_sucessfull{0};
  std::atomic<uint8_t> cnt_fail{0};
  std::atomic<uint8_t> cnt_done{0};
  std::atomic<uint64_t> cnt_test{0};
  std::atomic<uint8_t> cnt_lmSw{0};
  std::atomic<uint16_t> count_MSG_received{0};
  std::atomic<uint16_t> count_MSG_sended{0};
  uint8_t cntOmni = 0;
  std::atomic<bool> waitForResponse{false};

  int8_t rpm[4];
  int16_t encoder[4];
  float position[3];

  std::list<TLV_Frame> queue;
  std::atomic<bool> initialized{false};

  // Periodic cycle counter for RPM/encoder requests
  int m_periodicCycleCount{0};
  static constexpr int kRpmRequestInterval = 20; // Request every 20 cycles (~400ms at 50Hz)

  // Thread safety mutexes (still needed for data access)
  std::mutex queue_mutex;
  std::mutex data_mutex;

  void (*pDriveActionDoneCallback)() = NULL;
  void (*pLimitSwitchCallback)(uint8_t motorNr, uint8_t high) = NULL;

  // ----------------------------------------------------------------------
  // description:     handles TLV receives
  // parameters:      tag ... the tag of the received message
  //                  len ... the length of the received message
  //                  value ... the byte array received
  // return value:    -
  // ----------------------------------------------------------------------
  void handleTLVReceive(uint8_t tag, uint8_t len, uint8_t value[]);

  // ----------------------------------------------------------------------
  // description:     initializes the CAN connection
  // parameters:      -
  // return value:    -
  // ----------------------------------------------------------------------
  void init();

  // ----------------------------------------------------------------------
  // description:     WPILib Periodic - handles CAN communication
  //                  Called automatically by command scheduler at ~50Hz
  // parameters:      -
  // return value:    -
  // ----------------------------------------------------------------------
public:
  void Periodic() override;

  // ----------------------------------------------------------------------
  // description:     adds a TLV-Frame to the sending queue
  // parameters:      tag ... the tag of the message
  //                  len ... the length of the message
  //                  value ... the data of the message
  // return value:    -
  // ----------------------------------------------------------------------
  bool addTLVToList(const uint8_t tag, const uint8_t len,
                    std::list<uint8_t> value);

  // ----------------------------------------------------------------------
  // description:     reads messages from the mailbox
  // parameters:      -
  // return value:    -
  // ----------------------------------------------------------------------
  void readTLV();

  // ----------------------------------------------------------------------
  // description:     sends messages from the queue
  // parameters:      -
  // return value:    -
  // ----------------------------------------------------------------------
  bool sendTLV();
};