#include "subsystems/amcu/AMCU.h"
#include <iostream>

// Constructor starts thread for CAN
AMCU::AMCU()
{
  thread_finished = false;
  thr = std::thread(&AMCU::handleThread, this);
}

// Destructor stops CAN and ends thread
AMCU::~AMCU()
{
  HAL_CAN_CloseStreamSession(handleCAN);
  if (thr.joinable())
  {
    thread_finished = true;
    thr.join();
  }
}

// open the CAN
void AMCU::init()
{
  HAL_CAN_OpenStreamSession(&handleCAN, (id | ID_11_BIT), 0x00, MAX_MSG_STORE, &status);
}

// return the rpm from the given motor
int8_t AMCU::getRPM(enum Motor motor)
{
  std::lock_guard<std::mutex> lock(data_mutex);
  return rpm[motor];
}

// adds a TLV-frame to the queue, which sets the motor to the given rpm
void AMCU::setRPM(enum Motor motor, int8_t rpm)
{
  uint8_t abc = rpm;
  addTLVToList(TAG_SET_RPM, 2, {(uint8_t)motor, abc});
}

// registers a callback function for the limit switches
void AMCU::registerLimitSwitchCallback(void (*pCallback)(uint8_t motorNr, uint8_t high))
{
  pLimitSwitchCallback = pCallback;
}

// registers a callback function for the drive functions
void AMCU::registerDriveActionCallback(void (*pCallback)())
{
  pDriveActionDoneCallback = pCallback;
}

// adds a TLV-frame to the queue, which sets the robot base to a given speed in x-, y- or w-direction
void AMCU::speedDrive(uint8_t xSpeed_cms, uint8_t ySpeed_cms, uint8_t wSpeed_degs)
{
  // std::cout << "AMCU::speedDrive called: x=" << (int)xSpeed_cms << " y=" << (int)ySpeed_cms << " w=" << (int)wSpeed_degs << std::endl;
  addTLVToList(TAG_DR_SPEED, 3, {xSpeed_cms, ySpeed_cms, wSpeed_degs});
}

// adds a TLV-frame to the queue, which drives to robot base for a given distance
void AMCU::driveDistance(uint8_t xMeter, uint8_t yMeter, uint16_t omega_degree)
{
  uint8_t omega_degree1 = omega_degree & SMALL_HALF_OF_UINT16T;
  uint8_t omega_degree2 = (omega_degree & BIG_HALF_OF_UINT16T) >> 8;
  addTLVToList(TAG_DR_DIST, 4, {xMeter, yMeter, omega_degree1, omega_degree2});
}

// adds a TLv-Frame to the queue, which stops all activities
void AMCU::stop()
{
  addTLVToList(TAG_DR_STOP, 1, {0});
}

// splits the uint16_t value into two uint8_t values and adds a TLV-Frame to the queue to initialise a omnidirectional base
void AMCU::initOmniDriveBase(uint8_t wheelRadius_mm, uint16_t robotRadius_mm, enum Motor motorLeft, enum Motor motorRight, enum Motor motorBack)
{
  uint8_t robotRadius1 = robotRadius_mm & SMALL_HALF_OF_UINT16T;
  uint8_t robotRadius2 = (robotRadius_mm & BIG_HALF_OF_UINT16T) >> 8;
  addTLVToList(TAG_BASE_OMNI, 6, {wheelRadius_mm, robotRadius1, robotRadius2, (uint8_t)motorLeft, (uint8_t)motorRight, (uint8_t)motorBack});
}

// splits the uint16_t values into two uint8_t values and adds a TLV-Frame to the queue to initialise a omnidirectional base
void AMCU::initMecanumDriveBase(uint8_t wheelRadius_mm, uint16_t robotX_mm, uint16_t robotY_mm, enum Motor motorFrontLeft, enum Motor motorFrontRight, enum Motor motorBackLeft, enum Motor motorBackRight)
{
  uint8_t robotX1 = robotX_mm & SMALL_HALF_OF_UINT16T;
  uint8_t robotX2 = (robotX_mm & BIG_HALF_OF_UINT16T) >> 8;

  uint8_t robotY1 = robotY_mm & SMALL_HALF_OF_UINT16T;
  uint8_t robotY2 = (robotY_mm & BIG_HALF_OF_UINT16T) >> 8;

  addTLVToList(TAG_BASE_MEC, 9, {wheelRadius_mm, robotX1, robotX2, robotY1, robotY2, (uint8_t)motorFrontLeft, (uint8_t)motorFrontRight, (uint8_t)motorBackLeft, (uint8_t)motorBackRight});
}

// splits the uint16_t value into two uint8_t values and adds a TLV-Frame to the queue to initialise a omnidirectional base
void AMCU::initDifferentialDriveBase2Wheel(uint8_t wheelRadius_mm, uint16_t wheelDistance_mm, enum Motor motorLeft, enum Motor motorRight)
{
  uint8_t wheelDistance1 = wheelDistance_mm & SMALL_HALF_OF_UINT16T;
  uint8_t wheelDistance2 = (wheelDistance_mm & BIG_HALF_OF_UINT16T) >> 8;

  addTLVToList(TAG_BASE_DIFF2, 5, {wheelRadius_mm, wheelDistance1, wheelDistance2, (uint8_t)motorLeft, (uint8_t)motorRight});
}

// splits the uint16_t value into two uint8_t values and adds a TLV-Frame to the queue to initialise a omnidirectional base
void AMCU::initDifferentialDriveBase4Wheel(uint8_t wheelRadius_mm, uint16_t wheelDistance_mm, enum Motor motorFrontLeft, enum Motor motorFrontRight, enum Motor motorBackLeft, enum Motor motorBackRight)
{
  uint8_t wheelDistance1 = wheelDistance_mm & SMALL_HALF_OF_UINT16T;
  uint8_t wheelDistance2 = (wheelDistance_mm & BIG_HALF_OF_UINT16T) >> 8;

  addTLVToList(TAG_BASE_DIFF4, 7, {wheelRadius_mm, wheelDistance1, wheelDistance2, (uint8_t)motorFrontLeft, (uint8_t)motorFrontRight, (uint8_t)motorBackLeft, (uint8_t)motorBackRight});
}

// adds a TLV-frame to the list, to set the constants of the PID-controller
void AMCU::setPID(float kp, float ki, float kd)
{
  uint8_t Kp, Ki, Kd;
  Kp = kp * 10;
  Ki = ki * 10;
  Kd = kd * 10;
  addTLVToList(TAG_SET_PID, 3, {(uint8_t)Kp, Ki, Kd});
}

// adds a TLv-frame to the list, to configure a limit switch
void AMCU::setLimitSwitches(enum Motor motor, uint8_t high, uint8_t enable, uint8_t mode, uint8_t bounce)
{
  addTLVToList(TAG_SET_LIMITSW, 5, {(uint8_t)motor, high, enable, mode, bounce});
}

// returns the encoder value of the given motor
int16_t AMCU::getEncoder(enum Motor motor)
{
  return encoder[motor];
}

// adds a TLV-frame to the list, to reset the encoder value of the given motor
void AMCU::resetEncoder(enum Motor motor)
{
  addTLVToList(TAG_RESET_ENC, 1, {(uint8_t)motor});
}

// adds, a TLV-frame to the list, to set a given motor to a given speed in percent
void AMCU::setSpeed(enum Motor motor, int8_t percent)
{
  uint8_t abc = percent;
  addTLVToList(TAG_SET_SPEED, 2, {(uint8_t)motor, abc});
}

// adds a TLV-frame to the queue, to drive the robot base with a given speed for a given time
void AMCU::timeDrive(uint8_t xSpeed_cms, uint8_t ySpeed_cms, uint8_t wSpeed_degs, uint8_t time_s)
{
  addTLVToList(TAG_DR_TIME, 4, {xSpeed_cms, ySpeed_cms, wSpeed_degs, time_s});
}

// adds TLv-frames to the sending queue
bool AMCU::addTLVToList(const uint8_t tag, const uint8_t len, std::list<uint8_t> value)
{
  std::lock_guard<std::mutex> lock(queue_mutex);

  // create a new frame
  TLV_Frame frame;
  frame.tag = tag;
  frame.len = len;

  // if the data length is smaller than 7 (CAN frame supports up to 8 bytes -> 1Byte Tag + 1Byte Length + 6 Byte Data)
  if (len < 7)
  {
    frame.value.splice(frame.value.end(), value); // copies the pointers from the value list to the frame.value list
    frame.ext = NO;                               // not an extended frame
    queue.push_front(frame);                      // push frame to sending queue
  }
  else
  {
    auto start = value.begin();                               // store the begining of the value list
    auto end = value.begin();                                 // store the begining of the value list
    std::advance(end, 6);                                     // incerement the begining by 6 -> end for first frame
    frame.value.splice(frame.value.end(), value, start, end); // copy first 6 data elements to the frame.value list
    frame.ext = FIRST_PART;                                   // first part of the extended frame
    queue.push_front(frame);                                  // push frame to sending queue

    // create a second frame
    TLV_Frame frame2;
    frame2.tag = tag;
    frame2.len = len;

    frame2.value.splice(frame2.value.end(), value); // copy the rest of the data elements to the second frame.value list
    frame2.ext = SECOND_PART;                       // second part of the extended frame
    queue.push_front(frame2);                       // push frame to sending queue
  }
  return false;
}

// sends the stored TLV-frames
bool AMCU::sendTLV()
{
  // check if initlaized and if TLV-frames wait to be sendend and if no respones is waited for
  if (initialized == true && waitForResponse == false)
  {
    std::lock_guard<std::mutex> lock(queue_mutex);

    if (queue.size() > 0)
    {
      int32_t status;
      TLV_Frame frame = queue.back(); // get oldest entry of the queue
      queue.pop_back();               // delete oldest entry

      uint8_t len = frame.len + 2; // length is data length + 2 (Tag, Length)
      if (frame.ext == FIRST_PART) // if frame is first part of extended frame length is 8
        len = 6 + 2;
      else if (frame.ext == SECOND_PART) // if frame is second part of extended frame length is length -6 + 2
        len = (frame.len - 6) + 2;

      uint8_t *data = new uint8_t[len];
      data[0] = frame.tag; // store tag in first element
      data[1] = frame.len; // store length in second element

      if (frame.ext == NO) // if no extended frame
      {
        for (int i = 0; i < frame.len; i++) // store frame in sending array
        {
          data[i + 2] = (frame.value.front());
          frame.value.pop_front();
        }
      }
      else
      {
        // if first part of extended frame store first 6 data bytes in data
        if (frame.ext == FIRST_PART)
        {
          for (int i = 0; i < 6; i++)
          {
            data[i + 2] = (frame.value.front());
            frame.value.pop_front();
          }
        }
        // if second part of extended frame store the rest of the data bytes
        else
        {
          for (int i = 0; i < (frame.len - 6); i++)
          {
            data[i + 2] = (frame.value.front());
            frame.value.pop_front();
          }
        }
      }

      HAL_CAN_SendMessage(ID_11_BIT | CAN_ID, data, len, 0, &status); // send message over CAN
      count_MSG_sended++;                                             // increase counter
      waitForResponse = true;                                         // wait for a respones
    }
  }
  return true;
}

// reads TLV-frames from the buffer
void AMCU::readTLV()
{
  HAL_CANStreamMessage message;
  uint32_t read_msg = 0;

  HAL_CAN_ReadStreamSession(handleCAN, &message, 1, &read_msg, &status); // read a message from the buffer
  if (message.messageID == (0x446 | ID_11_BIT))                          // check if received message has given id
  {
    if (message.dataSize > 0)
    {
      count_MSG_received++; // increase counter for received messages

      // output values to dashboard (read with lock)
      {
        std::lock_guard<std::mutex> lock(data_mutex);
        frc::SmartDashboard::PutNumber("RPM 0", rpm[MOTOR_0]);
        frc::SmartDashboard::PutNumber("RPM 1", rpm[MOTOR_1]);
        frc::SmartDashboard::PutNumber("RPM 2", rpm[MOTOR_2]);
        frc::SmartDashboard::PutNumber("RPM 3", rpm[MOTOR_3]);

        frc::SmartDashboard::PutNumber("Encoder 0", encoder[MOTOR_0]);
        frc::SmartDashboard::PutNumber("Encoder 1", encoder[MOTOR_1]);
        frc::SmartDashboard::PutNumber("Encoder 2", encoder[MOTOR_2]);
        frc::SmartDashboard::PutNumber("Encoder 3", encoder[MOTOR_3]);
      }
    }

    // check if size of can frame -2 is the same as the length value of the TLV-Frame
    if (message.dataSize - 2 == message.data[1])
    {
      // if yes handle the received TLV-frame
      handleTLVReceive(message.data[0], message.data[1], &message.data[2]);
    }
  }
}

// handles received TLV-frame
void AMCU::handleTLVReceive(uint8_t tag, uint8_t len, uint8_t value[])
{
  switch (tag) // switch the received tag
  {
  case 0x40: // ok
  {
    cnt_sucessfull++;
    break;
  }

  case 0x41: // fail
  {
    cnt_fail++;
    break;
  }

  case 0x42: // getRPM response
  {
    std::lock_guard<std::mutex> lock(data_mutex);
    rpm[value[0]] = value[1];
    break;
  }

  case 0x43: // getEncoder response
  {
    std::lock_guard<std::mutex> lock(data_mutex);
    uint16_t u_enc = value[1] | (value[2] << 8);
    encoder[value[0]] = u_enc;
    break;
  }

  case 0x44: // done
  {
    cnt_done++;
    if (pDriveActionDoneCallback != NULL) // if a callback function is registered call it
      pDriveActionDoneCallback();
    break;
  }

  case 0x45: // if a limit switch is activated call the registered function
  {
    if (pLimitSwitchCallback != NULL)
    {
      cnt_lmSw++;
      pLimitSwitchCallback(value[0], value[1]);
    }
    break;
  }
  }
  if (tag != 0x44)
    waitForResponse = false;
}

// called for thread
void AMCU::handleThread()
{
  int cntGetRPM = 0;
  int storeQueueSize = 0;
  while (!thread_finished) // infinity loop while AMCU object exists
  {
    if (initialized == false) // if it hasnt been initiales initialise it
    {
      initialized = true;
      std::this_thread::sleep_for(350ms); // Wait until CAN is ready
      init();                             // init object

      // Read queue size with lock
      {
        std::lock_guard<std::mutex> lock(queue_mutex);
        storeQueueSize = queue.size();
      }
      frc::SmartDashboard::PutNumber("SizeQueue at Init", storeQueueSize);
    }
    cnt_test++;
    cntGetRPM++;
    // send a request for rpm and encoder every 1000 loops
    if (cntGetRPM > 1000)
    {
      addTLVToList(TAG_GET_RPM, 1, {0});
      addTLVToList(TAG_GET_RPM, 1, {1});
      addTLVToList(TAG_GET_RPM, 1, {2});
      addTLVToList(TAG_GET_RPM, 1, {3});

      addTLVToList(TAG_GET_ENC, 1, {0});
      addTLVToList(TAG_GET_ENC, 1, {1});
      addTLVToList(TAG_GET_ENC, 1, {2});
      addTLVToList(TAG_GET_ENC, 1, {3});
      cntGetRPM = 0;
    }

    readTLV(); // read a tlv-frame from buffer
    sendTLV(); // send a tlv-frame from the queue

    // Read queue size with lock
    {
      std::lock_guard<std::mutex> lock(queue_mutex);
      frc::SmartDashboard::PutNumber("SizeQueue", queue.size());
    }

    frc::SmartDashboard::PutNumber("countMSG received", count_MSG_received);
    frc::SmartDashboard::PutNumber("countMSG sended", count_MSG_sended);
    frc::SmartDashboard::PutNumber("cnt_OK", cnt_sucessfull);
    frc::SmartDashboard::PutNumber("cnt_FAIL", cnt_fail);
    frc::SmartDashboard::PutNumber("cnt_DONE", cnt_done);
    frc::SmartDashboard::PutNumber("cnt_LmSW", cnt_lmSw);
    std::this_thread::sleep_for(100us); // wait for 100µs
  }
}