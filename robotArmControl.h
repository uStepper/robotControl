#ifndef _ROBOTARMCONTROL_H_
#define _ROBOTARMCONTROL_H_

#include "robotArmComm.h"
#include "robotArmConfig.h"
#include "robotArmI2C.h"
#include <uStepperS.h>

#define FEEDRATETOANGULARFEEDRATE(x) x * 10.0
#define HOMEFEEDRATEFAST 20.0f
#define HOMEFEEDRATESLOW 5.0f

    /**
   * @brief	Interrupt routine for Servo.
   *
   *			This interrupt routine is in charge of generating pulses to the servo motor
  */
  extern "C" void TIMER4_COMPA_vect(void) __attribute__ ((signal,used,naked));
  extern "C" void TIMER4_OVF_vect(void) __attribute__ ((signal,used,naked));

class robotArmControl {

public:
  robotArmControl();

  void begin(void);

  /* Main control function, listening for gcode and reading commands when robot
   * is ready */
  void run(void);

  void execute(char *command);

  void setMotorAngle(uint8_t num, float angle);

  void setMotorSpeed(uint8_t num, float speed);

  void runContinously(uint8_t num, float speed);

  void setXYZ();

  void xyzToAngles(float &rot, float &left, float &right, float x_target,
                   float y_target, float z_target);

  void angleToxyz(float rot, float left, float right, float &x_actual,
                  float &y_actual, float &z_actual);

  void homeArm(void);

  bool checkLimits(float rot, float left, float right);

  void setPump(bool state);

  void setServo(uint8_t value);

  void sendXYZ(void);

  float direction = 1.0;
  void checkConnOrientation(void);

private:
  // Object for manipulating i2c slaves
  robotArmI2C bus;

  // Object for receiving and sending gcode
  robotArmComm comm;

  // I2C slave function when receiving a command
  static void busReceiveEvent(void);

  // I2C slave function for sending current angle position */
  static void busRequestEvent(void);

  uint8_t calcVelocityProfile(void);

  void calcVelocityProfileMovement(void);

  bool inRange( float value, float target, float limit );

  void setServo(float servoVal);

  bool isRecording = false;
  bool targetReached = true;

  int8_t baseTargetReached = 0;
  int8_t elbowTargetReached = 0;
  int8_t shoulderTargetReached = 0;

  // Current angles
  float angleBase = 0.0;
  float angleShoulder = 0.0;
  float angleElbow = 0.0;

  // Target angles
  float angleTargetBase = 0.0;
  float angleTargetShoulder = 0.0;
  float angleTargetElbow = 0.0;

  // Current position
  float x = 0.0;
  float y = 0.0;
  float z = 0.0;

  // Target position
  float tx = 145.0;
  float ty = 0.0;
  float tz = 145.0;

  // Target speed
  float sx = 0.0;
  float sy = 0.0;
  float sz = 0.0;

    // Target angular speed
  float targetBaseSpeed = 0.0;
  float targetElbowSpeed = 0.0;
  float targetShoulderSpeed = 0.0;

  bool targetPumpState = 0;
  float targetServo = 0.0;
  float currentServo = 0.0;
  bool currentPumpState = 0;
  // Feedrate in mm/s
  float feedrate = 10.0;

  // Used for timeout purposes
  uint32_t lastCommand = 0;

  bool valveOn = 0;
  int32_t valveOnTime;
};

#endif
