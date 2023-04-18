#include <SimpleFOC.h>
#include "adaptive_angle_adjustment.h"
#include "imu.h"
 
// create motor and driver instances
BLDCMotor bldcmotor = BLDCMotor(11);  // 11 = Pole pairs (the number of permanent magnets/2 on the rotor of a BLDC Motor). some bldc motors give a configuration like 24N22P (take the number of P and divide by 2 to get the pole pairs)
BLDCDriver3PWM motor_driver = BLDCDriver3PWM(10, 5, 6, 8);

// create the encoder instances
Encoder motor_encoder = Encoder(3, A4, 500);
// interrupt ruotine intialisation
void encoder_interupt_A() {  motor_encoder.handleA(); }
void encoder_interupt_B() {  motor_encoder.handleB(); }



bool cube_init_good = false;

//volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void setup() {
  // baud rate of the Serial port
  Serial.begin(250000);

  _delay(1000);
  // imu init and configure
  if ( !initIMU() ) {
    Serial.println(F("IMU init failed. Cube disabled."));
    return;
  }
  _delay(1000);

  

  // initialise encoder hardware
  motor_encoder.init();
  // link the 
  motor_encoder.enableInterrupts(encoder_interupt_A, encoder_interupt_B);

  
  // link the motor to the sensor
  bldcmotor.linkSensor(&motor_encoder);

  // power supply voltage [V]
  motor_driver.voltage_power_supply = 20;
  motor_driver.init();
  bldcmotor.linkDriver(&motor_driver);

  // set control loop type to be used
  // using voltage torque mode
  bldcmotor.controller = MotionControlType::torque;

  // enable monitoring
  bldcmotor.useMonitoring(Serial);

  

  // init motor
  bldcmotor.init();
  // align the encoder with the motor to get the right motor angle and velocity readings
  bldcmotor.initFOC();

  // set motor velocity to 0, for savety
  bldcmotor.target = 0;

  cube_init_good = true;
  Serial.println(F("Init finished. Cube is ready."));
}

float old_angle = 0;      // angle of previous iteration. used to calculate angle_velocity
float balance_angle = 0;  // the angle of the cube relative to the world
float angle_velocity = 0;      // the speed at whitch the angle is changeing
float motor_speed = 0;    // motor rotations per min. 

// used to calculate deltatime between iterations. used to calculate angle_velocity
unsigned long myTimeOld = 0;  
unsigned long myTime = 0;

// adaptive angle adjustment that will be added onto the balance angle to be closer to the actual 
// balance angle of the cube 
float triple_a = 0;

// bool that will be set true if the cube is placed in its balanced position. only if this is true the cube will
// start to actively balance itself. will be reset to false if the cube has fallen or moves out of its effective
// balance range of -0.3 < x < 0.3 (rad)
bool waitForBalancedPos = false;

void loop() {
  // should be executed as often as possible. makes sure the motor is geing operated as smooth as possible using FOC
  bldcmotor.loopFOC();

  // 
  bldcmotor.move();

  // check if everything was initialised correctly
  if (!cube_init_good) // init failled => do nothing
  { 
    bldcmotor.target = 0;
  } 
  else if (imuHasData())  // init good => start balancing routine 
  { 
    // when IMU has a new data package
    // read angle from the IMU
    old_angle = balance_angle;
    myTimeOld = myTime;

    balance_angle = getCubeAngle();
    myTime = millis();

    //adjust the angle so that the balanced position is at about 0 rad
    balance_angle = -(balance_angle + 2.29);
    // differenz quotient
    angle_velocity = 1000*(balance_angle - old_angle)/(myTime - myTimeOld);

    motor_speed = bldcmotor.shaftVelocity();
    
    Serial.print(balance_angle);
    Serial.print("\t");
    Serial.print(angle_velocity);
    Serial.print("\t");
    Serial.println(motor_speed);
      
    // safety to not let the motor spin at max speed all the time.
    // shoud only try to balance when it is within a reasonable range of the balance angle
    if (!waitForBalancedPos && balance_angle < 0.3 && balance_angle > -0.3)
    {
      // calculate the motor value to balance the cube based on the 3 state values
      float motorVoltage = calculateMotorControlleValue(balance_angle - triple_a, angle_velocity, motor_speed);
      // apply the motor value to the motor
      bldcmotor.move(motorVoltage);

      // calculate the triple_a
      triple_a = calculate_avg_angle(balance_angle);
    }
    else 
    {
      // stops the motor from turning
      bldcmotor.move(0);

      // wait to start balancing until the cube was placed in its balanced position
      if (waitForBalancedPos && balance_angle >= -0.001 && balance_angle <= 0.001)
      {
        waitForBalancedPos = false;
      }
      else 
      {
        waitForBalancedPos = true;
      }
    }
  }
}


// LQR that will take angle, angle velocity and the motor speed to calculate how the motor should react
// to hold the angle as close to 0 rad as possible 
float calculateMotorControlleValue(float cube_angle, float cube_angle_velocity, float motor_speed)
{
  // the maximum allowed motor value
  int maxVoltage = 17;
  
  // calculate the motor voltage needed to stabilize the pendulum
  // 20, 6, 0.1 and 1.8 are the parameters that can be used to tune the LQR
  float motorVal = (20*cube_angle + 6*cube_angle_velocity + 0.1*motor_speed)*1.8;

  // constraint motor voltage to maxVoltage
  if(abs(motorVal) > maxVoltage)
  {                          // just a sign function
    motorVal = maxVoltage * (motorVal/abs(motorVal));
  }

  return motorVal;
}
