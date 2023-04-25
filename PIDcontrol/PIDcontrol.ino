#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include "PIDcontrol.hpp"
#include <Wire.h>

namespace Settings
{
    //// The break signal to the dc.
    const int dcBreakSignal = 0;

    //// The maximimum signal to add or subtract relative to
    //// the break signal.
    const int dcUpperBound = 255;

    //// The minimum signal to add  or subtract relative to 
    //// the break signal, since BLDCs are unstable at low speeds.
    const int dcLowerBound = 0;

    // The angle at which the cube system should just quit.
    const double unsafeAngle = 360;

    // The angle at which the cube should begin balancing at.
    const double breakAngle = 20;

    // The sampling time of the PID controller in ms.
    const int PIDSampleTime = 1;//15;

    const int tol = 1;
};

struct State
 {
    // The current angle of the cube.
    double currentAngle;

    // The reference angle for the state.
    double referenceAngle;

    State() : currentAngle(0), referenceAngle(0){};
 };

// Controller

double Ku = 36;
double Tu = 0.357;
// Kp, Ki, Kd
//PID controller(25 * Ku, 0.1 * Tu, 0.1 * Tu, Settings::PIDSampleTime);
//PID controller(25 * Ku, 0.1 * Tu, 5 * Tu, Settings::PIDSampleTime);
//PID controller(25 * Ku, 30.1 * Tu, 0.01 * Tu, Settings::PIDSampleTime);
PID controller(5 * Ku, 20.1 * Tu, 12.01 * Tu, Settings::PIDSampleTime);

// The state of the system.
State state;

Adafruit_BNO055 bno;

// pins
int enA = 9;
int in1 = 8;
int in2 = 7;

void setup(void)
{
  Serial.begin(9600);
  delay(100);
  Serial.print("\n ===========Run 1================== \n");
  delay(100);
   // Setup motor control pins
  pinMode(enA,OUTPUT);
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
   
   // turn off to start
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
  bno.setExtCrystalUse(true);
  delay(1000);

}


void loop()
{
    state.currentAngle = getAngleFromIMU(bno);
    state.currentAngle = state.currentAngle +48; // offset
    Serial.print(String(state.referenceAngle) + "-" + String(state.currentAngle) + "\n");    
    if (isnan(state.currentAngle) || fabs(state.currentAngle) >= Settings::unsafeAngle)
    {
        // Some invalid reading was found, just exit.
        printErrorAndExit("Unsafe. Angle is past " + String(Settings::unsafeAngle) + 
                          " degrees. Current angle: "+ String(state.currentAngle));
    }
    else if (fabs(state.currentAngle) >= Settings::breakAngle)
    {
        delay(500);
        printErrorAndExit("Break. Angle is past " + String(Settings::unsafeAngle) + 
                          " degrees. Current angle: "+ String(state.currentAngle));
    }
    else
    {
        double roundedAngle = (float)((int)(state.currentAngle * 10))/10.0;
        double dcDiff = controller.compute(state.currentAngle, state.referenceAngle);
        dcDiff = dcDiff;
      
    if (dcDiff < Settings::breakAngle - Settings::tol /2) // real close
      {
        //Serial.print("\n negative angle");
        digitalWrite(in1,LOW);
        digitalWrite(in2,HIGH);
        analogWrite(enA,dcDiff);
      }

      else if (dcDiff < Settings::breakAngle - Settings::tol) // further
      {
        //Serial.print("\n negative angle");
        digitalWrite(in1,LOW);
        digitalWrite(in2,HIGH);
        analogWrite(enA,dcDiff);
      }
      else if (dcDiff > Settings::breakAngle + Settings::tol/2)
      {
        //Serial.print("\n positive angle");
        digitalWrite(in1,HIGH);
        digitalWrite(in2,LOW);
        analogWrite(enA,dcDiff);
      }
      else if (dcDiff > Settings::breakAngle + Settings::tol)
      {
        //Serial.print("\n positive angle");
        digitalWrite(in1,HIGH);
        digitalWrite(in2,LOW);
        analogWrite(enA,dcDiff);
      }
      else
      {
        //Serial.print("\n close enough");
      }
    }   
    delay(Settings::PIDSampleTime);
}

double getAngleFromIMU(Adafruit_BNO055& bno)
{
    imu::Vector<3> acceleration = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    double mx = acceleration.x();
    double my = acceleration.y();
    double theta = atan2(mx, my) * 180/PI;

    return theta-1.14; // balance offset
}

bool isAccelerationCalibrated(Adafruit_BNO055& bno)
{
    uint8_t acceleration = 0, gyro = 0;
    bno.getCalibration(nullptr, nullptr, &acceleration, &gyro);
    return acceleration > 0;
}

void printErrorAndExit(const String& message)
{
    //esc.writeMicroseconds(Settings::dcBreakSignal);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
     
    Serial.println(message);
    while(true);
}
