#include <Adafruit_BNO055.h>
#include "PIDcontrol.hpp"

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
    const double breakAngle = 15;

    // The sampling time of the PID controller in ms.
    const int PIDSampleTime = 15;
};

struct State
 {
    // The current angle of the cube.
    double currentAngle;

    // The reference angle for the state.
    double referenceAngle;

    State() : currentAngle(0), referenceAngle(0) {};
 };

// Controller
// 8 2 0.1
// 21, 0.18,  0.27
// 28, 0.18,  0.045
// 21, 0.18,  0.045
// 28, 0,  0.045

// Best ranked
// 0.6 * Ku, 0.5 * Tu, 0.75 * Tu

double Ku = 36;
double Tu = 0.357;
PID controller(0.6 * Ku, 0.6 * Tu, 0.83 * Tu, Settings::PIDSampleTime);

// The state of the system.
State state;

Adafruit_BNO055 bno;

//Servo dc;

// pins
int enA = 9;
int in1 = 8;
int in2 = 7;

void setup()
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

   //dc.attach(11);
   //delay(1000);
   //dc.writeMicroseconds(Settings::dcBreakSignal);

   // Setup BNOs.
   //if(!bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS))
   //{
   //    printErrorAndExit("Could not connect to imu.");
   //}

  //bno.setExtCrystalUse(true);

  //while (!isAccelerationCalibrated(bno))
  //{
  //  delay(200);
  //  Serial.print("Waiting for accleration calibration");
  //  delay(500);
  //}

   Serial.println("imu is calibrated");
   delay(1000);


   state.currentAngle = getAngleFromIMU(bno);
   Serial.println("System has initial angle: " + String(state.currentAngle));

  delay(500);
  digitalWrite(in1,HIGH);
  delay(200);
  analogWrite(enA,255);
  delay(6000);
  analogWrite(enA,0);
  digitalWrite(in1,LOW);
  delay(500);
  Serial.print("done test");
  delay(5000);

   Serial.println(0);
   Serial.println(Settings::breakAngle);
   Serial.println(-Settings::breakAngle);
}


void loop()
{
    state.currentAngle = getAngleFromIMU(bno);

    if (isnan(state.currentAngle) || fabs(state.currentAngle) >= Settings::unsafeAngle)
    {
        // Some invalid reading was found, just exit.
        //controller.reset();
        printErrorAndExit("Unsafe. Angle is past " + String(Settings::unsafeAngle) + 
                          " degrees. Current angle: "+ String(state.currentAngle));
    }
    else if (fabs(state.currentAngle) >= Settings::breakAngle)
    {
        // Send the break signal and do nothing.
        /*
        Serial.print(state.currentAngle);
        Serial.print(" ");
        Serial.print(0);
        Serial.print('\n');
        */
        //dc.writeMicroseconds(Settings::dcBreakSignal);
        //controller.reset();
        delay(500);
        printErrorAndExit("Break. Angle is past " + String(Settings::unsafeAngle) + 
                          " degrees. Current angle: "+ String(state.currentAngle));
    }
    else
    {
        double roundedAngle = (float)((int)(state.currentAngle * 10))/10.0;
        double dcDiff = controller.compute(state.currentAngle, state.referenceAngle);

        if (abs(dcDiff) > Settings::dcUpperBound)
        {
            dcDiff = dcDiff > 0 ? Settings::dcUpperBound : -Settings::dcUpperBound;
        }

        long dcVal = Settings::dcBreakSignal + Settings::dcUpperBound + dcDiff + Settings::dcLowerBound;
        analogWrite(enA,dcVal);
        Serial.print("dcVal is: " + String(dcVal));
        delay(50);
        //dc.writeMicroseconds(dcVal);

        Serial.print(roundedAngle);
        Serial.print(" ");
        Serial.print(map(dcDiff, 
                         -Settings::dcUpperBound,
                         Settings::dcUpperBound,
                         -Settings::breakAngle, Settings::breakAngle));
        Serial.print('\n');
    }
    delay(Settings::PIDSampleTime);
}

double getAngleFromIMU(Adafruit_BNO055& bno)
{
    imu::Vector<3> acceleration = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    double mx = acceleration.x();
    double my = acceleration.y();
    double theta = atan2(mx, my) * 180/PI;

    return theta + 3;
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
