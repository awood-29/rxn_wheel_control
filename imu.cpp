// for more info on how the MPU6050 chip of the imu works check out the example script "MPU6050_DMP6"

#include "imu.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include "Wire.h"
 
// IMU instance
MPU6050 mpu;
// MPU control/status vars
bool imuReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// imu related finctions
// check if IMU has received data
int imuHasData(){
  return imuReady && mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
}

// read the pitch value from the IMU
float getCubeAngle(){
  Quaternion q;   // [w, x, y, z] <- quaternion values (describes the full oriantation of the cube in 3D-space)
  
  // get the quaternion from the imu
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  
  // calculate the angle of the cube (x-axis) based on the quaternion
  float cube_angle = -_PI_2 + atan2(q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z, 2 * (q.y * q.z + q.w * q.x));
  //float cube_angle = asin(2*(q.w*q.y - q.z*q.x));  // use this if you balance around the y-axis
  
  return cube_angle;
}

// config and init for the DMP of the IMU
int initIMU() {
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setSleepEnabled(false);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    // mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    mpuIntStatus = mpu.getIntStatus();
    imuReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  delay(2000);
  Serial.println(F("Adjusting DMP sensor fusion gain..."));
  mpu.setMemoryBank(0);
  mpu.setMemoryStartAddress(0x60);
  mpu.writeMemoryByte(0);
  mpu.writeMemoryByte(0x20);
  mpu.writeMemoryByte(0);
  mpu.writeMemoryByte(0);
  
  return imuReady;
}
