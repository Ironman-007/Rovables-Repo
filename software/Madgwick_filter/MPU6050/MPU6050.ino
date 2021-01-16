#include <MadgwickAHRS.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!

Adafruit_MPU6050 mpu;

Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;

void setup() {
  Serial.begin(115200);

  // start the IMU and filter

  filter.begin(25);

  // Try to initialise and warn if we couldn't detect the chip
  if (!mpu.begin())
  {
    Serial.println("Oops ... unable to initialize the mpu6050. Check your wiring!");
    while (1);
  }
  Serial.println("Found mpu6050 9DOF");
  // Set the accelerometer range to 2G
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  // Set the gyroscope range to 250 degrees/second
//  CurieIMU.setGyroRange(250);

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 25;
  microsPrevious = micros();
}

void loop() {
  int aix, aiy, aiz;
  int gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float roll, pitch, heading;
  unsigned long microsNow;

  // check if it's time to read data and update the filter
  microsNow = micros();

//  lsm.read();  /* ask it to read in the data */ 

  /* Get a new sensor event */ 
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  if (microsNow - microsPrevious >= microsPerReading) {

    // read raw data from CurieIMU
//    CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

    // convert from raw data to gravity and degrees/second units
//    ax = convertRawAcceleration(aix);
//    ay = convertRawAcceleration(aiy);
//    az = convertRawAcceleration(aiz);
//    gx = convertRawGyro(gix);
//    gy = convertRawGyro(giy);
//    gz = convertRawGyro(giz);
      ax = a.acceleration.x;
      ay = a.acceleration.y;
      az = a.acceleration.z;
      gx = g.gyro.x*57.2958;
      gy = g.gyro.y*57.2958;
      gz = g.gyro.z*57.2958;
//      gx = 0.0;
//      gy = 0.0;
//      gz = 0.0;
//      mx = m.magnetic.x;
//      my = m.magnetic.y;
//      mz = m.magnetic.z;

    // update the filter, which computes orientation
//    filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    //    Serial.print("Orientation: ");
    if (Serial.available()) {
        char input = Serial.read();
        Serial.print(heading);
        Serial.print(",");
        Serial.print(pitch);
        Serial.print(",");
        Serial.println(roll);
      }
    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
  }
}

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767

  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}
