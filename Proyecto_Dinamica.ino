#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
// Include the library
#include <L298N.h>

// Pin definition
const unsigned int IN1 = 4;
const unsigned int IN2 = 2;
const unsigned int EN = 3;


// Create one motor instance
L298N motor(EN, IN1, IN2);

// Define the sensor objects with the library
VL53L0X tofsensor;
Adafruit_MPU6050 mpu;

const long runtime = 50; // Sensor reading interval, seconds
unsigned long currentTime = millis();

// Contains the same thing as setup, it is to avoid calling setup
void startup(){
  // MPU6050 initialization
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  //Serial.println("MPU6050 Inertial Measuring Unit Found!");

  // ToF Sensor initialization and timeout
  tofsensor.setTimeout(600);
  if (!tofsensor.init())
  {
    Serial.println("Failed to detect and initialize tofsensor!");
    while (1) {}
  }

  // Selection of MPU opereating modes (measuring ranges)
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  // Selection of MPU6050 angular velocity range
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  // Activation and selection of MPU6050 filter bandwith
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  
  // Wait for a start signal from the user
  char startSignal = Serial.read();


  if (startSignal == 's'){
    loop();
  }

  delay(2000);
}

void setup(){
  //Motor speed is set fixed here (temporarily) to avoid mishaps
  Serial.begin(250000);
  Wire.begin();
  motor.setSpeed(40);
  
  // MPU6050 initialization
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  //Serial.println("MPU6050 Inertial Measuring Unit Found!");

  // ToF Sensor initialization and timeout
  tofsensor.setTimeout(600);
  if (!tofsensor.init())
  {
    Serial.println("Failed to detect and initialize tofsensor!");
    while (1) {}
  }

  // Selection of MPU opereating modes (measuring ranges)
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  // Selection of MPU6050 angular velocity range
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  // Activation and selection of MPU6050 filter bandwith
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  
  // Wait for a start signal from the user
  char startSignal = Serial.read();


  if (startSignal == 's'){
    loop();
  }
  else{
    startup();
  }

  delay(2000);
}



void readsensors(){
  /* Read the MPU current values the values that matter for this task are acceleration
    x and y, and angular velocity in z */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    double a_x = a.acceleration.x; //Establish variables to get values out of the struct
    double a_y = a.acceleration.y;
    double w_z = g.gyro.z + 0.14; //Set the zero offsets by adding them
    double accel = hypot(a_x, a_y) - 1.297; 

    // Read the ToF current values and establish timeout
    double r = tofsensor.readRangeSingleMillimeters()-30;
    if (tofsensor.timeoutOccurred()) { Serial.print("ToF TIMEOUT"); }

    /*Print out the values */
    //Serial.print("Distance: ");
    Serial.print(currentTime, 4);
    Serial.print(",");
    Serial.print(r);
    Serial.print(",");
    Serial.print(accel, 4);
    Serial.print(",");
    Serial.print(w_z, 4);
    Serial.print(",");
    Serial.print("\n");
}

const long reading_interval = 50; // Sensor reading interval, milliseconds
unsigned long previousMillis = 0;  // will store last time sensor readings were updated

void loop()
{
  //This variable keeps track of the time since the arduino started
  unsigned long currentMillis = millis();

    // If the interval has passed, then output the move signal and read the sensors
    if (currentMillis - previousMillis >= reading_interval){
    // Update the time
    previousMillis = currentMillis;
    motor.forward();
    readsensors();
  }
  
}
  


  