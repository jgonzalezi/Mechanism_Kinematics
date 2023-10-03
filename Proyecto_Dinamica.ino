#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
// Include the library
#include <L298N.h>

// Pin definition
const unsigned int IN1 = 4;
const unsigned int IN2 = 2;


// Create one motor instance
L298N motor(IN1, IN2);

// Define the sensor objects with the library
VL53L0X tofsensor;
Adafruit_MPU6050 mpu;

const long runtime = 50; // Sensor reading interval, seconds
unsigned long currentTime = millis();
int movetime = 0;

// Contains the setup routine except for the comms. Improves readability
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
  

 while (1){ 
    char startSignal = Serial.read();
    if (startSignal == 's'){
      //Serial.println("The arduino program has started");
      while(1){
        //Serial.println("Please input the time in ms: ");
        movetime = Serial.parseInt();
        if (movetime > 0){
          //Serial.println("TIME RECEIVED");
          break;
        }
      }
      //Serial.println("Starting loop");
      break;
    }
  }

  delay(2000);
}

void setup(){
  Serial.begin(250000);
  Wire.begin();
  startup();
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
int times_ran = 0; //This variable keeps track of the times the loop has ran. It simplifies the timing

void loop()
{
  //This variable keeps track of the time since the arduino started
  unsigned long currentMillis = millis();

    // If the interval has passed, then output the move signal and read the sensors
    if (currentMillis - previousMillis >= reading_interval){
    
    previousMillis = currentMillis; // Update the time
    readsensors();
    motor.forward();     
    times_ran += 1; //We update the times the data collection has ran

    // We multiply times ran * read interval to get the time the mechanism has been moving and reading
    // If its bigger or equal to the defined moving time, we stop. (We do 10% more time just in case)
    if (times_ran * reading_interval >= movetime*1.1){
      Serial.println(times_ran);
      motor.stop();
      Serial.end();
      times_ran = 0; // We reset the times ran. Since it is a global variable, not resetting it causes glitches
      setup();  // Go back to the setup, wait for new data
    }
  }
  
}
  


  