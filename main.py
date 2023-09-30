# These libraries handle the serial comms with the arduino
import serial, time

# Regular expressions library will be used for dealing with the newline characters in the serial monitor
import re

# NumPy will be used to handle the different computations (speeds, positions, etc). It is imported as np by convention
# Note: Numpy handles numerical and array computations, similar to matlab. Be careful when operating with scalars
import numpy as np

# SciPy provides several signal processing and filtering capabilities
# it also provides cumulative integrals by trapezoidal rule
from scipy import integrate
from scipy import signal

# Matplotlib will handle data visualization
import matplotlib.pyplot as plt


import serial
from everywhereml.arduino import Sketch, Ino, H

movetime = int(input("Please enter the desired movement time, in seconds (integer): "))
#motorspeed = int(input("Please enter the desired motor speed in percentage: "))

sketch = Sketch(name="PyDuino")
sketch += Ino('''
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


void start(){
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
  tofsensor.setTimeout(500);
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
    setup();
  }
  delay(1000);
}

void setup(){
  start();
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
    Serial.print(accel);
    Serial.print(",");
    Serial.print(w_z);
    Serial.print("\\n");
}

const long runtime = 3000;
const long reading_interval = 50; // Sensor reading interval, milliseconds
unsigned long previousMillis = 0;  // will store last time sensor readings were updated

void loop()
{
  //This variable keeps track of the time since the arduino started
  unsigned long currentMillis = millis();
  unsigned long executionMillis = millis();

    // If the interval has passed, then output the move signal and read the sensors
    if (currentMillis - previousMillis >= reading_interval){
    // Update the time
    previousMillis = currentMillis;
    motor.forward();
    readsensors();
  }
  
}


  
''')

if sketch.compile(board='Arduino Mega or Mega').is_successful:
    print('Log', sketch.output)
    print('Sketch stats', sketch.stats)
else:
    print('ERROR', sketch.output)

"""
You can specify the exact port
"""
sketch.upload(port='COM7')


######################################################
# MAKE SURE BAUDS ARE EQUAL TO BAUDS IN ARDUINO CODE #
######################################################
arduino = serial.Serial('COM7', 250000, timeout=2)
print("Connected to arduino uno (master)")
time.sleep(1)


start_signal = input("Start movement and measuring? y/n:")
if start_signal == 'y':
    arduino.write(b's')
      

# If we know the amount of samples to collect we can assign it
# Remember, according to the Arduino code the board samples each 50 ms
samples = 20 * movetime

# Create numpy arrays to hold all the values to be read (a, r, omega)
# Shape refers to the array size, in this case its unidimensional with shape = size = number of samples
r = np.empty(shape=samples)
a = np.empty(shape=samples)
omega = np.empty(shape=samples)

# Create a numpy array to hold the time for easy plotting. Since we know each sample occurs each 50ms
# That is every 0.05 s. We can then fill it with a quick for loop to obtain the time vector in a simple fashion
t = np.empty(shape=samples)

t_0 = 0
for i in range(0,samples):
    t[i] = t_0
    t_0 = t_0 + 0.05


for i in range(0,samples):

    # We should read each data line from the arduino. It is decoded as ascii to get a string type object
    data_string = arduino.readline().decode('ascii')

    # The string is split by regex, in order to create a three element list [r, a, omega]
    # The slicing operation [0:-1] eliminates an empty value that appears when the string is split at the end (\n)
    data_string = re.split(',|\n',data_string)[0:-1]
    print(data_string)

    # Assign, for the current iteration, the values of the numpy arrays r, a, omega
    if [ r[i], a[i], omega[i] ] == []:
      continue
    r[i] = data_string[0]
    a[i] = data_string[1]
    omega[i] = data_string[2]


print(r)
print(a)
print(omega)

# PENDING: Filter the data using SciPy libs
r = signal.medfilt(r)
r = signal.wiener(r)
r = signal.medfilt(r)

a = signal.medfilt(a)
a = signal.wiener(a)
a = signal.medfilt(a)

# Find the r_dot and r_doubledot by deriving r using the np.gradient() 
r_dot = np.gradient(r)
r_doubledot = np.gradient(r_dot)

# Find the velocity by integrating a using np.cumsum(), this is the cumulative sum,
# effectively the inverse of the gradient operation
v = integrate.cumulative_trapezoid(a,initial=1)
print("V:")
print(v)

# Find the angle theta by integrating omega
theta = integrate.cumulative_trapezoid(omega,initial=1)

# Find the angular acceleration alpha by deriving omega
alpha = np.gradient(omega)

# Find the position. Use polar coordinates for simplicity, then project into cartesian
pos_x = r * np.cos(theta)
pos_y = r * np.sin(theta)


# PENDING: Plot the data
plt.subplot(2,2,1)
plt.plot(t,pos_x)
plt.plot(t,pos_y)

plt.subplot(2,2,2)
plt.plot(t,a)
plt.plot(t,v)

plt.subplot(2,2,3)
plt.plot(t,r)
plt.plot(t,r_dot)
plt.plot(t,r_doubledot)

plt.subplot(2,2,4)
plt.plot(t,theta)
plt.plot(t,omega)
plt.plot(t,alpha)
plt.show()
