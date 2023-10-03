# These libraries handle the serial comms with the arduino
import serial, time

# Regular expressions library is used for dealing with the newline characters in the serial monitor
import re

# NumPy handles the different computations (speeds, positions, etc). It is imported as np by convention
# Note: Numpy handles numerical and array computations, similar to matlab. Be careful when operating with scalars
import numpy as np

# SciPy provides several signal processing and filtering capabilities
# it also provides cumulative integrals by trapezoidal rule
from scipy import integrate
from scipy import signal

# Matplotlib handles data visualization
import matplotlib.pyplot as plt



#############################################
# MAKE SURE BAUDS ARE EQUAL TO ARDUINO CODE #
#############################################
arduino = serial.Serial('COM9', 250000, timeout=1)
print("Connected to arduino")
time.sleep(1)

start_signal = input("Start movement and measuring? y/n:")
if start_signal == 'y':
    movetime = input("Please enter the desired movement time, in milliseconds: ")
    arduino.write(b's')
    movetime = movetime.encode("utf-8")
    arduino.write(movetime)


listoflists = []

# Retrieve data from the arduino in the form of a list of lists
# Each element is a line [r, a, omega]. Made into list to prevent empty elements. See comments below
while 1:
    
    #Check if there is  data avaliable
    if arduino.in_waiting >= 0:
        
        # Decode each line as ascii to get a string type object
        data_string = arduino.readline().decode('ascii')

        # The string is split by regex, in order to create a three element list [r, a, omega]
        # The slicing operation [0:-1] eliminates an empty value that appears when the string is split at the end (\n)
        data_string = re.split(',|\n',data_string)[0:-1]

        # Check if data collection has ended since at the end the arduino will print out a single number (# of samples)
        if data_string.__len__() == 1:
            samples = int(data_string[0])
            break        
        print(data_string)
        listoflists.append(data_string)

        # Eliminate empty elements of the list so delays in serial communication will not affect the readings
        listoflists = [x for x in listoflists if x]
        
        print(listoflists)
        print(listoflists.__len__())      

# Extract single vectors with each parameter
r = [item[0] for item in listoflists]
a = [item[1] for item in listoflists]
omega = [item[2] for item in listoflists]

# Convert them to lists of floats, since its all strings up to this point
r = [float(item) for item in r]
a = [float(item) for item in a]
omega = [float(item) for item in omega]


# Create a numpy array to hold the time for easy plotting. Since we know each sample occurs each 50ms
# That is every 0.05 s. We can then fill it with a quick for loop to obtain the time vector in a simple fashion
t = np.empty(shape=samples)
t_0 = 0
for i in range(0,samples):
    t[i] = t_0
    t_0 = t_0 + 0.05



# Create numpy arrays to hold all the values to be read (a, r, omega)
# Shape refers to the array size, in this case its unidimensional with shape = size = number of samples
r = np.array(r)
a = np.array(a)
omega = np.array(omega)


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
