# Enviro3D
A 3D environment scanner build using the MSP432E401Y microcontroller

________________

## General Description: 


The Enviro3D is a 3D Scanner that utilizes a ToF sensor and other components to accurately create a 3D mapping of a given environment. The processes of generating a 3D render from start to finish can be grouped into three main sections: Data Acquisition, Data Processing, and Data Visualization.


During data acquisition, the device uses it’s ToF sensor to collect distance measurement of the environment. The ToF sensor uses a 940 nm invisible laser and calculates the distance of a surface based on the time taken for the beam of light to reflect and return to the sensor. Once this data is collected, it is sent to the computer via serial I2C protocol in which it is received by a python program that is capable of receiving serial data. The python is then parsed and stored for processing. This Repeats until the stepper motor the ToF is mounted on completes a 360 degree rotation to ensure that enough distance data points are gathered. 


During data processing and data visualization, the distance data received is converted into XYZ coordinates which are then stored in a XYZ file and imported in a 3D processing program to generate a 3D visualization. These processes will be analyzed in detail in the coming sections. 

### Device Overview:
  
![alt text](https://github.com/justinalexchan/Enviro3D/blob/master/images/image15.PNG)

### Block Diagram (Data flow graph):
![alt text](https://github.com/justinalexchan/Enviro3D/blob/master/images/image5.png "Logo Title Text 1")


### Device Characteristics:
![alt text](https://github.com/justinalexchan/Enviro3D/blob/master/images/image14.PNG)
![alt text](https://github.com/justinalexchan/Enviro3D/blob/master/images/image13.png)
### Detailed Description:
#### Distance Measurement:
A key component of measuring distances was the VL53L1X ToF Sensor. The VL53L1X uses a VCSEL (Vertical Cavity Surface Emitting Laser) to emit a 940 nm Infrared laser to time the reflection to the target. Although it’s distance is limited to 4 meters, it can actually measure distances with millimeter accuracy. (Distances are measured in mm)  
The following is the wiring diagram to connect the VL53L1X to the microcontroller. Only the SDA and SCL wires carry data. Looking at the circuit schematic (Figure 1), a better view of how the VL53L1X is connected with respect to other components of the system. 


![alt text](https://github.com/justinalexchan/Enviro3D/blob/master/images/image1.png "Logo Title Text 1")

VL53L1X Wiring Diagram. 2020. 




#### Communication:
The VL53L1X uses I2C interface to communicate with other devices. To use the I2C interface, the SDA and SCL pins have to be wired to GPIO pins on the microcontroller. Note the SDA pin is used for data transmitting (Digital input/output) while the SCL pin is only used as serial clock input (Digital input). 


Note communication between devices is done through the Universal Asynchronous Receiver/
Transmitter (UART) which must first be initialized via code. 


#### Data Acquisition and Transmission Process:
The following is a brief flowchart of the process:
![alt text](https://github.com/justinalexchan/Enviro3D/blob/master/images/image10.png "Logo Title Text 1")

The VL53L1X must first be initialized to begin its measurements. Initialization all happens within the code and includes processes like booting the ToF chip, clearing interrupts, setting up default settings and enabling ranging. The following step is to start the measurement and to wait for the data to be ready before. Once the data is ready, it is sent to the microcontroller which then sends it to a PC. This is done with the help of various UART circuits built into the microcontroller.


Once the process above is finished, the microcontroller tells the stepper motor to rotate by a set amount and the process repeats until the stepper motor is rotated 360 degrees.  The programming logic flowchart (Figure 2) shows how the logic of the process works from start to finish.


#### Conversion to the XYZ Plane:
![alt text](https://github.com/justinalexchan/Enviro3D/blob/master/images/image9.png "Logo Title Text 1")

Once data is sent from the microcontroller to the PC, a Python program receives it and starts parsing the data. 

#### About the Python program:
The program is written to do four things:
1. Receive serial data
2. Parse bits into distance data
3. Convert distance data to XYZ coordinates
4. Output XYZ coordinates to an XYZ file
The program utilizes various python libraries to accomplish these tasks, the most notable being the PySerial and Math libraries. PySerial is used to complete the task of receiving serial data. The incoming data is received in single bits and therefore must be separated by the new line ”\n”. This data is then stored in an array and converted into XYZ coordinates. This is done using the following equations:


#### Equations:
X = X

Y = sin(θ) * distance

Z = cos(θ) * distance
	
Note: The value of X is predetermined and increments accordingly while the value of θ is determined based on the turning angle of the motor and will also increment according (512 steps = 360 degrees, 8/512 steps  * 360 = 5.625 per motor turn)


#### Displacement Measurement:
In this system, displacement starts at zero and incremented by a predetermined value. For testing, the value of 100 (mm) was used. The value is recorded in the python program and will automatically increment after a full rotation has been completed by the stepper motor. 


The MPU-9250  IMU sensor can be implemented similarly to how the ToF sensor was implemented. The code would simply require various new methods to initialize, boot the sensor, and get various acceleration measurements. The device will have to be programmed to start taking measurements one stepper stops moving and stops when the motor starts moving again. The measured data can then be sent to a PC and parsed accordingly. (Note: The MPU-9250 has SDA and SCL pins which makes the wiring very similar to the ToF sensor).


Knowing that displacement is the double integral of acceleration, the x-displacement data can be calculated using the acceleration measurements taken by the IMU and by counting the total amount of measurements taken (which can be used to determine the total time by multiplying it with the frequency of measurements taken). 


#### Visualization:
Two methods of visualization were used in testing:
1. Using Open3D to plot the 3D contours based on the point cloud
2. Using MeshLab to construct a surface based on the point cloud generated 


Both of these methods were used on a computer with the following specs:
-        Processor: Intel i5 3.3GHz
-        Graphics: Intel HD620
-        Memory: 8GB
-        Storage: 256GB SSD
-        OS: Windows 10


The first step to visualize the scanned environment is generating a point cloud from the XYZ coordinates created during the data conversion process (refer to programming logic flowchart for a better idea). A simple way to do this is to store the XYZ coordinates in an XYZ file which can then be imported into programs for 3D processing. 

![alt text](https://github.com/justinalexchan/Enviro3D/blob/master/images/image4.png "Logo Title Text 1")

The first method of visualization uses a python library called Open3D. Open3D is an open sourced library that allows for the processing of 3D data. Using this library, the program can read the points stored in a given XYZ file and display it as a 3D model. Lines can also be generated that connect the various points to generate a 3D contour model of the scanned environment.  
*Note: Additional libraries can be imported such as trimesh which utilizes a ball pivoting algorithm to generate surfaces that can also be used to create more realistic looking models.


An issue with using the Open3D library is that it requires a lot of graphic processing power. Without a dedicated GPU rendering will often crash or not generate a 3D visualization at all.
  
![alt text](https://github.com/justinalexchan/Enviro3D/blob/master/images/image2.png "Logo Title Text 1")
Method two solves that issue by providing a more optimized program that can perform 3D renders with less graphic processing power. MeshLab is a mesh processing software that can generate detailed meshes from point clouds. By importing the XYZ file into the programming and using various tools like Poisson Surface Reconstruction and Ball Pivoting Surface reconstruction, detailed 3D renderings can be generated which can also be exported as STL files. 

#### Application:
An application of this device is using it to map the internal structure of a building. The following will be an example usage of the device in mapping a hallway inside a house. 
  
![alt text](https://github.com/justinalexchan/Enviro3D/blob/master/images/image11.png "Logo Title Text 1")
  

Images (1) point cloud of hallway (2) 3D render based on point cloud (3) birds eye view of 3D render


The following scan was created by taking 39 x-axis slices that were 10cm apart. The first image shows the point cloud generated by the python program while the second and third image shows different views of the 3D rendered model. Note that the two pieces extruding from the hallway are rooms with open doors.


Setup:
Follow the wiring schematic (Figure 1) to connect the various components together. When the device is assembled, ensure that 2DX4FinalProject.uvprojx is flashed onto the microcontroller. 


The following is steps to setup the software for data collection:
1. Ensure that the microcontroller is connected to a computer via USB
2. Open and run serialDataReciever.py (You must have python 3 installed)
3. Pressed the red Button connected to the microcontroller to begin scanning
4. Wait until scanning is finished
5. Repeat steps 3 and 4 until the environment is scanned


The XYZ coordinates data will be saved in an XYZ file named “coordinates3D.xyz” in the same directory as the python program.  The data can be imported into Meshlab in which a 3D model can be generated by going to: Filters > Remeshing,Simplification and Reconstruction > Surface Reconstruction: Ball Pivoting > Apply.


*Note: The XYZ planes are defined as:
* X → Forward movement
* Y  and Z → Vertical Plane Slice
The program utilizes serial communication with the following settings:
* COM port → COM3
* Baud Rate → 115200 BPS


#### Circuit Schematic:
![alt text](https://github.com/justinalexchan/Enviro3D/blob/master/images/image7.png "Logo Title Text 1")

Figure 1
________________
Programming Logic:
![alt text](https://github.com/justinalexchan/Enviro3D/blob/master/images/image12.png "Logo Title Text 1")

Figure 2




________________
