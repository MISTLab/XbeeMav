##Description

The "xbee_ros_node" package provides many tools (ROS nodes) to configure, test and communicate Xbee devices.
Four nodes are provided in the package:
  * config
  * xbee_mav
  * test_controller
  * test_buzz

The "test_controller" and the "test_buzz" are dummy nodes. They are used only for testing purposes. The "config" node is used to configure the Xbees. The "xbee_mav" is used to communicates between Xbee devices in different modes (e.g. SOLO and SWARM).

##Prerequisites

  * Linux OS
  * ROS
  * Serial device drivers. For FTDI, the Virtual COM port (VCP) drivers are mandatory (http://www.ftdichip.com/Drivers/VCP.htm).
      
##Configuration of Xbee devices

To configure an Xbee device, the "config" node is required. To build the "config" node:

  1. Uncomment the following lines in "xbee_ros_node/CMakeLists.txt":

        add_executable(config src/main.cpp src/XbeeModule.cpp src/XMLConfigParser.cpp)
        target_link_libraries(config ${catkin_LIBRARIES})
      
  2. Insert the correct path to the config file “Resources/Xbee_Config.xml” in “XMLConfigParser.cpp” (line 26):

        FILE_NAME = “/home/mistlab/catkin_ws/src/xbee_ros_node/Resources/Xbee_Config.xml”
      
  3. Build the package:

        $ cd ~/catkin_ws
        $ catkin_make
      
To configure an Xbee device run the "config" node. By default, the VCP and the baud rate are respectively "/dev/ttyUSB0" and "9600". Before running the “config” node, please make sure:

  1. The Xbee device is plugged.
  2. The serial adapter driver is installed.
  3. You have full access to the correspondent serial port (e.g. ttyUSB0):
            
            $ cd ~/../../dev/
            $ sudo chmod 666 ttyUSB0
            
Now you can run the “config” node (after running roscore):

            $ cd ~/catkin_ws
            $ rosrun xbee_ros_node config /dev/ttyUSB0 9600
            
If this command generates an error:

  * Error: open: No such file or directory:
    1. Make sure the Xbee device is plugged.
    2. Make sure the appropriate driver is installed correctly.
    3. Make sure the path to the VCP (e.g. /dev/ttyUSB0) is correct.
    4. Install the newest Xbee firmware with “XCTU” and Write the default config.
     
  * Error: open: Permission denied:
  
  You do not have full access to the serial port. Please execute these commands:
    
      $ cd ~/../../dev/
      $ sudo chmod 666 ttyUSB0
      
  * Time Out: The Xbee Module is Not Responding:
  
  Baud rate mismatch. Please make sure the introduced baud rate (e.g. 9600) matches the baud rate used by the Xbee (by default is 9600). If the Xbee was previously configured with the “config” node the baud rate will be 230400.
      
  * Error: Config File Not Found:
  
  The path to the config file is incorrect. Change it in “XMLConfigParser.cpp” (line 26):
    
      FILE_NAME = “/home/mistlab/catkin_ws/src/xbee_ros_node/Resources/Xbee_Config.xml”
      
The configuration will be loaded from "Resources/Xbee_Config.xml". Most of the existing values in the config file are optimized for Digi-Mesh. Some parameters are default values. However, all parameters can be edited. Please refer to **Table.1** before editing any parameter. More details can be found in “Xbee_Manual.pdf”. The commands table is in Chapter 3 (page 28).

**Table.1:** Most relevant Xbee Parameters.

| Key |	     Name     |	Chosen Parameter  |	Justification for chosen setting |
|:---:|:-------------:|:-----------------:|:---------------------------------:|
|CM   | Channel Mask  | 7FFF              | The channels should be allowed to jump on any available channel (Unless a particular channel is very bad).|
|HP	  | Preamble ID	  | 1	                |This parameter shouldn't be left as is as new not configured radios transmit through this default preamble setting and cause interference by default. Any value from 1-7 should be fine.|
|ID	  | Network ID	    | 5FFF	            | As with HP, this parameter should be changed from its default value because it is a factory setting that will introduce new not yet configured radios to the network. Any other value lower than 7FFF should be fine.|
|MT	  | Broadcast Multi-Transmits | 3	| MAC-level re-transmits. No reason for change until thoroughly tested.|
|PL	  | TX Power Level	| 4	| Should always be Highest [4] unless very high interference is noted (unlikely in our use case). Should only be decreased for range tests.|
|RR	  | Unicast Retries	| A	| MAC-level retries. No reason for change until thoroughly tested.|
| CE	| Routing/Messaging Mode |	0	|Should be standard for DigiMesh. (Needs validating)|
|BH	  | Broadcast Hops	| 0	| When set to 0, the broadcast hops will occur until NH is attained.|
| NH	| Network Hops |	7	| The maximum number of hops throughout the network. We could potentially link this parameter to the minimum necessary data bandwitdth (more hops = less bandwitdh).|
| MR	| Mesh Unicast Retries |	1	| No reason for change until thoroughly tested.|
|NN	 | Network Delay Slots |	3	| Random delay to alleviate network congestion, especially when broadcasting. Needs to be tested in a very dense mesh.|
|DH	 | Destination Addr.High	| 0	| Should be left to default value which will broadcast a message if entering AT mode (even though we will stay in API mode).|
|DL	 | Destination Addr. Low |	FFFF |	Should be left to default value which will broadcast a message if entering AT mode (even though we will stay in API mode).|
|NI	 | Node Identifier	| ‘Node 1’ |	This string could be set to a useful identifier for us. It allows up to a 20 characters long ASCII string. Its use needs to be evaluated with the team.|
|NT	 | Network Discov. Back-off|	82 |	This value represents a timeout for network discovery, the value is multiplied by 100ms. Network discovery is basically transparent to us (the user).|
|NO	 | Network Discov. Options |	0	| Bit 2 of this bitfield adds RSSI to signal, could be useful for us.|
|CI	 | Cluster ID	| 11	| Parameter to be studied. May be useful to identify swarms or camps.|
|EE	| Encryption Enable |	0	| Enabling encryption slightly impacts the performance (2% according to data from manual) and should be activated. This option almost completely disables the existing possibilities for accidental or voluntary (hacking) interference from other radios.|
|KY	| AES Encryption Key |	empty	| 128-bit AES Key. To be generated and shared across all radios.|
|BD	| Baud Rate	| 230400 |	To take advantage of the full transmission rate (theoretically 200 kbps), the serial communication baud rate should be adjusted according to the bandwidth.|
|NB	| Parity |	0 |	Default. |
|SB	| Stop Bits	| 0	| Default. |
|RO	| Packetization Timeout	| 3	|Default.|
|FT	| Flow Control Threshold	| 13F	| Since we are using the highest baud rate, we might need to increase the flow control threshold. Testing will tell us if the integrity of the data is fine.|
|AP	| API Enable |	1	| The XBee has to be set to API Mode to receive API frames. This allows recovering useful information in the headers of the frames such as source and destination address. We shouldn't ever need to escape API mode so we chose option [1]. Otherwise, the escape bytes would have to be escaped each time they occur. |
|AO	| API Options	| 0	| Will need to change to [1] (Explicit Rx Indicator) if we wand to be able to read Cluster ID. Otherwise, not very useful. |
 
## Test two Xbee devices

We consider the following setup (**Fig.1**). The block (Drone + Manifold) can be replaced by any Desktop/Laptop meeting the prerequisites.

**Fig.1:** Experimental Setup:
![][fig1]
[fig1]: https://github.com/MISTLab/XbeeMav/tree/master/Resources/Fig1.png "Fig.1"

One of the drones will behave as a Master while the other one will act as a Slave. The Master drone will send commands to the Slave drone.
Each drone is running a dummy flight controller node "test_controller". According to the drone type Master/Slave, the "test_controller" node will respectively send or receive and display a command. The commands in the Master drone will be introduced with the keyboard. When a command is received in the Slave drone, it will be printed on the screen. The following table (**Table.2**) depicts the keys of each command:

**Table.2:** Keys and Commands.

| Keys |	Commands |
|:----:|:---------:|
| 7	| Take Off|
| 8	| Land |
| 9	| Return To Launch |
| 21	| New Waypoint |
| 22	| Start Mission |
| 23	| Start Mission |


**Fig.2:** ROS nodes running on the drone:
![][fig2]
[fig2]: https://github.com/MISTLab/XbeeMav/tree/master/Resources/Fig2.png "Fig.2"

The communication between both drones is performed with Xbees. The “xbee_mav” node (**Fig.2**) will handle all communications with other ROS nodes (test_controller(Flight Controller) or test_buzz (ROS Buzz)) and the connected Xbee device. Therefore, both Xbees must be configured for Digi-Mesh with the maximum baud rate (230400).
We recognize two modes of communications:
  * SOLO mode: The "xbee_mav" will communicate only with the "test_controller" through services. By default, one of the following services will be active according to the drone type (Master/Slave):
    * “xbee_cmd”: The drone is Master (The server is implemented in the "xbee_mav" side while the client is implemented in the "test_controller" side).
    * “mav_dji_cmd”: The drone is Slave (The server is implemented in the "test_controller" side while the client is implemented in the "xbee_mav" side).
  * SWARM mode: The "xbee_mav" will communicate only with "test_buzz" through topics. (“inMavlink” and “outMavlink”).

All topics and services names of the “xbee_mav” node can be edited in the launch file "launch/xbeemav.launch". For other nodes (test_controller and test_buzz), topics and services names are hardcoded. Thus, they need to be modified in the source code.

####SOLO mode

The "test_controller" node is needed:
  1. Uncomment the following lines in "CMakeLists.txt":
        add_executable(test_controller src/TestController.cpp)
        target_link_libraries(test_controller ${catkin_LIBRARIES})
  2. Build the package:
        $ cd ~/catkin_ws
        $ catkin_make
  3. Run the "test controller" node in Matser/Slave (after running roscore):
        $ cd ~/catkin_ws
        $ rosrun xbee_ros_node test_controller master 
  4. Run the "xbee_mav" node in SOLO mode. The drone type (Master/Slave) and the communication mode (SOLO/SWARM) need to be specified for the "xbee_mav". You can change these parameters in the launch file “launch/xbeemav.launch”:
        $ cd ~/catkin_ws
        $ roslaunch xbee_ros_node xbeemav.launch
  5. Introduce some commands to the "test_controller". If the network was configured correctly, commands sent from one side should be received and displayed in the opposite side.

####SWARM mode

The "test_buzz" node is required:
  1. Uncomment the following lines in "CMakeLists.txt":
        add_executable(test_buzz src/TestBuzz.cpp)
        target_link_libraries(test_buzz ${catkin_LIBRARIES})
  2. Build the package
        $ cd ~/catkin_ws
        $ catkin_make
  3. Run the "ros_buzz" node. This node is independent of drone type:
        $ cd ~/catkin_ws
        $ rosrun xbee_ros_node test_buzz
  4. Run the "xbee_mav" node in SWARM mode. Run one drone as Master and the other one as Slave (Modify the launch file).
        $ cd ~/catkin_ws
        $ roslaunch xbee_ros_node xbeemav.launch

Random payloads (Mavlink messages) will be created in the Master drone and transferred through the Xbees to the Slave drone for display. Payloads are arrays with random sizes of random 64 bits integers.

## Communicate drones in a Swarm 

We consider the same setup in the testing phase (**Fig.1**). The "test_controller" and the "test_buzz" dummy nodes need to be replaced with real ones (“fligh_controller” and “ros_buzz”).
You can download the real nodes by clicking on the correspondent link:
  * flight_controller : (https://git.mistlab.ca/dasto/djiros_ws)  
  * ros_buzz : (https://github.com/MISTLab/ROSBuzz.git)

To run drones in SWARM mode you need to:
  1. Comment the following lines in “xbee_ros_node/CMakeLists.txt”:
  
        add_executable(config src/main.cpp src/XbeeModule.cpp src/XMLConfigParser.cpp)
        target_link_libraries(config ${catkin_LIBRARIES})
        add_executable(test_controller src/TestController.cpp)
        target_link_libraries(test_controller ${catkin_LIBRARIES})
        add_executable(test_buzz src/TestBuzz.cpp)
        target_link_libraries(test_buzz ${catkin_LIBRARIES})
        
  2. Build the three packages.
  3. Run the launch file (this will run the flight_controller, ros_buzz and xbee_mav). The "xbee_mav" should run in SWARM mode. The drone type (Master/Slave) is not needed.
