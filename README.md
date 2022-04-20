# Contents
- `Alex` Contains the Arduino code and header fike
- `calibration` Contains the Arduino code used to find the constants for ultrasonic sensor distance calibration
- `ros_epp` Contains the ROS workspace to run RPlidar and HectorSLAM nodes

# Compilation
Upload `Alex/Alex.ino` to the Arduino 
Compile `tls-alex-client.cpp` on the **Host Laptop**
```
g++ tls-alex-client.cpp make_tls_client.cpp tls_client_lib.cpp tls_pthread.cpp tls_common_lib.cpp -pthread -lssl -lcrypto -o tls-alex-client
```
Compile `tls-alex-server.cpp` on the **Raspberry Pi**
```
g++ tls-alex-server.cpp tls_server_lib.cpp tls_pthread.cpp make_tls_server.cpp tls_common_lib.cpp serial.cpp serialize.cpp -pthread -lssl -lcrypto -o tls-alex-server
```
# Running
## Initialise ROS
1. Setup ROS Master (Raspberry Pi) and Slave (Laptop) 
```
export ROS_MASTER_URI=http://<ipaddress>:<port>
export ROS_HOSTNAME=<ipaddress>
```
2. Navigate to `ros_epp` and run `source devel/setup.bash` on every terminal that you are going to use for ros
2. Run `roscore` on Master
3. Run `roslaunch rplidar_ros rplidar.launch` on Slave
4. Run `roslaunch hector_slam tutorial.launch` on Master

## Control Alex
1. Run `./tls-alex-server` on Raspberry Pi
2. Run `./tls-alex-client` on Laptop
3. Follow on screen instructions

