# Cyclone V1
Cyclone is a Small flight computer PCB with:
- An HP203B barometer
- An LSM6DSVETR IMU
- A SAM-M10Q GPS
- 1S LiPo Charging
- An SX1262 LoRa Radio Module
- An OV2640 Camera
- An SD card slot

And a software setup that handles:
- Kalman and complementary sensor filtering with 6DOF quaternion that allows for aerodynamic control systems.
- Over-the-air code uploads through WiFi
- Packet protocols over WiFi UDP and LoRa to handle pre-flight testing and in-flight data telemetry
- 4Mb of on-board data storage for detailed flight telemetry post-flight
- Serial IO through the secondary microcontroller that allows for turning the flight computer into a ground station using any Android phone
- Automatic triggering of video recording (about 320x240 at 30 fps) for flight-testing and data verification.