# Extended Kalman Filter

This [Extended Kalman Filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter#Predict) was used to fuse GPS and IMU sensor data using Bayesian estimation. This was done on an industrial power line robot used to inspect entire spans of electrical lines in an autonimous way. The novelty in the problem comes from the inability to use magnetometer data to reduce drfit in the accelerometer estimates as a result of high current carrying wires. 

Parameters estimated were:

    the temperature and load dependent sag of the line which is important for safety considerattions
    the horizontal displacement of the robot along the line (including the x;y co-ordinates)
    the height of the robot on the line
    the exact attitude (roll; pitch; yaw) of the robot sensor housing

It was shown that fusing sensor data in an optimal way improved the estimates beyond those which could be achieved from using either sensor in isolation. 

Future work will incorporate joint encoder data to follow the kinetic chain of the exact robot support arms. 


    
