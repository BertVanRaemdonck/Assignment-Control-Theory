SIMPLEROBOT README
==================
This project describes how the microOS and components library can be used to interface and control a robot. 

1. Building blocks
==================

The HAL
=======
A Hardware Abstraction Layer is used to make the software as hardware independent as possible. This can for instance be helpful when some pin assignments change, e.g. we wish to switch from 1 serial port to another one for a new hardware build, or we switch from 1 specific motor driver to another one. In order to keep everything streamlined, we make a HAL, which implements the required functionality with different components (picked from the components library?). If multiple hardware builds are expected, the HAL can be a mother class which has several children for different builds, such as HAL1\_1, HAL1\_2 and so on. 

The Communicator
================
The communicator implements serial communication. One can choose to implement a simple serial protocol, for instance with fixed message lengths, or make use of a more elaborate scheme, such as the mavlink protocol. This has been implemented as an example so that the user can start of using debug messages, thread messages, a fixed heartbeat and so on. However, if you which to implement your own protocol, you can make your own communicator object which should inherit from the communicator\_interface. It is recommended to implement some messages as a heartbeat, debug handler and thread info, although not strictly necessary.

The Robot
=========
The robot class is used as a convenience object to store all robot-related data and functionality. It can be a member of the communicator so that the latter can access the robot's data in a transparent way. The robot can for instance contain a updateHook() which runs specific robot functionality, e.g. a feedback controller or sensor updates. By means of a separate robot class and hardware class, various robots are easily defined with the same hardware. This effectively decouples hardware (components) and software (control).

The MicroOS
===========
The MicroOS is what packs it all together! It depends on the communicator to talk to the outside world and on the HAL to initialize the hardware. Most importantly, the microOS contains a list of threads, smaller functions that need sequential execution. The MicroOS makes it thus easy to make a 'multi-threaded' embedded program. Take for instance the feedback control of a robot which should be run at 100Hz. Also, we would like to send some info to the outside world every once in a while, let's say every 20ms (50Hz). Oh, and we should also print a nice text on the LCD screen, maybe every second. These can all be considered to be different threads, a control thread running at 100Hz, a communication thread running at 50Hz and a slow LCD thread running at 1Hz.
They are fitted in the MicroOS using the addThread() function. This function takes 4 arguments. 
* The priority can be (from low to high) LOWEST, BELOWNORMAL, NORMAL, ABOVENORMAL, HIGHEST
* The update period in us
* The update function, which can be any function which is defined static int myfunction(void), mostly defined in the main file
* start: either true when started immediately, or false when using a delayed start. Threads can be added in a stopped state, and activated all together using the MicroOS::start() command.
The MicroOS implements 2 standard threads by default. The first being a slow thread@1Hz, which makes the onboard led of the HALInterface flash and updates the microOSUpdateHook() in the HALInterface. Also, it contains a communication thread with a default speed of 1kHz which handles the communication specified in the communicator's update function. Any other functionality can be added by defining new functions and adding them as threads as is done in the case of the updateRobot().
After adding all threads and starting them, they are updated at the specified intervals by looping over the MicroOS::run() function. This either carries out a scheduled task, or waits for a new one. The list of threads can be monitored using the THREADINFO system request. This is useful to evaluate the thread's average execution time, latency, ... Threads with the highest priority should show the lowest latency, of the order of a few us. The lower the relative priority, the higher the latency will become.
The MicroOS also carries some general purpose input and output variables. They consist of 4 int32_t and 8 floats, which are sent to the world via the microOS' gpio message. They can be set using the functions setGPoutFloat(uint8\_t index, float value) and static void setGPoutInt(uint8\_t index, int32_t value).

2. Altering the code
====================
The example is fairly straight forwardly altered to match own taste. The main thing is to implement the robot.h and robot.cpp class by adding functionality. By means of the debug variables, you can directly communicate with the system. 
