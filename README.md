# FSE-100-Robot-App
See READ ME file for more details

This will be a robot that does automated sanitation work at a hospital using UV-C light. 
The Robot will start at a base, it will then drive to room 1. Once at room 1, it will turn 
left and enter the room. It will turn around and face the ultrasonic sensor towards the door 
to look for movement, if no movement is detected it will commence the cleaning. The robot 
will have two strong servo motors that will traverse and tilt with a UV-C light affixed to 
the end of the tilt lever. The servo motors will traverse 10 degrees at a time from 0 to 
180 degrees. Once the traverse reaches 180 degrees it will tilt the tilt servo to 30 degrees.
This process will repeat until it is either interupted by movement or both servo motors are at
180 degrees. The robot will then exit the room and repeat the process for the other two rooms. 
Once the last room has been disinfected the robot will return to base.

Other functions that will be included in the prototype. There will be a screen that prints out what 
would normally be "Serial.println()". It will print the status of a job and the position of the 
servo motors. There will be other information such as error warnings or movement or an object detected.
There will also be a line tracker to help the system detect borders and get back on track. The motors 
don't spin at the same rate so the analogWrite() need to be adjusted and tailors for certain movement
commands.
