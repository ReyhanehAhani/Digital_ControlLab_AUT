# Basic movements of a robot with RaspberryPi
+ In this code you can move the robot using WSAD keys on your keyboard. (forward/backward/left/right)
+  At first we have a minimal code to run commands.
+ What we need to do here is to let the robot know when and where to move. 
+  Since we want to support both upper and lower case, we change all of them to lower, by using .lower() method
+  Next we want to be able to quit raspberry environment by pressing 'q', so at first we check if the letter enterd is 'q' or not. If so, it won't check other conditions.
+  If the client has pressed anything but 'q', we'll continue to check whether that letter is one of WSAD or not, if so, we enter of those conditions and as a result a command will be sent to the robot. This is a simple command which +-100 indicated robot's velocity and its direction. The rest (15+00) is its standard format.
+  If none of WSAD or q were pressed, it means that we want the robot to stop, so we set the velocity +000; which should always have 3 digits and its maximum is 200 but 100 will suffice.


