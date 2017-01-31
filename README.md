# Reflow-Oven
My take on the diy relfow oven, inspired by the design by Karl Pitrich (karl@pitrich.com).

This reflow oven is still undergoing development, slowly, but works well enough to
bake some ICs. The reflowreader.py program is used to get a graphical output of the 
current temperature and setpoint (you will need to know which serial port the arduino
is connected to), it is written in python 2.7 and will need the correct interpreter.

The door does not open on its own yet and cannot autotune the PID but it can detect
the mains power frequency. The user will need to open the door manually for cooling.
Also fan control has not been implemented as my oven did not have any of that fanciness.
