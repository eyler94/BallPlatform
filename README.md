# BallPlatform
Ball platform software for sim and real world. Built for python3.

Ball tracking sources code from PyImageSearch. 
Requires the following libraries uncommon libraries: opencv-python and imutils.

Be aware, HSV values are input as follows.
(H: 0 - 180, S: 0 - 255, V: 0 - 255)

Also, hsv_detect was sourced from Aruldd at https://stackoverflow.com/questions/10948589/choosing-the-correct-upper-and-lower-hsv-boundaries-for-color-detection-withcv .

Arduino serial communication requires the following uncommon libraries: serial, pyserial.

Also, make sure you add the current user to the dialout group. This may need to be in the .bashrc 

See example beneath:

$sudo usermod -a -G dialout $USER

Then close the terminal and reopen it. 

Also, check the port number:

$ls /dev/ttyUSB*


