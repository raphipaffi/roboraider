# roboraider
This is a ROS package for my home-built robot. The package has been developed using
* [Xubuntu 14.04 LTS 64-bit OS](https://xubuntu.org/release/14-04/)
* [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu)
* [Setup ROS](http://www.xaxxon.com/documentation/view/oculus-prime-ros-installation)

Dependencies:
* depthimage_to_laserscan
* openni2_camera
* for a working teleop panel in rviz: modified visualization_tutorials by me

Arduino libraries needed to compile the Arduino code for the low level controller:
* BMSerial
* LiquidCrystal_I2C
* RoboClaw
* UM7 (extended by me)

# Installation of Xubuntu
(Most of these steps were copied from [OculusPrime Setup](http://www.xaxxon.com/documentation/view/oculus-prime-xubuntu-setup))
* Download [Xubuntu 14.04 LTS 64-bit OS](https://xubuntu.org/release/14-04/).
* When given the choice, set to LOGIN AUTOMATICALLY.
* Connect to your wifi network, set to “automatically connect when available” and use a static IP.
* Go to Settings > Software and Updates, change package repository reference to ‘Main’ or the United States.
* Update currently installed packages: $ sudo apt get update
* Install sshd for remote console connection: $ sudo apt-get install openssh-server
* Install a VNC server for remote desktop: $ sudo apt-get install x11vnc
* Run the server by entering: $ x11vnc -display :0
* Go to Settings > Light Locker Settings, turn everything to OFF, set sliders to 0 minutes/never.
* Go to Settings > Power Manager, uncheck ‘Monitor Power Management Control’ and go through the various tabs and set all the sliders to 0 minutes/never.
* If the system doesn’t shut down cleanly, sometimes the Grub boot manager will wait for a menu selection before booting. To avoid this, open the file: /etc/default/grub and add the line: GRUB_RECORDFAIL_TIMEOUT=0
* Run: $ sudo update-grub
* In case of disk checking on startup, you’ll want the system to repair any problems automatically. Edit the file /etc/default/rcS and find the line containing: FSCKFIX=no, change no to yes and uncomment if needed.
* when applications crash, they ask if you want to send a report. Disable this feature by editing the file /etc/default/apport and change the line: enabled=1 to 0
* Go to Settings > Software and Udates and click on the 'Updates' tab. Switch ‘Automatically Check for Updates’ to ‘Never’
* It should be OK to work on the system remotely from here onward.
* Go to [www.google.com/chrome/browser](http://www.google.com/chrome/browser), download the 64-bit ‘.deb’ package and install it using Software Center.
* RUN GOOGLE CHROME FROM THE MENU AT LEAST ONCE – choose the ‘set as default browser’ option when prompted.
* USB ports are only available to root by default in Ubuntu. To give yourself access: $ sudo adduser user_name dialout
* Substitute “user_name” with your username. NOTE: reboot is required for this to take effect.

# ROS Setup



