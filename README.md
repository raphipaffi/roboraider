# RoboRaider
This is a ROS package for my home-built robot. The package requires
* [Xubuntu 14.04 LTS 64-bit OS](https://xubuntu.org/release/14-04/)
* [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu)
* ROS Package depthimage_to_laserscan
* ROS Package openni2_camera
* Arduino 1.6.8
* for a working tele-op panel in rviz: modified visualization_tutorials

Arduino libraries needed to compile the Arduino code for the low level controller:
* BMSerial [github.com/KurtE/Orion/tree/master/Libraries/BMSerial](https://github.com/KurtE/Orion/tree/master/Libraries/BMSerial)
* LiquidCrystal_I2C [www.archiduino.com/liquidcrystal_i2c-h-library-arduino-ide-1-6-x/](http://www.archiduino.com/liquidcrystal_i2c-h-library-arduino-ide-1-6-x/)
* RoboClaw 2x15A Motor Controller [www.ionmc.com/downloads](http://www.ionmc.com/downloads)
* My own version of UM7 [github.com/raphipaffi/UM7](https://github.com/raphipaffi/UM7)

# Installation of Xubuntu
(Most of these steps were copied from [OculusPrime Setup](http://www.xaxxon.com/documentation/view/oculus-prime-xubuntu-setup))
* Download and install [Xubuntu 14.04 LTS 64-bit OS](https://xubuntu.org/release/14-04/).
* When given the choice, set to LOGIN AUTOMATICALLY.
* Connect to your wifi network, set to “automatically connect when available” and use a static IP.
* Go to Settings > Software and Updates, change package repository reference to ‘Main’ or the United States.
* Update currently installed packages: <br />
```
$ sudo apt get update
```
* Install sshd for remote console connection: <br />
```
$ sudo apt-get install openssh-server
```
* Install a VNC server for remote desktop: <br />
```
$ sudo apt-get install x11vnc
```
* Run the server by entering: <br />
```
$ x11vnc -display :0
```
* Go to Settings > Light Locker Settings, turn everything to OFF, set sliders to 0 minutes/never.
* Go to Settings > Power Manager, uncheck ‘Monitor Power Management Control’ and go through the various tabs and set all the sliders to 0 minutes/never.
* If the system doesn’t shut down cleanly, sometimes the Grub boot manager will wait for a menu selection before booting. To avoid this, open the file: `/etc/default/grub` and add the line: `GRUB_RECORDFAIL_TIMEOUT=0`
* Run: <br />
```
$ sudo update-grub
```
* In case of disk checking on startup, you’ll want the system to repair any problems automatically. Edit the file `/etc/default/rcS` and change the line containing: `FSCKFIX=no`, to `FSCKFIX=yes`.
* when applications crash, they ask if you want to send a report. Disable this feature by editing the file `/etc/default/apport` and change the line: `enabled=1` to `enabled=0`.
* Go to Settings > Software and Udates and click on the 'Updates' tab. Switch ‘Automatically Check for Updates’ to ‘Never’
* It should be OK to work on the system remotely from here onward.
* Go to [www.google.com/chrome/browser](http://www.google.com/chrome/browser), download the 64-bit ‘.deb’ package and install it using Software Center.
* RUN GOOGLE CHROME FROM THE MENU AT LEAST ONCE – choose the ‘set as default browser’ option when prompted.
* USB ports are only available to root by default in Ubuntu. To give yourself access:
```
$ sudo adduser user_name dialout
```
(Substitute “user_name” with your username. NOTE: reboot is required for this to take effect.)


# ROS Setup
(Most of these steps were copied from [OculusPrime Setup](http://www.xaxxon.com/documentation/view/oculus-prime-ros-installation))
* Install ROS Indigo following the instructions at [wiki.ros.org/indigo/Installation/Ubuntu](http://wiki.ros.org/indigo/Installation/Ubuntu). Choose 'Desktop-Full Install.'
* Install the required ROS Navigation packages: <br />
```
$ sudo apt-get install ros-indigo-move-base ros-indigo-map-server ros-indigo-amcl ros-indigo-openni2-launch ros-indigo-dwa-local-planner ros-indigo-gmapping
```
* Install Git: <br />
```
$ sudo apt-get install git
```
* Create a ROS Workspace in your home folder by entering: <br />
```
$ mkdir -p ~/catkin_ws/src <br />
$ cd ~/catkin_ws/src <br />
$ catkin_init_workspace <br />
$ cd ~/catkin_ws/ <br />
$ catkin_make <br />
$ cd ~/catkin_ws/src <br />
```
* Clone the modified depthimage_to_laserscan package: <br />
```
$ git clone https://github.com/xaxxontech/depthimage_to_laserscan.git
```
* Clone the default ROS openni2_camera package: <br />
```
$ git clone https://github.com/ros-drivers/openni2_camera.git
```
* For the Orbbec Astra camera, drivers have to be installed manually: <br />
```
mkdir ~/temp/ <br />
cd ~/temp/ <br />
wget http://www.xaxxon.com/downloads/orbbec_openni2_files.zip <br />
unzip orbbec_openni2_files.zip <br />
cd orbbec_openni2_files <br />
sudo mv 558-orbbec-usb.rules /etc/udev/rules.d/ <br />
sudo adduser oculus video <br />
sudo mv libopenni2.pc /usr/lib/pkgconfig/ <br />
cd /usr/lib/ <br />
sudo mv ~/temp/orbbec_openni2_files/libOpenNI2.so . <br />
cd OpenNI2/ <br />
sudo mv ~/temp/orbbec_openni2_files/* ./Drivers/ <br />
```
* Add the new workspace to the default ROS environment by doing: <br />
```
$ echo "source $HOME/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
* Reboot.







