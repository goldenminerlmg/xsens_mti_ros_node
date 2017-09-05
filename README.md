# Changes in this Fork

* Added non-custom messages for gps and IMU
* Publish both the RAW imu and the gravity subtracted INS imu readings
* Publish the covariance in the GPS fixed message
* Fixed getting data from MTI-G-700 sensor
* Fixed publishing of multiple types of the same readings (i.e. free and raw acceleration)
* Publish height and raw pressure readings
* Changed device class to the [ethzasl_xsens_driver](https://github.com/ethz-asl/ethzasl_xsens_driver) implementation
* Changed from "custom_msg" to "xsens_msgs" package for custom messages



## Running the Xsens MTi ROS Node
1. Clone into your ros workspace (tested on Kinetic)
2. Build the package (ensure you have gps common installed, should be by default)
3. Run the example launch file `roslaunch xsens_driver mti_G_700.launch`
4. Run `rostopic list` in a separate terminal to see topics



## Prerequisites

* Install the MTi USB Serial Driver
  ```sh
  $ git clone https://github.com/xsens/xsens_mt.git
  $ cd ~/xsens_mt
  $ make
  $ sudo modprobe usbserial
  $ sudo insmod ./xsens_mt.ko
  ```

* Install gps_common
  ```sh
  $ sudo apt-get install ros-distro-gps-common
  ```




## Troubleshooting

* The Mti1 (Motion Tracker Development Board) is not recognized.
  Support for the Development Board is present in recent kernels. (Since June 12, 2015).If your kernel does not support the Board, you can add this manually

    $ sudo /sbin/modprobe ftdi_sio
    $ echo 2639 0300 | sudo tee /sys/bus/usb-serial/drivers/ftdi_sio/new_id


* The device is recognized, but I cannot ever access the device
  Make sure you are in the correct group (often dialout or uucp) in order to access the device. You can test this with

        $ ls -l /dev/ttyUSB0
        crw-rw---- 1 root dialout 188, 0 May  6 16:21 /dev/ttyUSB0
        $ groups
        dialout audio video usb users plugdev

    If you aren't in the correct group, you can fix this in two ways.

    1. Add yourself to the correct group
        You can add yourself to it by using your distributions user management
        tool, or call

            $ sudo usermod -G dialout -a $USER

        Be sure to replace dialout with the actual group name if it is
        different. After adding yourself to the group, either relogin to your
        user, or call

            $ newgrp dialout

        to add the current terminal session to the group.

    2. Use udev rules
        Alternatively, put the following rule into /etc/udev/rules.d/99-custom.rules

            SUBSYSTEM=="tty", ATTRS{idVendor}=="2639", ACTION=="add", GROUP="$GROUP", MODE="0660"

        Change $GROUP into your desired group (e.g. adm, plugdev, or usb).


* The device is inaccessible for a while after plugging it in
    When having problems with the device being busy the first 20 seconds after
    plugin, purge the modemmanager application.
