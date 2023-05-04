# turtlebot-follower
Gesture recognition using a visual camera on a Turtlebot3 Burger.

# Turtlebot3 Setup
- [Required Resources](#required-resources)
- [Configure Remote PC](#configure-remote-pc)
- [Configure the TurtleBot3](#configure-the-turtlebot3)
- [Connect to the TurtleBot3 from the Remote PC](#connect-to-the-turtlebot3-from-the-remote-pc)

The procedure presented here was developed after pouring through documentaiton and after much trial and error. The main resourse used as a reference is the official Turtlebot [emanual](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup). Some additional steps are included in this procedure that were not very explicit in the emanual.
## Required Resources:
- Turtlebot3 Burger running ROS Noetic and Ubuntu 20.04
- Monitor + HDMI cable
- USB Keyboard
- PC running Ubuntu 20.04

## Configure Remote PC
On the PC, do the following:

1. Select a WiFi network. Using a personal hotspot has been easiest. A network with internet access is preferable since it will make downloading packages easier.

2. Open a terminal window, and make note of the current ip address by typing:
    
        >> ifconfig
	
    Look at some of the examples in the [emanual](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup) (make sure you click the “Noetic” tab at the top) to know what you’re looking for. You want the numbers that follow something like “inet addr:”

3. Next, open the configuration file you’ll need to edit by using:
		
        >> nano ~/.bashrc
    
    Go to the end of the file, and change the `ROS_MASTER_URI` to be `https://IP_OF_PC_HERE:11311`. For example, if the ip obtained from ifconfig was `192.168.122.182`, then `ROS_MASTER_URI=https://192.168.122.182:11311`

4. Similarly, change the next line, `ROS_HOSTNAME=IP_OF_PC_HERE`. For example, `ROS_HOSTNAME=192.168.122.182`.

5. Save and exit the file by using ctrl + x

6. Apply the changes by using:
		
        >> source ~/.bashrc
	(This might actually not be necessary on the PC, so if it doesn’t seem to work don’t worry about it)

## Configure the TurtleBot3
1. Plug a USB Keyboard into the TurtleBot3, and connect the bot to a monitor using an HDMI cable. Both the USB and HDMI ports are on the microcontroller on the top level of the bot.

2. Once connected, turn the robot on (note that in order for the HDMI display mode to activate, the robot MUST be plugged into the monitor BEFORE being turned on).

3. When prompted, login to the raspberry pi using the USB keyboard to input:

        username:
        >>  ubuntu
        password:
        >> turtlebot

4. Connect to the same WiFi network as the PC by editing the netplan config file:
        
        >> sudoedit /etc/netplan/50-cloud-init.yaml
    
    and look for a section that looks like:

        wifis:
                wlan0:
                    optional: true
                    access-points:
                        "YOUR-SSID-NAME":
                            password: "YOUR-NETWORK-PASSWORD"
                    dhcp4: true

    use your desired WiFi network name as `“YOUR-SSID-NAME”` and the password as `“YOUR-NETWORK-PASSWORD”`. 

5. Save and exit the file using ctrl + x. Then apply the changes by running
        
        >> sudo netplan apply
    
    If running into issues, look into answers like [this](https://askubuntu.com/questions/1143287/how-to-setup-of-raspberry-pi-3-onboard-wifi-for-ubuntu-server-with-netplan), as there are other explanations out there.

6. Now that the bot is connected to the same network as the PC, we make note of the robot ip address using the same command as before:
        
        >> ifconfig

7. Edit the ROS config file on the robot by using:
		
        >> nano ~/.bashrc

    Go to the end of the file, and change the ROS_MASTER_URI to be `https://IP_OF_PC_HERE:11311`. For example, if the ip obtained from `ifconfig` was `192.168.122.182`, then `ROS_MASTER_URI=https://192.168.122.182:11311`
    
    Change the next line, `ROS_HOSTNAME=IP_OF_ROBOT`, for example if the ROBOT has an ip of `192.168.122.225`, then `ROS_HOSTNAME=192.168.122.225`

8. Save and exit. Apply the changes by running:
        
        >> source ~/.bashrc
    
    Now that the robot is configured, you can unplug the peripherals. For any future changes on the robot, we can ssh into the raspberry pi (as long as it's turned on and on the same network). 

## Connect to the TurtleBot3 from the Remote PC
Now both the PC and robot are configured such that ROS will be able to connect them and run programs. Instructions for doing different tasks (ie tele-op) can be found in the TurtleBot3 tutorials. It's mostly a matter of copy-pasting the commands they list in the right order.

1. To start a remote connection with the TurtleBot from the PC, open a new terminal window and run:
        
        >> ssh ubuntu@IP_OF_ROBOT

    where `IP_OF_ROBOT` is the ip address of a turtlebot on the same network, for example is is ip address is `192.168.122.225` you’d put `ssh ubuntu@192.168.122.225`

    If the TurtleBot has only just recently been turned on, it may take several tries to connect successfully (the Raspberry Pi takes a minute to boot up). Eventually it will ask you if you want to add the bot as a known host or something, and so enter ‘yes’ or whatever it wants. 
    
2. Login to the bot by using:

        password:
        >> turtlebot

    Now this terminal window will run commands on the turtlebot.

From here, running a routine like tele-op is a matter of running the right commands/files on the PC and the robot. Typically you’ll start by following the bring-up instructions here, followed by the specific routine you want, like the tele-op instructions [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_operation/#basic-operation). Again, make sure that the instructions are specific to **ROS Noetic**, otherwise they may not be guaranteed to work.


