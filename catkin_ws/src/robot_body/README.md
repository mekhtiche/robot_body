# robot
The system is tested on ubuntu Xenial armhf
Image avalaible on MEGA Cloud https://mega.nz/#!oIFhUZLa decryption Key: !K1EM5aygbKRRNzjF8cbqpAs_myevqVCU3P7QGwaH8RE

this package need : 

                    ROS Kinetic 

                    pyserial
                    
                    enum
                    
                    python-tk
                    
Installation:

  ROS:
  
    install ROS Kinetic http://wiki.ros.org/kinetic/Installation/Ubuntu
        
  
  Python packages:
  
      $ sudo apt-get install python-pip

      $ sudo pip install pyserial

      $ sudo pip install enum

      $ sudo apt-get install python-tk

  now use git clone to download the package:

      $ cd catkin_ws/src

      $ git clone https://github.com/mekhtiche/robot_body.git

      $ cd ..

      $ catkin_make
  
  source the work space
  
      $ sudo nano .bashrc
    
    in the end of the file add "source ~/catkin_ws/devel/setup.bash"
    
    
  To launch the robot:

      $ roslaunch robot_body Robot_start.launch

      $ roslaunch robot_body Robot_start_stat.launch

  To record sign:

      $ roslaunch robot_body Recording.launch

      $ roslaunch robot_body Recording_stat.launch

  Network configurations:
  
    Create new Wi-Fi network and call it ROBOT and set the mode as hotspot.
    
    In the end of ,bashrc file add:
      
      export ROS_MASTER_URI=http://odroid:11311
      export ROSLAUNCH_SSH_UNKOWN=1
  
  if Pycharm not working you need to install Oracle Java JDK https://www.rosehosting.com/blog/how-to-install-java-on-ubuntu-16-04/ .
  
      $ sudo apt-get update && sudo apt-get -y upgrade
      $ sudo apt-get install software-properties-common
      $ sudo apt-add-repository ppa:webupd8team/java
      $ sudo apt-get update
      $ sudo apt install oracle-java8-installer
      $ java -version
      
      java version "1.8.0_121"
      Java(TM) SE Runtime Environment (build 1.8.0_121-b13)
      Java HotSpot(TM) 64-Bit Server VM (build 25.121-b13, mixed mode)
      
      
