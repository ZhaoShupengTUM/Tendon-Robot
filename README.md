# Tendon-Robot

## Introduction
 Bachelor program in CBL Lab in Tianjin University. I am a member of the group and am responsible for the Joystick design, singlechip programming and part of the windows app programming.
 
## Main Software architecture
 robot.h/cpp:  
    configuration part  
    Jacobian part  
    trajectory generation part  
    position and robot configuration estimation part  
    parameters setting part  

 TendonRobot.h/cpp:  
    the class to define the beahvior of the app  

 TendonRobotDlg.h/cpp:  
    a windows dialoge app based on MFC  
    a GUI to interact with the robot  
    trajectory generation modul  
    motion control modul  
    grab control modul  
    display modul  

 mscomm1.h/cpp:  
    automatic generated file from MFC lib  
    
 ## Literature
 Design of a Teleoperated Rod-driven Continuum Robot (link: https://link.springer.com/chapter/10.1007/978-3-030-27535-8_14)
    
   
   ![image](https://user-images.githubusercontent.com/74742676/180636712-c48e6507-b8fe-4706-8aed-ad7deb601c7e.png)


