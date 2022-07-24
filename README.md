# Tendon-Robot

 Bachelor program in CBL Lab
 The software architecture:
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