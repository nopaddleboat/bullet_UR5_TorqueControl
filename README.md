# bullet_UR5_TorqueControl
![cover_image](https://github.com/nopaddleboat/bullet_UR5_TorqueControl/blob/master/process_data/exp1_2.png)

Use bullet to simulate UR5, torque control is used.

1. The project is based on RobotSimulatorMain example, so you need to make sure that you can run RobotSimulatorMain provided by the official website. 

2. Two external libs are required, one is the lib provided by the modern robotics book [1] and the other is the Eigen lib, only the first lib is included, you need to download Eigen by yourself.

3. controllerPhase1.cpp contains the codes for manipulating. The codes are not well commented, I will add comments gradually. Feel free to contact me through swxie@outlook.com if you have any questions.

4. Note that you can easily modify the code to make it run on Mujoco, you may need to modify the urdf file, contact me if you need the xml file for mujoco simulation.

5. Note that a hybrid position/force controller is used in simulation, it is not the kind of hybrid controller using projection matrix.


[1] Lynch, Kevin M., and Frank C. Park. Modern Robotics. Cambridge University Press, 2017.
