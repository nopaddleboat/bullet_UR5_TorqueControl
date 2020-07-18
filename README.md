# bullet_UR5_TorqueControl
Use bullet the simulate UR5, torque control is used.

1. The project is based on RobotSimulatorMain example, so you need to make sure that you can run RobotSimulatorMain provided by the official website. 

2. Two external libs are required, one is the lib provide by the modern robotics book and the other is the Eigen lib, only the first lib is included, you need you download Eigen by yourself.

3. controllerPhase1.cpp contains the codes for manipulating. The codes are not well commented, I will add comments gradually. Feel free to contact me through swxie@outlook.com if you have any questions.

4. Note that you can easily modify the code to make it run on Mujoco, you may need to modify the urdf file, contact me if you want the xml file for mujoco simulation.

5. Note that a hybrid position/force controller is used in the simulation, it is not the kind of hybrid controller using projection matrix.


