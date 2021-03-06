/*
2020.5.12
To simulate the first phase.
*/
#pragma once


#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include<iostream>
#include "Eigen/Dense"
#include "modern_robotics.h"

#define Nj 6 

namespace UR5ctrl {
//these extern variable can be used in any source file which includes controller.h file.
extern double link01;
extern double link12;
extern double link23[2];
extern double link34;
extern double link56;
extern double link45;
extern double link67;
extern Eigen::MatrixXd Im;

extern Eigen::MatrixXd I3;
extern Eigen::MatrixXd ME00;
extern Eigen::MatrixXd ME01;
extern Eigen::MatrixXd ME02;
extern Eigen::MatrixXd ME03;
extern Eigen::MatrixXd ME04;
extern Eigen::MatrixXd ME05;
extern Eigen::MatrixXd ME06;
extern Eigen::MatrixXd ME07;

extern Eigen::MatrixXd Mm00;
extern Eigen::MatrixXd Mm01  ;
extern Eigen::MatrixXd Mm02 ;
extern Eigen::MatrixXd Mm03  ;
extern Eigen::MatrixXd Mm04  ;
extern Eigen::MatrixXd Mm05  ;
extern Eigen::MatrixXd Mm06  ;
extern Eigen::MatrixXd Mm07  ;

extern Eigen::MatrixXd Mm02_2  ;
extern Eigen::MatrixXd Mm03_2  ;

extern Eigen::MatrixXd M01  ;
extern Eigen::MatrixXd M12  ;
extern Eigen::MatrixXd M23  ;
extern Eigen::MatrixXd M34  ;
extern Eigen::MatrixXd M45  ;
extern Eigen::MatrixXd M56  ;
extern Eigen::MatrixXd M67  ;

extern Eigen::MatrixXd G0   ;
extern Eigen::MatrixXd G1   ;
extern Eigen::MatrixXd G2   ;
extern Eigen::MatrixXd G3   ;
extern Eigen::MatrixXd G4   ;
extern Eigen::MatrixXd G5   ;
extern Eigen::MatrixXd G6   ;

extern std::vector<Eigen::MatrixXd> Mlist;
extern std::vector<Eigen::MatrixXd> Glist;

extern Eigen::MatrixXd Slist;
extern Eigen::VectorXd thetalist;
extern Eigen::VectorXd dthetalist;
extern Eigen::VectorXd g;

extern Eigen::MatrixXd allq  ;  //all the historic joint angles
extern Eigen::MatrixXd alldq  ;
extern Eigen::MatrixXd alltime  ;

extern double ctrlTorque[Nj];


extern Eigen::MatrixXd eeTips0; 

extern Eigen::VectorXd eeX; 

extern Eigen::VectorXd f_QX(Eigen::MatrixXd T);

extern Eigen::MatrixXd txtMat;
	
void init_UR5();
void updateEEtips0(double* qpos, double* qvel);
void ctrl1(double* qpos, double* qvel, double* forceSensor, double* torqueSensor ,double curT);

Eigen::MatrixXd calTee(double* qpos, double* qvel);

Eigen::VectorXd getFT(double* qpos, double* qvel, double* forceSensor, double* torqueSensor);
Eigen::VectorXd getJfMInvJT(double* qpos, double* qvel, double* forceSensor, double* torqueSensor);
	
}
	
	
	
