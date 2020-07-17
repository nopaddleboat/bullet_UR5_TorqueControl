/*
2020.5.12
To simulate the first phase.
*/
#include "mujoco.h"
#include "glfw3.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include<iostream>
#include "Eigen/Dense"
#include "modern_robotics.h"
#include "controller_polish.h"
#include<fstream>  //file output
#include<string>
#include <math.h>

#define PI           3.14159265358979323846
#define simN 500*10   //10s with Ts=2e-3
#define Nj 6  //number of joints
#define Ncol_txtMat 11  //configuration file

namespace UR5ctrl {
 mjtNum link01;
 mjtNum link12;
 mjtNum link23[2];
 mjtNum link34;
 mjtNum link56;
 mjtNum link45;
 mjtNum link67;
 
  Eigen::MatrixXd Im(4,4);

  Eigen::MatrixXd I3;
  Eigen::MatrixXd ME00(4,4);
  Eigen::MatrixXd ME01(4,4);
  Eigen::MatrixXd ME02(4,4);
  Eigen::MatrixXd ME03(4,4);
  Eigen::MatrixXd ME04(4,4);
  Eigen::MatrixXd ME05(4,4);
  Eigen::MatrixXd ME06(4,4);
  Eigen::MatrixXd ME07(4,4);

  Eigen::MatrixXd Mm00(4,4);
  Eigen::MatrixXd Mm01(4,4);
  Eigen::MatrixXd Mm02(4,4);
  Eigen::MatrixXd Mm03(4,4);
  Eigen::MatrixXd Mm04(4,4);
  Eigen::MatrixXd Mm05(4,4);
  Eigen::MatrixXd Mm06(4,4);
  Eigen::MatrixXd Mm07(4,4);

  Eigen::MatrixXd Mm02_2(4,4);
  Eigen::MatrixXd Mm03_2(4,4);

  Eigen::MatrixXd M01(4,4);
  Eigen::MatrixXd M12(4,4);
  Eigen::MatrixXd M23(4,4);
  Eigen::MatrixXd M34(4,4);
  Eigen::MatrixXd M45(4,4);
  Eigen::MatrixXd M56(4,4);
  Eigen::MatrixXd M67(4,4);

  Eigen::MatrixXd G0(6,6);
  Eigen::MatrixXd G1(6,6);
  Eigen::MatrixXd G2(6,6);
  Eigen::MatrixXd G3(6,6);
  Eigen::MatrixXd G4(6,6);
  Eigen::MatrixXd G5(6,6);
  Eigen::MatrixXd G6(6,6);

  std::vector<Eigen::MatrixXd> Mlist;
  std::vector<Eigen::MatrixXd> Glist;

  Eigen::MatrixXd Slist(6, 6);
  Eigen::MatrixXd Blist(6, 6);
  
  Eigen::VectorXd thetalist(6);
  Eigen::VectorXd dthetalist(6);
  Eigen::VectorXd prev_dthetalist(6);
  
  Eigen::VectorXd g(3);

 Eigen::MatrixXd allq(simN,Nj)  ;  //all the historic joint angles
 Eigen::MatrixXd alldq (simN,Nj) ;
 Eigen::MatrixXd alltime(simN,1)  ;
 
 mjtNum ctrlTorque[Nj];
 
 Eigen::VectorXd intTheta(6);
 
  Eigen::VectorXd theta0(6); //initial config.  
  Eigen::VectorXd thetaT(6);//goal config
  Eigen::VectorXd thetaT1(6);//goal config
  
  Eigen::VectorXd FTlocal(6);
  
  //end effector points x4 (config at home position)
  Eigen::MatrixXd eeFrame(4,4);
  Eigen::MatrixXd eeTips(4,5);  //four tips plus the origin of the EE frame
  Eigen::MatrixXd eeTips0(4,5); //coordinates of eeTips in world frame
  
  Eigen::VectorXd eeX(6); 
  
  
 //controller parameters
 double Kv=4.3,Kp=300,Ki=400;	
 double Kv1=4.3,Kp1=300,Ki1=400;	
  double Kv2=4.3,Kp2=300,Ki2=400;	

 int phaseN=0;  //stages during the whole process
 
 double dt=1e-3;
 
 double Tstart_stage5=12;
 double TLastPhase=5;
 double LSlide=0.01;//slide 10mm
 double tipCy=0.0;
 
  
 //for controllers, for motion controller
 Eigen::MatrixXd T0(4,4);  
 Eigen::MatrixXd T1(4,4);
 Eigen::VectorXd intXe(6);
 bool setT0T1=false;
 bool setT0T1_2=false; //for 
 double phase1T=6;
 bool setPhaseN=false;
 bool setPhase4=false;
 bool setPhase5=false;
 bool setPhaseEnd=false;
  Eigen::VectorXd thetalistN(6);
  
 Eigen::MatrixXd test2eul(4,4);

 Eigen::MatrixXd txtMat(Ncol_txtMat,6);//matrix read from txt file
 
 double phase1Dur=3; //duration time for phase 1

Eigen::VectorXd Fd(6); //desired force; to be tracked;

Eigen::VectorXd intFe(6); //integration of force error

Eigen::VectorXd prev_dX_real(6);//previous dX_real

//parameters for phase 4
Eigen::VectorXd r_o2(3);  //from ee origin to tip 2 screwdriver
double total_angX=0.0;
double total_angY=0.0;
double total_angZ=0.0;

Eigen::MatrixXd T2_test4(4,4);

double eulerX=0.0;//radian
double eulerZ=0.0;//radian

Eigen::MatrixXd eeFrame_p4(4,4);

Eigen::MatrixXd ctrl_eig_prev(Nj,1);  //computed torque at previous time step
double RC=1.0/2/3.14/10;//parameters for LPF, cutoff frequency 50Hz
double fc=2*PI*20;//cutoff frequency
 double TRotx=2,TRotz=2,TSlide=5000;
 
 Eigen::MatrixXd Line1(4,1);
 Eigen::MatrixXd Line2(4,1);
 
 //3rd order LPF
 Eigen::VectorXd aLPF(4);
 Eigen::VectorXd bLPF(4);
 

Eigen::MatrixXd yPast(6,3);  //yk-1; y_k-2 ...
Eigen::MatrixXd uPast(6,3);  //will be initinilized later

//impedance controller paramerers
Eigen::VectorXd E_tmp_prev1(3);
Eigen::VectorXd E_tmp_prev2(3);
Eigen::VectorXd dthetalist_prev(6);
	

double TImpedance=5, Tstart_stage7=10000;
Eigen::MatrixXd Line3(4,1);
Eigen::Vector3d ImX0(3), ImVn(3);

bool reachEdge=false;

Eigen::MatrixXd eeFrame_p8(4,4); //for phase 8

Eigen::MatrixXd eeFrame_p11(4,4);

double TRotBack=3;
double TSlideBack=3;
bool stopSlideBack=false;
double Tfix=3;

double Tstart_stage10=0,TSlideBack2=4,TChangeContact=3,Tfinal_stage=100000;
double zeroForcePos=-1100;
double phase11=false;
double T11=3;
bool PhaseTC=false, stopRot=false;
double Vslide=0;
double TRoty1=3,TRotz2=3;

double flagTest=true;

double Fdz=-5;
double Kdfe=0.0, Kpfe=0.0, Kife=0.0;
double prevx6=0.0, intx6=0.0;

int polish_direction=0;
double directionV1[2]={cos(85.0/180.0*3.14), -sin(85.0/180.0*3.14)};
double directionV2[2]={directionV1[0],-directionV1[1]};
double polishV=5.0/1.5/1000.0;  // polishing velocity:  mm/1000/s
double globalT=0.0;//current time during running the controller
double zPolish=0.0;

Eigen::MatrixXd  readtxt(int Nrow, int Ncol,std::string path){
	Eigen::MatrixXd mat(Nrow,Ncol);
	 std::fstream input(path);
	 int i,j;
	for( i=0;i<Nrow;i++){
		for( j=0;j<Ncol;j++)
			input>>mat(i,j);
	}	 
	return mat;
}

void init_UR5()
{	
	//UR5 model parameters
	 link01=0.089159;
	 link12=0.13585;
	 link23[0]=0.425;
	 link23[1]=0.1197; 
	 link34=0.39225;
	 link56=0.09456;
	 link45=0.093;
	 link67=0.0823;

	I3=Eigen::MatrixXd::Identity(3,3);
	Im<<1, 0, 0, 0,
			 0,1,0,0,
			 0,0,1,0,
			 0,0,0,1;

	ME00=Im;
	ME01<<1,0,0, 0,
				 0,1,0,0,
				 0,0,1,link01,
				 0,0,0,1;
	ME02<<0,0,1, 0,
				 0,1,0,link12,
				 -1,0,0,link01,
				 0,0,0,1;
	ME03<<0,0,1, link23[0],
				 0,1,0,link12-link23[1],
				 -1,0,0,link01,
				 0,0,0,1;
	ME04<<-1,0,0, link23[0]+link34,
				 0,1,0,link12-link23[1],
				 0,0,-1,link01,
				 0,0,0,1;
	ME05<<-1,0,0, link23[0]+link34,
				 0,1,0,link12-link23[1]+link45,
				 0,0,-1,link01,
				 0,0,0,1;
	ME06<<-1,0,0, link23[0]+link34,
				 0,1,0,link12-link23[1]+link45,
				 0,0,-1,link01-link56,
				 0,0,0,1;
	ME07<<-1,0,0, link23[0]+link34,
				 0,1,0,link12-link23[1]+link45+link67,
				 0,0,-1,link01-link56,
				 0,0,0,1;

	Mm02_2<<1,0,0, 0,
				 0,1,0,0,
				 0,0,1,0.28,
				 0,0,0,1;
	Mm03_2<<1,0,0, 0,
				 0,1,0,0,
				 0,0,1,0.25,
				 0,0,0,1;			 

	Mm00=ME00*Im;
	Mm01=ME01*Im;
	Mm02=ME02*Mm02_2;
	Mm03=ME03*Mm03_2;
	Mm04=ME04*Im;
	Mm05=ME05*Im;
	Mm06=ME06*Im;

	M01=Mm00.colPivHouseholderQr().solve(Mm01);
	M12=Mm01.colPivHouseholderQr().solve(Mm02);
	M23=Mm02.colPivHouseholderQr().solve(Mm03);
	M34=Mm03.colPivHouseholderQr().solve(Mm04);
	M45=Mm04.colPivHouseholderQr().solve(Mm05);
	M56=Mm05.colPivHouseholderQr().solve(Mm06);
	M67=Mm06.colPivHouseholderQr().solve(Mm06);

	G0=Eigen::MatrixXd::Zero(6,6);
	G1=Eigen::MatrixXd::Zero(6,6);
	G2=Eigen::MatrixXd::Zero(6,6);
	G3=Eigen::MatrixXd::Zero(6,6);
	G4=Eigen::MatrixXd::Zero(6,6);
	G5=Eigen::MatrixXd::Zero(6,6);
	G6=Eigen::MatrixXd::Zero(6,6);

	G1.block<3,3>(0,0)=Eigen::Matrix3d(Eigen::Vector3d(0.010267,0.010267,0.00666).asDiagonal());
	G1.block<3,3>(3,3)=Eigen::Matrix3d(Eigen::Vector3d(3.7,3.7,3.7).asDiagonal());
	G2.block<3,3>(0,0)=Eigen::Matrix3d(Eigen::Vector3d(0.22689,0.22689,0.01511).asDiagonal());
	G2.block<3,3>(3,3)=Eigen::Matrix3d(Eigen::Vector3d(8.393,8.393,8.393).asDiagonal());
	G3.block<3,3>(0,0)=Eigen::Matrix3d(Eigen::Vector3d(0.04944,0.04944,0.004095).asDiagonal());
	G3.block<3,3>(3,3)=Eigen::Matrix3d(Eigen::Vector3d(2.275,2.275,2.275).asDiagonal());
	G4.block<3,3>(0,0)=Eigen::Matrix3d(Eigen::Vector3d(0.1112,0.1112,0.21942).asDiagonal());
	G4.block<3,3>(3,3)=Eigen::Matrix3d(Eigen::Vector3d(1.219,1.219,1.219).asDiagonal());
	G5.block<3,3>(0,0)=Eigen::Matrix3d(Eigen::Vector3d(0.1112,0.1112,0.21942).asDiagonal());
	G5.block<3,3>(3,3)=Eigen::Matrix3d(Eigen::Vector3d(1.219,1.219,1.219).asDiagonal());
	G6.block<3,3>(0,0)=Eigen::Matrix3d(Eigen::Vector3d(0.01714,0.01714,0.033822).asDiagonal());
	G6.block<3,3>(3,3)=Eigen::Matrix3d(Eigen::Vector3d(0.1879,0.1879,0.1879).asDiagonal());

	Mlist.push_back(M01);
	Mlist.push_back(M12);
	Mlist.push_back(M23);
	Mlist.push_back(M34);
	Mlist.push_back(M45);
	Mlist.push_back(M56);
	Mlist.push_back(M67);

	Glist.push_back(G1);
	Glist.push_back(G2);
	Glist.push_back(G3);
	Glist.push_back(G4);
	Glist.push_back(G5);
	Glist.push_back(G6);

	Slist<<0,0,0,0,0,0,
			   0,1,1,1,0,1,
			  1,0,0,0,-1,0,
			  0,-link01,-link01,-link01,-link12+link23[1]-link45,-link01+link56,
			  0,0,0,0,link23[0]+link34,0,
			  0,0,link23[0],link23[0]+link34,0,link23[0]+link34;
	
	eeFrame<<-1, 0, 0, link23[0]+link34,
			           0,1,0,link12-link23[1]+link45+link67,
			           0,0,-1,link01-link56,
			           0,0,0,1;
					   
	Blist=	Slist;
	for(int i=0;i<6;i++)
		Blist.block<6,1>(0,i)=mr::Adjoint( mr::TransInv(eeFrame) )*Slist.block<6,1>(0,i);
	
					   
	double sdY=0.05, sdX= 0.006, sdZ=0.0005;  //sd: screwdriver
	eeTips<<sdX, sdX,-sdX,-sdX,0,
	              sdY,sdY,sdY,sdY,0,
				  -sdZ,sdZ,sdZ,-sdZ,0,
				  1,1,1,1,1;
	r_o2<<sdX,sdY,sdZ;
	eeTips0=eeTips*0;
	
	eeFrame_p4<<-1, 0, 0, link23[0]+link34+sdX,   //eeFrame for phase 4, move to tip C
			           0,1,0,link12-link23[1]+link45+link67+sdY,
			           0,0,-1,link01-link56-sdZ,
			           0,0,0,1;
	eeFrame_p8<<-1, 0, 0, link23[0]+link34-sdX,   //eeFrame for phase 4, move to tip A
			           0,1,0,link12-link23[1]+link45+link67+sdY,
			           0,0,-1,link01-link56-sdZ,
			           0,0,0,1;
	
	eeFrame_p11<<-1, 0, 0, link23[0]+link34+sdX,   //eeFrame for phase 4, move to tip A
			           0,1,0,link12-link23[1]+link45+link67+sdY,
			           0,0,-1,link01-link56+sdZ,
			           0,0,0,1;
	

	dthetalist<<0, 0, 0, 0, 0, 0;
	thetalist << 0, 0, 0, 0, 0, 0;
	prev_dthetalist<<0, 0, 0, 0, 0, 0;
	thetalist=thetalist/180*PI;
	g << 0, 0, -9.81;	
	intTheta<<0,0,0,0,0,0;
	theta0<<0,0,0,0,0,0;
	
	std::string path1="../sample_swxie/process_data/parameters_polish.txt";
	txtMat=readtxt(Ncol_txtMat,6,path1);
	
	thetaT<<0,-1.29,0,0,0,0;
	
	//thetaT1<<1.13,-1.29,2.04,-2.42,-1.32,1.07;
	for(int i=0;i<6;i++)
		thetaT1(i)=txtMat(1,i);
	std::cout<<"txtMat "<<txtMat<<"\n";
	Kv=txtMat(2,0);
	Kp=txtMat(2,1);
	Ki=txtMat(2,2);
	
	Kv1=txtMat(2,3);
	Kp1=txtMat(2,4);
	Ki1=txtMat(2,5);
	
	Kv2=txtMat(5,0);
	Kp2=txtMat(5,1);
	Ki2=txtMat(5,2);
	
	Kdfe=txtMat(9,0);
	Kpfe=txtMat(9,1);
	Kife=txtMat(9,2);
	
	zPolish=txtMat(10,0);
	directionV1[0]=-sin(txtMat(10,1)/180.0*3.14);
	directionV1[1]=cos(txtMat(10,1)/180.0*3.14);
    directionV2[0]=-directionV1[0];
	directionV2[1]=directionV1[1];
		
	std::cout<<"Kv Kp Ki "<<Kv<<" "<<Kp<<" "<<Ki<<"\n";
	std::cout<<"Kv Kp Ki "<<Kv1<<" "<<Kp1<<" "<<Ki1<<"\n";
	std::cout<<"thetaT1 "<<thetaT1<<"\n";
	
	for(int i=0;i<Nj;i++){
		ctrlTorque[i]=0.0;
		ctrl_eig_prev(i)=0.0;
	}
	
	for(int i=0;i<6;i++)
		for(int j=0;j<3;j++)
		{
			uPast(i,j)=0.0;
			yPast(i,j)=0.0;
		}
	 //fc=20Hz 
	 //aLPF<<1.0000,   -2.6862 ,   2.4197,   -0.7302;
	 //bLPF<<0.0004 ,   0.0012 ,   0.0012,    0.0004;
	  //fc=50Hz 
	//aLPF<<1.0000 ,  -2.3741 ,   1.9294,   -0.5321;
	 //bLPF<< 0.0029 ,   0.0087,    0.0087 ,   0.0029;
	 
	aLPF<< 1.0000 ,  -1.1619,    0.6959,   -0.1378;
	bLPF<<  0.0495,    0.1486,    0.1486,    0.0495;

    phase1T=6;
	
	intFe<<0,0,0,0,0,0;
	
	prev_dX_real<<0,0,0,0,0,0;
	
	E_tmp_prev1<<0,0,0;
	E_tmp_prev2<<0,0,0;
	dthetalist_prev<<0,0,0,0,0,0;	
	
	/*test2eul<<0.572061402817684,	0.139279002165598,	-0.808298899518005, 3,
                             0.415626937777453,	-0.898807881668542,	0.139279002165598,4,
                            -0.707106781186548,	-0.415626937777453,	-0.572061402817684,5,
							0 ,0,0,1;*/
}

//compute the tips of the screwdriver
void updateEEtips0(const mjModel* m, mjData* d){	
     int i;
	 Eigen::VectorXd thetalist_tmp(6);
    for(i=0;i<Nj;i++){
		thetalist_tmp(i)=d->qpos[i];
	}
	Eigen::MatrixXd Tfkin=mr::FKinSpace(eeFrame,Slist,thetalist_tmp);
	eeTips0=Tfkin*eeTips;
	
	eeX=f_QX(  Tfkin );
}

Eigen::MatrixXd calTee(const mjModel* m, mjData* d){	
      int i;
	 Eigen::VectorXd thetalist_tmp(6);
    for(i=0;i<Nj;i++){
		thetalist_tmp(i)=d->qpos[i];
	}
	return mr::FKinSpace(eeFrame,Slist,thetalist_tmp);
}

//map rotation matrix to Euler angle
//Or this f mapping from Q to X
Eigen::VectorXd f_QX(Eigen::MatrixXd T){	
	  Eigen::VectorXd res(6);
	  double sy=sqrt( T(0,0)*T(0,0)+T(1,0)*T(1,0) );
	  bool singular=sy<1e-6;
	  double thx,thy,thz;
	  if(!singular){
		  thx=atan2( T(2,1),T(2,2)  );
		  thy=atan2( -T(2,0), sy  );
		  thz=atan2( T(1,0),T(0,0) );
	  }else{
		  thx=atan2( -T(1,2),T(1,1) );
		  thy=atan2( -T(2,0),sy );
		  thz=0;
	  }
	  res<<thx,thy,thz,T(0,3),T(1,3),T(2,3);
	  return res;
}

//desired end-effector trajectory 4x4 matrix
//T0: desired initial config.
//T1: desired end  config
//cT: time elapsed since beginning of this stages
//totalT: total time for this phase
Eigen::MatrixXd calTd(Eigen::MatrixXd T0, Eigen::MatrixXd T1, double cT, double totalT){
	Eigen::MatrixXd Xdmat(4,4);
	Xdmat=T0;	
	if(phaseN==6){  //we are polishing the surface
		if(polish_direction==0){
			 Eigen::Vector3d tmp;
			 tmp<<directionV1[0],directionV1[1],0.0;
			 Xdmat.block<3,1>(0,3)=T0.block<3,1>(0,3)+tmp*cT*polishV;
		}
		else{			
			 Eigen::Vector3d tmp;
			 tmp<<directionV2[0],directionV2[1],0.0;
			 Xdmat.block<3,1>(0,3)=T0.block<3,1>(0,3)+tmp*cT*polishV;
		}
	}else{
		Xdmat.block<3,1>(0,3)=T0.block<3,1>(0,3)+( T1.block<3,1>(0,3)-T0.block<3,1>(0,3)  )*cT/totalT;
	}
	return Xdmat;	
}

//calTd for phase 4
//totalThx: total angle to be rotated
Eigen::MatrixXd calTd4(Eigen::MatrixXd T0, double cT, double totalT, double totalThx){	
	double thx=-cT/totalT*totalThx;
	Eigen::MatrixXd rotx(4,4);
	rotx<<1,0,0,0,
	           0,cos(thx),sin(thx),0,
			   0,-sin(thx),cos(thx),0,
			   0,0,0,1;
	return T0*rotx;	
}


Eigen::MatrixXd calTd11(Eigen::MatrixXd T0, double cT, double totalT, double totalThy){	
	double thy=cT/totalT*totalThy;
	Eigen::MatrixXd roty(4,4);
	roty<<cos(thy),0,sin(thy),0,
	            0,1,0,0,
				-sin(thy),0,cos(thy),0,
			   0,0,0,1;
	return T0*roty;	
}

Eigen::MatrixXd calTd5(Eigen::MatrixXd T0, double cT, double totalT, double totalThz){	
	double thz=cT/totalT*totalThz;
	Eigen::MatrixXd rotz(4,4);
	rotz<<cos(thz),-sin(thz),0,0,
	          sin(thz),cos(thz),0,0,
	          0,0,1,0,				
			  0,0,0,1;
	return T0*rotz;	
}

//compute  Vs for desired trajectory; use calTd
Eigen::VectorXd caldXd4(Eigen::MatrixXd T0, double cT, double totalT, double totalThx){
	
	Eigen::VectorXd curXd=f_QX( calTd4(T0,cT,totalT,totalThx) );
	Eigen::VectorXd nextXd=f_QX( calTd4(T0,cT+dt,totalT,totalThx) );
	Eigen::VectorXd dXd=(nextXd-curXd)/dt;
	return dXd;		
}
Eigen::VectorXd caldXd5(Eigen::MatrixXd T0, double cT, double totalT, double totalThz){
	
	Eigen::VectorXd curXd=f_QX( calTd5(T0,cT,totalT,totalThz) );
	Eigen::VectorXd nextXd=f_QX( calTd5(T0,cT+dt,totalT,totalThz) );
	Eigen::VectorXd dXd=(nextXd-curXd)/dt;
	return dXd;		
}

Eigen::VectorXd caldXd11(Eigen::MatrixXd T0, double cT, double totalT, double totalThy){
	
	Eigen::VectorXd curXd=f_QX( calTd11(T0,cT,totalT,totalThy) );
	Eigen::VectorXd nextXd=f_QX( calTd11(T0,cT+dt,totalT,totalThy) );
	Eigen::VectorXd dXd=(nextXd-curXd)/dt;
	return dXd;		
}

Eigen::VectorXd calddXd4(Eigen::MatrixXd T0, double cT, double totalT, double totalThx){
	
	Eigen::VectorXd curVsd=caldXd4(T0,cT,totalT,totalThx);
	Eigen::VectorXd nextVsd=caldXd4(T0,cT+dt,totalT,totalThx);
	Eigen::VectorXd ddXd=(nextVsd-curVsd)/dt;
	return ddXd;		
}

Eigen::VectorXd calddXd5(Eigen::MatrixXd T0, double cT, double totalT, double totalThz){
	
	Eigen::VectorXd curVsd=caldXd5(T0,cT,totalT,totalThz);
	Eigen::VectorXd nextVsd=caldXd5(T0,cT+dt,totalT,totalThz);
	Eigen::VectorXd ddXd=(nextVsd-curVsd)/dt;
	return ddXd;		
}

Eigen::VectorXd calddXd11(Eigen::MatrixXd T0, double cT, double totalT, double totalThy){
	
	Eigen::VectorXd curVsd=caldXd11(T0,cT,totalT,totalThy);
	Eigen::VectorXd nextVsd=caldXd11(T0,cT+dt,totalT,totalThy);
	Eigen::VectorXd ddXd=(nextVsd-curVsd)/dt;
	return ddXd;		
}

//compute  Vs for desired trajectory; use calTd
Eigen::VectorXd caldXd(Eigen::MatrixXd T0, Eigen::MatrixXd T1, double cT, double totalT){
	
	Eigen::VectorXd curXd=f_QX( calTd(T0,T1,cT,totalT) );
	Eigen::VectorXd nextXd=f_QX( calTd(T0,T1,cT+dt,totalT) );
	Eigen::VectorXd dXd=(nextXd-curXd)/dt;
	return dXd;		
}

//derivative of Vsd
Eigen::VectorXd calddXd(Eigen::MatrixXd T0, Eigen::MatrixXd T1, double cT, double totalT){
	
	Eigen::VectorXd curVsd=caldXd(T0,T1,cT,totalT);
	Eigen::VectorXd nextVsd=caldXd(T0,T1,cT+dt,totalT);
	Eigen::VectorXd ddXd=(nextVsd-curVsd)/dt;
	return ddXd;		
}

//get the sensor data and transform to the world frame
Eigen::VectorXd getFT(const mjModel* m, mjData* d){
	Eigen::MatrixXd Tee=calTee(m,d);
	Eigen::VectorXd FTdata(6);
	Eigen::VectorXd TFb(6);
	TFb<<d->sensordata[3], d->sensordata[4], d->sensordata[5],d->sensordata[0], d->sensordata[1], d->sensordata[2];
	FTdata=( mr::Adjoint(   mr::TransInv(Tee) ) ).transpose()*TFb;
	FTlocal=TFb;
	return FTdata;
}



//compute the Jacobian in terms of f_QX
Eigen::MatrixXd JacobianF(Eigen::MatrixXd SlistMat, Eigen::MatrixXd thetalistVec){
	
	Eigen::MatrixXd Jf(6,6);
	Eigen::VectorXd deltaTh(6);
	Eigen::VectorXd X1(6);
	Eigen::VectorXd X2(6);
	deltaTh<<0,0,0,0,0,0;
	double dTh=0.001;
	int i,j;
	for(i=0;i<6;i++)
		for(j=0;j<6;j++)
		{
			deltaTh=deltaTh*0;
			deltaTh(j)=dTh;				
            if(phaseN==3 || phaseN==4 || phaseN==5 || phaseN==6 || phaseN==7 || phaseN==9 || phaseN==12){
				X1=f_QX( mr::FKinSpace(eeFrame_p4,Slist,thetalist+deltaTh) );
			    X2=f_QX( mr::FKinSpace(eeFrame_p4,Slist,thetalist) );	
			}else if(phaseN==8 ||phaseN==10 ){
				X1=f_QX( mr::FKinSpace(eeFrame_p8,Slist,thetalist+deltaTh) );
			    X2=f_QX( mr::FKinSpace(eeFrame_p8,Slist,thetalist) );	
			}
			else if(phaseN==11){
				X1=f_QX( mr::FKinSpace(eeFrame_p4,Slist,thetalist+deltaTh) );
			    X2=f_QX( mr::FKinSpace(eeFrame_p4,Slist,thetalist) );	
			}
			else{
				X1=f_QX( mr::FKinSpace(eeFrame,Slist,thetalist+deltaTh) );
			    X2=f_QX( mr::FKinSpace(eeFrame,Slist,thetalist) );	
			}
			Jf(i,j)=(X1(i)-X2(i))/dTh;
		}	
	return Jf;	
}

Eigen::VectorXd getJfMInvJT(const mjModel* m, mjData* d){
	for(int i=0;i<Nj;i++){
		thetalist(i)=d->qpos[i];
	}
		Eigen::MatrixXd Js=mr::JacobianSpace(Slist, thetalist);
		Eigen::MatrixXd Jf=JacobianF(Slist,thetalist); 
		Eigen::MatrixXd UR5M = mr::MassMatrix(thetalist,Mlist,Glist,Slist);
		Eigen::VectorXd  curF(6);
		curF=getFT(m,d);
		Eigen::MatrixXd  JfMInvJT=Jf*UR5M.colPivHouseholderQr().solve( Js.transpose() );
		Eigen::VectorXd res=JfMInvJT*curF;
		//res(5)=JfMInvJT(5,5);		//get epsilon
		res(5)=res(5)-JfMInvJT(5,5)*curF(5);		//get eta
		return  res;
}

//for constraints x-x1=0;y-x2=0,thx-x4=0;thy-x5=0;thz-x6=0
Eigen::MatrixXd  calJphi(){	
     	Eigen::MatrixXd A(1,6);
		/*A<<1,0,0,0,0,0,
		       0,1,0,0,0,0,
			   0,0,1,0,0,0,
			   0,0,0,1,0,0,
			   0,0,0,0,1,0,
			   0,0,0,0,0,1;*/
		A<<0,0,0,0,0,1;
        return A;			   
}

Eigen::VectorXd calht4(Eigen::MatrixXd T0, double cT, double totalT, double totalThx){	
	double thx=-cT/totalT*totalThx;
	Eigen::MatrixXd rotx(4,4);
	rotx<<1,0,0,0,
	           0,cos(thx),sin(thx),0,
			   0,-sin(thx),cos(thx),0,
			   0,0,0,1;
	Eigen::MatrixXd Xdmat(4,4);	
	Xdmat=T0*rotx;
	Eigen::VectorXd ht_tmp=f_QX(Xdmat);
	Eigen::VectorXd ht(5);
	ht<<ht_tmp(0),ht_tmp(1),ht_tmp(2),ht_tmp(3),ht_tmp(4);	
	return ht;	
}

Eigen::VectorXd calht11(Eigen::MatrixXd T0, double cT, double totalT, double totalThy){	
	double thy=cT/totalT*totalThy;
	Eigen::MatrixXd roty(4,4);
	roty<<cos(thy),0,sin(thy),0,
	            0,1,0,0,
				-sin(thy),0,cos(thy),0,
			   0,0,0,1;
	Eigen::MatrixXd Xdmat(4,4);	
	Xdmat=T0*roty;
	Eigen::VectorXd ht_tmp=f_QX(Xdmat);
	Eigen::VectorXd ht(5);
	ht<<ht_tmp(0),ht_tmp(1),ht_tmp(2),ht_tmp(3),ht_tmp(4);	
	return ht;	
}

Eigen::VectorXd calht5(Eigen::MatrixXd T0, double cT, double totalT, double totalThz){	
	double thz=cT/totalT*totalThz;
	Eigen::MatrixXd rotz(4,4);
	rotz<<cos(thz),-sin(thz),0,0,
	          sin(thz),cos(thz),0,0,
	          0,0,1,0,				
			  0,0,0,1;
	Eigen::MatrixXd Xdmat(4,4);	
	Xdmat=T0*rotz;
	Eigen::VectorXd ht_tmp=f_QX(Xdmat);
	Eigen::VectorXd ht(5);
	ht<<ht_tmp(0),ht_tmp(1),ht_tmp(2),ht_tmp(3),ht_tmp(4);	
	return ht;	
}

Eigen::VectorXd caldht4(Eigen::MatrixXd T0, double cT, double totalT, double totalThx){	
       Eigen::VectorXd  curht=calht4(T0,cT,totalT,totalThx);
	   Eigen::VectorXd  nextht=calht4(T0,cT+dt,totalT,totalThx);
	   return (nextht-curht)/dt;	   
}

Eigen::VectorXd caldht5(Eigen::MatrixXd T0, double cT, double totalT, double totalThz){	
       Eigen::VectorXd  curht=calht5(T0,cT,totalT,totalThz);
	   Eigen::VectorXd  nextht=calht5(T0,cT+dt,totalT,totalThz);
	   return (nextht-curht)/dt;	   
}

Eigen::VectorXd caldht11(Eigen::MatrixXd T0, double cT, double totalT, double totalThy){	
       Eigen::VectorXd  curht=calht11(T0,cT,totalT,totalThy);
	   Eigen::VectorXd  nextht=calht11(T0,cT+dt,totalT,totalThy);
	   return (nextht-curht)/dt;	   
}

//corresponding h(t)=[0,0,0,-D,vt-V_d]
/*phi(X)=
thx
thy
thz
Ax+By+Cz
V_x x+V_y y+V_z z
*/
Eigen::VectorXd caldht6(double v){
	Eigen::VectorXd dotHt(5,1);
	dotHt<<0,0,0,0,v;
	return dotHt;
}

Eigen::VectorXd caldht7(){
	Eigen::VectorXd dotHt(5,1);
	dotHt<<0,0,0,0,0;
	return dotHt;
}


Eigen::MatrixXd calX2T(Eigen::VectorXd Xphi){
	Eigen::MatrixXd resT(4,4);
	resT<<1,0,0,Xphi(3),
	            0,1,0,Xphi(4),
				0,0,1,Xphi(5),
				0,0,0,1;
	double angX=Xphi(0), angY=Xphi(1), angZ=Xphi(2);
	Eigen::MatrixXd rotX(3,3);
	Eigen::MatrixXd rotY(3,3);
	Eigen::MatrixXd rotZ(3,3);
	rotX<<1,0,0,
	           0,cos(angX),-sin(angX),
			   0,sin(angX), cos(angX);
	rotY<<cos(angY),0,sin(angY),
	            0,1,0,
				-sin(angY),0,cos(angY);
	rotZ<<cos(angZ), -sin(angZ),0,
	            sin(angZ),cos(angZ),0,
				0,0,1;
	resT.block<3,3>(0,0)=rotZ*rotY*rotX;
	
	return resT;	
}

Eigen::VectorXd f_phi(Eigen::VectorXd Xphi){
	Eigen::VectorXd Jphi(5);
	Jphi<<Xphi(0),Xphi(1),Xphi(2),Xphi(3),Xphi(4);	
	return Jphi;	
}

Eigen::MatrixXd f_Jphi(Eigen::VectorXd Xphi){
	Eigen::MatrixXd resJphi(5,6);
	resJphi<<1,0,0,0,0,0,     //thx
					 0,1,0,0,0,0,   //thy
		             0,0,1,0,0,0,   //thz
		             0,0,0,1,0,0,   //x
		             0,0,0,0,1,0;	 //y
	return resJphi;	
}

/*phi(X)=
thx
thy
thz
Ax+By+Cz
V_x x+V_y y+V_z z
*/
Eigen::MatrixXd f_Jphi6(Eigen::VectorXd Xphi){
	Eigen::MatrixXd resJphi(5,6);
	resJphi<<1,0,0,0,0,0,     //thx
					 0,1,0,0,0,0,   //thy
		             0,0,1,0,0,0,   //thz
		             0,0,0,Line1(0),Line1(1),Line1(2),   //line1
		             0,0,0,Line2(0),Line2(1),Line2(2);	 //line2
	return resJphi;	
}

Eigen::MatrixXd f_Jphi7(Eigen::VectorXd Xphi){
	Eigen::MatrixXd resJphi(5,6);
	resJphi<<1,0,0,0,0,0,     //thx
					 0,1,0,0,0,0,   //thy
		             0,0,1,0,0,0,   //thz
		             0,0,0,Line1(0),Line1(1),Line1(2),   //line1
		             0,0,0,Line3(0),Line3(1),Line3(2);	 //line3
	return resJphi;	
}

//calJphi for phase 4  
Eigen::MatrixXd  calJphi4(Eigen::VectorXd Xphi, Eigen::VectorXd dht, Eigen::VectorXd A_dXd){
	Eigen::MatrixXd A=f_Jphi(Xphi); 
    double 	tmp=A_dXd.transpose()*A_dXd;
    A=A-dht*A_dXd.transpose()/tmp;			
	return A;
}

Eigen::MatrixXd  calJphi6(Eigen::VectorXd Xphi, Eigen::VectorXd dht, Eigen::VectorXd A_dXd){
	Eigen::MatrixXd A=f_Jphi6(Xphi); 
    double 	tmp=A_dXd.transpose()*A_dXd;
    A=A-dht*A_dXd.transpose()/tmp;			
	return A;
}

Eigen::MatrixXd  calJphi7(Eigen::VectorXd Xphi, Eigen::VectorXd dht, Eigen::VectorXd A_dXd){
	Eigen::MatrixXd A=f_Jphi7(Xphi); 
    double 	tmp=A_dXd.transpose()*A_dXd;
    A=A-dht*A_dXd.transpose()/tmp;			
	return A;
}


Eigen::MatrixXd solA_XB(Eigen::MatrixXd A,Eigen::MatrixXd B){
	return (   (B.transpose()).colPivHouseholderQr().solve(A.transpose())  ).transpose();
}

void updateAngles(const mjModel* m, mjData* d)
{
	updateEEtips0(m,d);
	Eigen::Vector3d vco1=eeTips0.block<3,1>(0,1)-eeTips0.block<3,1>(0,2)  ; //4x5
	Eigen::Vector3d vco2=eeTips0.block<3,1>(0,3)-eeTips0.block<3,1>(0,2)   ; //4x5
	Eigen::Vector3d vZ;
	vZ<<0,0,1;
	Eigen::Vector3d Vn=vco1.cross(vZ);
	vco2=vco2/sqrt(vco2.dot(vco2));
	Vn=Vn/sqrt(Vn.dot(Vn));
	eulerX=acos(Vn.dot(vco2));	
	
	vco1=eeTips0.block<3,1>(0,2)-eeTips0.block<3,1>(0,3)  ; //4x5
	vco2=eeTips0.block<3,1>(0,1)-eeTips0.block<3,1>(0,2)   ; //4x5
	Vn=vco1.cross(vZ);
	vco2=vco2/sqrt(vco2.dot(vco2));
	Vn=Vn/sqrt(Vn.dot(Vn));
	eulerZ=acos(Vn.dot(vco2));	
}



//update ctrlTorque and then use it for control
void ctrl1(const mjModel* m, mjData* d, double curT){
	//update thetalist and dthetalist from d
	int i;
	double Ta=3,Ta1=3;// time span is 5sec.
	for(i=0;i<Nj;i++){
		thetalist(i)=d->qpos[i];
		prev_dthetalist(i)=dthetalist(i);
		dthetalist(i)=d->qvel[i];		
	}
	
	//std::cout<<"Time:"<< curT<<" thetalist"<<thetalist<<std::endl;
	
	Eigen::MatrixXd UR5M = mr::MassMatrix(thetalist,Mlist,Glist,Slist);
	Eigen::MatrixXd UR5c= mr::VelQuadraticForces(thetalist,dthetalist,Mlist,Glist,Slist);
	Eigen::VectorXd UR5grav = mr::GravityForces(thetalist,g,Mlist,Glist,Slist);	
	
	Eigen::VectorXd dthetaE(6);
	Eigen::VectorXd thetaD(6);//desired theta 
	 
	 Eigen::VectorXd FTdata(6);
	
	 //compute dthetaE=dtheta_d-dtheta
	 if(curT<Ta){
		 dthetaE=(thetaT-theta0)/Ta-dthetalist;
		 thetaD=theta0+(thetaT-theta0)/Ta*curT;
	 }else if(curT<Ta+Ta1){		
         dthetaE=(thetaT1-thetaT)/Ta1-dthetalist;	 
		 thetaD=thetaT+(thetaT1-thetaT)/Ta1*(curT-Ta);		 
	 }else if(curT<3+6){   //move to above of the screw
		 phaseN=1;  //test motion controller
	 } else if(curT>9 && curT<Tstart_stage5){   //move down towards the screw, until touch
		 phaseN=2;
		 phase1T=9;		
         FTdata=getFT(m,d);   
		 if(abs(FTdata(5))>0.01 && zeroForcePos <-1000){
			 //define zeroForcePos
			 updateEEtips0(m,d);
			 zeroForcePos=eeTips0(2,2);//tip C
			 std::cout<<"\n****zeroForcePos: "<<zeroForcePos<<"\n";
		 }
        if(  abs(FTdata(5))>txtMat(6,0)  ){       
			Tstart_stage5=curT;//touched the screw edge, signaled by increased downward force
			Fd=FTdata; //setup the desired force to be tracked
			//intFe=intFe*0;
            std::cout<<"Phase: towards the screw "<<Tstart_stage5<<"\n";
		    std::cout<<"Parameters for force controller: Kv2 Kp2 Ki2 "<<Kv2<<" "<<Kp2<<" "<<Ki2<<"\n";
			std::cout<<"desired force: "<<Fd.transpose()<<"\n ";
		}		 
	 }else if(curT<Tstart_stage5+2){   //fix the config., z is float, thx,thy,thz,x,y fixed
	     if(!setPhaseN){
			 std::cout<<"\n ~~~~~~ Fix current state for 2 secs: "<<curT<<"\n";
			 setPhaseN=true;
			 T0=mr::FKinSpace(eeFrame_p4,Slist,thetalist);
			 T1=T0;	
			 //T1(2,3)=T1(2,3)-0.0003;
			 std::cout<<"desired Fd: "<<Fd.transpose()<<" desired config: "<<f_QX(T0).transpose()<<"\n \n";					 
			 thetalistN=thetalist;  //desicred joint angles
			 phase1T=Tstart_stage5;
			 phase1Dur=2;
			// intXe=intXe*0;
		 }
		 phaseN=3;
		 dthetaE=-dthetalist;	 
		 thetaD=thetalistN;  
		 intTheta=intTheta*0;
	 }	 
    else if(curT<Tstart_stage5+2+TRotx){		  //rotate around x
		 if(!setPhase4){		
			 setPhase4=true;
			 phase1T=curT; 
			 T0=mr::FKinSpace(eeFrame_p4,Slist,thetalist);  
			 T1=T0;
            phase1Dur=TRotx;
			std::cout<<"\n ~~~~~~phase Rot X start time "<<curT<<"\n";
			//compute angle: thetax
		   updateAngles(m,d);
		   total_angX= txtMat(7,0)/180.0*3.1415;//txtMat
		   std::cout<<"angleX: "<<eulerX/3.1415*180.0<<" angleZ: "<<eulerZ/3.1415*180.0<<" "<<total_angX/3.1415*180.0<<" degree\n";    			
			setPhaseN=false;
			FTdata=getFT(m,d);   
			Fd=FTdata;
		 }
		 phaseN=4;
	 }	
	 else if(curT<Tstart_stage5+2+TRotx+TRotz){		  //rotate around z
		 if(!setPhaseN){		
			 setPhase4=false;
			 setPhaseN=true;			 
			 phase1T=curT; 
			 T0=mr::FKinSpace(eeFrame_p4,Slist,thetalist);  
			 T1=T0;
             phase1Dur=TRotz;
			 std::cout<<"\n ~~~~~~phase Rot Z start time "<<curT<<"\n";
			//compute angle: thetax
		    updateAngles(m,d);
		    total_angZ= txtMat(7,2)/180.0*3.1415;
		    std::cout<<"angleX: "<<eulerX/3.1415*180.0<<" angleZ: "<<eulerZ/3.1415*180.0<<" "<<total_angX/3.1415*180.0<<" degree\n";    						
		 }
		 phaseN=5;
	 }	
	 else if(curT<Tstart_stage5+2+TRotx+TRotz+TSlide && curT<Tstart_stage7){
		   if(!setPhase4){
			    std::cout<<"\n ~~~~~~slide on screw: "<<curT<<"\n ";
			   phase1T=curT;
			   phase1Dur=TSlide;
			   setPhase4=true;
			   setPhaseN=false;
			   updateEEtips0(m,d);
	           Eigen::Vector3d pxyz=eeTips0.block<3,1>(0,2);  //tip position
			   //normal direction of the plane
			   Eigen::Vector3d Vn=eeTips0.block<3,1>(0,3)-eeTips0.block<3,1>(0,2);  //norm direction of plane
			   Eigen::Vector3d Vxyz=eeTips0.block<3,1>(0,2)-eeTips0.block<3,1>(0,1);//sliding line
			   Vxyz=Vxyz/sqrt(Vxyz.dot(Vxyz));
			   Line1<<Vn(0),Vn(1),Vn(2),-Vn.dot(pxyz);
			   Line2<<Vxyz(0),Vxyz(1),Vxyz(2),-Vxyz.dot(pxyz);
			   //set desired configuration
			   Eigen::VectorXd deltaX(3);
			   Vxyz=Vxyz*LSlide;//move 10mm
			  deltaX<<Vxyz(0),Vxyz(1),Vxyz(2)*0;
			  T0=mr::FKinSpace(eeFrame_p4,Slist,thetalist);
			  T1=T0;
			  T1.block<3,1>(0,3)=	T1.block<3,1>(0,3)+	deltaX;	
			  std::cout<<"touching point coordinate: "<<T1.block<3,1>(0,3).transpose()<<"\n";
			  std::cout<<"polishing direction 1:"<<directionV1[0]<<" "<<directionV1[1]<<"\n";
			  std::cout<<"polishing direction 2:"<<directionV2[0]<<" "<<directionV2[1]<<"\n";
			   std::cout<<"polishing :"<<polishV<<"\n";
			   polish_direction=1;
		   }
		   //double TImpedance=5, Tstart_stage7=10000; Line3 Eigen::Vector3d ImX0 ImVn	   
		   updateEEtips0(m,d);
	       Eigen::Vector3d pxyz=eeTips0.block<3,1>(0,2);  //tip position   
		   
		   phaseN=6;
		   
		   Eigen::MatrixXd Tcur=mr::FKinSpace(eeFrame_p4,Slist,thetalist);
		   
		   if( Tcur(1,3)>0.8 )//stop polishing
			   Tstart_stage7=curT;
		   if(Tcur(2,3)<zPolish &&  curT-phase1T>5){
			   std::cout<<"Change direction!\n";
			   T0=mr::FKinSpace(eeFrame_p4,Slist,thetalist);
		       T1=T0;
			   phase1T=curT;
			   phase1Dur=TSlide;
			   if(polish_direction==0)
				   polish_direction=1;  
			   else
				   polish_direction=0;
		   }		   
		   
	 }	else{      //maintain current state
	    if(!setPhaseEnd){  //last phase
			 std::cout<<"\n <<<<<<Last phase, time"<<curT<<"\n ";
			 updateAngles(m,d);
			 FTdata=getFT(m,d);
			 Fd=FTdata;
			 T0=mr::FKinSpace(eeFrame,Slist,thetalist);
			 std::cout<<"phase last T0"<<f_QX(T0).transpose()<<"\n";
		     std::cout<<"angleX: "<<eulerX/3.1415*180.0<<" angleZ: "<<eulerZ/3.1415*180.0<<" degree\n";    
			 
			 setPhaseEnd=true;			
			 T0=mr::FKinSpace(eeFrame_p4,Slist,thetalist);
			 T1=T0;	
			 std::cout<<"start time: "<<Fd.transpose()<<" desired config: "<<f_QX(T0).transpose()<<"\n \n";			
			 phase1T=curT;
			 phase1Dur=1000;			 
		 }
		 phaseN=3;
	 }
	 
	 intTheta=intTheta+(thetaD-thetalist)*dt;
	
	//std::cout<<"Time:"<<curT<<"desired theta"<<dtheta_t<<std::endl;	
	Eigen::MatrixXd ctrl_eigff(Nj,1); //torque in eigen form
	Eigen::MatrixXd ctrl_eig(Nj,1); //torque in eigen form
	ctrl_eigff=UR5c+UR5grav; //feedforward term
	ctrl_eig=ctrl_eigff+UR5M*(Kv*dthetaE+Kp*(thetaD-thetalist)+Ki*intTheta);	
	
	//position controller
	Eigen::VectorXd ddXd(6);
	Eigen::VectorXd dXd(6);
	Eigen::VectorXd dXe(6);
	Eigen::VectorXd Xe(6);
	Eigen::MatrixXd Jf(6,6);
	Eigen::MatrixXd Js(6,6);
	Eigen::VectorXd  curF(6);
	
	if(phaseN==1 || phaseN==2)
	{
		//set T0 and T1 only once
		if(!setT0T1 && phaseN==1){
			std::cout<<"controller changed!  from 6-9 sec "<<phaseN<<"\n";
			Eigen::VectorXd deltaX(3);
			deltaX<<txtMat(3,0),txtMat(3,1),txtMat(3,2);
			T0=mr::FKinSpace(eeFrame,Slist,thetalist);
			T1=T0;
			T1.block<3,1>(0,3)=	T1.block<3,1>(0,3)+	deltaX;			
			intXe<<0,0,0,0,0,0;
			setT0T1=true;
			std::cout<<"T0: "<<T0<<"\n";
		}
		if(!setT0T1_2 && phaseN==2)
		{
			std::cout<<"controller changed!  from 9-12 sec \n";
			Eigen::VectorXd deltaX(3);
			deltaX<<txtMat(3,3),txtMat(3,4),txtMat(3,5);
			T0=mr::FKinSpace(eeFrame,Slist,thetalist);
			T1=T0;
			T1.block<3,1>(0,3)=	T1.block<3,1>(0,3)+	deltaX;			
			//intXe<<0,0,0,0,0,0;
			setT0T1_2=true;
			std::cout<<"T0: "<<T0<<"\n";
			std::cout<<"T0: "<<T1<<"\n";			
		}		
        //design PID controller for motion controller: T0-->T1	
		Js=mr::JacobianSpace(Slist, thetalist);
		curF=getFT(m,d);
		
        Eigen::MatrixXd  curT06=mr::FKinSpace(eeFrame,Slist,thetalist);
		Eigen::MatrixXd  nextT06=mr::FKinSpace(eeFrame,Slist,thetalist+dt*dthetalist);
		
		Jf=JacobianF(Slist,thetalist); 
		Eigen::VectorXd X1_real=f_QX(curT06);
		Eigen::VectorXd X2_real=f_QX(nextT06);
		Eigen::VectorXd dX_real=(X2_real-X1_real)/dt;
		
		Eigen::MatrixXd dJf=(JacobianF(Slist,thetalist+dt*dthetalist)-Jf)/dt;  //derivative of Jabobian			
		Eigen::MatrixXd Xd=f_QX( calTd(T0,T1,curT-phase1T,phase1Dur) );		
		
		dXd=caldXd(T0,T1,curT-phase1T,phase1Dur) ;			
		ddXd=calddXd(T0,T1,curT-phase1T,phase1Dur);							
		dXe=dXd-dX_real; 		
		Eigen::VectorXd Xe=Xd-X1_real;  					
		prev_dX_real=dX_real;		
		intXe=intXe+Xe*dt;		
		
		Eigen::MatrixXd dJfTh=dJf*dthetalist;
		Eigen::MatrixXd MJfInvdJfTh=UR5M*( Jf.colPivHouseholderQr().solve(dJfTh) );
		
		ctrl_eig=ctrl_eigff+UR5M*(Jf.colPivHouseholderQr().solve(ddXd+Kv1*dXe+Kp1*Xe+Ki1*intXe-dJf*dthetalist));			
		
		if(curT>6 && curT<6.01){
		}		
	}	
	//combine two controllers
	if(phaseN==3){ //relax the constraints on z
	     //compuate matrix A
		 Eigen::MatrixXd matA=calJphi();
	     //compute Lambda
		 Eigen::MatrixXd Lambda(6,6);
		 Js=mr::JacobianSpace(Slist, thetalist);
		 Jf=JacobianF(Slist,thetalist); 
		 Lambda=Js.transpose().colPivHouseholderQr().solve(  solA_XB(UR5M,Jf)   );		 
		 //compute matrix P
		 Eigen::MatrixXd S1=solA_XB(matA,Lambda);
		 Eigen::MatrixXd S2=( S1*( matA.transpose() ) ).colPivHouseholderQr().solve( S1 );
		 Eigen::MatrixXd matI6=Eigen::MatrixXd::Identity(6, 6); //6x6 identity matrix
		 Eigen::MatrixXd matP=matI6-matA.transpose()*S2;		 
		 Eigen::MatrixXd matI_P=matI6-matP; 	   	
		
        Eigen::MatrixXd  curT06=mr::FKinSpace(eeFrame_p4,Slist,thetalist);
		Eigen::MatrixXd  nextT06=mr::FKinSpace(eeFrame_p4,Slist,thetalist+dt*dthetalist);		
		
		Eigen::VectorXd X1_real=f_QX(curT06);
		Eigen::VectorXd X2_real=f_QX(nextT06);
		Eigen::VectorXd dX_real=(X2_real-X1_real)/dt;	
		
		Eigen::MatrixXd dJf=(JacobianF(Slist,thetalist+dt*dthetalist)-Jf)/dt;  //derivative of Jabobian			
		Eigen::MatrixXd Xd=f_QX( calTd(T0,T1,curT-phase1T,phase1Dur) );		
		
		dXd=caldXd(T0,T1,curT-phase1T,phase1Dur) ;			
		ddXd=calddXd(T0,T1,curT-phase1T,phase1Dur);							
		dXe=dXd-dX_real; 		
		Eigen::VectorXd Xe=Xd-X1_real;  					
		prev_dX_real=dX_real;		
		intXe=intXe+Xe*dt;		
		
		Eigen::VectorXd tau_motion=Js.transpose()*Lambda*(ddXd+Kv1*dXe+Kp1*Xe+Ki1*intXe-dJf*dthetalist); 
		
		curF=getFT(m,d);	
		double x6=Fdz-curF(5);	
		double dx6=(x6-prevx6)/dt;
		intx6=intx6+x6;
		T1(2,3)=T1(2,3)+dt*(Kdfe*dx6+Kpfe*x6+Kife*intx6);
		T0(2,3)=T0(2,3)+dt*(Kdfe*dx6+Kpfe*x6+Kife*intx6);
		prevx6=x6;		
		
		Eigen::MatrixXd dJfTh=dJf*dthetalist;
		Eigen::MatrixXd MJfInvdJfTh=UR5M*( Jf.colPivHouseholderQr().solve(dJfTh) );
		
		ctrl_eig=ctrl_eigff+tau_motion;//+tau_force;//tau_motion
		//if(setPhaseEnd){
		//	ctrl_eig=Js.transpose()*Lambda*(ddXd+Kv1*dXe+Kp1*Xe+Ki1*intXe-dJf*dthetalist);
		//}
		if(curT>10.6 && curT<10.72){
				//std::cout<<"phase3!tau_motion ctrl_eig"<<tau_motion.transpose()<<" "<<ctrl_eig.transpose()<<"\n \n";					
		}
			
		
	}
	
	if(phaseN==4){ //rotate around x
	
	    Eigen::MatrixXd  curT06=mr::FKinSpace(eeFrame_p4,Slist,thetalist);
		Eigen::MatrixXd  nextT06=mr::FKinSpace(eeFrame_p4,Slist,thetalist+dt*dthetalist);		
		
		Eigen::VectorXd X1_real=f_QX(curT06);	
		Eigen::VectorXd X2_real=f_QX(nextT06);
		Eigen::VectorXd dX_real=(X2_real-X1_real)/dt;		
	    
		Eigen::VectorXd dht=caldht4(T0, curT-phase1T, phase1Dur, total_angX);     
		
	     //compuate matrix A
		 //Eigen::MatrixXd matA=calJphi4(X1_real,dht, dX_real); //(Eigen::VectorXd Xphi, Eigen::VectorXd dht, Eigen::VectorXd A_dXd){
	     Eigen::MatrixXd matA=calJphi();
	     //compute Lambda
		 Eigen::MatrixXd Lambda(6,6);
		 Js=mr::JacobianSpace(Slist, thetalist);
		 Jf=JacobianF(Slist,thetalist); 
		 Lambda=Js.transpose().colPivHouseholderQr().solve(  solA_XB(UR5M,Jf)   );		 
		 //compute matrix P
		 Eigen::MatrixXd S1=solA_XB(matA,Lambda);
		 Eigen::MatrixXd S2=( S1*( matA.transpose() ) ).colPivHouseholderQr().solve( S1 );
		 Eigen::MatrixXd matI6=Eigen::MatrixXd::Identity(6, 6); //6x6 identity matrix
		 Eigen::MatrixXd matP=matI6-matA.transpose()*S2;		 
		 Eigen::MatrixXd matI_P=matI6-matP; 	   	        
		//std::cout<<"mark9\n";
		Eigen::MatrixXd dJf=(JacobianF(Slist,thetalist+dt*dthetalist)-Jf)/dt;  //derivative of Jabobian	
		
		Eigen::MatrixXd Xd=f_QX( calTd4(T0, curT-phase1T, phase1Dur, total_angX));			
		dXd=caldXd4(T0, curT-phase1T, phase1Dur, total_angX);		
        	
		ddXd=calddXd4(T0, curT-phase1T, phase1Dur, total_angX);		
       	
		dXe=dXd-dX_real; 		
		Eigen::VectorXd Xe=Xd-X1_real;  					
		prev_dX_real=dX_real;		
		intXe=intXe+Xe*dt;		
		
		Eigen::VectorXd tau_motion=Js.transpose()*Lambda*(ddXd+Kv1*dXe+Kp1*Xe+Ki1*intXe-dJf*dthetalist);//
		
		curF=getFT(m,d);
		double x6=Fdz-curF(5);	
		double dx6=(x6-prevx6)/dt;
		intx6=intx6+x6;
		T1(2,3)=T1(2,3)+dt*(Kdfe*dx6+Kpfe*x6+Kife*intx6);
		T0(2,3)=T0(2,3)+dt*(Kdfe*dx6+Kpfe*x6+Kife*intx6);
		prevx6=x6;
		
		Eigen::MatrixXd dJfTh=dJf*dthetalist;
		Eigen::MatrixXd MJfInvdJfTh=UR5M*( Jf.colPivHouseholderQr().solve(dJfTh) );		
		ctrl_eig=ctrl_eigff+tau_motion;
		
	}		
	
	if(phaseN==5 ){ //Rotate around Z
	
	    Eigen::MatrixXd  curT06=mr::FKinSpace(eeFrame_p4,Slist,thetalist);
		Eigen::MatrixXd  nextT06=mr::FKinSpace(eeFrame_p4,Slist,thetalist+dt*dthetalist);		
		
		Eigen::VectorXd X1_real=f_QX(curT06);	
		Eigen::VectorXd X2_real=f_QX(nextT06);
		Eigen::VectorXd dX_real=(X2_real-X1_real)/dt;		
	    
		Eigen::VectorXd dht=caldht5(T0, curT-phase1T, phase1Dur, total_angZ);     
		
	     //compuate matrix A
		 //Eigen::MatrixXd matA=calJphi4(X1_real,dht, dX_real); //(Eigen::VectorXd Xphi, Eigen::VectorXd dht, Eigen::VectorXd A_dXd){
		Eigen::MatrixXd matA=calJphi();
	     //compute Lambda
		 Eigen::MatrixXd Lambda(6,6);
		 Js=mr::JacobianSpace(Slist, thetalist);
		 Jf=JacobianF(Slist,thetalist); 
		 Lambda=Js.transpose().colPivHouseholderQr().solve(  solA_XB(UR5M,Jf)   );		 
		 //compute matrix P
		 Eigen::MatrixXd S1=solA_XB(matA,Lambda);
		 Eigen::MatrixXd S2=( S1*( matA.transpose() ) ).colPivHouseholderQr().solve( S1 );
		 Eigen::MatrixXd matI6=Eigen::MatrixXd::Identity(6, 6); //6x6 identity matrix
		 Eigen::MatrixXd matP=matI6-matA.transpose()*S2;		 
		 Eigen::MatrixXd matI_P=matI6-matP; 	   	        
		Eigen::MatrixXd dJf=(JacobianF(Slist,thetalist+dt*dthetalist)-Jf)/dt;  //derivative of Jabobian	
		
		Eigen::MatrixXd Xd=f_QX( calTd5(T0, curT-phase1T, phase1Dur, total_angZ));			
		dXd=caldXd5(T0, curT-phase1T, phase1Dur, total_angZ);	        	
		ddXd=calddXd5(T0, curT-phase1T, phase1Dur, total_angZ);		
       	
		dXe=dXd-dX_real; 		
		Eigen::VectorXd Xe=Xd-X1_real;  					
		prev_dX_real=dX_real;		
		intXe=intXe+Xe*dt;		
		
		Eigen::VectorXd tau_motion=Js.transpose()*Lambda*(ddXd+Kv1*dXe+Kp1*Xe+Ki1*intXe-dJf*dthetalist);//
		
		curF=getFT(m,d);
		curF=getFT(m,d);
		double x6=Fdz-curF(5);	
		double dx6=(x6-prevx6)/dt;
		intx6=intx6+x6;
		T1(2,3)=T1(2,3)+dt*(Kdfe*dx6+Kpfe*x6+Kife*intx6);
		T0(2,3)=T0(2,3)+dt*(Kdfe*dx6+Kpfe*x6+Kife*intx6);
		prevx6=x6;
			
		//intFe=intFe+Fe*dt;			
		//Eigen::VectorXd tau_force=Js.transpose()*matI_P*(Fd+Kp2*Fe+Ki2*intFe);
		
		Eigen::MatrixXd dJfTh=dJf*dthetalist;
		Eigen::MatrixXd MJfInvdJfTh=UR5M*( Jf.colPivHouseholderQr().solve(dJfTh) );		
		ctrl_eig=ctrl_eigff+tau_motion;	
		
	}	
	if(phaseN==6){ //slide the tip on the screw surface
	    globalT=curT-phase1T;
	    Eigen::MatrixXd  curT06=mr::FKinSpace(eeFrame_p4,Slist,thetalist);
		Eigen::MatrixXd  nextT06=mr::FKinSpace(eeFrame_p4,Slist,thetalist+dt*dthetalist);		
		
		Eigen::VectorXd X1_real=f_QX(curT06);	
		Eigen::VectorXd X2_real=f_QX(nextT06);
		Eigen::VectorXd dX_real=(X2_real-X1_real)/dt;		
	    
		//Eigen::VectorXd dht=caldht5(T0, curT-phase1T, phase1Dur, total_angZ);     
		Eigen::VectorXd dht=caldht6(LSlide/TSlide);
		
	     //compuate matrix A
		 Eigen::MatrixXd matA=calJphi6(X1_real,dht, dX_real); //(Eigen::VectorXd Xphi, Eigen::VectorXd dht, Eigen::VectorXd A_dXd){
	     //compute Lambda
		 Eigen::MatrixXd Lambda(6,6);
		 Js=mr::JacobianSpace(Slist, thetalist);
		 Jf=JacobianF(Slist,thetalist); //need to add phaseN=6 case
		 Lambda=Js.transpose().colPivHouseholderQr().solve(  solA_XB(UR5M,Jf)   );		 
		 //compute matrix P
		 Eigen::MatrixXd S1=solA_XB(matA,Lambda);
		 Eigen::MatrixXd S2=( S1*( matA.transpose() ) ).colPivHouseholderQr().solve( S1 );       
		Eigen::MatrixXd dJf=(JacobianF(Slist,thetalist+dt*dthetalist)-Jf)/dt;  //derivative of Jabobian	
		
		Eigen::MatrixXd Xd=f_QX(  calTd(T0,T1,curT-phase1T,phase1Dur)  );			
		dXd=caldXd(T0,T1,curT-phase1T,phase1Dur) ;	        	
		ddXd=calddXd(T0,T1,curT-phase1T,phase1Dur) ;		
       	
		dXe=dXd-dX_real; 		
		Eigen::VectorXd Xe=Xd-X1_real;  					
		prev_dX_real=dX_real;		
		intXe=intXe+Xe*dt;		
		
		Eigen::VectorXd tau_motion=Js.transpose()*Lambda*(ddXd+Kv1*dXe+Kp1*Xe+Ki1*intXe-dJf*dthetalist);//
				
		curF=getFT(m,d);
		double x6=Fdz-curF(5);	
		double dx6=(x6-prevx6)/dt;
		intx6=intx6+x6;
		T1(2,3)=T1(2,3)+dt*(Kdfe*dx6+Kpfe*x6+Kife*intx6);
		T0(2,3)=T0(2,3)+dt*(Kdfe*dx6+Kpfe*x6+Kife*intx6);
		prevx6=x6;
		
		Eigen::MatrixXd dJfTh=dJf*dthetalist;
		Eigen::MatrixXd MJfInvdJfTh=UR5M*( Jf.colPivHouseholderQr().solve(dJfTh) );		
		ctrl_eig=ctrl_eigff+tau_motion;
	}
	
	
	//pass tau to a low pass filter
	
	for(i=0;i<Nj;i++)
		ctrl_eig(i)=dt/RC*( ctrl_eig(i)-ctrl_eig_prev(i) )+ctrl_eig_prev(i);	
			
	for(i=0;i<Nj;i++){
		ctrl_eig_prev(i)=ctrl_eig(i);
		//if(!setPhaseEnd){
		ctrlTorque[i]=ctrl_eig(i);
		//}
	}
	
  }



} //end of namespace  ::mr










