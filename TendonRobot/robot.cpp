#include "stdafx.h"
#include<Eigen/Dense>
#include "math.h"
using namespace Eigen;
#include"robot.h"



Robot::Robot(double l, double r, double t1,double t2,double d1,double d2)
{
	Length=l; Radius=r; theta1=t1;theta2=t2;delta1=d1;delta2=d2;
	alpha=M_PI/3; beta=2*M_PI/3;
}

void Robot::SetRobot(double l, double r, double t1,double t2,double d1,double d2)
{
	Length=l; Radius=r; theta1=t1;theta2=t2;delta1=d1;delta2=d2;
	alpha=M_PI/3; beta=2*M_PI/3;
}


Matrix<double, 4, 3> Robot::JacobianPC(double t1,double d1,double t2,double d2)
{
	Matrix<double, 3, 4> J;

	double xtheta1, xtheta2, xdelta1, xdelta2,ytheta1, ytheta2, ydelta1, ydelta2,ztheta1, ztheta2, zdelta1, zdelta2;

	theta1=t1;theta2=t2;delta1=d1;delta2=d2;// update configuration

	if (theta1==M_PI/2&&theta2==M_PI/2)
	{
        
		xtheta1=-(3*Length*cos(delta1))/2;
		xdelta1=0;
		xtheta2=-(Length*cos(alpha + delta2))/2;
		xdelta2=0;
		ytheta1=-(3*Length*sin(delta1))/2;
		ydelta1=0;
		ytheta2=-(Length*sin(alpha + delta2))/2;
		ydelta2=0;
		ztheta1=0;
		zdelta1=0;
		ztheta2=(Length*cos(alpha - delta1 + delta2)*cos(delta1))/2;
		zdelta2=0;
    
		J<< xtheta1, xdelta1, xtheta2, xdelta2,
			ytheta1, ydelta1, ytheta2, ydelta2,
			ztheta1, zdelta1, ztheta2, zdelta2;
	}

	else if (theta1==M_PI/2&&theta2!=M_PI/2)
	{
    
		xtheta1= -(Length*cos(delta1)*(M_PI - 2*theta2 + 4*cos(theta2)))/(2*(M_PI - 2*theta2));
		xdelta1=0;
		xtheta2= -(2*Length*cos(alpha + delta2)*(2*sin(theta2) + M_PI*cos(theta2) - 2*theta2*cos(theta2) - 2))/pow((M_PI - 2*theta2),2);
		xdelta2=(Length*sin(alpha + delta2)*(sin(theta2) - 1))/(M_PI/2 - theta2);
		ytheta1= -(Length*sin(delta1)*(M_PI - 2*theta2 + 4*cos(theta2)))/(2*(M_PI - 2*theta2));
		ydelta1=0;
		ytheta2= -(2*Length*sin(alpha + delta2)*(2*sin(theta2) + M_PI*cos(theta2) - 2*theta2*cos(theta2) - 2))/pow((M_PI - 2*theta2),2);
		ydelta2= -(Length*cos(alpha + delta2)*(sin(theta2) - 1))/(M_PI/2 - theta2);
		ztheta1= -(2*Length*cos(alpha - delta1 + delta2)*(sin(theta2) - 1))/(M_PI - 2*theta2);
		zdelta1=0;
		ztheta2=(2*Length*(2*cos(theta2) - M_PI*sin(theta2) + 2*theta2*sin(theta2)))/pow((M_PI - 2*theta2),2);
		zdelta2=0;
		 
		J<< xtheta1, xdelta1, xtheta2, xdelta2,
			ytheta1, ydelta1, ytheta2, ydelta2,
			ztheta1, zdelta1, ztheta2, zdelta2;
	}

	else if (theta2==M_PI/2&&theta1!=M_PI/2)
	{
    
		xtheta1= - Length*cos(delta1)*sin(theta1) - (Length*cos(delta1)*cos(theta1))/(M_PI/2 - theta1) - (Length*cos(delta1)*(sin(theta1) - 1))/pow((M_PI/2 - theta1),2);
		xdelta1=(2*Length*sin(delta1)*(sin(theta1) - 1))/(M_PI - 2*theta1) - Length*sin(delta1)*cos(theta1);
		xtheta2=(Length*(sin(alpha - delta1 + delta2)*sin(delta1) - cos(alpha - delta1 + delta2)*cos(delta1)*sin(theta1)))/2;
		xdelta2=0;
		ytheta1= -Length*sin(delta1)*sin(theta1) - (Length*sin(delta1)*(sin(theta1) - 1))/pow((M_PI/2 - theta1),2) - (Length*sin(delta1)*cos(theta1))/(M_PI/2 - theta1);
		ydelta1=Length*cos(delta1)*cos(theta1) - (2*Length*cos(delta1)*(sin(theta1) - 1))/(M_PI - 2*theta1);
		ytheta2=-(Length*(sin(alpha - delta1 + delta2)*cos(delta1) + cos(alpha - delta1 + delta2)*sin(delta1)*sin(theta1)))/2;
		ydelta2=0;
		ztheta1=Length*cos(theta1) + (Length*cos(theta1))/pow((M_PI/2 - theta1),2) - (Length*sin(theta1))/(M_PI/2 - theta1);
		zdelta1=0;
		ztheta2=(Length*cos(alpha - delta1 + delta2)*cos(theta1))/2;
		zdelta2=0;

		J<< xtheta1, xdelta1, xtheta2, xdelta2,
			ytheta1, ydelta1, ytheta2, ydelta2,
			ztheta1, zdelta1, ztheta2, zdelta2;
	}
	else
	{
		xtheta1= -(Length*cos(delta1)*cos(theta1))/(M_PI/2 - theta1) - (Length*cos(delta1)*(sin(theta1) - 1))/pow((M_PI/2 - theta1),2) - (Length*cos(delta1)*cos(theta2)*sin(theta1))/(M_PI/2 - theta2) - (Length*cos(alpha - delta1 + delta2)*cos(delta1)*cos(theta1)*(sin(theta2) - 1))/(M_PI/2 - theta2);
		xdelta1=(2*Length*(sin(theta2) - 1)*(sin(alpha - 2*delta1 + delta2) - cos(alpha - 2*delta1 + delta2 - theta1)/2 + cos(alpha - 2*delta1 + delta2 + theta1)/2))/(M_PI - 2*theta2) + (2*Length*sin(delta1)*(sin(theta1) - 1))/(M_PI - 2*theta1) - (2*Length*sin(delta1)*cos(theta1)*cos(theta2))/(M_PI- 2*theta2);
		xtheta2=(Length*cos(theta2)*(sin(alpha - delta1 + delta2)*sin(delta1) - cos(alpha - delta1 + delta2)*cos(delta1)*sin(theta1)))/(M_PI/2 - theta2) + (Length*(sin(theta2) - 1)*(sin(alpha - delta1 + delta2)*sin(delta1) - cos(alpha - delta1 + delta2)*cos(delta1)*sin(theta1)))/pow((M_PI/2 - theta2),2) + (Length*cos(delta1)*cos(theta1)*cos(theta2))/pow((M_PI/2 - theta2),2) - (Length*cos(delta1)*cos(theta1)*sin(theta2))/(M_PI/2 - theta2);
		xdelta2=(Length*(sin(theta2) - 1)*(cos(alpha - delta1 + delta2)*sin(delta1) + sin(alpha - delta1 + delta2)*cos(delta1)*sin(theta1)))/(M_PI/2 - theta2);
		ytheta1= -(Length*sin(delta1)*(sin(theta1) - 1))/pow((M_PI/2 - theta1),2) - (Length*sin(delta1)*cos(theta1))/(M_PI/2 - theta1) - (Length*sin(delta1)*cos(theta2)*sin(theta1))/(M_PI/2 - theta2) - (Length*cos(alpha - delta1 + delta2)*sin(delta1)*cos(theta1)*(sin(theta2) - 1))/(M_PI/2 - theta2);
		ydelta1=(2*Length*(sin(theta2) - 1)*(sin(alpha - 2*delta1 + delta2 - theta1)/2 + cos(alpha - 2*delta1 + delta2) - sin(alpha - 2*delta1 + delta2 + theta1)/2))/(M_PI - 2*theta2) - (2*Length*cos(delta1)*(sin(theta1) - 1))/(M_PI - 2*theta1) + (2*Length*cos(delta1)*cos(theta1)*cos(theta2))/(M_PI- 2*theta2);
		ytheta2=(Length*sin(delta1)*cos(theta1)*cos(theta2))/pow((M_PI/2 - theta2),2) - (Length*(sin(theta2) - 1)*(sin(alpha - delta1 + delta2)*cos(delta1) + cos(alpha - delta1 + delta2)*sin(delta1)*sin(theta1)))/pow((M_PI/2 - theta2),2) - (Length*cos(theta2)*(sin(alpha - delta1 + delta2)*cos(delta1) + cos(alpha - delta1 + delta2)*sin(delta1)*sin(theta1)))/(M_PI/2 - theta2) - (Length*sin(delta1)*cos(theta1)*sin(theta2))/(M_PI/2 - theta2);
		ydelta2= -(Length*(sin(theta2) - 1)*(cos(alpha - delta1 + delta2)*cos(delta1) - sin(alpha - delta1 + delta2)*sin(delta1)*sin(theta1)))/(M_PI/2 - theta2);
		ztheta1=(Length*cos(theta1))/pow((M_PI/2 - theta1),2) - (Length*sin(theta1))/(M_PI/2 - theta1) + (Length*cos(theta1)*cos(theta2))/(M_PI/2 - theta2) - (Length*cos(alpha - delta1 + delta2)*sin(theta1)*(sin(theta2) - 1))/(M_PI/2 - theta2);
		zdelta1=(Length*sin(alpha - delta1 + delta2)*cos(theta1)*(sin(theta2) - 1))/(M_PI/2 - theta2);
		ztheta2=(Length*cos(theta2)*sin(theta1))/pow((M_PI/2 - theta2),2) - (Length*sin(theta1)*sin(theta2))/(M_PI/2 - theta2) + (Length*cos(alpha - delta1 + delta2)*cos(theta1)*(sin(theta2) - 1))/pow((M_PI/2 - theta2),2) + (Length*cos(alpha - delta1 + delta2)*cos(theta1)*cos(theta2))/(M_PI/2 - theta2);
		zdelta2=-(Length*sin(alpha - delta1 + delta2)*cos(theta1)*(sin(theta2) - 1))/(M_PI/2 - theta2);

		J<< xtheta1, xdelta1, xtheta2, xdelta2,
			ytheta1, ydelta1, ytheta2, ydelta2,
			ztheta1, zdelta1, ztheta2, zdelta2;
	}
    

	return moorepenrose(J);
}

Matrix<double, 6, 4> Robot::JacobianCL(double t1,double d1,double t2,double d2)
{
	Matrix<double, 6, 4> J;

	theta1=t1;theta2=t2;delta1=d1;delta2=d2;

		J<< Radius*cos(delta1),        -Radius*(theta1-M_PI/2)*sin(delta1),          0, 0,
			Radius*cos(delta1+beta),   -Radius*(theta1-M_PI/2)*sin(delta1+beta),     0, 0,
			Radius*cos(delta1+2*beta), -Radius*(theta1-M_PI/2)*sin(delta1+2*beta),   0, 0,
			Radius*cos(delta1+alpha),  -Radius*(theta1-M_PI/2)*sin(delta1+alpha),          Radius*cos(delta2+alpha),   -Radius*(theta2-M_PI/2)*sin(delta2+alpha),
			Radius*cos(delta1+beta+alpha),   -Radius*(theta1-M_PI/2)*sin(delta1+beta+alpha),     Radius*cos(delta2+beta+alpha),   -Radius*(theta2-M_PI/2)*sin(delta2+beta+alpha),
			Radius*cos(delta1+2*beta+alpha), -Radius*(theta1-M_PI/2)*sin(delta1+2*beta+alpha),   Radius*cos(delta2+2*beta+alpha),   -Radius*(theta2-M_PI/2)*sin(delta2+2*beta+alpha);

    return J;
}

MatrixXd Robot::TrajectoryGeneration(double dx, double dy, double dz, int T, int F )
{
	//F means control frequency
	//T means control time span
	double A=dx/T, B=dy/T, C=dz/T,omega=2*M_PI/T;
	ArrayXd t=ArrayXd::LinSpaced(T*F,0,T);
	ArrayXd a=ArrayXd::LinSpaced(T*F,A,A);
	ArrayXd b=ArrayXd::LinSpaced(T*F,B,B);
	ArrayXd c=ArrayXd::LinSpaced(T*F,C,C);
	ArrayXd traa=(omega*t).cos()*(-A)+a;
	ArrayXd trab=(omega*t).cos()*(-B)+b;
	ArrayXd trac=(omega*t).cos()*(-C)+c;
	MatrixXd Tra(3,T*F);
	Tra.row(0)=traa.matrix().transpose();
	Tra.row(1)=trab.matrix().transpose();
	Tra.row(2)=trac.matrix().transpose();
		return Tra;
}

MatrixXd Robot::TrajectoryGeneration(double dt1, double dd1, double dt2, double dd2,int T,int F)
{
	//F means control frequency
	//T means control time span
	double A=dt1/T,B=dd1/T,C=dt2/T,D=dd2/T, omega=2*M_PI/T;
	ArrayXd t=ArrayXd::LinSpaced(T*F,0,T);
	ArrayXd a=ArrayXd::LinSpaced(T*F,A,A);
	ArrayXd b=ArrayXd::LinSpaced(T*F,B,B);
	ArrayXd c=ArrayXd::LinSpaced(T*F,C,C);
	ArrayXd d=ArrayXd::LinSpaced(T*F,D,D);
	ArrayXd traa=(omega*t).cos()*(-A)+a;
	ArrayXd trab=(omega*t).cos()*(-B)+b;
	ArrayXd trac=(omega*t).cos()*(-C)+c;
	ArrayXd trad=(omega*t).cos()*(-D)+d;
	MatrixXd Tra(4,T*F);
	Tra.row(0)=traa.matrix().transpose();
	Tra.row(1)=trab.matrix().transpose();
	Tra.row(2)=trac.matrix().transpose();
	Tra.row(3)=trad.matrix().transpose();
	return Tra;
}

Vector4d Robot::Configuration(double L11,double L12,double L13,double L21,double L22,double L23)
{   
	double dl1, dl2, dl3, x21, x22, x23, pdelta2=0, ptheta2=0;
	Vector4d configuration;
	if (L11==L12&&L12==L13)
	{
		delta1=0;
	    theta1=M_PI/2;
	}
	else
	{
		delta1=atan2((L12-Length)-(L11-Length)*cos(beta),-(L11-Length)*sin(beta));
        theta1=M_PI/2+(L11-Length)/Radius/cos(delta1);
	}

	dl1=Length+Radius*cos(delta1+alpha)*(theta1-M_PI/2);
	dl2=Length+Radius*cos(delta1+beta+alpha)*(theta1-M_PI/2);
	dl3=Length+Radius*cos(delta1+2*beta+alpha)*(theta1-M_PI/2);
	L21=L21-dl1; L22=L22-dl2; L23=L23-dl3;
	if (L21==L22&&L22==L23)
	{
		delta2=0;
	    theta2=M_PI/2;
	}
	else
	{
	pdelta2=atan2((L22-Length)-(L21-Length)*cos(beta),-(L21-Length)*sin(beta));
	ptheta2=M_PI/2+(L21-Length)/Radius/cos(pdelta2);
	x21=Length+Radius*cos(pdelta2-alpha)*(ptheta2-M_PI/2);
	x22=Length+Radius*cos(pdelta2+beta-alpha)*(ptheta2-M_PI/2);
	x23=Length+Radius*cos(pdelta2+2*beta-alpha)*(ptheta2-M_PI/2);

	delta2=atan2((x22-Length)-(x21-Length)*cos(beta),-(x21-Length)*sin(beta));
	theta2=M_PI/2+(x21-Length)/Radius/cos(delta2);
	}


	configuration(0)=theta1; configuration(1)=delta1; configuration(2)=theta2; configuration(3)=delta2;
	
    return configuration;

}

Vector3d Robot::Position(double L11,double L12,double L13,double L21,double L22,double L23)
{   
	double dl1, dl2, dl3;
	Vector3d position;
	if (L11==L12&&L12==L13)
		delta1=0;
	else
		delta1=atan2((L12-Length)-(L11-Length)*cos(beta),-(L11-Length)*sin(beta));

	theta1=M_PI/2+(L11-Length)/Radius/cos(delta1);

	dl1=Length+Radius*cos(delta1+alpha)*(theta1-M_PI/2);
	dl2=Length+Radius*cos(delta1+beta+alpha)*(theta1-M_PI/2);
	dl3=Length+Radius*cos(delta1+2*beta+alpha)*(theta1-M_PI/2);
	L21=L21-dl1; L22=L22-dl2; L23=L23-dl3;

	if (L21==L22&&L22==L23)
		delta2=0;
	else
		delta2=atan2((L22-Length)-(L21-Length)*cos(beta),-(L21-Length)*sin(beta));

	theta2=M_PI/2+(L21-Length)/Radius/cos(delta2);
	
	//ÆæÒìÎ»×ËÎ´¿¼ÂÇ£¡£¡£¡£¡
	position(0)=Length=(Length*(sin(theta2) - 1)*(sin(alpha - delta1 + delta2)*sin(delta1) - cos(alpha - delta1 + delta2)*cos(delta1)*sin(theta1)))/(M_PI/2 - theta2) - (Length*cos(delta1)*(sin(theta1) - 1))/(M_PI/2 - theta1) + (Length*cos(delta1)*cos(theta1)*cos(theta2))/(M_PI/2 - theta2);
    position(1)=Length=(Length*sin(delta1)*cos(theta1)*cos(theta2))/(M_PI/2 - theta2) - (Length*(sin(theta2) - 1)*(sin(alpha - delta1 + delta2)*cos(delta1) + cos(alpha - delta1 + delta2)*sin(delta1)*sin(theta1)))/(M_PI/2 - theta2) - (Length*sin(delta1)*(sin(theta1) - 1))/(M_PI/2 - theta1);
	position(2)=Length=(Length*cos(theta1))/(M_PI/2 - theta1) + (Length*cos(theta2)*sin(theta1))/(M_PI/2 - theta2) + (Length*cos(alpha - delta1 + delta2)*cos(theta1)*(sin(theta2) - 1))/(M_PI/2 - theta2);

    return position;

}