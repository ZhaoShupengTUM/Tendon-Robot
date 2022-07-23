#include<Eigen/Dense>
using namespace Eigen; 

#ifndef ROBOT_H
#define ROBOT_H

class Robot
{
private:

	double Length, Radius, theta1, theta2, delta1,delta2, alpha, beta;


	Matrix<double, 4, 3> moorepenrose(MatrixXd J)
		{	MatrixXd S;
			Matrix<double, 4, 3> invJ;
			JacobiSVD<MatrixXd> svd(J, ComputeThinU | ComputeThinV);
			int a=svd.singularValues().rows();
			int b=svd.nonzeroSingularValues();
			S.resize(a,a);
			S<<MatrixXd::Zero(a,a);
			int i=0;
			while(i<a)
			{
				if (i<b)
				{
					S(i,i)=1/(svd.singularValues()(i));
					i++;
				}
				else
				{
					S(i,i)=0;
					i++;
				}
			}		
			invJ=svd.matrixV()*S*svd.matrixU().transpose();
			return invJ;
		};// pseudo-inverse

public:
	Robot(double l, double r, double t1,double t2,double d1,double d2); // Constructuin function
	Matrix<double, 4, 3> JacobianPC(double t1,double d1,double t2,double d2);// Jacobian from position to configuration space
	Matrix<double, 6, 4> JacobianCL(double t1,double d1,double t2,double d2);// Jacobian from configuration to length space
	MatrixXd TrajectoryGeneration(double dx, double dy, double dz, int T,int F); // 重载轨迹发生函数（position）
	MatrixXd TrajectoryGeneration(double dt1, double dd1, double dt2, double dd2, int T,int F);//重载轨迹发生函数（configuration）
	Vector4d Configuration(double L11,double L12,double L13,double L21,double L22,double L23);
	Vector3d Position(double L11,double L12,double L13,double L21,double L22,double L23);
	void SetRobot(double l, double r, double t1,double t2,double d1,double d2);
};

#endif
