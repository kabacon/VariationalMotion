#include <cmath>
#include <iostream>
#include <fstream>

#include "Quaternion.h"

using namespace std;

Quaternion::Quaternion()

{
	q[0]=q[1]=q[2]=q[3] = 0.0f;
}

Quaternion::Quaternion(const double re[4])
{
	for (int i=0; i<4; i++)
		q[i] = re[i];
}

Quaternion::Quaternion(double re1, double re2, double re3, double re4)
{
	q[0] = re1;
	q[1] = re2;
	q[2] = re3;
	q[3] = re4;
}



Quaternion operator+( Quaternion& arg1, Quaternion& arg2)
{ 
Quaternion sum;

	for (int i=0; i<4; i++)
		sum.q[i] = arg1.q[i] + arg2.q[i];

	return sum;
}

Quaternion operator-( Quaternion& arg1, Quaternion& arg2)
{ 
Quaternion diff;

	for (int i=0; i<4; i++)
		diff.q[i] = arg1.q[i] - arg2.q[i];

	return diff;
}


double Quaternion::Modulus()
{
double sum = 0.0;

	for (int i=0; i<4; i++)
		sum += q[i]*q[i];
	

	return sum;
}
	

Quaternion Quaternion::Conjugate()
{

	Quaternion conju(-q[0], -q[1], -q[2], q[3]);

return conju;
}


Quaternion Quaternion::Inverse()
{

	return Conjugate()/Modulus();
}

Quaternion operator/(Quaternion& arg1, double arg2)
{
Quaternion division;
	
	for (int i=0; i<4; i++)
	division.q[i] = arg1.q[i] / arg2;

return division;

}


Quaternion operator*( Quaternion& arg1, Quaternion& arg2)
{
Quaternion product;

product.q[0] = arg1.q[3]*arg2.q[0] + arg2.q[3]*arg1.q[0] +
			   arg1.q[1]*arg2.q[2] - arg1.q[2]*arg2.q[1];

product.q[1] = arg1.q[3]*arg2.q[1] + arg2.q[3]*arg1.q[1] +
			   arg1.q[2]*arg2.q[0] - arg1.q[0]*arg2.q[2];

product.q[2] = arg1.q[3]*arg2.q[2] + arg2.q[3]*arg1.q[2] +
			   arg1.q[0]*arg2.q[1] - arg1.q[1]*arg2.q[0];

product.q[3] = arg1.q[3]*arg2.q[3] - 
				(arg1.q[0]*arg2.q[0] + arg1.q[1]*arg2.q[1] + arg1.q[2]*arg2.q[2]);

return product;
}

Quaternion operator*( double arg1, Quaternion& arg2)
{
Quaternion product;
	
	for (int i=0; i<4; i++)
	product.q[i] = arg1 * arg2.q[i];

return product;

}

Quaternion operator*(Quaternion& arg1, double arg2)
{
Quaternion product;
	
	for (int i=0; i<4; i++)
	product.q[i] = arg2 * arg1.q[i];

return product;

}

Quaternion& Quaternion::operator=(Quaternion arg1)
{
	for (int i=0; i<4; i++)
		q[i] = arg1.q[i];

return *this;
}

ifstream &operator>>(ifstream &input, Quaternion & arg)
{
	for (int i=0; i<4; i++)
		input>>arg.q[i];

	return input;

}
ofstream &operator<<(ofstream &output, const Quaternion & arg)		   
{
	//for (int i=0; i<4; i++)
	//	output<<arg.q[i];
	output << "(" << arg.q[0] << ", " << arg.q[1] << ", " << arg.q[2] << ", " << arg.q[3] << ")" << endl; 
	return output;
}
			
istream &operator>>(istream &input, Quaternion & arg)
{
	for (int i=0; i<4; i++)
		input>>arg.q[i];

	return input;

}
ostream &operator<<(ostream &output, const Quaternion & arg)		   
{
	//for (int i=0; i<4; i++)
	//	output<<arg.q[i];
	output << "(" << arg.q[0] << ", " << arg.q[1] << ", " << arg.q[2] << ", " << arg.q[3] << ")"; 
	return output;
}

Matrix Quaternion::quaternionToRotationMatrix()
{
    Matrix R(3,3);
	double k;
	k = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    R.m[0][0] = (q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3])/k;
	R.m[0][1] = 2*(q[0]*q[1] - q[2]*q[3])/k;
	R.m[0][2] = 2*(q[0]*q[2] + q[1]*q[3])/k;
	R.m[1][0] = 2*(q[0]*q[1] + q[2]*q[3])/k;
	R.m[1][1] = (- q[0]*q[0] + q[1]*q[1] - q[2]*q[2] + q[3]*q[3])/k;
	R.m[1][2] = 2*(q[1]*q[2] - q[0]*q[3])/k;
	R.m[2][0] = 2*(q[0]*q[2] - q[1]*q[3])/k;
	R.m[2][1] = 2*(q[1]*q[2] + q[0]*q[3])/k;
	R.m[2][2] = (- q[0]*q[0] - q[1]*q[1] + q[2]*q[2] + q[3]*q[3])/k;

	return (R);
}