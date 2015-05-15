#include <cmath>
#include <iostream>
#include <fstream>

#include "hPoint.h"
#include "hMatrix.h"
#include "dualQuaternion.h"

using namespace std;

hPoint& hPoint::operator=( hPoint arg1 )
{
	for( int i = 0; i < 4; i ++ )
		coord[i] = arg1.coord[i];

	return *this;
}

hPoint operator*( const hMatrix& arg1, const hPoint& arg2 )
{
	hPoint mul;

	mul.Clear();

	for( int i = 0; i < 4; i ++ )
		for( int j = 0; j < 4; j ++ )
			mul.coord[i] += arg1.m[i][j] * arg2.coord[j];
		
	return mul;
}

hPoint operator*( const hPoint& arg1, double arg2 )
{
	hPoint mul;

	for( int i = 0; i < 4; i ++ )
			mul.coord[i] = arg1.coord[i] * arg2;
		
	return mul;
}

hPoint operator*( double arg1, const hPoint& arg2 )
{
	hPoint mul;

	for( int i = 0; i < 4; i ++ )
			mul.coord[i] = arg2.coord[i] * arg1;
		
	return mul;
}

hPoint operator*( const hPoint& h1, const hPoint& h2 )
{
	hPoint res;

	res.coord[0] = h1.coord[1]*h2.coord[2] - h1.coord[2]*h2.coord[1];
	res.coord[1] = h1.coord[2]*h2.coord[0] - h1.coord[0]*h2.coord[2];
	res.coord[2] = h1.coord[0]*h2.coord[1] - h1.coord[1]*h2.coord[0];
	res.coord[3] = 0.0f;

	return res;
}

hPoint operator/( const hPoint& arg1, double arg2 )
{
	hPoint mul;

	for( int i = 0; i < 4; i ++ )
			mul.coord[i] = arg1.coord[i] / arg2;
		
	return mul;
}

hPoint operator+( const hPoint& arg1, const hPoint& arg2 )
{
	hPoint sum;

	for( int i = 0; i < 4; i ++ )
			sum.coord[i] = arg1.coord[i] + arg2.coord[i];
		
	return sum;
}

hPoint operator-( const hPoint& arg1, const hPoint& arg2 )
{
	hPoint sum;

	for( int i = 0; i < 4; i ++ )
			sum.coord[i] = arg1.coord[i] - arg2.coord[i];
		
	return sum;
}

void hPoint::Clear()
{
	for( int i = 0; i < 4; i ++ )	coord[i] = 0.0;

	return;
}

void hPoint::PointNormalize()
{
	for( int i = 0; i < 4; i ++ )	coord[i] = coord[i]/coord[3];

	return;
}

Point& Point::operator=( Point arg1 )
{
	for( int i=0; i < 3; i ++ )
		coord[i] = arg1.coord[i];

	for( int i = 0; i < 3; i ++ )
		normal[i] = arg1.normal[i];

	visible = arg1.visible;
		
	return *this;
}

Point operator*( const hMatrix& arg1, const Point& arg2 )
{
	hPoint	mul, temp;
	Point	res;

	for( int i = 0; i < 3; i ++ )	temp.coord[i] = arg2.coord[i];
	temp.coord[3] = 1.0f;

	mul = arg1 * temp;

	for(int i = 0; i < 3; i ++ )	res.coord[i] = mul.coord[i] / mul.coord[3];

	return res;
}

Point operator*( const Point& arg1, double arg2 )
{
	Point	res;

	for( int i = 0; i < 3; i ++ )	res.coord[i] = arg1.coord[i] * arg2;

	return res;
}

Point operator*( double arg1, const Point& arg2 )
{
	Point	res;

	for( int i = 0; i < 3; i ++ )	res.coord[i] = arg1 * arg2.coord[i];

	return res;
}

Point operator/( const Point& arg1, double arg2 )
{
	Point	res;

	for( int i = 0; i < 3; i ++ )	res.coord[i] = arg1.coord[i] / arg2;

	return res;
}

Point operator+( const Point& arg1, const Point& arg2 )
{
	Point	res;

	for( int i = 0; i < 3; i ++ )
		res.coord[i] = arg1.coord[i] + arg2.coord[i];

	return res;
}

Point operator-( const Point& arg1, const Point& arg2 )
{
	Point res;

	for( int i = 0; i < 3; i ++ )
		res.coord[i] = arg1.coord[i] - arg2.coord[i];

	return res;
}

double operator^( const hPoint& arg1, const hPoint& arg2 )
{
	double res = 0.0;

	for( int i = 0; i < 4; i ++ )
		res = res + arg1.coord[i] * arg2.coord[i];

	return res;
}

void Point::Normalize( )
{
	double sum = sqrt( normal[0]*normal[0] + normal[1]*normal[1] + normal[2]*normal[2] );

	normal[0] /= sum;
	normal[1] /= sum;
	normal[2] /= sum;

	return;
}

hPoint hPoint::PointTransform( DualQuaternion Q )
{
	DualQuaternion QQ, QR;
	hPoint hp = *this;

	for( int i = 0; i < 4; i ++ )
	{
		QQ.dual[i].SetupDual( Q.dual[i].GetReal(), 0 );
		QR.dual[i].SetupDual( Q.dual[i].GetDual(), 0 );
	}

	DualQuaternion qq = QQ*hp*QQ.Conjugate( )+QR*QQ.Conjugate()-QQ*QR.Conjugate();

	hPoint hpp(qq.dual[0].GetReal(), qq.dual[1].GetReal(), qq.dual[2].GetReal(), qq.dual[3].GetReal() );
	hpp.PointNormalize( );

	return hpp;
}

Plane hPoint::PlaneTransform( DualQuaternion Q )
{
	DualQuaternion QQ, QR;
	Plane hp = *this;

	for( int i = 0; i < 4; i ++ )
	{
		QQ.dual[i].SetupDual( Q.dual[i].GetReal(), 0 );
		QR.dual[i].SetupDual( Q.dual[i].GetDual(), 0 );
	}

	Vector M( hp.coord[0], hp.coord[1], hp.coord[2], 0 );

	DualQuaternion qq = QQ*hp*QQ.Conjugate( )+QR*M*QQ.Conjugate()-QQ*M*QR.Conjugate();

	hPoint hpp(qq.dual[0].GetReal(), qq.dual[1].GetReal(), qq.dual[2].GetReal(), qq.dual[3].GetReal() );
	hpp.PlaneNormalize( );

	return hpp;
}

void hPoint::PlaneNormalize( )
{
	double temp = 0;

	for( int i = 0; i < 3; i ++ )
		temp += coord[i]*coord[i];

	temp = sqrt( temp );

	for( int i = 0; i <= 3; i ++ )
		coord[i] /= temp;

	return;
}

ifstream &operator>>(ifstream &input, hPoint & arg)
{
	for (int i=0; i<4; i++)
		input>>arg.coord[i];

	return input;

}
ofstream &operator<<(ofstream &output, const hPoint & arg)		   
{
	output << "(" << arg.coord[0] << ", " << arg.coord[1] << ", " << arg.coord[2] << ", " << arg.coord[3] << ")" << endl; 
	return output;
}
			
istream &operator>>(istream &input, hPoint & arg)
{
	for (int i=0; i<4; i++)
		input>>arg.coord[i];

	return input;

}
ostream &operator<<(ostream &output, const hPoint & arg)		   
{
	output << "(" << arg.coord[0] << ", " << arg.coord[1] << ", " << arg.coord[2] << ", " << arg.coord[3] << ")"; 
	return output;
}