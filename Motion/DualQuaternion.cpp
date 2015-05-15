#define _CRT_SECURE_NO_DEPRECATE
#include <cmath>
#include <iostream>
#include <fstream>

#include "Quaternion.h"
#include "DualQuaternion.h"
using namespace std;

DualQuaternion::DualQuaternion(Quaternion& real, Vector& translation)
{
	Quaternion d(translation.coord);
	Quaternion R;
	R =  0.5 * (d * real);
	for( int i = 0; i < 4; i++ )
		this->dual[i].SetupDual(real.q[i], R.q[i]);

}



DualQuaternion::DualQuaternion(const Quaternion& real, const Quaternion& dual)
{
	for( int i = 0; i < 4; i++ )
		this->dual[i].SetupDual(real.q[i], dual.q[i]);

}

DualQuaternion::DualQuaternion()
{
	for( int i = 0; i < 4; i ++ )
		dual[i].SetupDual( 0.0f, 0.0f );
}

DualQuaternion::DualQuaternion( double re[4], double du[4] )
{
	for( int i = 0; i < 4; i ++ )
		dual[i].SetupDual( re[i], du[i] );
}

Dual operator^( DualQuaternion& arg1, DualQuaternion& arg2 )
{
	Dual res;

	for( int i = 0; i < 4; i ++ )
		res = res + arg1.dual[i] * arg2.dual[i];

	return res;
}

DualQuaternion operator+( DualQuaternion& arg1, DualQuaternion& arg2 )
{
	DualQuaternion sum;

	for( int i = 0; i < 4; i ++ )   
		sum.dual[i] = arg1.dual[i] + arg2.dual[i];	

	return sum;
}

DualQuaternion operator+( DualQuaternion& arg1, double arg2 )
{
	DualQuaternion res;

	for( int i = 0; i < 3; i ++ )	
		res.dual[i].SetupDual( arg1.dual[i].GetReal(), arg1.dual[i].GetDual() );

	res.dual[3].SetupDual( arg1.dual[3].GetReal()+arg2, arg1.dual[3].GetDual() );

	return res;
}

DualQuaternion operator+( double arg1, DualQuaternion& arg2 )
{
	DualQuaternion res;

	for( int i = 0; i < 3; i ++ )	
		res.dual[i].SetupDual( arg2.dual[i].GetReal(), arg2.dual[i].GetDual() );

	res.dual[3].SetupDual( arg2.dual[3].GetReal()+arg1, arg2.dual[3].GetDual() );

	return res;
}

DualQuaternion operator-( DualQuaternion& arg1, DualQuaternion& arg2 )
{
	DualQuaternion sum;

	for( int i = 0; i < 4; i ++ )   
		sum.dual[i] = arg1.dual[i] - arg2.dual[i];
	
	return sum;
}

DualQuaternion operator-( DualQuaternion& arg1, double arg2 )
{
	DualQuaternion res;

	for( int i = 0; i < 3; i ++ )	
		res.dual[i].SetupDual( arg1.dual[i].GetReal(), arg1.dual[i].GetDual() );

	res.dual[3].SetupDual( arg1.dual[3].GetReal()-arg2, arg1.dual[3].GetDual() );

	return res;
}

DualQuaternion operator-( double arg1, DualQuaternion& arg2 )
{
	DualQuaternion res;

	for( int i = 0; i < 3; i ++ )	
		res.dual[i].SetupDual( arg2.dual[i].GetReal(), arg2.dual[i].GetDual() );

	res.dual[3].SetupDual( arg2.dual[3].GetReal()-arg1, arg2.dual[3].GetDual() );

	return res;
}

DualQuaternion operator*( DualQuaternion& arg1, DualQuaternion& arg2 )
{
	DualQuaternion mul;

	mul.dual[0] = arg1.dual[3]*arg2.dual[0] + arg1.dual[0]*arg2.dual[3]
		+ arg1.dual[1]*arg2.dual[2] - arg1.dual[2]*arg2.dual[1];

	mul.dual[1] = arg1.dual[3]*arg2.dual[1] + arg1.dual[1]*arg2.dual[3]
		+ arg1.dual[2]*arg2.dual[0] - arg1.dual[0]*arg2.dual[2];

	mul.dual[2] = arg1.dual[3]*arg2.dual[2] + arg1.dual[2]*arg2.dual[3]
		+ arg1.dual[0]*arg2.dual[1] - arg1.dual[1]*arg2.dual[0];

	mul.dual[3] = arg1.dual[3]*arg2.dual[3] - arg1.dual[0]*arg2.dual[0]
		- arg1.dual[1]*arg2.dual[1] - arg1.dual[2]*arg2.dual[2];
	
	return mul;
} 

DualQuaternion& DualQuaternion::operator=( DualQuaternion arg1 )
{
	for( int i=0; i < 4; i ++ )
		dual[i].SetupDual( arg1.dual[i].GetReal(), arg1.dual[i].GetDual() );

	return *this;
}

Dual DualQuaternion::Length()
{
	Dual sum, res;
	double r, d;
	
	for( int i = 0; i < 4; i ++ )
		sum = sum + dual[i] * dual[i];	
	
	r = sqrt( sum.GetReal() );
	d = sum.GetDual() / r / 2.0f;

	res.SetupDual( r, d );

	return res;
}

DualQuaternion DualQuaternion::Conjugate( )
{
	DualQuaternion conju;
		
	for( int i = 0; i < 3 ; i ++ )
		conju.dual[i].SetupDual( -dual[i].GetReal(), -dual[i].GetDual() );
	conju.dual[3].SetupDual( dual[3].GetReal(), dual[3].GetDual() );

	return conju;
}

DualQuaternion DualQuaternion::Inverse( )
{
	return Conjugate( )/Length()/Length();	
}

Quaternion DualQuaternion::GetReal() const
{
	Quaternion real(dual[0].GetReal(), dual[1].GetReal(), dual[2].GetReal(), dual[3].GetReal());
	
	return real;
}

Quaternion DualQuaternion::GetDual() const
{
	Quaternion dual(dual[0].GetDual(), dual[1].GetDual(), dual[2].GetDual(), dual[3].GetDual());

	return dual;
}

void DualQuaternion::Print( )
{
	FILE* fp = fopen("Quat.dat", "a+");
	
	for (int i =0; i<4; i++)
	{
		fprintf(fp,"%lf  ", dual[i].GetReal() );
		fprintf(fp, " dual:%lf\n", dual[i].GetDual() );
	}
	
	fprintf(fp, "\n");
}


void DualQuaternion::Clear()
{
	for( int i = 0; i < 4; i ++ )
		dual[i].SetupDual( 0.0f, 0.0f );

	return;
}

DualQuaternion operator*( DualQuaternion& arg1, double arg2 )
{
	DualQuaternion mul;
	
	for( int i = 0; i < 4; i ++ )
		mul.dual[i].SetupDual( arg1.dual[i].GetReal()*arg2, arg1.dual[i].GetDual()*arg2 );
	
	return mul;
}

DualQuaternion operator*( double arg1, DualQuaternion& arg2)
{
	DualQuaternion mul;
	
	for( int i = 0; i < 4; i ++ )
		mul.dual[i].SetupDual( arg2.dual[i].GetReal()*arg1, arg2.dual[i].GetDual()*arg1 );
	
	return mul;
}

DualQuaternion operator*( Dual& arg1, DualQuaternion& arg2 )
{
	DualQuaternion mul;
	
	for( int i = 0; i < 4; i ++ )
		mul.dual[i].SetupDual( arg2.dual[i] * arg1 );
	
	return mul;
}

DualQuaternion operator*( DualQuaternion& arg1, Dual& arg2 )
{
	DualQuaternion mul;
	
	for( int i = 0; i < 4; i ++ )
		mul.dual[i].SetupDual( arg1.dual[i] * arg2 );
	
	return mul;
}

DualQuaternion operator/(DualQuaternion& arg1, double arg2 )
{
	DualQuaternion div;

	for( int i = 0; i < 4; i ++ )
		div.dual[i].SetupDual( arg1.dual[i].GetReal()/arg2, arg1.dual[i].GetDual()/arg2 );
	
	return div;
}		

DualQuaternion operator/( DualQuaternion& arg1, DualQuaternion& arg2 )
{
	double deno = 0.0f;
	DualQuaternion res;

	for( int i = 0; i < 4; i ++ )
		deno += ( arg2.dual[i]*arg2.dual[i] ).GetReal();

	res.dual[0] = ( arg1.dual[0]*arg2.dual[3] - arg1.dual[3]*arg2.dual[0]
		+ arg1.dual[2]*arg2.dual[1] - arg1.dual[1]*arg2.dual[2] ) / deno;
	res.dual[1] = ( arg1.dual[1]*arg2.dual[3] - arg1.dual[3]*arg2.dual[1]
		+ arg1.dual[0]*arg2.dual[2] - arg1.dual[2]*arg2.dual[0] ) / deno;
	res.dual[2] = ( arg1.dual[2]*arg2.dual[3] - arg1.dual[3]*arg2.dual[2]
		+ arg1.dual[1]*arg2.dual[0] - arg1.dual[0]*arg2.dual[1] ) / deno;
	res.dual[3] = ( arg1.dual[0]*arg2.dual[0] + arg1.dual[1]*arg2.dual[1]
		+ arg1.dual[2]*arg2.dual[2] + arg1.dual[3]*arg2.dual[3] ) / deno;

	return res;
}

DualQuaternion operator/( double arg1, DualQuaternion& arg2 )
{
	DualQuaternion q;

	for( int i = 0; i < 3; i ++ )	q.dual[i].SetupDual( 0.0f, 0.0f );
	q.dual[3].SetupDual( arg1, 0.0f );

	return q/arg2;
}

DualQuaternion operator/( DualQuaternion& arg1, Dual& arg2 )
{	
	return arg1 * ( 1.0f / arg2 );
}

DualQuaternion operator*( DualQuaternion& arg1, hPoint& arg2 )
{
	DualQuaternion mul;

	mul.dual[0] = arg1.dual[3]*arg2.coord[0] + arg1.dual[0]*arg2.coord[3]
		+ arg1.dual[1]*arg2.coord[2] - arg1.dual[2]*arg2.coord[1];

	mul.dual[1] = arg1.dual[3]*arg2.coord[1] + arg1.dual[1]*arg2.coord[3]
		+ arg1.dual[2]*arg2.coord[0] - arg1.dual[0]*arg2.coord[2];

	mul.dual[2] = arg1.dual[3]*arg2.coord[2] + arg1.dual[2]*arg2.coord[3]
		+ arg1.dual[0]*arg2.coord[1] - arg1.dual[1]*arg2.coord[0];

	mul.dual[3] = arg1.dual[3]*arg2.coord[3] - arg1.dual[0]*arg2.coord[0]
		- arg1.dual[1]*arg2.coord[1] - arg1.dual[2]*arg2.coord[2];
	
	return mul;
}

DualQuaternion operator*( hPoint& arg1, DualQuaternion& arg2 )
{	
	DualQuaternion mul;

	mul.dual[0] = arg1.coord[3]*arg2.dual[0] + arg1.coord[0]*arg2.dual[3]
		+ arg1.coord[1]*arg2.dual[2] - arg1.coord[2]*arg2.dual[1];

	mul.dual[1] = arg1.coord[3]*arg2.dual[1] + arg1.coord[1]*arg2.dual[3]
		+ arg1.coord[2]*arg2.dual[0] - arg1.coord[0]*arg2.dual[2];

	mul.dual[2] = arg1.coord[3]*arg2.dual[2] + arg1.coord[2]*arg2.dual[3]
		+ arg1.coord[0]*arg2.dual[1] - arg1.coord[1]*arg2.dual[0];

	mul.dual[3] = arg1.coord[3]*arg2.dual[3] - arg1.coord[0]*arg2.dual[0]
		- arg1.coord[1]*arg2.dual[1] - arg1.coord[2]*arg2.dual[2];

	return mul;
}

ifstream &operator>>(ifstream &input, DualQuaternion & arg)		   
{
	input>>arg.GetReal()>>arg.GetDual();
	
	return input;
}

istream &operator>>(istream &input, DualQuaternion & arg)		   
{
	input>>arg.GetReal()>>arg.GetDual();
	
	return input;
}

ofstream &operator<<(ofstream &output, const DualQuaternion & arg)		   
{
	output<<arg.GetReal()<<" "<<arg.GetDual();
	
	return output;
}

ostream &operator<<(ostream &output, const DualQuaternion & arg)		   
{
	output<<arg.GetReal()<<" "<<arg.GetDual();
	
	return output;
}


hMatrix DualQuaternion::dualQuaternionToHomogeneousMatrix(void)
{
	Quaternion Q = GetReal();
	//cout<<"Q="<<Q<<endl;
	Quaternion R = GetDual();
	//cout<<"R="<<R<<endl;
	Quaternion T = (R*Q.Conjugate() - Q*R.Conjugate())/Q.Modulus(); // recover translation component
	//cout<<"T="<<T<<endl;

	Matrix RotMatrix = Q.quaternionToRotationMatrix();
	hMatrix H;
	for (int i=0; i<3; i++)
		for (int j=0; j<3; j++)
			H.m[i][j]=RotMatrix.m[i][j];
	for (int i=0; i<3; i++)
		H.m[i][3]=T.q[i];
	for (int j=0; j<3; j++)
		H.m[3][j]=0.0;
	H.m[3][3]=1;
	//cout<<"H="<<H<<endl;
	
	return H;
	
}