#ifndef DUALQUATERNION_H
#define DUALQUATERNION_H


#include "dualNumber.h"
#include "hPoint.h"
#include "Quaternion.h"

class DualQuaternion		//define a quaternion
{
public:
	Dual dual[4];

public:
	DualQuaternion(Quaternion& ,  Vector&);
	DualQuaternion( double re[4], double du[4] );
	DualQuaternion(const Quaternion&, const Quaternion&);
	DualQuaternion();


	friend istream &operator>>(istream &, DualQuaternion &);
	friend ifstream &operator>>(ifstream &, DualQuaternion &);
	friend ostream &operator<<(ostream &, const DualQuaternion &);
	friend ofstream &operator<<(ofstream &, const DualQuaternion &);

	friend DualQuaternion operator+( DualQuaternion&, DualQuaternion& );
	friend DualQuaternion operator+( DualQuaternion&, double );
	friend DualQuaternion operator+( double, DualQuaternion& );
	friend DualQuaternion operator-( DualQuaternion&, DualQuaternion& );
	friend DualQuaternion operator-( DualQuaternion&, double );
	friend DualQuaternion operator-( double, DualQuaternion& );
	friend DualQuaternion operator*( DualQuaternion&, DualQuaternion& );
	friend DualQuaternion operator*( DualQuaternion&, hPoint& );
	friend DualQuaternion operator*( hPoint&, DualQuaternion& );
	friend DualQuaternion operator*( DualQuaternion&, double );
	friend DualQuaternion operator*( double, DualQuaternion& );
	friend DualQuaternion operator*( DualQuaternion&, Dual& );
	friend DualQuaternion operator*( Dual&, DualQuaternion& );
	friend Dual		  operator^( DualQuaternion&, DualQuaternion& );	//dot product
	friend DualQuaternion operator/( DualQuaternion&, double );
	friend DualQuaternion operator/( const DualQuaternion&, const DualQuaternion& );
	friend DualQuaternion operator/( double, DualQuaternion& );
	friend DualQuaternion operator/( DualQuaternion&, Dual& );
	DualQuaternion& operator=( DualQuaternion );

	void   Clear();		//make all the member of dual 0

	Dual	   Length( );		// calculate the length of a dual quaternion
	DualQuaternion Inverse( );		//calculate the inverse of a quaternion
	DualQuaternion Conjugate( );	//calculate the conjuagate of a quaternion
	Quaternion GetReal( ) const;		//create a quaternion from the real part of this quaternion
	Quaternion GetDual( ) const;		//create a quaternion from the dual part of this quaternion
	hMatrix dualQuaternionToHomogeneousMatrix(void);
	void Print();
};

#endif