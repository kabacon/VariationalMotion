// hpoint.h defines all operations for 3D point and homogeneous point

#ifndef HPOINT_H
#define HPOINT_H

#include <iostream>
#include <fstream>

class hMatrix;
class DualQuaternion;

using namespace std;

// 3D point
class Point
{
public:
	double	coord[3];
	double	normal[3];
	bool	visible;

public:
	Point() { for( int i = 0; i < 3; i ++ )	coord[i] = 0.0f; }
	Point( double x, double y, double z )	{ coord[0] = x; coord[1] = y; coord[2] = z; }

	// hMatrix is homogeneous matrix
	friend Point operator*( const hMatrix&, const Point& );
	friend Point operator*( const Point&, double );
	friend Point operator*( double, const Point& );
	friend Point operator/(	const Point&, double );
	friend Point operator+( const Point&, const Point& );
	friend Point operator-( const Point&, const Point& );
	Point&	operator=( Point );

	void Normalize( );
};

//homogeneous representation of a point or a plane
class hPoint
{
public:
	double coord[4];

public:
	hPoint() 
	{ 
		for( int i=0; i < 3; i ++ )  coord[i] = 0.0f; 
		coord[3] = 1.0f;
	}
	hPoint(double arg1, double arg2, double arg3 ) 
	{ 
		coord[0] = arg1; coord[1] = arg2; coord[2] =arg3; coord[3] = 1.0f; 
	}
	hPoint(double arg1, double arg2, double arg3, double arg4 ) 
	{ 
		coord[0] = arg1; coord[1] = arg2; coord[2] =arg3; coord[3] = arg4; 
	}
	hPoint(double arg1[3] )
	{
		coord[0] = arg1[0]; coord[1] = arg1[1]; coord[2] = arg1[2]; coord[3] = 1.0f;
	}

	friend ifstream &operator>>(ifstream &, hPoint &);
	friend istream &operator>>(istream &, hPoint &);
	friend ofstream &operator<<(ofstream &, const hPoint &);
	friend ostream &operator<<(ostream &, const hPoint &);

	friend hPoint operator*( const hMatrix&, const hPoint& );
	friend hPoint operator*( const hPoint&, const hPoint& );	//cross product
	friend hPoint operator+( const hPoint&, const hPoint& );
	friend hPoint operator-( const hPoint&, const hPoint& );
	friend hPoint operator*( const hPoint&, double );
	friend hPoint operator*( double, const hPoint& );
	friend hPoint operator/( const hPoint&, double );
	friend double operator^( const hPoint&, const hPoint& );
	hPoint& operator=( hPoint );

	void Clear();
	void PointNormalize();
	void PlaneNormalize();

	hPoint PointTransform( DualQuaternion Q );
	hPoint  PlaneTransform( DualQuaternion Q );
};

#define Vector hPoint
#define Plane  hPoint

#endif
