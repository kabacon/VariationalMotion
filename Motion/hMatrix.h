// hmatrix.h defines all operations of a homogeneous matrix

#ifndef HMATRIX_H
#define HMATRIX_H

#include <iostream>
using namespace std;

class hPoint;
class Point;

// 4*4 homogeneous matrix
class hMatrix
{
public:
	double m[4][4];

public:
	hMatrix();
	friend hMatrix operator+( const hMatrix&, const hMatrix& );
	friend hMatrix operator-( const hMatrix&, const hMatrix& );
	friend hMatrix operator*( const hMatrix&, const hMatrix& );
	friend hMatrix operator*( const hMatrix&, const double );
	friend hMatrix operator*( const double, const hMatrix& );
	friend hMatrix operator/( const hMatrix&, const double );
	friend hPoint operator*( const hMatrix&, const hPoint& ); 
	friend Point  operator*( const hMatrix&, const Point& );
	friend ostream &operator<<(ostream&, hMatrix&);
	hMatrix& operator=( hMatrix ); 
	void   Clear();
	hMatrix Inverse( );		// return the inverse of a 4*4 matrix
	hMatrix transpose();
};

// any arbitrary matrix
class Matrix
{
public:
	int row;
	int column;
	double **m;

public:
	Matrix();
	Matrix( int, int );
	Matrix( const Matrix& );
	~Matrix( );
	friend Matrix operator+( const Matrix&, const Matrix& );
	friend Matrix operator-( const Matrix&, const Matrix& );
	friend Matrix operator*( const Matrix&, const Matrix& );
	friend Matrix operator*( const Matrix&, const double );
	friend Matrix operator*( const double, const Matrix& );
	friend Matrix operator/( const Matrix&, const double );
	void   operator=( const Matrix& );
	void   Clear();
	Matrix Inverse(  );
	Matrix Transpose( );

	bool   MallocSpace( );
	void   DeleteSpace( );
};


#endif