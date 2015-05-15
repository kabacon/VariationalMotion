#ifndef QUATERNION_H
#define QUATERNION_H

#include <iostream>
#include <fstream>

#include "hMatrix.h"


class Quaternion
{
public:
	double q[4];

public:
	Quaternion(const double re[4]);
	Quaternion(double r1, double r2, double r3, double r4);
	Quaternion();

	friend ifstream &operator>>(ifstream &, Quaternion &);
	friend istream &operator>>(istream &, Quaternion &);
	friend ofstream &operator<<(ofstream &, const Quaternion &);
	friend ostream &operator<<(ostream &, const Quaternion &);

	friend Quaternion operator+(Quaternion&, Quaternion&);
	friend Quaternion operator-(Quaternion&, Quaternion&);
	friend Quaternion operator*(Quaternion&, Quaternion&);
	friend Quaternion operator*(Quaternion&, double);
	friend Quaternion operator*(double, Quaternion&);
	friend Quaternion operator/(Quaternion&, double);
	Quaternion& operator=(Quaternion);

	Matrix quaternionToRotationMatrix();
	double Modulus();
	Quaternion Conjugate();
	Quaternion Inverse();

};

#endif