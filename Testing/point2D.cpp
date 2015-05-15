// point2D.cpp
// point2D member-function definition

#include "Point2D.h"

// Point2D constructor
Point2D::Point2D( double xx, double yy )
{
	x = xx;
	y = yy;
}

double Point2D::getPx()
{
	return x;
}

double Point2D::getPy()
{
	return y;
}

void Point2D::setPx( double xValue )
{
	x = xValue;
}

void Point2D::setPy( double yValue )
{
	y = yValue;
}
