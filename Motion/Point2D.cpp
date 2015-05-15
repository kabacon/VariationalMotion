/*
	Point2D.cpp
*/

#include "Point2D.h"

/*
	Constructor
*/
Point2D::Point2D(double x, double y) {
	this->x = x;
	this->y = y;
}

/*
	Accessor methods
*/
double Point2D::getX()			{ return x;		}
double Point2D::getY()			{ return y;		}
void Point2D::setX(double x)	{ this->x = x;	}
void Point2D::setY(double y)	{ this->y = y;	}
