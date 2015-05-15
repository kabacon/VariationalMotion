/*
	Point2D.h - Defines a two dimensional point.
*/

#pragma once

class Point2D {
private:
	double x, y;

public:
	Point2D() : x(0.0f), y(0.0f) {}
	Point2D(double x, double y);
	double getX();
	double getY();
	void setX(double x);
	void setY(double y);

};