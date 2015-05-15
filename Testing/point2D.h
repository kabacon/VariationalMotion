// point2D.h
// point2D class definition

#ifndef POINT2D_H
#define POINT2D_H

class Point2D
{
public:
	Point2D(double=0.0, double=0.0);
	double getPx(void);
	double getPy(void);
	void setPx(double);
	void setPy(double);

private:
	double x, y;
};

#endif