#include "dualNumber.h"

void Dual::SetupDual( double arg1, double arg2 )
{
	real = arg1;	dual = arg2;

	return;
}

void Dual::SetupDual( Dual& arg1 )
{
	real  = arg1.GetReal();
	dual = arg1.GetDual();

	return;
}

Dual operator+( const Dual& arg1, const Dual& arg2 )
{
	Dual sum;
	
	sum.real = arg1.real + arg2.real;
	sum.dual = arg1.dual + arg2.dual;

	return sum;
}

Dual operator+( const Dual& arg1, double arg2 )
{
	Dual sum;
	
	sum.real = arg1.real + arg2;
	sum.dual = arg1.dual;

	return sum;
}

Dual operator+( double arg1, const Dual& arg2 )
{
	Dual sum;
	
	sum.real = arg1 + arg2.real;
	sum.dual = arg2.dual;

	return sum;
}

Dual operator-( const Dual& arg1, const Dual& arg2 )
{
	Dual sum;
	
	sum.real = arg1.real - arg2.real;
	sum.dual = arg1.dual - arg2.dual;

	return sum;
}

Dual operator-( const Dual& arg1, double arg2 )
{
	Dual sum;
	
	sum.real = arg1.real - arg2;
	sum.dual = arg1.dual;

	return sum;
}

Dual operator-( double arg1, const Dual& arg2 )
{
	Dual sum;
	
	sum.real = arg1 - arg2.real;
	sum.dual = 0.0f - arg2.dual;

	return sum;
}

Dual operator*( const Dual& arg1, double arg2 )
{
	Dual sum;

	sum.real = arg1.real * arg2;
	sum.dual = arg1.dual * arg2;

	return sum;
}

Dual operator*( double arg1, const Dual& arg2 )
{
	Dual sum;

	sum.real = arg1 * arg2.real;
	sum.dual = arg1 * arg2.dual;

	return sum;
}

Dual operator*( const Dual& arg1, const Dual& arg2 )
{
	Dual sum;

	sum.real = arg1.real * arg2.real;
	sum.dual = arg1.real*arg2.dual + arg2.real*arg1.dual;

	return sum;
}

Dual operator/( const Dual& arg1, const Dual& arg2 )
{
	Dual sum;

	sum.real = arg1.real/arg2.real;
	sum.dual = ( arg2.real*arg1.dual - arg1.real*arg2.dual ) / ( arg2.real*arg2.real );

	return sum;
}

Dual operator/( const Dual& arg1, double arg2 )
{
	Dual sum;

	sum.real = arg1.real / arg2;
	sum.dual = arg1.dual / arg2;

	return sum;
}

Dual operator/( double arg1, const Dual& arg2 )
{
	Dual sum;

	sum.real = arg1 / arg2.real;
	sum.dual = -arg1 * arg2.dual / arg2.real / arg2.real;

	return sum;
}

Dual& Dual::operator=( Dual arg1 )
{
	real = arg1.real;
	dual = arg1.dual;

	return *this;
}
