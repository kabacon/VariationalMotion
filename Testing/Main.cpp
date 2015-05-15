/*
	Matthew Fink, MEC 572, Spring 2015

	This code implements a few of the equations from section 2 of 
	the motion design paper.
*/

#include <iostream>
#include <armadillo>
#include <list>
#include <vector>
#include "DualQuaternion.h"

using std::list;
using std::vector;
using std::cout;
using std::endl;
using arma::mat;
using arma::vec;
using arma::cx_vec;
using arma::trans;
using arma::zeros;
using arma::endr;

list<cx_vec> calculateSpecialPoints(const vector<vec> points);
vec barycenter(const vector<vec> points);
double S(int x, int y, const vector<vec>& X, const vector<vec>& Y);
vector<vec> variationalSubdivision(const vector<vec> points, int iterations);
vector<vec> variationalSubdivision(const vector<vec> points, vector<double> weights, double qWeights, int iterations);
int ipow(int base, int exp);
mat* getAMatrix(int size);
mat* getBMatrix(int size);
mat* getLMatrix(int size, vec* lambdas);
mat* buildAaMatrix(mat* A, mat* B, mat* L, int size);

/*
	This is all from section 2, not fully implemented, but just trying to 
	translate the math into code.
*/
int main() {
	// Take input of a moving body at several time instances

	// Choose four affinely independent feature points from the model. 
	// These points are arbitrary for now, but should be chosen from a model later
	vector<vec> X;
	X.push_back("0.0 0.0 0.0");
	X.push_back("1.0 2.0 1.0");
	X.push_back("2.0 1.0 2.0");
	X.push_back("3.0 2.0 1.0");


	// Compute the barycenter of the feature points xBar
	vec xBar = barycenter(X);
	cout << "Barycenter of X:" << endl;
	xBar.print(); cout << endl;

	// Compute the intertia tensor J and the 6 special points xSpecial
	list<cx_vec> xSpecial = calculateSpecialPoints(X);

	// Compute the barycentric coordinates of xSpecial w.r.t. X
	cx_vec cxXBar(xBar, zeros<vec>(3));
	vector<cx_vec> xSpecialBary;
	for (cx_vec v : xSpecial) {
		xSpecialBary.push_back(v - cxXBar);
	}


	// Now get the four sets of homologous points and apply the variational 
	// subdivision algorithm to each set of points

	// Get the position of the feature points at each time instance
	vector<vector<vec>> positions;
	// .......

	// Some arbitrary positions for testings
	vector<vec> p1;
	vector<vec> p2;
	vector<vec> p3;
	vector<vec> p4;
	p1.push_back("0.0 0.0 0.0");
	p2.push_back("1.0 2.0 1.0");
	p3.push_back("2.0 1.0 2.0");
	p4.push_back("3.0 2.0 1.0");

	p1.push_back("1.0 0.0 0.0");
	p2.push_back("2.0 2.0 1.0");
	p3.push_back("3.0 1.0 2.0");
	p4.push_back("4.0 2.0 1.0");

	p1.push_back("0.0 1.0 0.0");
	p2.push_back("1.0 3.0 1.0");
	p3.push_back("2.0 2.0 2.0");
	p4.push_back("3.0 3.0 1.0");

	p1.push_back("0.0 0.0 1.0");
	p2.push_back("1.0 2.0 2.0");
	p3.push_back("2.0 1.0 3.0");
	p4.push_back("3.0 2.0 2.0");

	positions.push_back(p1);
	positions.push_back(p2);
	positions.push_back(p3);
	positions.push_back(p4);

	// Rearrange the positions into sets of homologous points
	vector<vector<vec>> homologousPoints;
	for (int i = 0; i < positions[0].size(); i++) {
		vector<vec> homologousSet;
		for (int j = 0; j < positions.size(); j++) {
			homologousSet.push_back(positions[j][i]);
		}
		homologousPoints.push_back(homologousSet);
	}

	// Employ the variational subdivision algorithm on each set of 
	// homologous points, giving the four feature curves.
	vector<vector<vec>> curves;
	for (vector<vec> homologousSet : homologousPoints) {
		// This is interpolary
		curves.push_back(variationalSubdivision(homologousSet, 5));
	}

	homologousPoints.clear();

	// Rearrange the curves into positions
	vector<vector<vec>> newPositions;
	for (int i = 0; i < curves[0].size(); i++) {
		vector<vec> position;
		for (int j = 0; j < curves.size(); j++) {
			position.push_back(curves[j][i]);
		}
		newPositions.push_back(position);
	}

	// Determine the affine map?? The affine images??



	// Create local coordinate systems relative to the barycenters
	// Eq. 4
	vector<vec> Xprime;
	for (int i = 0; i < X.size(); i++) {
		vec v = X[i] - xBar;
		Xprime.push_back(v);
	}



	// The point cloud of feature points at a different time instance
	vector<vec> Y;
	Y.push_back("0.3 0.5 0.1");
	Y.push_back("1.2 0.0 0.3");
	Y.push_back("0.2 1.0 0.1");
	Y.push_back("0.1 0.0 1.0");

	cout << "Barycenter of Y:" << endl;
	vec yBar = barycenter(Y);
	yBar.print(); cout << endl;

	vector<vec> Yprime;
	for (int i = 0; i < Y.size(); i++) {
		vec v = Y[i] - yBar;
		Yprime.push_back(v);
	}

	// Begin calculating the rotation matrix R
	double Sxx = S(0, 0, Xprime, Yprime);
	double Sxy = S(0, 1, Xprime, Yprime);
	double Sxz = S(0, 2, Xprime, Yprime);
	double Syx = S(1, 0, Xprime, Yprime);
	double Syy = S(1, 1, Xprime, Yprime);
	double Syz = S(1, 2, Xprime, Yprime);
	double Szx = S(2, 0, Xprime, Yprime);
	double Szy = S(2, 1, Xprime, Yprime);
	double Szz = S(2, 2, Xprime, Yprime);

	// M is this (symmetric) matrix, Eq. 5
	mat M = zeros(4, 4);
	M	<< Sxx + Syy + Szz << Syz - Szy << Szx - Sxz << Sxy - Syz << endr
		<< Syz - Szy << Sxx - Syy - Szz << Sxy + Syx << Szx + Sxz << endr
		<< Szx - Sxz << Sxy + Syx << -Sxx + Syy - Szz << Syz + Szy << endr
		<< Sxy - Syz << Szx + Sxz << Syz + Szy << -Sxx - Syy + Szz << endr;
	cout << "M:" << endl;
	M.print(); cout << endl;

	// Find the maximal eigenvalue and corresponding unit eigenvector
	vec eigVals;
	mat eigVecs;
	eig_sym(eigVals, eigVecs, M);

	vec a = eigVecs.col(3);
	cout << "Rotation Unit Quaternion: " << endl;
	a.print(); cout << endl;

	// Create the rotation matrix, Eq. 7
	mat R = zeros(3, 3);
	R	<< a(0)*a(0) + a(1)*a(1) - a(2)*a(2) - a(3)*a(3) << 2*(a(1)*a(2) + a(0)*a(3)) << 2*(a(1)*a(3) - a(0)*a(2)) << endr
		<< 2*(a(1)*a(2) - a(0)*a(3)) << a(0)*a(0) - a(1)*a(1) + a(2)*a(2) - a(3)*a(3) << 2*(a(2)*a(3) + a(0)*a(1)) << endr
		<< 2*(a(1)*a(3) + a(0)*a(2)) << 2*(a(2)*a(3) - a(0)*a(1)) << a(0)*a(0) - a(1)*a(1) - a(2)*a(2) - a(3)*a(3) << endr;
	cout << "Rotation matrix:" << endl;
	R.print(); cout << endl;

	// Get the translation vector, Eq. 8
	vec t = yBar - R * xBar;
	cout << "Translation vector:" << endl;
	t.print(); cout << endl;

	// Turn this into a Dual Quaternion. The dual quaternion represents 
	// a motion from the first point cloud to the second point cloud.
	double r[4] = { a(0), a(1), a(2), a(3) };
	Quaternion real(r);
	double tr[3] = { t(0), t(1), t(2) };
	hPoint trans(tr);
	DualQuaternion dq(real, trans);


	// Calculate the "Special points" given in eq. 10
	list<cx_vec> sp = calculateSpecialPoints(X);


	vector<vec> points;
	points.push_back("0.0 0.0 0.0");
	points.push_back("1.0 2.0 1.0");
	points.push_back("2.0 1.0 2.0");
	points.push_back("3.0 2.0 1.0");
	vector<double> weights;
	weights.push_back(10);
	weights.push_back(10);
	weights.push_back(10);
	weights.push_back(10);

	vector<vec> newPoints;
	//newPoints = variationalSubdivision(points, weights, 1.0, 5);
	newPoints = variationalSubdivision(points, 5);

	// Cin to keep the command line open
	int d;
	std::cin >> d;
}

double S(int x, int y, const vector<vec>& X, const vector<vec>& Y) {
	double s = 0.0f;
	for (int i = 0; i < X.size(); i++) {
		s += (X[i](x) * Y[i](y));
	}
	return s;
}

/*
	Compute the barycenter of a point cloud.
*/
vec barycenter(const vector<vec> points) {
	if (points.empty()) return vec();
	vec xBar = zeros<vec>(points.front().size());
	for (vec v : points) {
		xBar += v;
	}
	return xBar / points.size();
}

/*
	Calculate the "six special points"
*/
list<cx_vec> calculateSpecialPoints(const vector<vec> points) {
	// Make a copy of the points
	vector<vec> pointsCopy(points);

	// Find the barycenter of the points;
	vec xBar = barycenter(pointsCopy);

	cx_vec cxXBar(xBar, zeros<vec>(3));

	// Calculate the inertia tensor for any number of points
	// Eq. 9
	mat J = zeros<mat>(3, 3);
	for (vec v : pointsCopy) {
		J += (v * trans(v));
	}
	pointsCopy.clear();
	//cout << "Inertia Tensor:" << endl;
	//J.print(); cout << endl;
	
	// Find the eigenvalues and eigenvectors of the sym. matrix
	vec eigVals;
	mat eigVecs;
	eig_sym(eigVals, eigVecs, J);

	cx_vec cEigVals(eigVals, zeros<vec>(3));

	/*cout << "Eigenvalues:" << endl;
	eigVals.print();
	cout << endl << "Eigenvectors:" << endl;
	eigVecs.print();
	cout << endl;*/

	// Separate the eigenvectors
	for (int i = 0; i < 3; i++) {
		vec v = eigVecs.col(i);
		pointsCopy.push_back(v);
	}

	// Change the points to complex points. Since Eq. 10 takes the square 
	// root of the eigenvector, if its coefficients are negative, it will 
	// produce complex numbers.
	list<cx_vec> newPoints;
	for (vec v : pointsCopy) {
		cx_vec c(v, zeros<vec>(3));
		newPoints.push_back(c);
	}

	// Calcuate the 6 points used for registration
	for (int i = 0; i < 3; i++) {
		cx_vec x = cxXBar + arma::sqrt(newPoints.front() / 2.0f) * cEigVals[i];
		newPoints.push_back(x);
		x = cxXBar - arma::sqrt(newPoints.front() / 2.0f) * cEigVals[i];
		newPoints.push_back(x);
		newPoints.pop_front();
	}

	/*cout << "Special points:" << endl;
	for (cx_vec v : newPoints) {
		v.print();
		cout << endl;
	}*/
	return newPoints;

}

/*
	Generate a curve using Variational Subdivision, interpolating the list of points. For 
	a list of N points and m iterations, the curve will contain (N - 1) * 2^m + 1 points.
*/
vector<vec> variationalSubdivision(const vector<vec> points, int iterations) {
	
	// Separate the x, y, z coordinates
	vec* x = new vec(points.size());
	vec* y = new vec(points.size());
	vec* z = new vec(points.size());
	for (int i = 0; i < points.size(); i++) {
		(*x)(i) = points[i](0);
		(*y)(i) = points[i](1);
		(*z)(i) = points[i](2);
	}

	// Set up the A, B, L, and Aa matrices
	mat* A = nullptr;
	mat* B = nullptr;

	// Insert points for the given number of iterations
	for (int iter = 0; iter < iterations; iter++) {

		// Size here is actually N - 1, which is the size of the A matrix.
		int size = x->size() - 1;

		// Construct A matrix
		A = getAMatrix(size);

		// Construct B matrix
		B = getBMatrix(size);

		// Calculate the new points
		vec qX = arma::solve(*A, (*B)*(*x));
		vec qY = arma::solve(*A, (*B)*(*y));
		vec qZ = arma::solve(*A, (*B)*(*z));

		delete A;
		delete B;

		// Rearrange the points
		vec* oldX = x;
		vec* oldY = y;
		vec* oldZ = z;
		x = new vec(2 * size + 1);
		y = new vec(2 * size + 1);
		z = new vec(2 * size + 1);
		for (int i = 0; i < size + 1; i++) {
			(*x)(2 * i) = (*oldX)(i);
			if (i != size) (*x)(2 * i + 1) = qX(i);
			(*y)(2 * i) = (*oldY)(i);
			if (i != size) (*y)(2 * i + 1) = qY(i);
			(*z)(2 * i) = (*oldZ)(i);
			if (i != size) (*z)(2 * i + 1) = qZ(i);
		}
		delete oldX;
		delete oldY;
		delete oldZ;
	}

	// Convert the x, y, z coordinates to points and return them
	vector<vec> curve;
	for (int i = 0; i < x->size(); i++) {
		vec v(3);
		v(0) = (*x)(i);
		v(1) = (*y)(i);
		v(2) = (*z)(i);
		curve.push_back(v);
	}
	delete x;
	delete y;
	delete z;

	return curve;
}

/*
	Generate a curve using Variational Subdivision, approximating the list of points with the
	given weights. Inserted points will receive a weight corresponding to qWeights. Higher 
	weights will more closely approximate the given points. For a list of N points and m 
	iterations, the curve will contain (N - 1) * 2^m + 1 points.
*/
vector<vec> variationalSubdivision(const vector<vec> points, const vector<double> weights, double qWeights, int iterations) {
	if (points.size() != weights.size()) {
		cout << "Number of points and weights do not match!" << endl;
		return points;
	}

	// Separate the x, y, z coordinates
	vec* x = new vec(points.size());
	vec* y = new vec(points.size());
	vec* z = new vec(points.size());
	for (int i = 0; i < points.size(); i++) {
		(*x)(i) = points[i](0);
		(*y)(i) = points[i](1);
		(*z)(i) = points[i](2);
	}

	// Make a copy of the weights
	vec* lambdas = new vec(weights);

	// Set up the A, B, L, and Aa matrices
	mat* A = nullptr;
	mat* B = nullptr;
	mat* L = nullptr;
	mat* Aa = nullptr;

	// Insert points for the given number of iterations
	for (int iter = 0; iter < iterations; iter++) {

		// Size here is actually N - 1, which is the size of the A matrix.
		int size = x->size() - 1;

		// Construct A matrix
		A = getAMatrix(size);

		// Construct B matrix
		B = getBMatrix(size);
		(*B) = -(*B);

		// Construct L matrix
		L = getLMatrix(size, lambdas);

		// Construct the large Aa matrix
		Aa = buildAaMatrix(A, B, L, size);

		// Assemble b for x, y, z
		vec bX = zeros<vec>(2 * size + 1);
		vec bY = zeros<vec>(2 * size + 1);
		vec bZ = zeros<vec>(2 * size + 1);
		for (int i = 0; i < size + 1; i++) {
			bX(size + i) = (*x)(i) * (*lambdas)(i);
			bY(size + i) = (*y)(i) * (*lambdas)(i);
			bZ(size + i) = (*z)(i) * (*lambdas)(i);
		}

		// Calculate the new points
		vec newX = arma::solve(*Aa, bX);
		vec newY = arma::solve(*Aa, bY);
		vec newZ = arma::solve(*Aa, bZ);
		
		delete Aa;

		// Rearrange the points
		delete x;
		delete y;
		delete z;
		x = new vec(2 * size + 1);
		y = new vec(2 * size + 1);
		z = new vec(2 * size + 1);
		for (int i = 0; i < size + 1; i++) {
			(*x)(2 * i) = newX(size + i);
			if (i != size) (*x)(2 * i + 1) = newX(i);
			(*y)(2 * i) = newY(size + i);
			if (i != size) (*y)(2 * i + 1) = newY(i);
			(*z)(2 * i) = newZ(size + i);
			if (i != size) (*z)(2 * i + 1) = newZ(i);
		}

		// Get the set of weights
		vec* newLambdas = new vec(2 * size + 1);
		newLambdas->ones();
		*(newLambdas) *= qWeights;
		for (int i = 0; i < points.size(); i++) {
			(*newLambdas)(i * ipow(2, iter + 1)) = (*lambdas)[i * ipow(2, iter)];
		}
		delete lambdas;
		lambdas = newLambdas;

	}

	// Convert the x, y, z coordinates to points and return them
	vector<vec> curve;
	for (int i = 0; i < x->size(); i++) {
		vec v(3);
		v(0) = (*x)(i);
		v(1) = (*y)(i);
		v(2) = (*z)(i);
		curve.push_back(v);
	}
	delete x;
	delete y;
	delete z;
	delete lambdas;

	return curve;
}

/*
	Helper method to construct the A matrix for variational subdivision
*/
mat* getAMatrix(int size) {
	mat* A = new mat(size, size);
	A->zeros();
	(*A)(0, 0) = 5;
	(*A)(0, 1) = 1;
	(*A)(size - 1, size - 2) = 1;
	(*A)(size - 1, size - 1) = 5;
	for (int row = 1; row < size - 1; row++) {
		(*A)(row, row - 1) = 1;
		(*A)(row, row) = 6;
		(*A)(row, row + 1) = 1;
	}
	return A;
}

/*
	Helper method to construct the B matrix for variational subdivision
*/
mat* getBMatrix(int size) {
	mat* B =new mat(size, size + 1);
	B->zeros();
	(*B)(0, 0) = 2;
	(*B)(0, 1) = 4;
	(*B)(size - 1, size - 1) = 4;
	(*B)(size - 1, size) = 2;
	for (int row = 1; row < size - 1; row++) {
		(*B)(row, row) = 4;
		(*B)(row, row + 1) = 4;
	}
	return B;
}

/*
	Helper method to construct the L matrix for approximation variational subdivision
*/
mat* getLMatrix(int size, vec* lambdas) {
	mat* L = new mat(size + 1, size + 1);
	L->zeros();
	(*L)(0, 0) = 1 + (*lambdas)(0);
	(*L)(0, 1) = 1;
	(*L)(size, size - 1) = 1;
	(*L)(size, size) = 1 + (*lambdas)(size);
	for (int row = 1; row < size; row++) {
		(*L)(row, row - 1) = 1;
		(*L)(row, row) = 6 + (*lambdas)(row);
		(*L)(row, row + 1) = 1;
	}
	return L;
}

/*
	Helper method to build the Aa matrix for approximation variational subdivision
*/
mat* buildAaMatrix(mat* A, mat* B, mat* L, int size) {
	mat* Aa = new mat(2 * size + 1, 2 * size + 1);
	Aa->zeros();
	for (int r = 0; r < 2 * size + 1; r++) {
		for (int c = 0; c < 2 * size + 1; c++) {
			if (r < size) {		// Top half of the matrix
				if (c < size) {	// Top left is the A matrix
					(*Aa)(r, c) = (*A)(r, c);
				} else {				// Top right is the -B matrix
					(*Aa)(r, c) = (*B)(r, c - size);
				}
			} else {					// Bottom half of the matrix
				if (c < size) {	// Bottom left is -B.t matrix
					(*Aa)(r, c) = (*B)(c, r - size);
				} else {				// Bottom right is L matrix
					(*Aa)(r, c) = (*L)(r - size, c - size);
				}
			}
		}
	}
	delete A;
	delete B;
	delete L;

	return Aa;
}

/*
	Integer based power function, taken direction from StackOverflow:
	http://stackoverflow.com/questions/101439/the-most-efficient-way-to-implement-an-integer-based-power-function-powint-int
*/
int ipow(int base, int exp) {
	int result = 1;
	while (exp) {
		if (exp & 1)
			result *= base;
		exp >>= 1;
		base *= base;
	}

	return result;
}