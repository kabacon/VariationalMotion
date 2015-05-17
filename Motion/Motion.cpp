/*
	Motion.cpp - See Motion.h for description.
*/

#include <iostream>
#include <fstream>

#include "Motion.h"
#include "freeglut\freeglut.h"

using arma::zeros;
using arma::cx_mat;


Motion::Motion() {
	// Load the control positions from a text file.
	fileName = "input.txt";
	if (readControlPositions()) {
		for (int i = 0; i < controlPositions.size(); i++) {
			variationalWeights.push_back(10.0f);
		}
		initFeaturePoints();
	}
	variationalIterations = 5;
	updateScrewMotion();
	updateBezierMotion();
	updateVariationalCurve();
	updateVariationalMotion();
}

bool Motion::readControlPositions() {

	ifstream inFile(fileName, ios::in);

	if (!inFile)
	{
		cerr << "File " << fileName << " could not be opened" << endl;
		return false;
	}

	controlPositions.clear();
	int i;
	int numberOfPositions;

	inFile >> numberOfPositions;

	Quaternion *RotationQuaternion = new Quaternion[numberOfPositions];
	Vector *TranslationVector = new Vector[numberOfPositions];

	for (i = 0; i<numberOfPositions; i++)
		inFile >> RotationQuaternion[i];

	for (i = 0; i<numberOfPositions; i++)
		inFile >> TranslationVector[i];

	for (i = 0; i<numberOfPositions; i++)
	{
		DualQuaternion dQ(RotationQuaternion[i], TranslationVector[i]);
		controlPositions.push_back(dQ);
		//cout << "first position from input: " << controlPositions[i] << endl;
	}

	delete[] RotationQuaternion;
	delete[] TranslationVector;
	return true;
}


void Motion::initFeaturePoints() {

	featurePoints.clear();
	// These feature points are chosen from freeglut_teapot_data.h
	// Randomly selected and assumed to be affinely independent
	vec v1("2.1 3.6 0.0 1.0");
	vec v2("1.561 0.1424 1.561 1.0");
	vec v3("-2.006 3.711 0.8532 1.0");
	vec v4("5.001 3.711 0.1687 1.0");

	vector<vec> base;
	base.push_back(v1);
	base.push_back(v2);
	base.push_back(v3);
	base.push_back(v4);

	// The glutSolidTeapot function takes the teapot vertices and 
	// submits them to these transformations
	mat rotate(4, 4);
	double c = cos(270 * 0.0174532925);
	double s = sin(270 * 0.0174532925);
	rotate << 1 << 0 << 0 << 0 << arma::endr
		<< 0 << c << -s << 0 << arma::endr
		<< 0 << s << c << 0 << arma::endr
		<< 0 << 0 << 0 << 1;
	mat scale(4, 4);
	scale << 0.15*0.15 << 0 << 0 << 0 << arma::endr
		<< 0 << 0.15*0.15 << 0 << 0 << arma::endr
		<< 0 << 0 << 0.15*0.15 << 0 << arma::endr
		<< 0 << 0 << 0 << 1;
	mat translation(4, 4);
	translation << 1 << 0 << 0 << 0 << arma::endr
		<< 0 << 1 << 0 << 0 << arma::endr
		<< 0 << 0 << -1.5 << 0 << arma::endr
		<< 0 << 0 << 0 << 1;

	mat transform = rotate * scale * translation;

	// Create an hMatrix from this
	hMatrix transf;
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			transf.m[i][j] = transform(i, j);
		}
	}
	/*
	for (int i = 0; i < 4; i++) {
	vec v = base[i];
	v = rotate * scale * trans * v;
	base[i] = v;
	}*/

	// Get the four feature points at each of the control positions.
	hMatrix hm0 = controlPositions[0].dualQuaternionToHomogeneousMatrix();	// Start with the first position
	for (int i = 0; i < controlPositions.size(); i++) {						// For each control position
		vector<vec> position;
		for (int j = 0; j < 4; j++) {										// For each of the four points
			hPoint hp(base[j](0), base[j](1), base[j](2), base[j](3));		// Convert the point to an hPoint
			hp = hm0 * transf * hp;											// Transform the point
			vec v(3);
			for (int k = 0; k < 3; k++) {									// Convert back to a vec
				v(k) = hp.coord[k];
			}
			position.push_back(v);											// Add the point
		}
		featurePoints.push_back(position);									// Add the four feature points

		// Move up to the next position
		if (i < controlPositions.size() - 1) {
			hm0 = controlPositions[i + 1].dualQuaternionToHomogeneousMatrix();
		}
	}
}



void Motion::updateScrewMotion() {
	screwPositions.clear();

	int numberOfPositions = controlPositions.size();
	for (int i = 0; i < numberOfPositions - 1; i++) {
		for (int j = 1; j < screwDivisions; j++) {
			double t = (double)j / (double)screwDivisions;
			DualQuaternion inter = (1 - t) * controlPositions[i] + t * controlPositions[i + 1];
			hMatrix interp = inter.dualQuaternionToHomogeneousMatrix().transpose();

			screwPositions.push_back(interp);
		}
	}
	glutPostRedisplay();
}


/*
	Bezier motion between control points
*/
void Motion::updateBezierMotion() {
	bezierPositions.clear();

	double t;
	DualQuaternion dq;
	hMatrix interp;
	for (int i = 1; i < controlPositions.size() * bezierDivisions; i++) {
		t = (double)i / (double)(controlPositions.size() * bezierDivisions);
		dq = deCasteljau(t);

		// Convert to hMatrix
		interp = dq.dualQuaternionToHomogeneousMatrix().transpose();

		bezierPositions.push_back(interp);
	}
	glutPostRedisplay();
}

void Motion::updateVariationalCurve() {
	if (featurePoints.empty()) return;

	// Rearrange the positions into sets of homologous points
	vector<vector<vec>> homologousPoints;
	for (int i = 0; i < featurePoints[0].size(); i++) {
		vector<vec> homologousSet;
		for (int j = 0; j < featurePoints.size(); j++) {
			homologousSet.push_back(featurePoints[j][i]);
		}
		homologousPoints.push_back(homologousSet);
	}

	variationalCurves.clear();
	if (approximateVariational) {
		for (vector<vec> homologous : homologousPoints) {
			variationalCurves.push_back(variationalSubdivision(homologous, variationalWeights, insertedWeight, variationalIterations));
		}
	} else {
		for (vector<vec> homologous : homologousPoints) {
			variationalCurves.push_back(variationalSubdivision(homologous, variationalIterations));
		}
	}
	glutPostRedisplay();

}

void Motion::updateVariationalMotion() {
	variationalPositions.clear();

	// Rearrange the feature curves into sets of positions
	vector<vector<vec>> positions;
	for (int i = 0; i < variationalCurves[0].size(); i++) {
		vector<vec> position;
		for (int j = 0; j < variationalCurves.size(); j++) {
			position.push_back(variationalCurves[j][i]);
		}
		positions.push_back(position);
	}

	
	// Begin with the first position, find the barycenter 
	vector<vec> X = featurePoints[0];
	vec xBar = barycenter(X);

	// Create a local coordinate system
	vector<vec> Xprime;
	for (int i = 0; i < X.size(); i++) {
		vec v = X[i] - xBar;
		Xprime.push_back(v);
	}

	// Calculate the special points
	list<cx_vec> specialX = calculateSpecialPoints(X);
	vector<cx_vec> specialXvec;

	// Convert the special points to barycentric coords w.r.t X ??
	cx_vec cxXBar(xBar, zeros<vec>(3));
	for (int i = 0; i < specialX.size(); i++) {
		cx_vec v = specialX.front();
		specialX.pop_front();
		v -= cxXBar;
		specialXvec.push_back(v);
	}

	// For every position after the first
	for (int i = 0; i < positions.size(); i++) {
		// Get the position, find the barycenter.
		vector<vec> Y = positions[i]; 
		vec yBar = barycenter(Y);
		
		// Try just getting a translation??
		vec tra = yBar - xBar;

		Quaternion qQ = controlPositions[0].GetReal();
		//cout<<"Q="<<Q<<endl;
		Quaternion qR = controlPositions[0].GetDual();
		//cout<<"R="<<R<<endl;
		Quaternion qT = (qR*qQ.Conjugate() - qQ*qR.Conjugate()) / qQ.Modulus(); // recover translation component
		
		vec tra2(3);
		for (int i = 0; i < 3; i++) {
			tra2(i) = qT.q[i];
		}
		// Translation from control position 0 to the current point
		double r[4] = { 0.0, 0.0, 0.0, 1.0 };
		Quaternion real(r);
		double tr[3] = { tra(0) + tra2(0), tra(1) + tra2(1), tra(2) + tra2(2) };
		hPoint trans(tr);
		DualQuaternion dq(real, trans);

		// Create a local coordinate system
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

		// M is this (symmetric) matrix
		mat M = zeros(4, 4);
		M << Sxx + Syy + Szz << Syz - Szy << Szx - Sxz << Sxy - Syz << arma::endr
			<< Syz - Szy << Sxx - Syy - Szz << Sxy + Syx << Szx + Sxz << arma::endr
			<< Szx - Sxz << Sxy + Syx << -Sxx + Syy - Szz << Syz + Szy << arma::endr
			<< Sxy - Syz << Szx + Sxz << Syz + Szy << -Sxx - Syy + Szz;

		// Find the maximal eigenvalue and corresponding unit eigenvector
		vec eigVals;
		mat eigVecs;
		eig_sym(eigVals, eigVecs, M);

		// A is the rotation unit quaternion
		vec a = eigVecs.col(3);

		// Create the rotation matrix R
		mat R = zeros(3, 3);
		double a0 = a(0);
		double a1 = a(1);
		double a2 = a(2);
		double a3 = a(3);
		R << a0*a0 + a1*a1 - a2*a2 - a3*a3 << 2 * (a1*a2 + a0*a3) << 2 * (a1*a3 - a0*a2) << arma::endr
			<< 2 * (a1*a2 - a0*a3) << a0*a0 - a1*a1 + a2*a2 - a3*a3 << 2 * (a2*a3 + a0*a1) << arma::endr
			<< 2 * (a1*a3 + a0*a2) << 2 * (a2*a3 - a0*a1) << a0*a0 - a1*a1 - a2*a2 - a3*a3;

		// Get the translation vector from x to y
		vec t = yBar - R * xBar;
		
		// Make a translation matrix
		mat translation = zeros(4, 4);
		translation << 1 << 0 << 0 << t(0) << arma::endr
			<< 0 << 1 << 0 << t(1) << arma::endr
			<< 0 << 0 << 1 << t(2) << arma::endr
			<< 0 << 0 << 0 << 1;
		/*
		// Use the rotation quaternion and the translation vectoro to build a Dual Quaternion
		double r[4] = { a(0), a(1), a(2), a(3) };
		//double r[4] = { 0.0, 0.0, 0.0, 1.0 };
		Quaternion real(r);
		double tr[3] = { t(0), t(1), t(2) };
		hPoint trans(tr);
		DualQuaternion dq(real, trans);

		DualQuaternion dq2 = controlPositions[0] * dq;
		// Convert this to an hMatrix and add it to the list
		hMatrix hm1 = dq.dualQuaternionToHomogeneousMatrix();*/
		variationalPositions.push_back(dq.dualQuaternionToHomogeneousMatrix().transpose());
	}
	
}


/*
	Draw the control positions
*/
void Motion::drawControlPositions() {
	vector <hMatrix> homogeneousMatricesForCtrlPositions;
	int numberOfPositions = controlPositions.size();
	glLineWidth(1.0f);
	for (int i = 0; i< numberOfPositions; i++) {
		homogeneousMatricesForCtrlPositions.push_back(controlPositions[i].dualQuaternionToHomogeneousMatrix().transpose());
		double MatrixforOpenGLStack[16];

		for (int i1 = 0; i1<4; i1++)
			for (int i2 = 0; i2<4; i2++)
				MatrixforOpenGLStack[4 * i1 + i2] = homogeneousMatricesForCtrlPositions.at(i).m[i1][i2];

		glPushMatrix();
		glMultMatrixd(MatrixforOpenGLStack);
		glutSolidTeapot(0.15);
		glPopMatrix();
	}
	glutPostRedisplay();

}

/*
	Linear interpolation between control positions
*/
void Motion::drawScrewMotion() {
	
	for (hMatrix interp : screwPositions) {
		// Convert to OpenGL matrix
		double oglMatrix[16];
		for (int i1 = 0; i1<4; i1++)
			for (int i2 = 0; i2<4; i2++)
				oglMatrix[4 * i1 + i2] = interp.m[i1][i2];

		glPushMatrix();
		glMultMatrixd(oglMatrix);
		glutSolidTeapot(0.15);
		glPopMatrix();
	}
}

void Motion::drawBezierMotion() {
	for (hMatrix interp : bezierPositions) {
		// Convert to OpenGL matrix
		double oglMatrix[16];
		for (int i1 = 0; i1<4; i1++)
			for (int i2 = 0; i2<4; i2++)
				oglMatrix[4 * i1 + i2] = interp.m[i1][i2];

		glPushMatrix();
		glMultMatrixd(oglMatrix);
		glutSolidTeapot(0.15);
		glPopMatrix();
	}
}

void Motion::drawVariationalCurve() {
	glLineWidth(1.0f);
	glColor3f(1.0f, 1.0f, 1.0f);
	for (vector<vec> curve : variationalCurves) {
		glBegin(GL_LINE_STRIP);
		for (vec v : curve) {
			glVertex3f(v(0), v(1), v(2));
		}
		glEnd();
	}
}

void Motion::drawVariationalMotion() {
	for (hMatrix interp : variationalPositions) {
		// Convert to OpenGL matrix
		double oglMatrix[16];
		for (int i1 = 0; i1<4; i1++)
			for (int i2 = 0; i2<4; i2++)
				oglMatrix[4 * i1 + i2] = interp.m[i1][i2];

		glPushMatrix();
		glMultMatrixd(oglMatrix);
		glutSolidTeapot(0.15);
		glPopMatrix();
	}

}




DualQuaternion Motion::deCasteljau(double t) {
	vector<DualQuaternion> dqs;
	int n = controlPositions.size() - 1;

	for (int i = 0; i < n + 1; i++) {
		dqs.push_back(controlPositions[i]);
	}

	for (int r = 1; r <= n; r++) {
		for (int i = 0; i <= n - r; i++) {
			dqs[i] = (1.0 - t)*dqs[i] + t*dqs[i + 1];
		}
	}

	return dqs[0];
}





void Motion::setScrewDivisions(int divisions) { 
	if (screwDivisions != divisions) {
		screwDivisions = divisions;
		updateScrewMotion();
	}
}
void Motion::setBezierDivisions(int divisions) {
	if (bezierDivisions != divisions) {
		bezierDivisions = divisions;
		updateBezierMotion();
	}
}
void Motion::setVariationalIterations(int iters) {
	if (variationalIterations != iters) {
		variationalIterations = iters;
		updateVariationalCurve();
		updateVariationalMotion();
	}
}

void Motion::setApproximateVariational(int approx) {
	if (approximateVariational != approx) {
		approximateVariational = approx;
		updateVariationalCurve();
		updateVariationalMotion();
	}
}

void Motion::setVariationalWeights(int id, double weight) {
	if (variationalWeights.size() > id) {
		variationalWeights[id] = weight;
		updateVariationalCurve();
		updateVariationalMotion();
	}
}

void Motion::setInsertedWeights(double weight) {
	if (insertedWeight > weight + 0.0001 || insertedWeight < weight - 0.0001) {
		insertedWeight = weight;
		updateVariationalCurve();
		updateVariationalMotion();
	}
}

bool Motion::setFileName(string file) {
	if (fileName.compare(file) != 0) {
		fileName = file;
		if (readControlPositions()) {
			initFeaturePoints();
			variationalWeights.clear();
			for (int i = 0; i < controlPositions.size(); i++) {
				variationalWeights.push_back(10.0f);
			}
			return true;
		}
	}
	return false;
}

void Motion::updateAll() {
	updateScrewMotion();
	updateBezierMotion();
	updateVariationalCurve();
	updateVariationalMotion();
	updateBezierMotion();
}

/*
	Helper methods for variational subdivision
*/
/*
	Compute the barycenter of a point cloud.
*/
vec Motion::barycenter(const vector<vec> points) {
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
list<cx_vec> Motion::calculateSpecialPoints(const vector<vec> points) {
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
vector<vec> Motion::variationalSubdivision(const vector<vec> points, int iterations) {

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
vector<vec> Motion::variationalSubdivision(const vector<vec> points, const vector<double> weights, double qWeights, int iterations) {
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
			bX(size + i) = (*x)(i)* (*lambdas)(i);
			bY(size + i) = (*y)(i)* (*lambdas)(i);
			bZ(size + i) = (*z)(i)* (*lambdas)(i);
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
mat* Motion::getAMatrix(int size) {
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
mat* Motion::getBMatrix(int size) {
	mat* B = new mat(size, size + 1);
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
mat* Motion::getLMatrix(int size, vec* lambdas) {
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
mat* Motion::buildAaMatrix(mat* A, mat* B, mat* L, int size) {
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
int Motion::ipow(int base, int exp) {
	int result = 1;
	while (exp) {
		if (exp & 1)
			result *= base;
		exp >>= 1;
		base *= base;
	}

	return result;
}


/*
	Helper methods for registration
*/
/*
	Compute the S_,_ coefficients of the M matrix
*/
double Motion::S(int x, int y, const vector<vec>& X, const vector<vec>& Y) {
	double s = 0.0f;
	for (int i = 0; i < X.size(); i++) {
		s += (X[i](x) * Y[i](y));
	}
	return s;

}

/*
	Compute the S_,_ coefficients of the M matrix
*/
cx_double Motion::S(int x, int y, const vector<cx_vec>& X, const vector<cx_vec>& Y) {
	cx_double s(0.0f, 0.0f);
	for (int i = 0; i < X.size(); i++) {
		s += (X[i](x) * Y[i](y));
	}
	return s;
}