/*
	Matthew Fink, MEC 572, Spring 2015

	Motion.h - Defines the operations for motion using several different algorithms. The 
	main focus is motion using variational subdivision and registration. Screw motions 
	and Bezier motion are also included, as a comparison against the variational motion.

*/

#pragma once

#include <vector>
#include <list>
#include <armadillo>
#include <string>
#include "DualQuaternion.h"

using std::list;
using std::vector;
using arma::mat;
using arma::vec;
using arma::cx_vec;
using arma::cx_double;

class Motion {
private:
	vector<DualQuaternion> controlPositions;
	vector<hMatrix> screwPositions;
	vector<hMatrix> bezierPositions;
	vector<vector<vec>> variationalCurves;
	vector<hMatrix> variationalPositions;
	vector<double> variationalWeights;
	vector<vector<vec>> featurePoints;
	double insertedWeight;
	string fileName;
	//hMatrix initialRotation;

	int screwDivisions;
	int bezierDivisions;
	int variationalIterations;
	bool approximateVariational;

public:
	Motion();

	void updateScrewMotion();
	void updateBezierMotion();
	void updateVariationalCurve();
	void updateVariationalMotion();

	void drawControlPositions();
	void drawScrewMotion();
	void drawBezierMotion();
	void drawVariationalCurve();
	void drawVariationalMotion();

	void setScrewDivisions(int divisions);
	void setBezierDivisions(int divisions);
	void setVariationalIterations(int iter);
	void setApproximateVariational(int approx);
	void setVariationalWeights(int id, double weight);
	void setInsertedWeights(double weight);
	bool setFileName(string file);
	void updateAll();

	int getNumControlPositions() { return controlPositions.size(); }

private:
	bool readControlPositions();
	void initFeaturePoints(); 
	DualQuaternion deCasteljau(double t);

	vec barycenter(const vector<vec> points);

	// Helper methods for variational subdivision
	vector<vec> variationalSubdivision(const vector<vec> points, int iterations);
	vector<vec> variationalSubdivision(const vector<vec> points, vector<double> weights, double qWeights, int iterations);
	list<cx_vec> calculateSpecialPoints(const vector<vec> points);
	int ipow(int base, int exp);
	mat* getAMatrix(int size);
	mat* getBMatrix(int size);
	mat* getLMatrix(int size, vec* lambdas);
	mat* buildAaMatrix(mat* A, mat* B, mat* L, int size);

	// Helper methods for registration
	double S(int x, int y, const vector<vec>& X, const vector<vec>& Y); 
	cx_double S(int x, int y, const vector<cx_vec>& X, const vector<cx_vec>& Y);

};