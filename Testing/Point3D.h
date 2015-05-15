#pragma once
#include <string>
using std::wstring;
using std::to_wstring;

class Point3D {
public:
	double x, y, z;
	Point3D() : x(0.0f), y(0.0f), z(0.0f) {}
	Point3D(double x, double y, double z) : x(x), y(y), z(z) {}
	wstring toString() { return (L"(" + to_wstring(x) + L", " + to_wstring(y) + L", " + to_wstring(z) + L")"); }
};