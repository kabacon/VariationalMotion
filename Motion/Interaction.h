/*
	Matthew Fink, MEC 572, Spring 2015
	Adapted from code provided by Prof. Anurag Purwar

	Interaction.h - Defines the mouse and keyboard interactions with the GLUT window.
*/

#pragma once

#include "Point2D.h"

class Interaction
{
public:
	static double m_xRotate;
	static double m_yRotate;
	static double m_zRotate;
	static double m_xTrans;
	static double m_yTrans;
	static double m_zTrans;

	static void mouseButtonEvents(int, int, int, int);
	static void mouseMotionEvents(int, int);
	static void keyboard(unsigned char, int, int);
	static void mouseWheel(int wheel, int direction, int x, int y); 


private:
	static Point2D m_LDownPos;
	static Point2D m_RDownPos;
	static Point2D m_MDownPos;
	static int m_LButtonDown;
	static int m_RButtonDown;
	static int m_MButtonDown;

};