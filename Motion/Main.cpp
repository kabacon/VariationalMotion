/*
	Matthew Fink, MEC 572, Spring 2015

	This program implements a motion design algorithm using variational subdivision 
	to generate feature curves, and registration to map rigid body transformations 
	to the sets of feature curves. From the paper "From curve design algorithms to 
	the design of rigid body motions" by Hofer et al.
*/

#pragma once

#include <iostream>
#include "freeglut/freeglut.h"
#include "glui/glui.h"
#include "Interaction.h"
#include "Motion.h"


void init();
void setupLight();
void display();
void updateSettings(); 
void updateFile(int id);
void updateWeights(int id);
void reshape(int w, int h);

struct Settings {
	int screwDiv = 5;
	int bezierDiv = 5;
	int variationalIt = 5;
	int showControlPositions = true;
	int showScrewMotion = false;
	int showBezierMotion = false;
	int showVariationalCurve = true;
	int showVariationalMotion = false;
	int approximateVariational = false;
	float variationalWeights[50];
	float insertedWeights = 10.0; 
	GLUI_EditText* fileText;
	GLUI_String* fileName;
};

Settings settings;
Motion* m;

/*
	Main method, set up GLUT, GLUI, and run the main loop.
*/
int main(int argc, char** argv) {
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_ALPHA);
	glutInitWindowSize(1280, 720);
	glutInitWindowPosition(320, 180);
	int win_id = glutCreateWindow("From Curve Design to Motion Design");

	init();
	setupLight();
	glutDisplayFunc(display);
	GLUI_Master.set_glutReshapeFunc(reshape);
	GLUI_Master.set_glutMouseFunc(Interaction::mouseButtonEvents);
	GLUI_Master.set_glutKeyboardFunc(Interaction::keyboard);
	glutMotionFunc(Interaction::mouseMotionEvents); 
	glutMouseWheelFunc(Interaction::mouseWheel);

	m = new Motion();

	GLUI* glui = GLUI_Master.create_glui_subwindow(win_id, GLUI_SUBWINDOW_RIGHT);
	glui->add_statictext("Options");
	glui->add_separator();
	settings.fileText = glui->add_edittext("File", GLUI_EDITTEXT_TEXT, settings.fileName, 999, updateFile);
	settings.fileText->set_text("input.txt");

	GLUI_Panel* showPanel = glui->add_panel("Show");
	glui->add_checkbox_to_panel(showPanel, "Control Positions", &settings.showControlPositions);
	glui->add_checkbox_to_panel(showPanel, "Screw Motion", &settings.showScrewMotion);
	glui->add_checkbox_to_panel(showPanel, "Bezier Motion", &settings.showBezierMotion);
	glui->add_checkbox_to_panel(showPanel, "Variational Curve", &settings.showVariationalCurve);
	glui->add_checkbox_to_panel(showPanel, "Variational Motion", &settings.showVariationalMotion);


	GLUI_Panel* divPanel = glui->add_panel("Subdivisions");
	GLUI_Spinner* screwDivisionsSpinner = glui->add_spinner_to_panel(divPanel, "Screw", GLUI_SPINNER_INT, &settings.screwDiv);
	screwDivisionsSpinner->set_int_limits(0, 20);
	GLUI_Spinner* bezierDivisionsSpinner = glui->add_spinner_to_panel(divPanel, "Bezier", GLUI_SPINNER_INT, &settings.bezierDiv);
	bezierDivisionsSpinner->set_int_limits(0, 20);
	GLUI_Spinner* variationalIterationsSpinner = glui->add_spinner_to_panel(divPanel, "Variational", GLUI_SPINNER_INT, &settings.variationalIt);
	variationalIterationsSpinner->set_int_limits(0, 10);


	GLUI_Panel* varPanel = glui->add_panel("Variational Settings");
	glui->add_checkbox_to_panel(showPanel, "Approximational", &settings.approximateVariational);

	GLUI_Spinner* weightSpinner;
	for (int i = 0; i < 11; i++) {
		settings.variationalWeights[i] = 10.0f;
		char label[10];
		sprintf_s(label, "Weight %d", i);
		weightSpinner = glui->add_spinner_to_panel(varPanel, label, GLUI_SPINNER_FLOAT, &settings.variationalWeights[i], i, updateWeights);
		weightSpinner->set_float_val(10.0f);
		weightSpinner->set_float_limits(0.001, 1E8);
	}
	GLUI_Spinner* insertedWeightSpinner;
	insertedWeightSpinner = glui->add_spinner_to_panel(varPanel, "Weight Q", GLUI_SPINNER_FLOAT, &settings.insertedWeights);
	insertedWeightSpinner->set_float_val(10.0f);
	insertedWeightSpinner->set_float_limits(0.001, 1E8);


	glutMainLoop();

	return 0;
}


void init() {

	glClearColor(0.1, 0.1, 0.1, 0.1);

	GLUquadricObj *qobj = gluNewQuadric();
	gluQuadricDrawStyle(qobj, GLU_FILL);
	gluQuadricNormals(qobj, GLU_SMOOTH);

	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);

	// specify the back of the buffer as clear depth
	glClearDepth(1.0f);

	double eqn[] = { 0.01f, 0.0f, 0.01f, -1.0f };
	// enable clip plane	
	glClipPlane(GL_CLIP_PLANE0, eqn);

}

void setupLight() {

	float LightAmbient[] = { 1.0f, 1.0f, 1.0f, 1.0 };
	float LightDiffuse[] = { 0.7f, 0.7f, 0.7f, 1.0f };
	float LightSpecular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	float LightPosition[] = { 3.0f, 3.0f, 3.0f, 1.0f };
	float LightPosition2[] = { -3.0f, -3.0f, 0.0f, 1.0f };

	float RedSurface[] = { 1.0f, 0.0f, 0.0f, 1.0f };
	float GreenSurface[] = { 0.0f, 1.0f, 0.0f, 1.0f };
	float BlueSurface[] = { 0.0f, 0.0f, 1.0f, 1.0f };

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glShadeModel(GL_SMOOTH);

	glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmbient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpecular);
	glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);
	glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse);
	glLightfv(GL_LIGHT1, GL_SPECULAR, LightSpecular);
	glLightfv(GL_LIGHT1, GL_POSITION, LightPosition2);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	//glEnable(GL_LIGHT1);

	glDepthFunc(GL_LESS);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_NORMALIZE);

	float no_mat[] = { 0.0f, 0.0f, 0.0f, 1.0f };
	float mat_ambient[] = { 0.7f, 0.7f, 0.7f, 1.0f };
	float mat_ambient_color[] = { 0.8f, 0.3f, 0.7f, 1.0f };
	float mat_diffuse[] = { 0.7f, 0.7f, 0.7f, 1.0f };

	float mat_specular[] = { 0.7f, 0.7f, 0.7f, 1.0f };

	float no_shininess[] = { 0.0 };
	float low_shininess[] = { 5.0 };
	float high_shininess[] = { 100.0 };
	float mat_emission[] = { 0.2f, 0.1f, 0.1f, 0.0f };

	glMaterialfv(GL_FRONT, GL_AMBIENT, no_mat);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, high_shininess);
	glMaterialfv(GL_FRONT, GL_EMISSION, mat_emission);

}

void display(void) {

	updateSettings();

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glPushMatrix();
	glTranslated(Interaction::m_xTrans, Interaction::m_yTrans, Interaction::m_zTrans);
	glRotated(Interaction::m_xRotate, 1.0, 0.0, 0.0);
	glRotated(Interaction::m_yRotate, 0.0, 1.0, 0.0);
	glRotated(Interaction::m_zRotate, 0.0, 0.0, 1.0);

	static float BROWN[3] = { 0.7f, 0.5f, 0.2f };
	glMaterialfv(GL_FRONT, GL_DIFFUSE, BROWN);
	if (settings.showControlPositions)
		m->drawControlPositions();

	static float GREY[3] = { 0.5f, 0.5f, 0.5f };
	glMaterialfv(GL_FRONT, GL_DIFFUSE, GREY);
	if (settings.showScrewMotion)
		m->drawScrewMotion();

	static float LIGHT_GREY[3] = { 0.8f, 0.8f, 0.8f };
	glMaterialfv(GL_FRONT, GL_DIFFUSE, LIGHT_GREY);
	if (settings.showBezierMotion)
		m->drawBezierMotion();

	if (settings.showVariationalCurve)
		m->drawVariationalCurve();

	static float BLUE[3] = { 0.5f, 0.5f, 1.0f };
	glMaterialfv(GL_FRONT, GL_DIFFUSE, BLUE);
	if (settings.showVariationalMotion)
		m->drawVariationalMotion();

	glPopMatrix();

	glutSwapBuffers();
}

void updateSettings() {
	m->setScrewDivisions(settings.screwDiv);
	m->setBezierDivisions(settings.bezierDiv);
	m->setVariationalIterations(settings.variationalIt);
	m->setApproximateVariational(settings.approximateVariational);
	m->setInsertedWeights(settings.insertedWeights);
}

void updateWeights(int id) {
	m->setVariationalWeights(id, settings.variationalWeights[id]);
}

void updateFile(int id) {
	string text = settings.fileText->get_text();
	if (m->setFileName(text)) {
		m->updateAll();
	}

}

void reshape(int w, int h) {
	//Setup Viewing Transform: This calculates windows coordinates
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);

	//Setup Projection: Device coordinates
	::glMatrixMode(GL_PROJECTION);
	::glLoadIdentity();
	gluPerspective(30.0f, (double)w / (double)h, 0.1f, 1000.0f);

	//Set up Modeling and Viewing transform: Get Eye coordinates
	::glMatrixMode(GL_MODELVIEW);
	::glLoadIdentity();
	::glTranslatef(0.0f, 0.0f, -10.0f);

}