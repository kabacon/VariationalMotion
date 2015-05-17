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
void updateGUI();
void printControls();

struct Settings {
	int screwDiv = 5;
	int bezierDiv = 5;
	int variationalIt = 3;
	int showControlPositions = true;
	int showScrewMotion = false;
	int showBezierMotion = false;
	int showVariationalCurve = false;
	int showVariationalMotion = false;
	int approximateVariational = false;
	float variationalWeights[50];
	float insertedWeights = 10.0; 
	GLUI_EditText* fileText;
	GLUI_String* fileName;
};

struct GUI {
	GLUI* glui;
	GLUI_Panel* showPanel; 
	GLUI_Panel* divPanel;
	GLUI_Spinner* screwDivisionsSpinner;
	GLUI_Spinner* bezierDivisionsSpinner;
	GLUI_Spinner* variationalIterationsSpinner;

	GLUI_Panel* varPanel;
	vector<GLUI_Spinner*> weightSpinners;
	GLUI_Spinner* insertedWeightSpinner;

	GLUI_EditText* fileText;
	GLUI_String* fileName;
};

Settings settings;
GUI gui;
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

	gui.glui = GLUI_Master.create_glui_subwindow(win_id, GLUI_SUBWINDOW_RIGHT);
	gui.glui->add_statictext("Options");
	gui.glui->add_separator();
	settings.fileText = gui.glui->add_edittext("File", GLUI_EDITTEXT_TEXT, settings.fileName, 999, updateFile);
	settings.fileText->set_text("input.txt");

	gui.showPanel = gui.glui->add_panel("Show");
	gui.glui->add_checkbox_to_panel(gui.showPanel, "Control Positions", &settings.showControlPositions);
	gui.glui->add_checkbox_to_panel(gui.showPanel, "Screw Motion", &settings.showScrewMotion);
	gui.glui->add_checkbox_to_panel(gui.showPanel, "Bezier Motion", &settings.showBezierMotion);
	gui.glui->add_checkbox_to_panel(gui.showPanel, "Variational Curve", &settings.showVariationalCurve);
	gui.glui->add_checkbox_to_panel(gui.showPanel, "Variational Motion", &settings.showVariationalMotion);


	gui.divPanel = gui.glui->add_panel("Subdivisions");
	gui.screwDivisionsSpinner = gui.glui->add_spinner_to_panel(gui.divPanel, "Screw", GLUI_SPINNER_INT, &settings.screwDiv);
	gui.screwDivisionsSpinner->set_int_limits(0, 20);
	gui.bezierDivisionsSpinner = gui.glui->add_spinner_to_panel(gui.divPanel, "Bezier", GLUI_SPINNER_INT, &settings.bezierDiv);
	gui.bezierDivisionsSpinner->set_int_limits(0, 20);
	gui.variationalIterationsSpinner = gui.glui->add_spinner_to_panel(gui.divPanel, "Variational", GLUI_SPINNER_INT, &settings.variationalIt);
	gui.variationalIterationsSpinner->set_int_limits(0, 8);


	gui.varPanel = gui.glui->add_panel("Variational Settings");
	gui.glui->add_checkbox_to_panel(gui.varPanel, "Approximation", &settings.approximateVariational);

	for (int i = 0; i < 11; i++) {
		GLUI_Spinner* weightSpinner;
		settings.variationalWeights[i] = 10.0f;
		char label[10];
		sprintf_s(label, "Weight %d", i);
		weightSpinner = gui.glui->add_spinner_to_panel(gui.varPanel, label, GLUI_SPINNER_FLOAT, &settings.variationalWeights[i], i, updateWeights);
		weightSpinner->set_float_val(10.0f);
		weightSpinner->set_float_limits(0.001, 1E8);
		weightSpinner->set_speed(0.0000001f);
		gui.weightSpinners.push_back(weightSpinner);
	}
	gui.insertedWeightSpinner = gui.glui->add_spinner_to_panel(gui.varPanel, "Weight Q", GLUI_SPINNER_FLOAT, &settings.insertedWeights);
	gui.insertedWeightSpinner->set_float_val(10.0f);
	gui.insertedWeightSpinner->set_float_limits(0.001, 1E8);
	gui.insertedWeightSpinner->set_speed(0.000001f);

	updateGUI();

	printControls();

	glutMainLoop();

	return 0;
}

void updateGUI() {
	for (int i = 0; i < m->getNumControlPositions(); i++) {
		gui.weightSpinners[i]->enable();
		gui.weightSpinners[i]->set_float_val(10.0f);
	}
	for (int i = m->getNumControlPositions(); i < 11; i++) {
		gui.weightSpinners[i]->disable();
	}

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
	float LightDiffuse[] = { 0.8f, 0.8f, 0.8f, 0.8f };
	float LightSpecular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	float LightPosition[] = { 0.0f, 0.0f, 0.0f, 1.0f };
	float LightPosition2[] = { -3.0f, -3.0f, 0.0f, 1.0f };

	float RedSurface[] = { 1.0f, 0.0f, 0.0f, 1.0f };
	float GreenSurface[] = { 0.0f, 1.0f, 0.0f, 1.0f };
	float BlueSurface[] = { 0.0f, 0.0f, 1.0f, 1.0f };

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glShadeModel(GL_SMOOTH);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmbient);
	glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpecular);


	glDepthFunc(GL_LESS);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_NORMALIZE);

	float no_mat[] = { 0.0f, 0.0f, 0.0f, 1.0f };
	float mat_ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f };
	float mat_ambient_color[] = { 0.8f, 0.3f, 0.7f, 1.0f };
	float mat_diffuse[] = { 0.8f, 0.8f, 0.8f, 1.0f };

	float mat_specular[] = { 0.7f, 0.7f, 0.7f, 1.0f };

	float no_shininess[] = { 0.0 };
	float low_shininess[] = { 5.0 };
	float high_shininess[] = { 100.0 };
	float mat_emission[] = { 0.1f, 0.1f, 0.1f, 0.0f };

	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
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

	static float BLUE[3] = { 0.1f, 0.1f, 1.0f };
	static float blue_ambient[] = { 0.0f, 0.0f, 0.1f, 1.0f };
	static float low_specular[] = { 0.2f, 0.2f, 0.2f, 1.0f };
	glMaterialfv(GL_FRONT, GL_AMBIENT, blue_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, low_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, BLUE);
	if (settings.showControlPositions)
		m->drawControlPositions();

	static float GREY[3] = { 0.2f, 0.2f, 0.2f };
	static float mat_ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f };
	static float mat_specular[] = { 0.7f, 0.7f, 0.7f, 1.0f };
	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, GREY);
	if (settings.showScrewMotion)
		m->drawScrewMotion();

	static float LIGHT_GREY[3] = { 0.4f, 0.4f, 0.4f };
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, LIGHT_GREY);
	if (settings.showBezierMotion)
		m->drawBezierMotion();

	static float emission[] = { 0.3f, 0.3f, 0.3f, 0.0f };
	glMaterialfv(GL_FRONT, GL_EMISSION, emission);
	if (settings.showVariationalCurve)
		m->drawVariationalCurve();

	static float ORANGE[3] = { 1.0f, 0.5f, 0.1f };
	static float mat_emission[] = { 0.1f, 0.1f, 0.1f, 0.0f };
	glMaterialfv(GL_FRONT, GL_EMISSION, mat_emission);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, ORANGE);
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
		updateGUI();
	}

}

void reshape(int w, int h) {
	//Setup Viewing Transform: This calculates windows coordinates
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);

	//Setup Projection: Device coordinates
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(30.0f, (double)w / (double)h, 0.1f, 1000.0f);

	//Set up Modeling and Viewing transform: Get Eye coordinates
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(0.0f, 0.0f, -10.0f);

}

void printControls() {
	cout << "Matthew Fink, MEC 572, Spring 2015" << endl;
	cout << "Motion Design using Variational Subdivision and Registration" << endl;
	cout << endl;
	cout << "Based on the paper:" << endl;
	cout << "'From curve design algorithms to the design of rigid body motions'" << endl;
	cout << "   by Hofer et al." << endl;
	cout << endl << endl;
	cout << "Controls:" << endl << endl;
	cout << "Left Mouse:  Rotate window" << endl;
	cout << "Right Mouse: Translate window" << endl;
	cout << "Mouse Wheel: Zoom" << endl;
	cout << endl;
	cout << "Q: Quit" << endl;

}