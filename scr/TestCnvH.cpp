// TextCnvH.cpp : Defines the entry point for the console application.

//debug depenancies
#include <iostream>
#include <fstream>
#include <assert.h>
//dummy depenancies
#include "FVector.h"		// Simple float Vector 3D implementation
#include "AThruster.h"		// Dummy spaceship thruster implementation
//true dependancies
#include <GLutilities.h>
#include "CnvH.h"			// Convex Hull of vector combinations

int main(int argc, char **argv) {
	CnvH CH_empty;
	int counter = 0;
	std::ifstream inputFile;
	inputFile.open("../Input/inputTest_2.txt");
	while (!inputFile.eof()){
		float x, y, z;
		inputFile >> x >> y >> z;
		FVector vec = { x, y, z };
		CH_empty.Add(vec, counter);
		counter++;
	};
	inputFile.close();

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutCreateWindow("red 3D lighted cube");
	glutDisplayFunc(display);
	init();
	glutMainLoop();

    return 0;
}