// TextCnvH.cpp : Defines the entry point for the console application.

//debug depenancies
#include <string>
#include <iostream>
#include <fstream>
#include <assert.h>
//dummy depenancies
#include "FVector.h"		// Simple float Vector 3D implementation
#include "AThruster.h"		// Dummy spaceship thruster implementation
//true dependancies
#include "CnvH.h"			// Convex Hull of vector combinations

int main() {
	CnvH CH_empty;
	std::ifstream in;
	in.open("../Input/inputTest.txt");
	while (!in.eof()){
		float x,y,z;
		in >> x >> y >> z;
		FVector vec = { x, y, z };
		CH_empty.Add(vec);
	};
	in.close();
	std::ofstream out;
	out.open("../Input/CnvH.obj");
	out << "#Vertices" << std::endl;
	out << CH_empty.GetPointStr();
	out << "#Quads" << std::endl;
	out << CH_empty.GetQuadStr();
	out.close();
    return 0;
}