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
	out << CH_empty;
	out.close();
	while (true) {
		std::cout << std::endl;
		std::cout << "Enter a vector to disolve: ";
		FVector vec;
		std::cin >> vec.x >> vec.y >> vec.z;
		if (vec == FV_ZERO) break;
		CH_empty.Disolve(vec);
	}
    return 0;
}