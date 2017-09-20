// TextCnvH.cpp : Defines the entry point for the console application.

//debug depenancies
#include <iostream>
#include <fstream>
#include <assert.h>
//dummy depenancies
#include "FVector.h"		// Simple float Vector 3D implementation
#include "AThruster.h"		// Dummy spaceship thruster implementation
//true dependancies
#include "CnvH.h"			// Convex Hull of vector combinations

//UNIT TEST DATA
#define DATA_SIZE 11
const AThruster engines[DATA_SIZE] = {
	/* Below are defined 11 thrusters, each with his name, position and thrust vectors. */
	/* Vectors are deined in an orthonormal coordinate system where:	+X is front/forward, +Y is right and +Z is top/up.*/
	/* (Please note that the thrust is in the opposite direction of the movement it will produce!) */
	//	{ "name",					position{x,y,z},		thrust{x,y,z}        }
	{ "+X top front-left",			{ 1.0f, 2.0f,   0.5f },	{ 1.0f, 0.0f, 0.0f } },
	{ "+X bottom front-left",		{ 1.0f, 2.0f,  -0.5f },	{ 1.0f, 0.0f, 0.0f } },
	{ "+X top front-rignt",			{ 1.0f,-2.0f,   0.5f },	{ 1.0f, 0.0f, 0.0f } },
	{ "+X bottom front-right",		{ 1.0f,-2.0f,  -0.5f },	{ 1.0f, 0.0f, 0.0f } },
	{ "+Y back left",				{-3.0f, 1.0f,   0.0f },	{ 0.0f, 1.0f, 0.0f } },
	{ "-Y back right",				{-3.0f,-1.0f,   0.0f },	{ 0.0f,-1.0f, 0.0f } },
	{ "+Z back top",				{-1.0f, 0.0f,   1.0f },	{ 0.0f, 0.0f, 1.0f } },
	{ "-Z back bottom",				{-1.0f, 0.0f,  -1.0f },	{ 0.0f, 0.0f,-1.0f } },
	{ "-X main top",				{-3.0f, 0.0f,   2.0f },	{-3.0f, 0.0f, 0.0f } },
	{ "-X main bottom-left",		{-3.0f, 1.732f,-1.0f },	{-3.0f, 0.0f, 0.0f } },
	{ "-X main bottom-down",		{-3.0f,-1.732f,-1.0f },	{-3.0f, 0.0f, 0.0f } },
};
//end UNIT TEST DATA



int main() {
	std::ifstream inputFile;
	inputFile.open("../Input/inputTest_1.txt");
	CnvH CH_empty;
	int idx = 0;
	float A, B, C;
	FVector vec;
	do{
		std::cout << "Enter extrusion vector: ";
		std::cin >> A >> B >> C;
		vec = { A, B, C };
		CH_empty.Add(vec, idx);
		idx++;
	} while (vec != FV_ZERO);
	inputFile.close();
    return 0;
}