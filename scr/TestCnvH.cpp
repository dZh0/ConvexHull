// Vectors.cpp : Defines the entry point for the console application.

//dummy depenancies
#include "FVector.h"		// Simple float Vector 3D implementation
#include "AThruster.h"		// Dummy spaceship thruster implementation
//true dependancies
#include "CnvH.h"			// Convex Hull of vector combinations

//UNIT TEST DATA
/* Below are defined 11 thrusters, each with his name, position and thrust vectors.
Vectors are deined in an orthonormal coordinate system where:
+X is front/forward, +Y is right and +Z is top/up.
(Please note that the thrust is in the opposite direction of the movement it will produce!)*/
#define DATA_SIZE 11
const AThruster engines[DATA_SIZE] = {
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
	//Create force and momentum arrays.
	FVector force[DATA_SIZE];
	FVector momentum[DATA_SIZE];
	for (int i = 0; i < DATA_SIZE; i++) {
		const AThruster* E = engines+i;
		force[i] = E->thrust;
		momentum[i] = cross(E->position, E->thrust);
	}
	/* Create force combination and momentum combination
	convex hulls from force and momentum arrays.*/
	CnvH CH_maxForce(force, DATA_SIZE);
	CnvH CH_maxMomentum(momentum, DATA_SIZE);

    return 0;
}