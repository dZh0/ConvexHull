// Convex Hull of vector combinations
#pragma once

//true dependancies
#include <vector>
#include <map>
#include "FVector.h"

class CnvH {
public:
	CnvH();											// Default constructor
	CnvH(FVector const* p_arr, const int _size);	// Array constructor
	void add(FVector const _vec, const int collectionIdx);
	
private:
	// Holds a vector and a weight map from collection that constructs it
	struct point 
	{
		FVector vec;
		std::map<int, float> weight;

		//constructors
		point() : vec(FV_ZERO) {};
		point(FVector _vec, int _idx, float _power = 1.0f) : vec(_vec) { weight[_idx] = _power; };
	};
	// Holds the 4 indices of points that make up the quad and a normal vector
	struct quad {
		size_t pointIdx[4];
		FVector normal;
	};
	enum geometry { empty, linear, planar, volume };

	geometry state;									// Geometric state: linear, planar etc.
	std::vector<FVector> collection;				// Collection of references TODO: set template class container
	std::vector<point> points;						// Points of the Convex Hull (INDEXED CONTAINER!)
	std::vector<quad> quads;						// Quads of the Convex Hull
};