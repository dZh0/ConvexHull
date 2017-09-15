// Convex Hull of vector combinations

#pragma once
//true dependancies
#include <vector>
#include <map>
#include "FVector.h"

class CnvH {
public:

// Internal structures:
	enum geometry {empty, linear, planar, volume};

	// Holds a vector and a weight map from collection that constructs it
	struct point {
		FVector vec;
		std::map<int,float> weight;

		//constructors
		point() : vec(FV_ZERO) {};
		point(FVector _vec, int _idx, float _power = 1.0f) : vec(_vec) { weight[_idx] = _power; }; 
	};
	struct quad {
		// Holds the 4 indices of points that make up the quad and a normal vector
		size_t pointIdx[4];
		FVector normal;
	};

	CnvH();											// Default constructor
	CnvH(FVector const* p_arr, const int _size);	// Array constructor
	void add(FVector const* p_vec, const int idx);
	
private:
	geometry state;									// Geometric state: linear, planar etc.
	std::vector<FVector const*> collection;			// Collection of references TODO: set template class container
	std::vector<point> points;						// Points of the Convex Hull (INDEXED CONTAINER!)
	std::vector<quad> quads;						// Quads of the Convex Hull
};