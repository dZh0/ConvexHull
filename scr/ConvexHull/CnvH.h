// Convex Hull of vector combinations

#pragma once
//true dependancies
#include <vector>
#include <map>
#include <unordered_set>
#include "FVector.h"

template <class T = FVector>
class CnvH {
public:

// Internal structures:
	enum geometry {empty, linear, planar, volume};

	// Holds a vector and a weight array from colection that constructs it
	struct point {
		FVector vec;
		std::map<int,float> weight;

		//constructors
		point() : vec(FV_ZERO) {};
		point(FVector _vec, int _idx, float _power = 1.0f) : vec(_vec) { weight[_idx] = _power; }; 
	};
	struct quad {
		// Holds the 4 indecies of points thatmake up the quad and a normal vector
		size_t pointIdx[4];
		FVector normal;
		std::unordered_set<quad*> neighbour;
		inline size_t& operator[] (int x) { return pointIdx[x]; };
	};

	CnvH() :state(empty) {};						// Default constructor
	CnvH(T const* arr, const int _size);		// Array constructor
	void add(FVector const* p_vec, const int idx);
	
private:
	geometry state;									// Geometric state: linear, planae etc.
	std::vector<T const*> collection;			// Colection of references TODO: set template class container
	std::vector<point> points;						// Points of the Convex Hull (INDEXED CONTAINER!)
	std::vector<quad> quads;						// Quads of the Convex Hull
};