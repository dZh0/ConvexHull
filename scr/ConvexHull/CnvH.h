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
	enum geometry { empty, linear, planar, volume };
	geometry state;

	std::vector<FVector> collection;	// Collection of references TODO: set template class container

	struct point {
		FVector vec;
		std::map<int, float> weight;

		//constructors
		point() : vec(FV_ZERO) {};
		point(FVector _vec, int _idx, float _power = 1.0f) : vec(_vec) { weight[_idx] = _power; };
	};
	std::vector<point> points;			// Points of the Convex Hull (INDEXED CONTAINER!)

	struct quad {
		size_t pointIdx[4];
		FVector normal;
	};
	std::vector<quad> quads;			// Quads of the Convex Hull

	typedef std::pair<size_t, size_t> edge;

	std::vector<edge> findOpenEdges(const std::vector<quad*>& quadArray);
	void sortEdges(std::vector<edge>& edgeArray);
	quad buildQuad(edge e1, edge e2, FVector dist);
	quad flipQuad(const quad& q);
};