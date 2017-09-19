// Convex Hull of vector combinations
#pragma once

//true dependancies
#include <vector>
#include <map>
#include "FVector.h"

class CnvH {
public:
	CnvH();											// Default constructor
	CnvH(FVector const* p_arr, int _size);			// Array constructor

	void Add(FVector extrusion, int collectionIdx);

private:
	enum geometry { empty, linear, planar, volume };
	geometry state;

	std::vector<FVector> collection;	// Collection of references TODO: set template class container

	struct point {
		FVector vec;
		std::map<int, float> weight;

		point() : vec(FV_ZERO) {};
	};
	std::vector<point> hullPoints;			// Points of the Convex Hull (MUST BE INDEXED CONTAINER!)

	struct quad {
		size_t pointIdx[4];
		FVector normal;
	};
	std::vector<quad> hullQuads;			// Quads of the Convex Hull

	struct edge{
		size_t startIdx;
		size_t endIdx;
	};

	std::vector<edge>
	FindOpenEdges(const std::vector<quad*>& quadArray);

	void
	SortEdges(std::vector<edge>& edgeArray);

	quad
	BuildQuad(edge e1, edge e2, FVector dist);

	quad
	FlipQuad(const quad& q);
};