// Convex Hull of vector combinations
#pragma once

//true dependancies
#include <iostream>
#include <vector>
#include <list>
#include <map>
#include "FVector.h"

class CnvH {
public:
	CnvH() :state(empty) {};						// Default constructor
	CnvH(FVector const* p_arr, int _size);			// Array constructor

	void Add(FVector extrusion);
	// .obj return functions
	std::string GetPointStr();
	std::string GetQuadStr();

private:
	enum geometry { empty, linear, planar, volume };
	geometry state;

	std::vector<FVector> collection;		// Collection of references TODO: set template class container

	struct point {
		FVector vec;
		std::map<int, float> weight;

		point() : vec(FV_ZERO) {};
		inline std::ostream& operator<<(std::ostream& ostr) {
			ostr << "v " << vec.x << " " << vec.y << " " << vec.z;
			return ostr;
		}
	};
	std::vector<point> hullPoints;			// Points of the Convex Hull (MUST BE INDEXED CONTAINER!)

	struct quad {
		size_t pointIdx[4];
		FVector normal;
		inline std::ostream& operator<<(std::ostream& ostr) {
			ostr << "v ";
			for (int i = 0; i < 4; i++) {
				ostr << pointIdx[i];
				if (i < 3) ostr << " ";
			}
			return ostr;
		}
	};
	std::vector<quad> hullQuads;			// Quads of the Convex Hull

	struct edge{
		size_t pointIdx[2];
	};

	std::list<edge> FindOpenEdges(const std::list<quad*>& quadArr);
	quad BuildQuad(edge e1, edge e2, FVector dist);
	quad FlipQuad(const quad& q);
	friend bool operator==(const CnvH::edge& A, const CnvH::edge& B);
};