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
	CnvH();
	CnvH(FVector const* p_arr, int _size);

	void Add(FVector extrusion);
	void Disolve(FVector V);
	// .obj return functions
	friend std::ofstream& operator<<(std::ofstream& ostr, const CnvH& hull);

private:
	enum geometry { empty, linear, planar, volume };
	geometry state;

	std::vector<FVector> collection;		// Collection of references TODO: set template class container

	struct point {
		FVector vec;
		std::map<size_t, float> weight;

		point(FVector _vec = FV_ZERO) : vec(_vec) {};
		point& operator *= (float a) { for (auto w : weight) w.second *= a;	return *this; };
	};
	std::vector<point> hullPoints;			// Points of the Convex Hull (MUST BE INDEXED CONTAINER!)

	struct quad {
		const CnvH* parent;
		size_t pointIdx[4];
		FVector normal;

		quad(size_t a, size_t b, size_t c, size_t d, FVector _norm, CnvH* _parent) : normal(_norm) {
			pointIdx[0] = a; pointIdx[1] = b; pointIdx[2] = c; pointIdx[3] = d;
		}
		const point& getPnt(int i) const { return parent->hullPoints[pointIdx[i]]; };
	};
	std::vector<quad> hullQuads;			// Quads of the Convex Hull

	struct edge{
		size_t pointIdx[2];
	};

	std::list<edge> FindOpenEdges(std::list<quad*>& quadArr);
	quad BuildQuad(edge e1, edge e2, FVector dist);
	quad FlipQuad(const quad& q);
	friend bool operator==(const CnvH::edge& A, const CnvH::edge& B);
	friend point operator*(float A, const point& B);
	friend point operator*(const point& A, float B);
	friend point operator+(const point& A, const point& B);
	friend point operator-(const point& A, const point& B);
};