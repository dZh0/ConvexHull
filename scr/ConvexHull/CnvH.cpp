// debug dependancies
#ifndef _IOSTREAM_
#include <iostream>
#endif
#include <assert.h>
// true dependancies
#include <algorithm>
#include <unordered_set>
#include "CnvH.h"

CnvH::point operator+(const CnvH::point& A, const CnvH::point& B);


/* TODO: Revert beack default constructor after tests */
CnvH::CnvH(void) {
	state = volume;
	point A;	A.vec = { 1.0f,-1.0f,-1.0f };	points.push_back(A);
	point B;	B.vec = { 1.0f, 1.0f,-1.0f };	points.push_back(B);
	point C;	C.vec = {-1.0f, 1.0f,-1.0f };	points.push_back(C);
	point D;	D.vec = {-1.0f,-1.0f,-1.0f };	points.push_back(D);
	point E;	E.vec = { 1.0f,-1.0f, 1.0f };	points.push_back(E);
	point F;	F.vec = { 1.0f, 1.0f, 1.0f };	points.push_back(F);
	point G;	G.vec = {-1.0f, 1.0f, 1.0f };	points.push_back(G);
	point H;	H.vec = {-1.0f,-1.0f, 1.0f };	points.push_back(H);

	quad q1 = { { 0, 3, 2, 1 } , { 0.0f, 0.0f,-1.0f } };	quads.push_back(q1);
	quad q2 = { { 4, 5, 6, 7 } , { 0.0f, 0.0f, 1.0f } };	quads.push_back(q2);
	quad q3 = { { 2, 3, 7, 6 } , {-1.0f, 0.0f, 0.0f } };	quads.push_back(q3);
	quad q4 = { { 0, 1, 5, 4 } , { 1.0f, 0.0f, 0.0f } };	quads.push_back(q4);
	quad q5 = { { 0, 4, 7, 3 } , { 0.0f,-1.0f, 0.0f } };	quads.push_back(q5);
	quad q6 = { { 1, 2, 6, 5 } , { 0.0f, 1.0f, 0.0f } };	quads.push_back(q6);
}

CnvH::CnvH(FVector const* arr, int _size) : state(empty){
	collection.reserve(_size);
	for (int i = 0; i < _size; i++)
		add(arr + i, i);
}

void CnvH::add(FVector const* p_vec, const int idx) {
	collection.push_back(p_vec);
	if (*p_vec == FV_ZERO) return; // Dont change geometry if vector is {0,0,0}
	point newPnt(*p_vec, idx);
	switch (state) {

	/////////////////////////////////
	//    Convex Hull is empty:    //
	/////////////////////////////////
	case empty: {
		std::cout << "Empty" << std::endl;
/*
		state = linear;
		points.push_back(point());
		points.push_back(newPnt);
*/
	} break;

	/////////////////////////////////
	//   Convex Hull is a line:    //
	/////////////////////////////////
	case linear: {
		std::cout << "Linear" << std::endl;
/*
		auto it = points.rbegin();
		if (isColinear(newPnt.vec, it->vec)) {						// if new point IS on the same line
			auto it_selected = it;
			float direction = dot(newPnt.vec, it_selected->vec);
			while (it < points.rend()){
				float newDirection = dot(newPnt.vec, it->vec);
				if (newDirection > direction){
					direction = newDirection;
					it_selected = it;
				}
				++it;
			}
			if (it_selected->vec == FV_ZERO)	points.push_back(newPnt);
			else *it_selected = *it_selected + newPnt;
		}
		else {													// if new point is NOT on the same line
			size_t size = points.size();
			// TODO: Build quads
			state = planar;
		}
*/
	} break;

	/////////////////////////////////
	//   Convex Hull is a plane:   //
	/////////////////////////////////
	case planar: {
		std::cout << "Planar" << std::endl;
/*
		state = volume;
		if (cross(newPnt.vec, quads[0].normal) == FV_ZERO) {	// if new point IS in the same plane
			state = volume;
			// TODO: Select edge
			// TODO: Clone verecies
			// TODO: Build quads
			// TODO: Move
		}
		else {													// if new point is NOT in the same plane
			state = volume;
			std::cout << "Plane to volume extrude - " << newPnt.vec << std::endl;

			std::vector<quad> facingQuads;
			std::unordered_set<size_t> facingPoints;

			std::vector<quad> nonFacingQuads;
			std::unordered_set<size_t> nonFacingPoints;

			// TODO: Flip Normals
			// TODO: Select edge
			// TODO: Clone verecies
			// TODO: Build quads
			// TODO: Move
		}
*/
	} break;

	/////////////////////////////////
	// Convex Hull is a 3D volume: //
	/////////////////////////////////
	case volume: {
		std::cout << "Volume extrude - " << newPnt.vec << std::endl;

		// split quads and point ids in facing and non-facing containers based on the quads' normal and the given point newPnt

		std::vector<quad> facingQuads;
		std::unordered_set<size_t> facingPoints;

		std::vector<quad> nonFacingQuads;
		std::unordered_set<size_t> nonFacingPoints;

		for (const quad& qua : quads) {
			std::vector<quad> *p_quads;
			std::unordered_set<size_t> *p_points;
			// if quad IS facing the direction: Select facing quad and point id containers
			if (dot(newPnt.vec, qua.normal) >= 0.0f) {
				p_quads = &facingQuads;
				p_points = &facingPoints;
			}
			// if quad is NOT facing the direction: Select non-facing quad and point id containers
			else {
				p_quads = &nonFacingQuads;
				p_points = &nonFacingPoints;
			}
			// fill selected containers
			p_quads->push_back(qua);
			for (const size_t& i : qua.pointIdx)
				p_points->insert(i);
		}

		// create edge point and move point containers from facing and non-facing points

		std::unordered_set<size_t>& movePoints = facingPoints;		// ids of points which will be MOVED during extrusion
		std::unordered_set<size_t>& edgePoints = nonFacingPoints;	// ids of points which will be CLONED during extrusion

		/* if point id is ONLY in edge container: Remove point id from edge container */
		for (auto it_edge = edgePoints.begin(); it_edge != edgePoints.end();) {
			if (movePoints.find(*it_edge) == movePoints.end()) it_edge = edgePoints.erase(it_edge);
			else ++it_edge;
		}

		/* if point id is in edge container: Remove point id from move container */
		for (auto it_move = movePoints.begin(); it_move != movePoints.end();) {
			if (edgePoints.find(*it_move) != edgePoints.end()) it_move = movePoints.erase(it_move);
			else ++it_move;
		}

		/* order the edge point in a loop container so each point MUST share a quad with the next point in the container */

		std::unordered_set<size_t> edgeLoop;
		assert(!edgePoints.empty());
		size_t loopPoint = *edgePoints.begin();

		while (edgeLoop.size() < edgePoints.size()) {
			for (const quad& qua : facingQuads){
				for (int i = 0; i < 4; i++){
					if (qua.pointIdx[i] == loopPoint) {
						size_t nextIdx = (i + 1 < 4) ? i + 1 : i - 3;
						while (edgePoints.find(qua.pointIdx[nextIdx]) != edgePoints.end()) {
							loopPoint = qua.pointIdx[nextIdx];
							edgeLoop.insert(loopPoint);
							nextIdx = (nextIdx + 1 < 4) ? nextIdx + 1 : nextIdx - 3;
						}
					}
				}
			}
		}

		/* create extrusion points */

		std::unordered_set<size_t> newEdgeLoop;
		for (auto it_edge = edgeLoop.begin(); it_edge != edgeLoop.end(); ++it_edge) {
			size_t edgeIdx = *it_edge;
			size_t newIdx = points.size();
			point pnt = points[edgeIdx];

			/* duplicate point and add its index to the new edge loop and the move container */
			points.push_back(pnt);
			movePoints.insert(newIdx);
			newEdgeLoop.insert(newIdx);

			/* rebind facing quads' edge points to the new edge loop */
			for (quad& qua : facingQuads)
				for (int i = 0; i < 4; i++)
					if (qua.pointIdx[i] == edgeIdx) qua.pointIdx[i] = newIdx;
		}

		/* create edge quads from the old and the new edge points */

		std::vector<quad> edgeQuads;
		edgeQuads.reserve(edgeLoop.size());

		// 4 point iterators
		auto it_0 = edgeLoop.begin();			// newEdgeLoop	----- it_3 <-- it_2 -----
		auto it_1 = std::next(it_0);			//				        |        ^
		auto it_3 = newEdgeLoop.begin();		//				        v        |
		auto it_2 = std::next(it_3);			// edgeLoop		----- it_0 --> it_1 -----

		while (it_0 != edgeLoop.end() && it_3 != newEdgeLoop.end()) {
			// TODO normals are undefined as both points are on the same spot
			FVector A = points[*it_1].vec - points[*it_0].vec;
			FVector B = points[*it_2].vec - points[*it_0].vec;
			FVector normal = norm(cross(A, B));
			quad qua = { { *it_0, *it_1, *it_2, *it_3 }, normal };
			edgeQuads.push_back(qua);
			{ // next iterators
				++it_0;
				++it_1;
				++it_3;
				++it_2;
				if (it_1 == edgeLoop.end() || it_2 == newEdgeLoop.end()) {
					it_1 = edgeLoop.begin();
					it_2 = newEdgeLoop.begin();
				}
			}
		}

		// Replace CnvH.quads container

		quads.clear();
		quads.reserve(facingQuads.size() + nonFacingQuads.size() + edgeQuads.size());
		quads.insert(quads.end(),	facingQuads.begin(),	facingQuads.end());
		quads.insert(quads.end(),	nonFacingQuads.begin(),	nonFacingQuads.end());
		quads.insert(quads.end(),	edgeQuads.begin(),		edgeQuads.end());

		// Move the points in move container based on newPnt

		for (auto it_mov = movePoints.begin(); it_mov != movePoints.end(); ++it_mov)
			points[*it_mov] = points[*it_mov] + newPnt;

	}break;	//end "case volume"
	default:
		assert("Convex Hull state not properly defined!");
		break;
	}
}

/////////////////////////////////
// Point operations
/////////////////////////////////
CnvH::point operator+(const CnvH::point& A, const CnvH::point& B) {
	CnvH::point result;
	result.vec = A.vec + B.vec;
	result.weight.insert(A.weight.begin(), A.weight.end());
	for (auto it = B.weight.begin(); it != B.weight.end(); ++it) {
		auto insert = result.weight.insert(*it);
		if (!insert.second)
			insert.first->second += it->second;
	}
	return result;
}