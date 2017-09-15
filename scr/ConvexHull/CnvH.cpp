// debug dependencies
#ifndef _IOSTREAM_
#include <iostream>
#endif
#include <assert.h>
// true dependencies
#include <algorithm>
#include "CnvH.h"

CnvH::point operator+(const CnvH::point& A, const CnvH::point& B);
void sortEdges(std::vector<std::pair<size_t, size_t>>& arr);

CnvH::CnvH(void){
	state = volume;
	/* Test cube 2 by 2 by 2 */
	points.push_back(point({  1.0f, -1.0f, -1.0f }, 0));
	points.push_back(point({  1.0f,  1.0f, -1.0f }, 1));
	points.push_back(point({ -1.0f,  1.0f, -1.0f }, 2));
	points.push_back(point({ -1.0f, -1.0f, -1.0f }, 3));
	points.push_back(point({  1.0f, -1.0f,  1.0f }, 4));
	points.push_back(point({  1.0f,  1.0f,  1.0f }, 5));
	points.push_back(point({ -1.0f,  1.0f,  1.0f }, 6));
	points.push_back(point({ -1.0f, -1.0f,  1.0f }, 7));

	quads.push_back(quad{{ 0, 3, 2, 1 }, {  0.0f,  0.0f, -1.0f }});
	quads.push_back(quad{{ 4, 5, 6, 7 }, {  0.0f,  0.0f,  1.0f }});
	quads.push_back(quad{{ 2, 3, 7, 6 }, { -1.0f,  0.0f,  0.0f }});
	quads.push_back(quad{{ 0, 1, 5, 4 }, {  1.0f,  0.0f,  0.0f }});
	quads.push_back(quad{{ 0, 4, 7, 3 }, {  0.0f, -1.0f,  0.0f }});
	quads.push_back(quad{{ 1, 2, 6, 5 }, {  0.0f,  1.0f,  0.0f }});
}

CnvH::CnvH(FVector const* arr, int _size) : state(empty){
	collection.reserve(_size);
	for (int i = 0; i < _size; i++)
		add(arr + i, i);
}

void CnvH::add(FVector const* p_vec, const int _idx) {
	collection.push_back(p_vec);
	if (*p_vec == FV_ZERO) return; // Don't change geometry if vector is {0,0,0}
	FVector extrusion = *p_vec;
	switch (state) {

	/////////////////////////////////
	//    Convex Hull is empty:    //
	/////////////////////////////////
	case empty: {
		std::cout << "Empty" << std::endl;
		state = linear;
		points.push_back(point());
		points.push_back(point(extrusion, _idx));
	} break;

	/////////////////////////////////
	//   Convex Hull is a line:    //
	/////////////////////////////////
	case linear: {
		std::cout << "Linear" << std::endl;
/*
		auto it = points.rbegin();
		if (isColinear(extrusion, it->vec)) {						// if new point IS on the same line
			auto it_selected = it;
			float direction = dot(extrusion, it_selected->vec);
			while (it < points.rend()){
				float newDirection = dot(extrusion, it->vec);
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
			// TODO: Clone vertices
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
			// TODO: Clone vertices
			// TODO: Build quads
			// TODO: Move
		}
*/
	} break;

	/////////////////////////////////
	// Convex Hull is a 3D volume: //
	/////////////////////////////////
	case volume: {
		std::cout << "Volume extrude - " << extrusion << std::endl;

		/* get quads facing the extrusion direction */
		std::vector<quad> facingQuads;
		for (const quad& qua : quads) {
			if (dot(extrusion, qua.normal) >= 0.0f) {				// if quad IS facing the direction
				facingQuads.push_back(qua);
			}
		}

		/* find open edges */
		std::vector<std::pair<size_t, size_t>> edges;				// TODO: find optimal container for find() and erase()
		for (const quad& qua : facingQuads) {
			for (int i = 0; i < 4; i++){
				int j = (i + 1 < 4) ? j = i + 1 : j = i - 3;
				std::pair<size_t, size_t> edge = { qua.pointIdx[i], qua.pointIdx[j] };
				std::pair<size_t, size_t> edge_reverse = { qua.pointIdx[j], qua.pointIdx[i] };
				auto it_found = std::find(edges.begin(), edges.end(), edge_reverse);
				if (it_found == edges.end()){
					edges.push_back(edge);
				}
				else{
					edges.erase(it_found);
				}
			}
		}
		sortEdges(edges);
		bool isEdgeLoop = edges.front.first == edges.back.second;

		/* make new points and re-bind facing quads to them */
		for (auto it = edges.begin(); it != edges.end(); ++it) {
			size_t oldIdx = it->first;
			size_t newIdx = points.size();
			it->second = newIdx;
			point newPoint = points[oldIdx];
			points.push_back(newPoint);
			for (quad& qua : facingQuads)
				for (int i = 0; i < 4; i++)
					if (qua.pointIdx[i] == oldIdx){
						qua.pointIdx[i] = newIdx;
					}
		}

		/* make edge quads */
		quads.reserve(quads.size() + edges.size());
		for (auto it = edges.begin(); it + 1 != edges.end(); ++it){		//ASK: it + 1 != edges.end();
			size_t
				p1_idx = it->first,
				p2_idx = (it + 1)->first,
				p3_idx = (it + 1)->second,
				p4_idx = it->second;
			FVector begVec = points[p1_idx].vec;
			FVector endVec = points[p2_idx].vec;
			FVector normal = norm(cross(endVec - begVec, extrusion));

			quads.push_back(quad{ { p1_idx, p2_idx, p3_idx, p4_idx }, normal });
		}

	}break;	//end "case volume"

	default:
		assert("Convex Hull state not properly defined!");
		break;
	}
}

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

void sortEdges(std::vector<std::pair<size_t, size_t>>& edgeArray){
	assert(!edgeArray.empty());
	for (
		auto it = edgeArray.begin(), it_next = next(it); it_next != edgeArray.end(); ++it, ++it_next){
		if (it->second != it_next->first){
			for (auto it_seek = next(it_next); it_seek != edgeArray.end(); ++it_seek){
				if (it->second == it_seek->first){
					std::iter_swap(it_next, it_seek);
					break;
				}
			}
		}	
	}
}