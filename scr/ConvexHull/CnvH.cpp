// debug dependencies
#include <iostream>
#include <assert.h>
// true dependencies
#include "CnvH.h"

typedef std::pair<size_t, size_t> edge;
void sortEdges(std::vector<edge>& arr);

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
		add(arr[i], i);
}

void CnvH::add(FVector _vec, int collectionIdx) {
	collection.push_back(_vec);
	if (_vec == FV_ZERO) return; // Don't change geometry if vector is {0,0,0}
	FVector extrusion = _vec;
	switch (state) {

	/////////////////////////////////
	//    Convex Hull is empty:    //
	/////////////////////////////////
	case empty: {
		std::cout << "Empty" << std::endl;
		state = linear;
		points.push_back(point());
		points.push_back(point(extrusion, collectionIdx));
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

		/* get pointers to quads facing the extrusion direction */
		std::vector<quad*> facingQuads;
		for (quad& qua : quads) {
			if (dot(extrusion, qua.normal) >= 0.0f) {				// if quad IS facing the direction
				facingQuads.push_back(&qua);
			}
		}

		/* find open edges */
		std::vector<edge> edges;				// TODO: find optimal container for find() and erase()
		for (const quad* qua : facingQuads) {
			for (int i = 0; i < 4; i++){
				int j = (i + 1 < 4) ? j = i + 1 : j = i - 3;
				edge trueEdge = { qua->pointIdx[i], qua->pointIdx[j] };
				edge revEdge = { qua->pointIdx[j], qua->pointIdx[i] };
				auto it_found = std::find(edges.begin(), edges.end(), revEdge);
				if (it_found == edges.end()){
					edges.push_back(trueEdge);
				}
				else{
					edges.erase(it_found);
				}
			}
		}
		sortEdges(edges);
		bool isEdgeLoop = edges.front().first == edges.back().second; // Is the edge a closed loop?

		/* make new points and re-bind facing quads to them */
		for (auto it = edges.begin(); it != edges.end(); ++it) {
			size_t oldIdx = it->first;
			size_t newIdx = points.size();
			it->second = newIdx;				// insert in edges
			point newPoint = points[oldIdx];	// duplicate point
			points.push_back(newPoint);
			for (quad* qua : facingQuads)		//re-bind quads
				for (int i = 0; i < 4; i++)
					if (qua->pointIdx[i] == oldIdx){
						qua->pointIdx[i] = newIdx;
					}
		}

		/* get pointers to points belonging to quads facing the extrusion direction */
		std::vector<point*> extrPoints;
		for (quad* const qua : facingQuads) {
			for (size_t idx : qua->pointIdx) {
				point* p_point = &points[idx];
				auto it_found = std::find(extrPoints.begin(), extrPoints.end(), p_point);
				if (it_found == extrPoints.end()) {
					extrPoints.push_back(p_point);		// add pointer only if unique
				}
			}
		}

		/* make quads along the edge */
		quads.reserve(quads.size() + edges.size());
		for (auto it = edges.begin(); next(it) != edges.end(); ++it){		//ASK: it + 1 != edges.end();
			size_t
				idx_1 =	it->first,
				idx_2 = next(it)->first,
				idx_3 = next(it)->second,
				idx_4 =	it->second;
			FVector vec = points[idx_2].vec - points[idx_1].vec;
			FVector normal = norm(cross(vec, extrusion));
			quads.push_back(quad{ { idx_1, idx_2, idx_3, idx_4 }, normal });	// !!! invalidates facingQuads
		}
		/* make a quad closing the loop */
		if (isEdgeLoop) {
			size_t
				idx_1 = edges.back().first,
				idx_2 = edges.front().first,
				idx_3 = edges.front().second,
				idx_4 = edges.back().second;
			FVector vec = points[idx_2].vec - points[idx_1].vec;
			FVector normal = norm(cross(vec, extrusion));
			quads.push_back(quad{ { idx_1, idx_2, idx_3, idx_4 }, normal });	// !!!! invalidates facingQuads
		}

		/* move points in the extrusion direction and add weight */
		for (point* p_point : extrPoints) {
			p_point->vec += extrusion;
			auto insResult = p_point->weight.insert({collectionIdx, 1.0f});
			if (!insResult.second) {		// inserion failed
				insResult.first->second += 1.0f;
			}
		}

	}break;	//end "case volume"

	default:
		assert("Convex Hull state not properly defined!");
		break;
	}
}
/*
point operator+(const point& A, const point& B) {
	point result;
	result.vec = A.vec + B.vec;
	result.weight.insert(A.weight.begin(), A.weight.end());
	for (auto it = B.weight.begin(); it != B.weight.end(); ++it) {
		auto insert = result.weight.insert(*it);
		if (!insert.second)
			insert.first->second += it->second;
	}
	return result;
}
*/

void sortEdges(std::vector<edge>& edgeArray){
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