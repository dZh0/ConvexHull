// debug dependencies
#include <iostream>
#include <assert.h>
#include <queue>
#include <unordered_set>
// true dependencies
#include "CnvH.h"

CnvH::CnvH(void) {
	state = empty;
}
/*
CnvH::CnvH(void){
	state = volume;
	// Test cube 2 by 2 by 2
	hullPoints.push_back(point({  1.0f, -1.0f, -1.0f }, 0));
	hullPoints.push_back(point({  1.0f,  1.0f, -1.0f }, 1));
	hullPoints.push_back(point({ -1.0f,  1.0f, -1.0f }, 2));
	hullPoints.push_back(point({ -1.0f, -1.0f, -1.0f }, 3));
	hullPoints.push_back(point({  1.0f, -1.0f,  1.0f }, 4));
	hullPoints.push_back(point({  1.0f,  1.0f,  1.0f }, 5));
	hullPoints.push_back(point({ -1.0f,  1.0f,  1.0f }, 6));
	hullPoints.push_back(point({ -1.0f, -1.0f,  1.0f }, 7));

	hullQuads.push_back(quad{{ 0, 3, 2, 1 }, {  0.0f,  0.0f, -1.0f }});
	hullQuads.push_back(quad{{ 4, 5, 6, 7 }, {  0.0f,  0.0f,  1.0f }});
	hullQuads.push_back(quad{{ 2, 3, 7, 6 }, { -1.0f,  0.0f,  0.0f }});
	hullQuads.push_back(quad{{ 0, 1, 5, 4 }, {  1.0f,  0.0f,  0.0f }});
	hullQuads.push_back(quad{{ 0, 4, 7, 3 }, {  0.0f, -1.0f,  0.0f }});
	hullQuads.push_back(quad{{ 1, 2, 6, 5 }, {  0.0f,  1.0f,  0.0f }});
}
*/
CnvH::CnvH(FVector const* p_arr, int _size) : state(empty){
	collection.reserve(_size);
	for (int i = 0; i < _size; i++) {
		Add(p_arr[i], i);
	}
}

void CnvH::Add(FVector extrusion, int collectionIdx) {
	collection.push_back(extrusion);
	if (extrusion == FV_ZERO) return;		// Don't change geometry if vector is {0,0,0}
	
	std::queue<point> newPoints;			// Points to be added during the extrusion (FiFo)
	std::queue<quad> newQuads;				// Quads to be added during the extrusion (FiFo)
	std::unordered_set<size_t> moveIdx;		// Unique indecis of points to be moved during the extrusion
	size_t newIdx = hullPoints.size();		// Keeps track of the index of the newly added points

	switch (state) {
	/////////////////////////////////
	//    Convex Hull is empty:    //
	/////////////////////////////////
	case empty: {
		std::cout << "Empty" << std::endl;
		state = linear;
		newPoints.emplace();
		newIdx++;
		newPoints.emplace();
		moveIdx.insert(newIdx);
	} break;	//end "case empty"

	/////////////////////////////////
	//   Convex Hull is a line:    //
	/////////////////////////////////
	case linear: {
		FVector hullLine = hullPoints[1].vec;

		if (isColinear(extrusion, hullLine)) {	// If the extrusion direction is on the hull line:
			state = linear;
			std::cout << "Linear extrusion - " << extrusion << std::endl;

			// Provide the index of the furthest point on the hullLine in the extrude direction
			size_t endIdx = 0;
			float endPrj = dot(hullPoints[endIdx].vec, hullLine);
			for (size_t i = 1; i < hullPoints.size(); i++){
				if (dot(hullPoints[i].vec, hullLine) < endPrj){
					endIdx = i;
					endPrj = dot(hullPoints[endIdx].vec, hullLine);
				}
			}
			// If the selected point IS the origin: clone it and mark the cloned point for translation
			if (hullPoints[endIdx].vec == FV_ZERO) {
				newPoints.emplace();
				moveIdx.insert(newIdx);
			}
			// If the selected point is NOT the origin: mark it for translation
			else {
				moveIdx.insert(endIdx);
			}
		}
		else {	// If the extrusion direction is NOT on the hull line:
			state = planar;
			std::cout << "Line to plane extrusion - " << extrusion << std::endl;

			// Build edge_1 from existing points (not a loop)
			std::list<size_t> edge_1 = { 0 };
			for (size_t i = 1; i < hullPoints.size(); i++){
				float iProj = dot(hullPoints[i].vec, hullLine);
				auto it = edge_1.begin();
				while (iProj < dot(hullPoints[*it].vec, hullLine) && it != edge_1.end()){
					++it;
				}
				edge_1.insert(it, i);
			}

			// Clone the points from edge_1 to edge 2 and mark them for move
			std::list<size_t> edge_2;
			for (size_t i : edge_1) {
				edge_2.push_back(newIdx);
				newPoints.push(hullPoints[i]);
				moveIdx.insert(newIdx);
				newIdx++;
			}

			// Build quads betwean edge_1 and edge_2
			for (auto it_1 = edge_1.begin(), it_2 = edge_2.begin(); next(it_1) != edge_1.end(); ++it_1, ++it_2) {
				FVector edgeVec = hullPoints[*next(it_1)].vec - hullPoints[*it_1].vec;
				FVector normal = norm(cross(edgeVec, extrusion));
				newQuads.push(quad{ *it_1, *next(it_1), *next(it_2), *it_2, normal });
			}
			
		}
	} break;	//end "case linear"

	/////////////////////////////////
	//   Convex Hull is a plane:   //
	/////////////////////////////////
	case planar: {
		FVector hullNormal = hullQuads[0].normal;

		// Select all quads:
		std::list<quad*> selectQuads;
		for (quad& q : hullQuads) {
			selectQuads.push_back(&q);
		}

		// Find open edges:
		std::list<edge> edge_1 = FindOpenEdges(selectQuads);

		if (dot(extrusion, hullNormal) == 0.0f) {	// If the extrusion direction IS on the hull plane:
			state = planar;
			std::cout << "Planar extrude - " << extrusion << std::endl;

			// Delete edges that are not facing the extrusion direction (no longer loop)
			FVector direction = cross(extrusion, hullNormal);
			for (auto it = edge_1.begin(); it != edge_1.end();) {
				FVector edgeVec = hullPoints[it->endIdx].vec - hullPoints[it->startIdx].vec;
				if (dot(edgeVec, direction) <= 0.0f) {
					it = edge_1.erase(it);
				}
				else {
					++it;
				}
			}

			// Clone the points from edge_1 to edge 2 and mark them for move
			std::map<size_t,size_t> indexMap; // Keeps map of indecies from edge_1 to edge_2
			std::list<edge> edge_2;
			for (const edge& e : edge_1) {

			}

			// Build quads betwean edge_1 and edge_2
			for (auto it_1 = edge_1.begin(), it_2 = edge_2.begin(); it_1 != edge_1.end(); ++it_1, ++it_2) {
				newQuads.push(BuildQuad(*it_1, *it_2, extrusion));
			}

		}
		else {	// If the extrusion direction is NOT on the hull plane:
			state = volume;
			std::cout << "Plane to volume extrude - " << extrusion << std::endl;

			// Build edge_2 edge with new point indecies
			std::vector<edge> edge_2 = edge_1;
			for (edge& e : edge_2) {
				e.startIdx += newIdx;
				e.endIdx += newIdx;
			}

			// Clone all points and mark them for moving
			moveIdx.reserve(hullPoints.size());
			for (const point& p : hullPoints) {
				newPoints.push(p);
				moveIdx.insert(newIdx);
				newIdx++;
			}

			// Clone all quads of the hull, flip their normals and rebind their indecies
			bool isFacing = dot(extrusion, hullNormal) > 0.0f;	// if the hull IS facing the direction
			for (quad& q : hullQuads) {
				quad newQ = q;
				if (isFacing) { q = FlipQuad(q); }	// Flip the original quad
				else { newQ = FlipQuad(newQ);	}	// Flip the new quad
				for (int i = 0; i < 4; i++) {
					newQ.pointIdx[i] += newIdx;
				}
				newQuads.push(newQ);
			}

			// Build quads betwean edge_1 and edge_2
			for (auto it_1 = edge_1.begin(), it_2 = edge_2.begin(); it_1 != edge_1.end(); ++it_1, ++it_2) {		//ASK: it + 1 != edges.end();
				newQuads.push(BuildQuad(*it_1, *it_2, extrusion));
			}
		}
	} break;

	/////////////////////////////////
	// Convex Hull is a 3D volume: //
	/////////////////////////////////
	case volume: {
		std::cout << "Volume extrude - " << extrusion << std::endl;

		//Select quads facing the extrusion direction
		std::vector<quad*> selectQuads;
		for (quad& q : hullQuads) {
			if (dot(extrusion, q.normal) >= 0.0f) {	// if quad IS facing the direction
				selectQuads.push_back(&q);
			}
		}

		// Find open edges:
		std::vector<edge> edge_1 = FindOpenEdges(selectQuads);

		// Build edge_2 edge with new point indecies
		std::vector<edge> edge_2 = edge_1;
		for (edge& e : edge_2) {
			e.startIdx += newIdx;
			e.endIdx += newIdx;
		}

		// Build quads betwean edge_1 and edge_2
		for (auto it_1 = edge_1.begin(), it_2 = edge_2.begin(); it_1 != edge_1.end(); ++it_1, ++it_2) {
			newQuads.push(BuildQuad(*it_1, *it_2, extrusion));
		}

		// make new points and re-bind facing quads to them
		for (auto it_1 = edge_1.begin(), it_2 = edge_2.begin(); it_1 != edge_1.end(); ++it_1, ++it_2) {
			newQuads.push(BuildQuad(*it_1, *it_2, extrusion));
			size_t oldIdx = it_1->startIdx;
			point newPnt = hullPoints[oldIdx];		// duplicate point
			hullPoints.push_back(newPnt);
			for (quad* ptr_q : selectQuads) {		//re-bind quads
				for (int i = 0; i < 4; i++)
					if (ptr_q->pointIdx[i] == oldIdx) {
						ptr_q->pointIdx[i] = newIdx;
					}
			}
			newIdx++;
		}

	}break;	//end "case volume"

	default:
		assert("Convex Hull state not properly defined!");
		break;
	}
	// Add new points
	hullPoints.reserve(hullPoints.size() + newPoints.size());
	while (!newPoints.empty()) {
		hullPoints.push_back(newPoints.front());
		newPoints.pop();
	}
	// Add new quads
	hullQuads.reserve(hullQuads.size() + newQuads.size());
	while (!newQuads.empty()) {
		hullQuads.push_back(newQuads.front());
	}
	// Move points
	if (!moveIdx.empty()) {
		for (const size_t idx: moveIdx) {
			hullPoints[idx].vec += extrusion;
			auto insertRes = hullPoints[idx].weight.insert({ collectionIdx, 1.0f });
			if (!insertRes.second) {		// inserion failed
				insertRes.first->second += 1.0f;
			}
		}
	}
}
/*
// Finds the open edges in selection of quads.
std::list<CnvH::edge> CnvH::FindOpenEdges(const std::list<quad*>& quadArray) {
	std::list<edge> openEdges;				// TODO: find optimal container for find() and erase()
	for (const quad* ptr_q : quadArray) {
		for (int i = 0; i < 4; i++) {
			int j = (i < 3) ? j = i + 1 : j = i - 3;
			edge trueEdge = { ptr_q->pointIdx[i], ptr_q->pointIdx[j] };
			edge revEdge = { ptr_q->pointIdx[j], ptr_q->pointIdx[i] };
			auto it_found = std::find(openEdges.begin(), openEdges.end(), revEdge);
			if (it_found == openEdges.end()) {
				openEdges.push_back(trueEdge);
			}
			else {
				openEdges.erase(it_found);
			}
		}
	}
	return openEdges;
}
*/
// Finds the open edges in selection of quads.
std::list<CnvH::edge> CnvH::FindOpenEdges(const std::list<quad*>& quadArr) {
	std::list<edge> openEdges;				// TODO: find optimal container for find() and erase()
	for (const quad* ptr_q : quadArr) {
		for (int i = 0; i < 4; i++) {
			int j = (i < 3) ? j = i + 1 : j = i - 3;
			edge trueEdge = { ptr_q->pointIdx[i], ptr_q->pointIdx[j] };
			edge revEdge = { ptr_q->pointIdx[j], ptr_q->pointIdx[i] };
			auto it_found = std::find(openEdges.begin(), openEdges.end(), revEdge);
			if (it_found == openEdges.end()) {
				openEdges.push_back(trueEdge);
			}
			else {
				openEdges.erase(it_found);
			}
		}
	}
	return openEdges;
}

// Sorts a list of edges and returns an ordered list of indices
std::list<size_t> CnvH::SortEdges(std::list<edge>& edgeArr) {
	// Find the first edge 
	// startIdx of the edge must not be endIdx of any other edge)
	bool isFound = true;
	auto it_first = edgeArr.begin();
	while (it_first != edgeArr.end() && isFound) {
		isFound = false;
		auto it_seek = edgeArr.begin();
		while (it_seek != edgeArr.end() && !isFound) {
			if (it_seek->endIdx == it_first->startIdx) {
				isFound = true;
			}
			++it_seek;
		}
	}
	// Sort the rest of the edges and fill the list of indexes
	if (it_first != edgeArr.begin() && it_first != edgeArr.end()) {
		std::iter_swap(edgeArr.begin(), it_first);
	}
	auto it_sorted = edgeArr.begin();
	std::list<size_t> idxList;
	idxList.push_back(it_sorted->startIdx);
	idxList.push_back(it_sorted->endIdx);
	while (it_sorted != prev(edgeArr.end())) {
		auto it_seek = next(it_sorted);
		while (it_sorted->endIdx != it_seek->startIdx) {
			++it_seek;
		}
		idxList.push_back(it_seek->endIdx);
		if (it_seek != next(it_sorted)) {
			std::iter_swap(next(it_sorted), it_seek);
		}
		++it_sorted;
	}
	if (edgeArr.front().startIdx == edgeArr.back().endIdx) { //If the loop is closed
		idxList.push_back(edgeArr.front().startIdx);
	}
	return idxList;
}

// Builds a quad between 2 edges
CnvH::quad CnvH::BuildQuad(	edge e1, edge e2, FVector dist )
{
	size_t idx[4] = {
		e1.startIdx,
		e1.endIdx,
		e2.endIdx,
		e2.startIdx
	};
	FVector vec = hullPoints[idx[1]].vec - hullPoints[idx[0]].vec;
	FVector normal = norm(cross(vec, dist));
	return quad{ { idx[0], idx[1], idx[2], idx[3] }, normal };
}

// Returns a quad with reverse normal
CnvH::quad CnvH::FlipQuad(const quad& q) {
	size_t idx[4] = {
		q.pointIdx[0],
		q.pointIdx[3],
		q.pointIdx[2],
		q.pointIdx[1],
	};
	FVector normal = q.normal * -1.0f;
	return quad{ { idx[0], idx[1], idx[2], idx[3] }, normal };
}