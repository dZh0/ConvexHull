// debug dependencies
#include <iostream>
#include <assert.h>
// true dependencies
#include "CnvH.h"

CnvH::CnvH(void) {
	state = empty;
}

/*
CnvH::CnvH(void){
	state = volume;
	// Test cube 2 by 2 by 2
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
*/
CnvH::CnvH(FVector const* arr, int _size) : state(empty){
	collection.reserve(_size);
	for (int i = 0; i < _size; i++) {

		add(arr[i], i);
	}
}

void CnvH::add(const FVector extrusion, int collectionIdx) {
	collection.push_back(extrusion);
	if (extrusion == FV_ZERO) return;	// Don't change geometry if vector is {0,0,0}
	
	std::vector<point> NewPoints;		// Points to be added during the extrusion
	std::vector<quad> NewQuads;			// Quads to be added during the extrusion
	std::vector<size_t> MoveIdx;		// Indexes of points to be moved during the extrusion
	size_t IndexOffset = points.size();	// The first index of the new points
	size_t NewIdx = IndexOffset;		// Keeps track of the tindex of the NewPoints

	switch (state) {
	/////////////////////////////////
	//    Convex Hull is empty:    //
	/////////////////////////////////
	case empty: {
		std::cout << "Empty" << std::endl;
		state = linear;
		NewPoints.push_back(point());
		NewPoints.push_back(point());
		MoveIdx.push_back(++NewIdx);
	} break;

	/////////////////////////////////
	//   Convex Hull is a line:    //
	/////////////////////////////////
	case linear: {
		FVector hullLine = points[1].vec;

		if (isColinear(extrusion, hullLine)) {	// If the extrusion direction is on the hull line:
			state = linear;
			std::cout << "Linear extrusion - " << extrusion << std::endl;

			// Select by index the furthest point in the extrude direction:
			size_t selectedIdx = 0;
			float maxProjection = 0.0f;
			for (size_t i = 1; i < points.size(); i++) {
				float projection = dot(extrusion, points[i].vec);
				if (projection > maxProjection){
					maxProjection = projection;
					selectedIdx = i;
				}
			}
			// If the selected point IS the origin: clone it and mark it for translation
			if (points[selectedIdx].vec == FV_ZERO) {
				NewPoints.push_back(point());
				MoveIdx.push_back(IndexOffset);
			}
			// If the selected point is NOT the origin: mark it for translation
			else {
				MoveIdx.push_back(selectedIdx);
			}
		}
		else {	// If the extrusion direction is NOT on the hull line:
			state = planar;
			std::cout << "Line to plane extrusion - " << extrusion << std::endl;

			// Clone all points and mark them for moving
			NewPoints.reserve(points.size());
			MoveIdx.reserve(points.size());
			for (const point& p : points) {
				NewPoints.push_back(p);
				MoveIdx.push_back(NewIdx);
				NewIdx++;
			}

			// Build edge_1 from existing point indecies
			std::vector<edge> edge_1;
			edge_1.reserve(points.size());
			FVector lineDirection = points[1].vec - points[0].vec;
			for (size_t startId = 0; startId < points.size(); startId++) {
				size_t endId = startId;
				float minDist = 0.0f;
				bool minDistSet = false;
				for (size_t i = 0; i < points.size(); i++) {
					float distance = dot(points[i].vec, lineDirection) - dot(points[startId].vec, lineDirection);
					if (distance > 0.0f && ( distance < minDist || !minDistSet)) {
						endId = i;
						minDist = distance;
					}
				}
				if (startId != endId) {
					edge_1.push_back({startId,endId});
				}
			}

			// Build edge_2 edge with new point indecies
			std::vector<edge> edge_2 = edge_1;
			for (edge& e : edge_2) {
				e.first += IndexOffset;
				e.second += IndexOffset;
			}

			// Build quads betwean edge_1 and edge_2
			NewQuads.reserve(edge_1.size());
			for (auto it_1 = edge_1.begin(), it_2 = edge_2.begin(); it_1 != edge_1.end(); ++it_1, ++it_2) {		//ASK: it + 1 != edges.end();
				NewQuads.push_back(buildQuad(*it_1, *it_2, extrusion));
			}
		}
	} break;	//end "case linear"

	/////////////////////////////////
	//   Convex Hull is a plane:   //
	/////////////////////////////////
	case planar: {
		FVector hullNormal = quads[0].normal;

		if (dot(extrusion, hullNormal) == 0.0f) {	// If the extrusion direction IS on the hull plane:
			state = planar;
			std::cout << "Planar extrude - " << extrusion << std::endl;

			// Select all quads:
			std::vector<quad*> selectQuads;
			selectQuads.reserve(quads.size());
			for (quad& q : quads) {
				selectQuads.push_back(&q);
			}

			// Find open edges:
			std::vector<edge> edge_1 = findOpenEdges(selectQuads);

			// Delete edges that are not facing the extrusion direction
			FVector direction = cross(extrusion, hullNormal);
			for (auto it = edge_1.begin(); it != edge_1.end();) {
				FVector startPnt = points[it->first].vec;
				FVector endPnt = points[it->second].vec;
				FVector vecEdge = endPnt - startPnt;
				if (dot(vecEdge, direction) <= 0.0f) {
					it = edge_1.erase(it);
				}
				else {
					++it;
				}
			}

			// Build edge_2 edge with new point indecies
			std::vector<edge> edge_2 = edge_1;
			for (edge& e : edge_2) {
				e.first += IndexOffset;
				e.second += IndexOffset;
			}

			// Clone edge points of the hull and mark them for translation
			NewPoints.reserve(edge_1.size());
			for (const edge& e : edge_1) {
				NewPoints.push_back(points[e.first]);
				MoveIdx.push_back(NewIdx);
				NewIdx++;
			}
			// Clone the last point as edge_1 must be open (not a loop) TODO
			assert(edge_1.front().first != edge_1.back().second);
			NewPoints.push_back(points[edge_1.back().second]);
			MoveIdx.push_back(NewIdx);

			// Build quads betwean edge_1 and edge_2
			NewQuads.reserve(edge_1.size());
			for (auto it_1 = edge_1.begin(), it_2 = edge_2.begin(); it_1 != edge_1.end(); ++it_1, ++it_2) {		//ASK: it + 1 != edges.end();
				NewQuads.push_back(buildQuad(*it_1, *it_2, extrusion));
			}
		}
		else {	// If the extrusion direction is NOT on the hull plane:
			state = volume;
			std::cout << "Plane to volume extrude - " << extrusion << std::endl;

			// Select all quads:
			std::vector<quad*> selectQuads;
			selectQuads.reserve(quads.size());
			for (quad& q : quads) {
				selectQuads.push_back(&q);
			}

			// Find open edges:
			std::vector<edge> edge_1 = findOpenEdges(selectQuads);

			// Build edge_2 edge with new point indecies
			std::vector<edge> edge_2 = edge_1;
			for (edge& e : edge_2) {
				e.first += IndexOffset;
				e.second += IndexOffset;
			}

			// Clone all points and mark them for moving
			NewPoints.reserve(points.size());
			MoveIdx.reserve(points.size());
			for (const point& p : points) {
				NewPoints.push_back(p);
				MoveIdx.push_back(NewIdx);
				NewIdx++;
			}

			// Clone all quads of the hull, flip their normals and rebind their indecies
			NewQuads.reserve(quads.size() + edge_1.size());
			bool isFacing = dot(extrusion, hullNormal) > 0.0f;	// if the hull IS facing the direction
			for (quad& q : quads) {
				quad newQ = q;
				if (isFacing) { q = flipQuad(q); }	// Flip the original quad
				else { newQ = flipQuad(newQ);	}	// Flip the new quad
				for (int i = 0; i < 4; i++) {
					newQ.pointIdx[i] += IndexOffset;
				}
				NewQuads.push_back(newQ);
			}

			// Build quads betwean edge_1 and edge_2
			NewQuads.reserve(edge_1.size());
			for (auto it_1 = edge_1.begin(), it_2 = edge_2.begin(); it_1 != edge_1.end(); ++it_1, ++it_2) {		//ASK: it + 1 != edges.end();
				NewQuads.push_back(buildQuad(*it_1, *it_2, extrusion));
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
		for (quad& q : quads) {
			if (dot(extrusion, q.normal) >= 0.0f) {	// if quad IS facing the direction
				selectQuads.push_back(&q);
			}
		}

		// Find open edges:
		std::vector<edge> edge_1 = findOpenEdges(selectQuads);

		// Build edge_2 edge with new point indecies
		std::vector<edge> edge_2 = edge_1;
		for (edge& e : edge_2) {
			e.first += IndexOffset;
			e.second += IndexOffset;
		}

		// Build quads betwean edge_1 and edge_2
		NewQuads.reserve(edge_1.size());
		for (auto it_1 = edge_1.begin(), it_2 = edge_2.begin(); it_1 != edge_1.end(); ++it_1, ++it_2) {
			NewQuads.push_back(buildQuad(*it_1, *it_2, extrusion));
		}

		// make new points and re-bind facing quads to them
		NewPoints.reserve(edge_1.size());
		for (auto it_1 = edge_1.begin(), it_2 = edge_2.begin(); it_1 != edge_1.end(); ++it_1, ++it_2) {
			NewQuads.push_back(buildQuad(*it_1, *it_2, extrusion));
			size_t oldIdx = it_1->first;
			point newPnt = points[oldIdx];		// duplicate point
			points.push_back(newPnt);
			for (quad* ptr_q : selectQuads) {		//re-bind quads
				for (int i = 0; i < 4; i++)
					if (ptr_q->pointIdx[i] == oldIdx) {
						ptr_q->pointIdx[i] = NewIdx;
					}
			}
			NewIdx++;
		}

	}break;	//end "case volume"

	default:
		assert("Convex Hull state not properly defined!");
		break;
	}
	// Add new points
	if (!NewPoints.empty()) {
		points.reserve(points.size() + NewPoints.size());
		points.insert(points.end(), NewPoints.begin(), NewPoints.end());
	}
	// Add new quads
	if (!NewQuads.empty()) {
		quads.reserve(points.size() + NewQuads.size());
		quads.insert(quads.end(), NewQuads.begin(), NewQuads.end());
	}
	// Move points
	if (!MoveIdx.empty()) {
		for (const size_t idx: MoveIdx) {
			points[idx].vec += extrusion;
			auto insertRes = points[idx].weight.insert({ collectionIdx, 1.0f });
			if (!insertRes.second) {		// inserion failed
				insertRes.first->second += 1.0f;
			}
		}
	}
}

// Finds the open edges in selection of quads.
std::vector<CnvH::edge> CnvH::findOpenEdges(const std::vector<quad*>& quadArray) {
	std::vector<edge> openEdges;				// TODO: find optimal container for find() and erase()
	for (const quad* ptr_q : quadArray) {
		for (int i = 0; i < 4; i++) {
			int j = (i + 1 < 4) ? j = i + 1 : j = i - 3;
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

// Sorts edges.
void CnvH::sortEdges(std::vector<edge>& edgeArray){
	assert(!edgeArray.empty());
	for ( auto it = edgeArray.begin(), it_next = next(it); it_next != edgeArray.end(); ++it, ++it_next){
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

// Builds a quad between 2 edges
CnvH::quad CnvH::buildQuad(	edge e1, edge e2, FVector dist )
{
	size_t idx[4] = {
		e1.first,
		e1.second,
		e2.second,
		e2.first
	};
	FVector vec = points[idx[1]].vec - points[idx[0]].vec;
	FVector normal = norm(cross(vec, dist));
	return quad{ { idx[0], idx[1], idx[2], idx[3] }, normal }; // ASK: Can I place the array directly?
}

CnvH::quad CnvH::flipQuad(const quad& q) {
	size_t idx[4] = {
		q.pointIdx[0],
		q.pointIdx[3],
		q.pointIdx[2],
		q.pointIdx[1],
	};
	FVector normal = q.normal * -1.0f;
	return quad{ { idx[0], idx[1], idx[2], idx[3] }, normal };
}