// debug dependencies
#include <assert.h>
#include <sstream>
#include <queue>
#include <unordered_set>
// true dependencies
#include "CnvH.h"

// Creates CnvH from C-style FVector array.
CnvH::CnvH(FVector const* p_arr, int _size) : state(empty){
	collection.reserve(_size);
	for (int i = 0; i < _size; i++) {
		Add(p_arr[i]);
	}
}

// Adds a FVector to the hull.
void CnvH::Add(FVector extrusion) {
	collection.push_back(extrusion);
	if (extrusion == FV_ZERO) return;		// Don't change geometry if vector is {0,0,0}
	size_t collectionIdx;
	
	std::queue<point> newPoints;			// Points to be added during the extrusion (FiFo)
	std::queue<quad> newQuads;				// Quads to be added during the extrusion (FiFo)
	std::unordered_set<size_t> moveIdx;		// Unique indecis of points to be moved during the extrusion
	size_t newIdx = hullPoints.size();		// Keeps track of the index of the newly added points

	// Find if there is already a vector with the same direcion in colection
	auto found = collection.begin();
	while (found != collection.end()-1) {	// collection.end()-1 as we already added the extrusion in the collection
		if (isColinear(*found, extrusion) && dot(*found, extrusion) > 0.0f) {
			break;
		}
		++found;
	}
	if (found != collection.end()-1) {		// If vector in the same direction IS found in the collection
		collectionIdx = std::distance(collection.begin(), found);
		for (size_t i = 0; i < hullPoints.size(); i++) {
			const auto& map = hullPoints[i].weight;
			if (map.find(collectionIdx) != map.end()) {
				moveIdx.insert(i);
			}
		}
	}
	else {									// If vector in the same direction is NOT found in the collection
		collectionIdx = collection.size();
		switch (state) {
			/////////////////////////////////
			//    Convex Hull is empty:    //
			/////////////////////////////////
		case empty: {
			std::cout << "Add first point - " << extrusion << std::endl;
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
				for (size_t i = 1; i < hullPoints.size(); i++) {
					if (dot(hullPoints[i].vec, hullLine) < endPrj) {
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
				for (size_t i = 1; i < hullPoints.size(); i++) {
					float iProj = dot(hullPoints[i].vec, hullLine);
					auto it = edge_1.begin();
					while (iProj < dot(hullPoints[*it].vec, hullLine)) {
						++it;
						if (it == edge_1.end()) break;
					}
					edge_1.insert(it, i);
				}

				// Clone the points from edge_1 to edge 2 and mark them for move
				std::list<size_t> edge_2;
				moveIdx.reserve(edge_1.size());
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
					newQuads.push(quad{ { *it_1, *next(it_1), *next(it_2), *it_2 }, normal });
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
				FVector direction = cross(hullNormal, extrusion);
				for (auto it = edge_1.begin(); it != edge_1.end();) {
					FVector edgeVec = hullPoints[it->pointIdx[1]].vec - hullPoints[it->pointIdx[0]].vec;
					if (dot(edgeVec, direction) <= 0.0f) {
						it = edge_1.erase(it);
					}
					else {
						++it;
					}
				}

				// Clone the points from edge_1 to edge 2 and mark them for move
				std::list<edge> edge_2;
				moveIdx.reserve(edge_1.size() + 1);
				std::map<size_t, size_t> indexMap; // Keeps map of indecies from edge_1 to edge_2
				for (const edge& e : edge_1) {
					edge e2;
					for (int i = 0; i < 2; i++) {
						auto insResult = indexMap.insert({ e.pointIdx[i],newIdx });
						if (insResult.second) {
							e2.pointIdx[i] = newIdx;
							newPoints.push(hullPoints[e.pointIdx[i]]);
							moveIdx.insert(newIdx);
							newIdx++;
						}
						else {
							e2.pointIdx[i] = insResult.first->second;
						}
					}
					edge_2.push_back(e2);
				}

				// Build quads betwean edge_1 and edge_2
				for (auto it_1 = edge_1.begin(), it_2 = edge_2.begin(); it_1 != edge_1.end(); ++it_1, ++it_2) {
					quad q = BuildQuad(*it_1, *it_2, extrusion);
					newQuads.push(FlipQuad(q));
				}

			}
			else {	// If the extrusion direction is NOT on the hull plane:
				state = volume;
				std::cout << "Plane to volume extrude - " << extrusion << std::endl;

				// As the whole hull will be cloned all new points have index = old_index + index_offset 
				const size_t indexOffset = newIdx;

				// Build edge_2 edge with new incremented point indecies
				std::list<edge> edge_2 = edge_1;
				for (edge& e : edge_2) {
					e.pointIdx[0] += indexOffset;
					e.pointIdx[1] += indexOffset;
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
					else { newQ = FlipQuad(newQ); }	// Flip the new quad
					for (int i = 0; i < 4; i++) {
						newQ.pointIdx[i] += indexOffset;
					}
					newQuads.push(newQ);
				}

				// Build quads betwean edge_1 and edge_2
				for (auto it_1 = edge_1.begin(), it_2 = edge_2.begin(); it_1 != edge_1.end(); ++it_1, ++it_2) {		//ASK: it + 1 != edges.end();
					quad q = BuildQuad(*it_1, *it_2, extrusion);
					if (!isFacing) {
						q = FlipQuad(q);
					}
					newQuads.push(q);
				}
			}
		} break; // end "case planar"

		/////////////////////////////////
		// Convex Hull is a 3D volume: //
		/////////////////////////////////
		case volume: {
			std::cout << "Volume extrude - " << extrusion << std::endl;

			//Select quads facing the extrusion direction
			std::list<quad*> selectQuads;
			for (quad& q : hullQuads) {
				if (dot(extrusion, q.normal) > 0.0f) {	// if quad IS facing the direction
					selectQuads.push_back(&q);
				}
			}

			// Find open edges:
			std::list<edge> edge_1 = FindOpenEdges(selectQuads);

			// Clone the points from edge_1 to edge 2
			std::list<edge> edge_2;
			std::map<size_t, size_t> indexMap; // Keeps map of indecies from edge_1 to edge_2
			for (const edge& e1 : edge_1) {
				edge e2;
				for (int i = 0; i < 2; i++) {
					auto insResult = indexMap.insert({ e1.pointIdx[i],newIdx });
					if (insResult.second) {
						e2.pointIdx[i] = newIdx;
						newPoints.push(hullPoints[e1.pointIdx[i]]);
						newIdx++;
					}
					else {
						e2.pointIdx[i] = insResult.first->second;
					}
				}
				edge_2.push_back(e2);
			}

			// Re-bind selectQuads quads to edge_2 and mark all for move
			for (quad* ptr_q : selectQuads) {
				for (int i = 0; i < 4; i++) {
					auto it_found = indexMap.find(ptr_q->pointIdx[i]);
					if (it_found != indexMap.end()) {
						ptr_q->pointIdx[i] = it_found->second;
					}
					moveIdx.insert(ptr_q->pointIdx[i]);
				}
			}


			// Build quads betwean edge_1 and edge_2
			for (auto it_1 = edge_1.begin(), it_2 = edge_2.begin(); it_1 != edge_1.end(); ++it_1, ++it_2) {
				newQuads.push(BuildQuad(*it_1, *it_2, extrusion));
			}
		}break;	//end "case volume"

		default:
			assert("Convex Hull state not properly defined!");
			break;
		}
	} // end if vector in the same direction is NOT found in the collection

	///////////////////////////////////////
	//   Constructs the new geometry:    //
	///////////////////////////////////////

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
		newQuads.pop();
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
	std::cout << "Points: " << hullPoints.size() <<"      Quads: " << hullQuads.size() << std::endl;
	std::cout << std::endl;
}

std::map<size_t, float> CnvH::Disolve(FVector vec) {
	std::map<size_t, float> result;
	float k, m, n;					// vP = k*vec = vA + m*vB + n*vC
	FVector vP;						// The intersection point of the vector and the quad plane
	const quad *ptr_q = nullptr;	// Poiner to the quad containing vP
	std::list<quad*> selectQuads;

	//Find the quad that the vector intersects
	for (quad& q : hullQuads) {
		if (dot(vec, q.normal) < 0.0f) continue;				// If quad IS backfacing: try next quad
		selectQuads.push_back(&q);
		FVector vA = hullPoints[q.pointIdx[0]].vec;
		k = dot(vA, q.normal) / dot(vec, q.normal);
		vP = k * vec;
		FVector vB = hullPoints[q.pointIdx[1]].vec - vA;
		FVector vC = hullPoints[q.pointIdx[3]].vec - vA;
		// vP = m*vB + n*vC =>
		m = (vP.y - vP.x*vC.y / vC.x) / (vB.y - vB.x*vC.y / vC.x);
		n = vP.x / vC.x - m*vB.x / vC.x;
		if ( m<0.0f || n<0.0f || m>1.0f || n>1.0f ) continue;	// If vP is OUTSIDE the quad: try next quad
		ptr_q = &q;
	}
	if (ptr_q == nullptr) return result;						// If intersection is NOT found: end the function
	// Remove quads not belonging to polygon
	auto it = selectQuads.begin();
	while ( it != selectQuads.end()) {
		if ((*it)->normal != ptr_q->normal){
			it = selectQuads.erase(it);
		}
		else{
			++it;
		}
	}
	if (selectQuads.size == 1){									// If quad only member of polygon
		
	}
	else{														// If quad belongs to a plygon
		float p,q;						// vP = k*vec = vA + p*vQ = vQ + p*vM + q*p*(vN-vM)
		FVector vQ;						// The intersection point of the vector and the poligon edge
		std::list<edge> polyEdge = FindOpenEdges(selectQuads);
		for (edge& e : polyEdge){
			q = ()
		}
	}
}

std::string CnvH::GetPointStr() {
	std::string str = "";
	for (const point& p : hullPoints) {
		str += "v " + std::to_string(p.vec.x) + " " + std::to_string(p.vec.y) + " " + std::to_string(p.vec.z) + "\n";
	}
	return str;
};
std::string CnvH::GetQuadStr() {
	std::string str = "";
	for (const quad& q : hullQuads) {
		str += "f ";
		for (size_t i : q.pointIdx) {
			str += std::to_string(i+1) + " ";
		}
		str += "\n";
	}
	return str;
}

// Finds the open edges in selection of quads.
std::list<CnvH::edge> CnvH::FindOpenEdges(std::list<quad*>& quadArray) {
	std::list<edge> openEdges;
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

// Builds a quad between 2 edges
CnvH::quad CnvH::BuildQuad(	edge e1, edge e2, FVector dist )
{
	size_t idx[4] = {
		e1.pointIdx[0],
		e1.pointIdx[1],
		e2.pointIdx[1],
		e2.pointIdx[0]
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

bool operator==(const CnvH::edge& A, const CnvH::edge& B) {
	if (&A == &B) return true;
	return  A.pointIdx[0] == B.pointIdx[0] && A.pointIdx[1] == B.pointIdx[1];
}