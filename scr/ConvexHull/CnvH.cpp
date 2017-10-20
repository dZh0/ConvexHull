// debug dependencies
#include <assert.h>
// true dependencies
#include <fstream>
#include <queue>
#include <unordered_set>
#include "CnvH.h"

/*
CnvH::CnvH(void){
	state = volume;
	hullPoints.reserve(8);
	hullPoints.emplace_back(this, FVector{ 0.0f, 0.0f, 0.0f });
	hullPoints.emplace_back(this, FVector{ 1.0f, 0.0f, 0.0f });
	hullPoints.emplace_back(this, FVector{ 1.0f, 1.0f, 0.0f });
	hullPoints.emplace_back(this, FVector{ 0.0f, 1.0f, 0.0f });
	hullPoints.emplace_back(this, FVector{ 0.0f, 0.0f, 1.0f });
	hullPoints.emplace_back(this, FVector{ 1.0f, 0.0f, 1.0f });
	hullPoints.emplace_back(this, FVector{ 1.0f, 1.0f, 1.0f });
	hullPoints.emplace_back(this, FVector{ 0.0f, 1.0f, 1.0f });
	hullQuads.reserve(6);
	hullQuads.emplace_back(this, 0, 3, 2, 1, FVector{ 0.0f, 0.0f, -1.0f });
	hullQuads.emplace_back(this, 4, 5, 6, 7, FVector{ 0.0f, 0.0f, 1.0f });
	hullQuads.emplace_back(this, 0, 1, 5, 4, FVector{ 0.0f, -1.0f, 0.0f });
	hullQuads.emplace_back(this, 2, 3, 7, 6, FVector{ 0.0f, 1.0f, 0.0f });
	hullQuads.emplace_back(this, 0, 4, 7, 3, FVector{ -1.0f, 0.0f, 0.0f });
	hullQuads.emplace_back(this, 1, 2, 6, 5, FVector{ 1.0f, 0.0f, 0.0f });
};
*/

// Creates CnvH from C-style FVector array.
CnvH::CnvH(FVector const* p_arr, int _size) : CnvH::CnvH(){
	collection.reserve(_size);
	for (int i = 0; i < _size; i++) {
		Add(p_arr[i]);
	}
}

// Adds a FVector to the hull.
void CnvH::Add(FVector extrusion) {
	size_t collectionIdx;
	std::cout << "#" << collection.size() << "  " << extrusion;
	collection.push_back(extrusion);
	if (extrusion == FV_ZERO) return;		// Don't change geometry if vector is {0,0,0}
	
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
		std::cout << "  Move points:             ";
		collectionIdx = std::distance(collection.begin(), found);
		for (size_t i = 0; i < hullPoints.size(); i++) {
			const auto& map = hullPoints[i].weight;
			if (map.find(collectionIdx) != map.end()) {
				moveIdx.insert(i);
			}
		}
	}
	else {									// If vector in the same direction is NOT found in the collection
		collectionIdx = collection.size()-1;
		switch (state) {
			/////////////////////////////////
			//    Convex Hull is empty:    //
			/////////////////////////////////
		case empty: {
			std::cout << "  Add first point:         ";
			state = linear;
			newPoints.emplace(this);
			newIdx++;
			newPoints.emplace(this);
			moveIdx.insert(newIdx);
		} break;	//end "case empty"

		/////////////////////////////////
		//   Convex Hull is a line:    //
		/////////////////////////////////
		case linear: {
			FVector hullLine = hullPoints[1].vec;

			if (isColinear(extrusion, hullLine)) {	// If the extrusion direction is on the hull line:
				state = linear;
				std::cout << "  Linear extrusion:        ";

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
					newPoints.emplace(this);
					moveIdx.insert(newIdx);
				}
				// If the selected point is NOT the origin: mark it for translation
				else {
					moveIdx.insert(endIdx);
				}
			}
			else {	// If the extrusion direction is NOT on the hull line:
				state = planar;
				std::cout << "  Line to plane extrusion: ";

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
					newQuads.emplace(this, *it_1, *next(it_1), *next(it_2), *it_2, normal);
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
				std::cout << "  Planar extrude:          ";

				// Delete edges that are not facing the extrusion direction (no longer loop)
				FVector direction = cross(hullNormal, extrusion);
				for (auto it = edge_1.begin(); it != edge_1.end();) {
					FVector edgeVec = hullPoints[it->pointIdx[1]].vec - hullPoints[it->pointIdx[0]].vec;
					if (dot(edgeVec, direction) <= 0.0f) it = edge_1.erase(it);
					else ++it;
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
				std::cout << "  Plane to volume extrude: ";

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
			std::cout << "  Volume extrude:          " << extrusion << std::endl;

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
		default: assert("Convex Hull state not properly defined!"); break;
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
	std::cout << " points: " << hullPoints.size() <<" quads: " << hullQuads.size() << std::endl;
}

// Returns the determinant of a 3x3 matrix constructed by the column vector parameters
inline float det3(FVector a, FVector b, FVector c) {
	return a.x*b.y*c.z + a.z*b.x*c.y + a.y*b.z*c.x - a.x*b.z*c.y - a.y*b.x*c.z - a.z*b.y*c.x;
}

void CnvH::Disolve(FVector vec) {
	point pnt(this);
	float scale;
	switch (state){
	case linear: {
		float del = 0.0f;
		for (const point& p : hullPoints){
			float newDel = dot(vec, p.vec);
			if (newDel > del){
				del = newDel;
				pnt = p;
			}
		}
		if (del == 0.0f){
			std::cout << vec << " does not intersect the hull!" << std::endl;
			return;
		}
		float lenP = Length(pnt.vec);
		float lenV = Length(vec);

	}break;
	case planar: {
		FVector normal = hullQuads[0].normal;
		std::list<quad*> selectQuads;
		for (quad& q : hullQuads) selectQuads.push_back(&q);
		std::list<edge> edge_1 = FindOpenEdges(selectQuads);
		point base = point(this);
		// Vector P is the projection of vec on the plane -> P = vec - n*normal
		// Point Q is part of the edge E1:E2 and part of line define by P -> E1 + q*(E2-E1) = k * P
		// -> k*vec - n*k*normal + q*(E1-E2) = E1
		// and n, k, and q are real number coaficents
		float n, k, q, nk;
		FVector vBase = FV_ZERO;
		for (edge& e : edge_1){
			FVector vE1 = hullPoints[e.pointIdx[0]].vec;
			FVector vE2 = hullPoints[e.pointIdx[1]].vec;
			float det = det3(vec, -normal, vE1 - vE2);
			if (det == 0.0f) continue;
			k = det3(vE1, -normal, vE1 - vE2) / det;
			nk = det3(vec, vE1, vE1 - vE2) / det;
			q = det3(vec, -normal, vE1) / det;
			if (k == 0.0f)
			{
				pnt = base;
				continue;
			}
			n = nk / k;
			if (n != 0.0f)
			{
				pnt = base;
				std::cout << vec << " does not intersect the hull!" << std::endl;
				break;
			}
			if (q >= 0.0f && q <= 1.0f){
				pnt = hullPoints[e.pointIdx[0]] + q*(hullPoints[e.pointIdx[1]] - hullPoints[e.pointIdx[0]]);
				break;
			}
		}
		scale = (k > 1.0f) ? k : 1.0f;
	} break;
	case volume: {
		// Intersection point P is part of the line vec -> P = k*vec
		// Intersection point P is part of the plane OAB -> P = O + m*A + n*B
		// -> k*vec - m*A - n*B = O
		// Where A and B are quad edges and O is their common point
		// and k, m and n are real number coaficents
		float k, m, n;
		quad* found = nullptr;
		for (quad& q : hullQuads) {
			if (dot(vec, q.normal) < 0.0f) continue;
			FVector vO = q.getPnt(0).vec;
			FVector vA = q.getPnt(1).vec - q.getPnt(0).vec;
			FVector vB = q.getPnt(3).vec - q.getPnt(0).vec;
			float det = det3(vec, -vA, -vB);
			if (det == 0.0f) continue;	// vec lays on OAB
			k = det3(vO, -vA, -vB) / det;
			assert(k > 0.0f);			// must always be true if dot(vec, q.normal) >= 0.0f
			m = det3(vec, vO, -vB) / det;
			n = det3(vec, -vA, vO) / det;
			if (m >= 0.0f && n >= 0.0f && m <= 1.0f && n <= 1.0f){
				found = &q;
				pnt = (1.0f - m - n)*found->getPnt(0) + m*found->getPnt(1) + n*found->getPnt(3);
				break;
			}
		}
		if (!found){
			std::cout << vec << " does not intersect the hull!" << std::endl;
			return;
		}
		std::list<quad*> selectQuads;
		for (quad& q : hullQuads) {
			if (q.normal == found->normal){
				selectQuads.push_back(&q);
			}
		}
		if (selectQuads.size() > 1){
			// Find open edges:
			std::list<edge> edge_1 = FindOpenEdges(selectQuads);
			point base = BasePoint(edge_1);
			// Intersection point P is part of the line vec -> P = k*vec
			// The line through P and the polygon base point intersects an edge (with end points E1 and E2) at poin Q ->
			// -> Q = E1 + q*(E2-E1) and P = base + p*(Q - base)
			// -> k*vec = base + p*(E1 + q*(E2-E1) - base)
			// -> base = k*vec + p*(base - E1) + p*q*(E1 - E2)
			// where k, p and q are real number coaficents
			float k, p, q, pq;
			FVector vBase = base.vec;
			for (edge& e : edge_1){
				FVector vE1 = hullPoints[e.pointIdx[0]].vec;
				FVector vE2 = hullPoints[e.pointIdx[1]].vec;
				float det = det3(vec, vBase - vE1, vE1 - vE2);
				if (det == 0.0f) continue;
				k = det3(vBase, vBase - vE1, vE1 - vE2) / det;
				p = det3(vec, vBase, vE1 - vE2) / det;
				pq = det3(vec, vBase - vE1, vBase) / det;
				if (p == 0.0f)
				{
					pnt = base;
					break;
				}
				q = pq / p;
				assert(k >= 0.0f);
				if (q >= 0.0f && q <= 1.0f){
					pnt = base + p*(hullPoints[e.pointIdx[0]] - base) + pq*(hullPoints[e.pointIdx[1]] - hullPoints[e.pointIdx[0]]);
					break;
				}
			}
		}
		scale = (k > 1.0f) ? k : 1.0f;
	}break;
	default: assert("Convex Hull state not properly defined!"); break;
	}
	pnt *= scale;
	std::cout << pnt << std::endl;
	std::cout << "= " << 1.0f/scale << " * " << pnt.vec << std::endl;
}

std::ofstream& operator<<(std::ofstream& ostr, const CnvH& hull){
	ostr.setf(std::ios::fixed, std::ios::floatfield);
	ostr.setf(std::ios::showpoint);
	ostr << "#Vertecies" <<std::endl;
	for (const CnvH::point& p : hull.hullPoints) {
		ostr << "v " << p.vec.x << " " << p.vec.y << " " << p.vec.z << std::endl;
	};
	ostr << "#Inecies" << std::endl;
	for (const CnvH::quad& q : hull.hullQuads) {
		ostr << "f " << q.pointIdx[0] + 1 << " " << q.pointIdx[1] + 1 << " " << q.pointIdx[2] + 1 << std::endl;
		ostr << "f " << q.pointIdx[0] + 1 << " " << q.pointIdx[2] + 1 << " " << q.pointIdx[3] + 1 << std::endl;
	}
	return ostr;
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
	return quad(this, idx[0], idx[1], idx[2], idx[3], normal);
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
	return quad(this, idx[0], idx[1], idx[2], idx[3], normal);
}

bool operator==(const CnvH::edge& A, const CnvH::edge& B) {
	if (&A == &B) return true;
	return  A.pointIdx[0] == B.pointIdx[0] && A.pointIdx[1] == B.pointIdx[1];
}

CnvH::point CnvH::BasePoint(const std::list<edge>& edges){
	auto it = edges.begin();
	point result = hullPoints[it->pointIdx[0]];
	++it;
	while (it != edges.end()){
		result = Common(result, hullPoints[it->pointIdx[0]]);
		++it;
	}
	return result;
}

CnvH::point CnvH::Common(const point& A, const point& B){
	point result = A;
	auto it = result.weight.begin();
	while (it != result.weight.end()) {
		auto found = B.weight.find(it->first);
		if (found != B.weight.end()) {
			float scale = (it->second < found->second) ? it->second : found->second;
			it->second = scale;
			result.vec -= (1.0f - scale)*collection[found->first];
			++it;
		}
		else{
			result.vec -= collection[it->first];
			it = result.weight.erase(it);
		}
	}
	return result;
}

CnvH::point operator*(float A, const CnvH::point& B){
	CnvH::point result = B;
	result.vec *= A;
	for (auto& w : result.weight) {
		w.second *= A;
	}
	return result;
}

CnvH::point operator*(const CnvH::point& A, float B){
	CnvH::point result = A;
	result.vec *= B;
	for (auto& w : result.weight) {
		w.second *= B;
	}
	return result;
}

CnvH::point operator+(const CnvH::point& A, const CnvH::point& B){
	if (B.vec == FV_ZERO) return A;
	CnvH::point pnt = A;
	pnt.vec += B.vec;
	for (auto w : B.weight) {
		auto ret = pnt.weight.insert(w);
		if (!ret.second) {
			ret.first->second += w.second;
			if (ret.first->second == 0.0f) pnt.weight.erase(ret.first);
		}
	}
	return pnt;
}

CnvH::point operator-(const CnvH::point& A, const CnvH::point& B){
	if (B.vec == FV_ZERO) return A;
	CnvH::point pnt = A;
	pnt.vec -= B.vec;
	for (auto w : B.weight) {
		w.second *= -1.0f;
		auto ret = pnt.weight.insert(w);
		if (!ret.second) {
			ret.first->second += w.second;
			if (ret.first->second == 0.0f) pnt.weight.erase(ret.first);
		}
	}
	return pnt;
}

#include <iomanip>
std::ostream& operator<<(std::ostream& ostr, const CnvH::point& p){
	for (auto w : p.weight){
		ostr << std::fixed << std::setprecision(6);
		ostr << w.second << " * #" << w.first << " " << p.parent->collection[w.first] << std::endl;
	}
	return ostr;
}