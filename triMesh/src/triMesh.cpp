/*

This file is part of the TriMesh library.

The TriMesh library is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

The TriMesh library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

This link provides the exact terms of the GPL license <https://www.gnu.org/licenses/>.

The author's interpretation of GPL 3 is that if you receive money for the use or distribution of the TriMesh Library or a derivative product, GPL 3 no longer applies.

Under those circumstances, the author expects and may legally pursue a reasoble share of the income. To avoid the complexity of agreements and negotiation, the author makes
no specific demands in this regard. Compensation of roughly 1% of net or $5 per user license seems appropriate, but is not legally binding.

In lay terms, if you make a profit by using the TriMesh library (violating the spirit of Open Source Software), I expect a reasonable share for my efforts.

Robert R Tipton - Author

Dark Sky Innovative Solutions http://darkskyinnovation.com/

*/

#include <list>
#include <tm_defines.h>

#include <algorithm>
#include <fstream>
#include <iomanip>

#define _USE_MATH_DEFINES
#include <cmath>

#include <../../stlReader/include/readStl.h>
#include <triMesh.h>
#include <MultiCoreUtil.h>

using namespace std;

namespace TriMesh {

atomic<size_t> CMesh::_statId = 0;

CMesh::CMesh() 
	: _id(_statId++)
	, _changeNumber(0)
{
}

CMesh::CMesh(const Vector3d& min, const Vector3d& max)
	: _id(_statId++)
	, _vertTree(BoundingBox(min, max))
	, _edgeTree(BoundingBox(min, max))
	, _triTree(BoundingBox(min, max))
{
}

void CMesh::reset(const BoundingBox& bbox) {
	BoundingBox bb(bbox);
	bb.grow(SAME_DIST_TOL);
	_vertTree.reset(bb);
	_edgeTree.reset(bb);
	_triTree.reset(bb);
}

void CMesh::dumpTris(const wstring& filename) const
{
#ifdef WIN32
	wofstream out(filename);
#else
	wofstream out(CReadSTL::fromWString(filename));
#endif // WIND32

	out << setprecision(15);
	out << numVertices() << "\n";
	out << numTris() << "\n";
	out << numEdges() << "\n";
	for (const auto& vert : _vertices) {
		out << vert._pt[0] << " " << vert._pt[1] << " " << vert._pt[2] << "\n";
	}
	for (const auto& tri : _tris) {
		out << tri[0] << " " << tri[1] << " " << tri[2] << "\n";
	}
	for (const auto& edge : _edges) {
		out << edge._vertIndex[0] << " " << edge._vertIndex[1] << " " << edge._numFaces << " " << edge._faceIndices[0];
		if (edge._numFaces == 2)
			out << " " << edge._faceIndices[1];
		out << "\n";
	}
}

bool CMesh::compareDumpedTris(const wstring& filename) const
{
#ifdef WIN32
	wifstream in(filename);
#else
	wifstream in(CReadSTL::fromWString(filename));
#endif // WIND32
	size_t nVerts, nTris, nEdges;
	in >> nVerts;
	in >> nTris;
	in >> nEdges;
	if (numVertices() != nVerts)
		return false;
	if (numTris() != nTris)
		return false;
	if (numEdges() != nEdges)
		return false;

	for (const auto& vert : _vertices) {
		CVertex v;
		in >> v._pt[0] >> v._pt[1] >> v._pt[2];
		if ((vert._pt - v._pt).norm() > 1.0e-15) {
			return false;
		}
	}
	for (const auto& tri : _tris) {
		Vector3i t;
		in >> t[0] >> t[1] >> t[2];
		if (t != tri) {
			return false;
		}
	}
	for (const auto& edge : _edges) {
		CEdge e;
		size_t a, b, c, d;
		int numFaces;
		in >> a >> b >> numFaces >> c;
		if (numFaces == 2)
			in >> d;

		if (a != edge._vertIndex[0]) {
			return false;
		}
		if (b != edge._vertIndex[1]) {
			return false;
		}
		if (numFaces != edge._numFaces) {
			return false;
		}

		if (numFaces == 2 && d != edge._faceIndices[1])
			return false;
	}
	return true;
}

void CMesh::dumpTree(const wstring& filename) const
{
#ifdef WIN32
	wofstream out(filename);
#else
	wofstream out(CReadSTL::fromWString(filename));
#endif
	out << setprecision(15);
	_triTree.dump(out);

}

bool CMesh::compareDumpedTree(const wstring& filename) const
{
	return true;
}

size_t CMesh::findEdge(const CEdge& edge) const
{
	auto iter = _edgeToIdxMap.find(edge);
	if (iter != _edgeToIdxMap.end())
		return iter->second;
	return -1;
}

size_t CMesh::findEdge(size_t vertIdx0, size_t vertIdx1) const
{
	return findEdge(CEdge(vertIdx0, vertIdx1));
}

bool CMesh::triContainsVertex(size_t triIdx, size_t vertIdx) const
{
	if (triIdx < _tris.size()) {
		const auto& tri = _tris[triIdx];
		for (int i = 0; i < 3; i++) {
			if (tri[i] == vertIdx)
				return true;
		}
	}
	return false;
}

bool CMesh::triContainsEdge(size_t triIdx, size_t edgeIdx) const
{
	if (triIdx < _tris.size() && edgeIdx < _edges.size()) {
		const auto& tri = _tris[triIdx];
		const auto& testEdge = _edges[edgeIdx];

		for (int i = 0; i < 3; i++) {
			int j = (i + 1) % 3;
			CEdge edge(tri[i], tri[j]);
			if (testEdge == edge)
				return true;
		}
	}
	return false;
}

bool CMesh::edgeContainsVert(size_t edgeIdx, size_t vertIdx) const
{
	if (edgeIdx < _edges.size() && vertIdx < _vertices.size()) {
		const auto& edge = _edges[edgeIdx];
		for (int i = 0; i < 2; i++) {
			if (edge._vertIndex[i] == vertIdx)
				return true;
		}
	}
	return false;
}

bool CMesh::edgeReferencesTri(size_t edgeIdx, size_t triIdx) const
{
	if (edgeIdx < _edges.size() && triIdx < _tris.size()) {
		const auto& edge = _edges[edgeIdx];
		for (int i = 0; i < edge._numFaces; i++) {
			if (edge._faceIndices[i] == triIdx)
				return true;
		}
	}
	return false;
}

bool CMesh::vertReferencesTri(size_t vertIdx, size_t triIdx) const
{
	if (vertIdx < _vertices.size() && triIdx < _tris.size()) {
		const auto& vertIds = _vertices[vertIdx]._faceIndices;
		if (find(vertIds.begin(), vertIds.end(), triIdx) != vertIds.end())
			return true;
		return false;
	}
	return false;
}

bool CMesh::vertReferencesEdge(size_t vertIdx, size_t edgeIdx) const
{
	if (vertIdx < _vertices.size() && edgeIdx < _edges.size()) {
		const auto& edgeIds = _vertices[vertIdx]._edgeIndices;
		if (find(edgeIds.begin(), edgeIds.end(), edgeIdx) != edgeIds.end())
			return true;
		return false;
	}
	return false;
}

void CMesh::squeezeEdge(size_t idx)
{
	auto& edge = _edges[idx];
	if (edge._numFaces != 2)
		return;

	Vector3d dir = getEdgesLineSeg(idx).calcDir();

	double minCp[] = { DBL_MAX, DBL_MAX };
	double minDist[] = { DBL_MAX, DBL_MAX };
	for (size_t i = 0; i < 2; i++) {
		const auto& vert = _vertices[edge._vertIndex[i]];
		for (size_t edgeIdx : vert._edgeIndices) {
			if (edgeIdx == idx)
				continue;
			auto seg = getEdgesLineSeg(edgeIdx);
			Vector3d v = seg.calcDir();
			double cp = v.cross(dir).norm();
			if (cp < minCp[i]) {
				minCp[i] = cp;
				minDist[i] = seg.calLength();
			}
		}
	}

	// squeeze to the longer edge so the shorter edge gets longer. This reduces the aspect ratio of the tri with the short 
	// edge so it's less likely we need to squeeze it too.
	size_t vertIdxToKeep, vertIdxToRemove;
	if (minDist[0] > minDist[1]) {
		vertIdxToKeep = edge._vertIndex[0];
		vertIdxToRemove = edge._vertIndex[1];
	} else {
		vertIdxToKeep = edge._vertIndex[1];
		vertIdxToRemove = edge._vertIndex[0];
	}

	size_t faceIdx0 = edge._faceIndices[0];
	size_t faceIdx1 = edge._faceIndices[1];

	// Remove the larger one first. If not, the first one can cause the second to move
	if (faceIdx1 < faceIdx0)
		swap(faceIdx0, faceIdx1);

	removeTri(faceIdx1);
	removeTri(faceIdx0);
	mergeVertices(vertIdxToKeep, vertIdxToRemove);
}

bool CMesh::removeTri(size_t triIdx)
{
	bool result = true;
	assert(triIdx < _tris.size());
	
	BoundingBox triBBox = getTriBBox(triIdx);
	_triTree.remove(triBBox, triIdx);

	set<size_t> edgesToRemove;
	const auto& tri = _tris[triIdx];
	for (int i = 0; i < 3; i++) {
		int j = (i + 1) % 3;
		size_t vertIdx0 = tri[i];
		size_t vertIdx1 = tri[j];
		size_t edgeIdx = findEdge(vertIdx0, vertIdx1);

		auto& vert0 = _vertices[vertIdx0];
		vert0.removeFaceIndex(triIdx);

		auto& edge = _edges[edgeIdx];
		edge.removeFaceIndex(triIdx);
		if (edge._numFaces == 0) {
			auto& vert1 = _vertices[vertIdx1];
			vert0.removeEdgeIndex(edgeIdx);
			vert1.removeEdgeIndex(edgeIdx);
			edgesToRemove.insert(edgeIdx);
		}
	}
	if (!deleteTriFromStorage(triIdx))
		result = false;

	while (!edgesToRemove.empty()) {
		if (!deleteEdgeFromStorage(*edgesToRemove.begin())) {
			result = false;
			break;
		}
		edgesToRemove.erase(edgesToRemove.begin());
	}

	return result;
}

bool CMesh::deleteTriFromStorage(size_t triIdx)
{
	size_t srcIdx = _tris.size() - 1;

	BoundingBox bbox = getTriBBox(triIdx), srcBBox = getTriBBox(srcIdx);
	_triTree.remove(srcBBox, srcIdx);

	if (triIdx == _tris.size() - 1) {
		_tris.pop_back();
		return true;
	}
	_tris[triIdx] = _tris[srcIdx];
	_tris.pop_back();

	{
		auto& tri = _tris[triIdx];
		for (int i = 0; i < 3; i++) {
			int j = (i + 1) % 3;
			size_t vertIdx0 = tri[i];
			size_t vertIdx1 = tri[j];
			size_t edgeIdx = findEdge(vertIdx0, vertIdx1);

			auto& vert0 = _vertices[vertIdx0];
			vert0.changeFaceIndex(srcIdx, triIdx);
			vert0.changeEdgeIndex(srcIdx, triIdx);

			auto& edge = _edges[edgeIdx];
			edge.changeFaceIndex(srcIdx, triIdx);
		}
		BoundingBox bbox = getTriBBox(triIdx);
		_triTree.add(bbox, triIdx);
	}

	verifyTris(triIdx);
	return true;
}

bool CMesh::deleteEdgeFromStorage(size_t edgeIdx)
{
	size_t srcIdx = _edges.size() - 1;

	// Must delete from BOTH _edgeTree and _edgeToIdxMap
	BoundingBox bbox = getEdgeBBox(edgeIdx), srcBbox = getEdgeBBox(srcIdx);
	_edgeTree.remove(bbox, edgeIdx);
	_edgeToIdxMap.erase(_edges[edgeIdx]);
	_edgeTree.remove(srcBbox, srcIdx);
	_edgeToIdxMap.erase(_edges[srcIdx]);

	{
		auto& edge = _edges[edgeIdx];

		size_t vertIdx0 = edge._vertIndex[0];
		size_t vertIdx1 = edge._vertIndex[1];
		auto& vert0 = _vertices[vertIdx0];
		auto& vert1 = _vertices[vertIdx0];

		assert(!vert0.containsEdgeIndex(edgeIdx));
		assert(!vert1.containsEdgeIndex(edgeIdx));
		assert(edge._numFaces == 0);
	}

	if (edgeIdx == _edges.size() - 1) {
		_edges.pop_back();
	} else {
		_edges[edgeIdx] = _edges[srcIdx];
		_edges.pop_back();

		auto& edge = _edges[edgeIdx];
		size_t vertIdx0 = edge._vertIndex[0];
		size_t vertIdx1 = edge._vertIndex[1];
		auto& vert0 = _vertices[vertIdx0];
		auto& vert1 = _vertices[vertIdx1];
		vert0.changeEdgeIndex(srcIdx, edgeIdx);
		vert1.changeEdgeIndex(srcIdx, edgeIdx);

		// Must add back to BOTH _edgeTree and _edgeToIdxMap
		_edgeTree.add(srcBbox, edgeIdx);
		_edgeToIdxMap.insert(make_pair(edge, edgeIdx));
	}

	return true;
}

void CMesh::mergeVertices(size_t vertIdxToKeep, size_t vertIdxToRemove)
{
	auto& vertToRemove = _vertices[vertIdxToRemove];
	auto triIdsToModify = vertToRemove._faceIndices;

	for (size_t triIdx : triIdsToModify) {
		auto tri = _tris[triIdx];
		int vertIdxToSwap = -1;
		for (int i = 0; i < 3; i++) {
			if (tri[i] == vertIdxToRemove) {
				vertIdxToSwap = i;
				break;
			}
		}
		if (vertIdxToSwap != -1) {
			removeTri(triIdx);
			tri[vertIdxToSwap] = vertIdxToKeep;
			addTriangle(tri);
		}
	}
}

bool CMesh::verifyTopology(bool allowEmptyEdges) const
{
	for (size_t triIdx = 0; triIdx < _tris.size(); triIdx++) {
		if (!verifyTris(triIdx))
			return false;
	}

	for (size_t vertIdx = 0; vertIdx < _vertices.size(); vertIdx++) {
		if (!verifyVerts(vertIdx))
			return false;
	}

	for (size_t edgeIdx = 0; edgeIdx < _edges.size(); edgeIdx++) {
		if (!verifyEdges(edgeIdx, allowEmptyEdges))
			return false;
	}

	return true;
}

bool CMesh::verifyTris(size_t triIdx) const
{
	{
		BoundingBox bbox = getTriBBox(triIdx);
		auto foundTris = _triTree.find(bbox);
		bool found = false;
		for (const auto& ent : foundTris) {
			if (ent.getIndex() == triIdx) {
				found = true;
				break;
			}
		}
		if (!found)
			return false;
	}

	const auto& tri = _tris[triIdx];
	for (size_t i = 0; i < 3; i++) {
		size_t j = (i + 1) % 3;
		size_t vertIdx0 = tri[i];
		size_t vertIdx1 = tri[j];
		size_t edgeIdx = findEdge(vertIdx0, vertIdx1);
		// 4319 deleted, but still in use by triIdx == 0
		if (!vertReferencesTri(vertIdx0, triIdx))
			return false;
		if (!vertReferencesEdge(vertIdx0, edgeIdx))
			return false;
		if (!edgeReferencesTri(edgeIdx, triIdx))
			return false;
	}

	return true;
}

bool CMesh::verifyVerts(size_t vertIdx) const
{
	{
		BoundingBox bbox = getVertBBox(vertIdx);
		auto foundTris = _vertTree.find(bbox);
		bool found = false;
		for (const auto& ent : foundTris) {
			if (ent.getIndex() == vertIdx) {
				found = true;
				break;
			}
		}
		if (!found)
			return false;
	}

	const auto& vert = _vertices[vertIdx];

	for (auto triIdx : vert._faceIndices) {
		if (!vertReferencesTri(vertIdx, triIdx))
			return false;
	}

	for (auto edgeIdx : vert._edgeIndices) {
		if (!vertReferencesEdge(vertIdx, edgeIdx))
			return false;
	}

	return true;
}

bool CMesh::verifyEdges(size_t edgeIdx, bool allowEmpty) const
{
	{
		BoundingBox bbox = getEdgeBBox(edgeIdx);
		auto foundTris = _edgeTree.find(bbox);
		bool found = false;
		for (const auto& ent : foundTris) {
			if (ent.getIndex() == edgeIdx) {
				found = true;
				break;
			}
		}
		if (!found)
			return false;
	}

	const auto& edge = _edges[edgeIdx];
	if (edge._numFaces == 0)
		return allowEmpty;

	if (edge._numFaces > 2)
		return false;

	for (int i = 0; i < 2; i++) {
		if (!vertReferencesEdge(edge._vertIndex[i], edgeIdx))
			return false;
	}

	for (int i = 0; i < edge._numFaces; i++) {
		bool found = false;
		if (!triContainsEdge(edge._faceIndices[i], edgeIdx))
			return false;
	}

	return true;
}

size_t CMesh::addEdge(size_t vertIdx0, size_t vertIdx1) {
	CEdge edge(vertIdx0, vertIdx1);
	auto iter = _edgeToIdxMap.find(edge);
	if (iter != _edgeToIdxMap.end())
		return iter->second;
	size_t result = _edges.size();
	_edges.push_back(edge);
	_edgeToIdxMap.insert(make_pair(edge, result));

	auto& vert0 = _vertices[vertIdx0];
	auto& vert1 = _vertices[vertIdx1];

	vert0.addEdgeIndex(result);
	vert1.addEdgeIndex(result);

	BoundingBox edgeBox;
	edgeBox.merge(vert0._pt);
	edgeBox.merge(vert1._pt);
	_edgeTree.add(edgeBox, result);

	return result;
}

bool CMesh::sameTri(const Vector3i& tri0, const Vector3i& tri1)
{
	for (int i = 0; i < 3; i++) {
		bool found = false;
		for (int j = 0; j < 3; j++) {
			if (tri0[i] == tri1[j]) {
				found = true;
				break;
			}
		}
		if (!found)
			return false;
	}

	return true;
}

size_t CMesh::addTriangle(const Vector3i& tri) {
	BoundingBox triBox;
	for (int i = 0; i < 3; i++) {
		const auto& vert = _vertices[tri[i]];
		triBox.merge(vert._pt);
	}

	auto triIndices = _triTree.find(triBox);
	for (const auto& triEntry : triIndices) {
		size_t triIdx = triEntry.getIndex();
		if (sameTri(_tris[triIdx], tri))
			return triIdx;
	}

	size_t triIdx = _tris.size();
	_tris.push_back(tri);
	for (int i = 0; i < 3; i++) {
		int j = (i + 1) % 3;
		size_t edgeIdx = addEdge(tri[i], tri[j]); // addEdge adds itself to its vertices

		auto& vert = _vertices[tri[i]];
		vert.addFaceIndex(triIdx);

		_edges[edgeIdx].addFaceIndex(triIdx);
	}
	triBox.grow(SAME_DIST_TOL);
	_triTree.add(triBox, triIdx);

	return triIdx;
}

CMesh::BoundingBox CMesh::getTriBBox(size_t triIdx) const
{
	BoundingBox result;
	const auto& tri = _tris[triIdx];
	for (int i = 0; i < 3; i++)
		result.merge(_vertices[tri[i]]._pt);

	return result;
}

CMesh::BoundingBox CMesh::getEdgeBBox(size_t edgeIdx) const
{
	BoundingBox result;
	const auto& edge = _edges[edgeIdx];
	for (int i = 0; i < 2; i++)
		result.merge(_vertices[edge._vertIndex[i]]._pt);

	return result;
}

CMesh::BoundingBox CMesh::getVertBBox(size_t vertIdx) const
{
	BoundingBox result;
	result.merge(_vertices[vertIdx]._pt);

	return result;
}

LineSegment CMesh::getEdgesLineSeg(size_t edgeIdx) const 
{
	const CEdge& edge = _edges[edgeIdx];
	size_t idx0 = edge._vertIndex[0];
	size_t idx1 = edge._vertIndex[1];
	return LineSegment(_vertices[idx0]._pt, _vertices[idx1]._pt);
}

bool CMesh::isEdgeSharp(size_t edgeIdx, double sinEdgeAngle) const {
	const CEdge& edge = _edges[edgeIdx];
	if (edge._numFaces < 2)
		return true;
	const LineSegment seg = getEdgesLineSeg(edgeIdx);
	const Vector3d edgeV = seg.calcDir();
	if (edge._faceIndices[0] == edge._faceIndices[1]) {
		cout << "Duplicated faceIndex\n";
	}
	const Vector3d norm0 = triUnitNormal(edge._faceIndices[0]);
	const Vector3d norm1 = triUnitNormal(edge._faceIndices[1]);

	double sinTheta = norm0.cross(norm1).norm();
	bool isSharp = sinTheta > sinEdgeAngle;
	return isSharp;
}

const vector<size_t>& CMesh::getSharpEdgeIndices(double edgeAngleRadians) const
{
	if (edgeAngleRadians < 1.0e-6) {
		return _sharpEdgeIndices;
	}

	double sinEdgeAngle = sin(edgeAngleRadians);
	buildNormals();

	vector<bool> sharps;
	sharps.resize(_edges.size());
	MultiCore::runLambda([this, &sharps, sinEdgeAngle](size_t index)->bool {
		sharps[index] = isEdgeSharp(index, sinEdgeAngle);
		return true;
	}, _edges.size(), true);

	_sharpEdgeIndices.clear();
	_sharpEdgeIndices.reserve(_edges.size());
	for (size_t i = 0; i < sharps.size(); i++) {
		if (sharps[i]) {
			_sharpEdgeIndices.push_back(i);
		}
	}

	_sharpEdgeIndices.shrink_to_fit();
	return _sharpEdgeIndices;
}

size_t CMesh::numVertices() const {
	return _vertices.size();
}

size_t CMesh::numEdges() const {
	return _edges.size();
}

size_t CMesh::numTris() const {
	return _tris.size();
}

size_t CMesh::numLaminarEdges() const {
	size_t result = 0;
	for (const auto& edge : _edges) {
		if (edge._numFaces == 1)
			result++;
	}
	return result;
}

bool CMesh::isClosed() const {
	return numLaminarEdges() == 0;
}

size_t CMesh::rayCast(const Ray& ray, vector<RayHit>& hits, bool biDir) const {
	buildNormals();
	vector<size_t> hitIndices;
	if (_triTree.biDirRayCast(ray, hitIndices) > 0) {
		for (size_t triIdx2 : hitIndices) {
			const auto& tri = _tris[triIdx2];
			Vector3d const* const pts[] = {
				&_vertices[tri[0]]._pt,
				&_vertices[tri[1]]._pt,
				&_vertices[tri[2]]._pt,
			};

			RayHit hit;
			if (intersectRayTri(ray, pts, hit)) {
				if (biDir || hit.dist > 0) {
					hit.triIdx = triIdx2;
					hits.push_back(hit);
				}
			}
		}
	}
	sort(hits.begin(), hits.end());
	return hits.size();
}

size_t CMesh::rayCast(const LineSegment& seg, vector<RayHit>& hits, double tol) const {
	auto segLen = seg.calLength();
	vector<size_t> hitIndices;
	if (_triTree.biDirRayCast(seg.getRay(), hitIndices) > 0) {
		for (size_t triIdx2 : hitIndices) {
			const auto& tri = _tris[triIdx2];
			Vector3d const* const pts[] = {
				&_vertices[tri[0]]._pt,
				&_vertices[tri[1]]._pt,
				&_vertices[tri[2]]._pt,
			};

			RayHit hit;
			if (intersectLineSegTri(seg, pts, hit)) {
				if ((hit.dist >= -tol) && (hit.dist <= segLen + tol)) {
					if (hit.dist < 0)
						hit.dist = 0;
					else if (hit.dist > segLen)
						hit.dist = segLen;

					hit.triIdx = triIdx2;
					hits.push_back(hit);
				}
			}
		}
	}
	sort(hits.begin(), hits.end());
	return hits.size();
}

size_t CMesh::rayCast(size_t triIdx, vector<RayHit>& hits, bool biDir) const {
	Vector3d ctr = triCentroid(triIdx);
	Vector3d norm = triUnitNormal(triIdx);
	Ray ray(ctr, norm);

	vector<RayHit> temp;
	rayCast(ray, temp);

	for (const auto& hit : temp) {
		if (hit.triIdx != triIdx)
			hits.push_back(hit);
	}
	return hits.size();
}

double CMesh::findTriMinimumGap(size_t i) const {
	vector<RayHit> hits;
	if (rayCast(i, hits) == 0)
		return FLT_MAX;
	else
		return fabs(hits[0].dist);
}

double CMesh::findMinGap(double tol, bool multiCore) const {
	vector<double> minGapVec;

	minGapVec.resize(MultiCore::getNumCores(), DBL_MAX);

	MultiCore::runLambda([this, tol, &minGapVec](size_t threadNum, size_t numThreads)->bool {
		auto& minGap = minGapVec[threadNum];
		size_t num = numTris();
		for (size_t triIdx = threadNum; triIdx < num; triIdx += numThreads) {
			vector<RayHit> hits;
			if (rayCast(triIdx, hits) > 0) {
				double d = fabs(hits[0].dist);
				if (d > tol && d < minGap)
					minGap = d;
			}
		}
		return true;
	}, multiCore);

	double minGap = DBL_MAX;
	for (double d : minGapVec) {
		if (d < minGap) {
			minGap = d;
		}
	}

	return minGap;
}

void CMesh::getGapHistogram(const vector<double>& binSizes, vector<size_t>& bins, bool multiCore) const {
	buildCentroids(multiCore);
	buildNormals(multiCore);

	bins.clear();
	bins.resize(binSizes.size(), 0);
	vector<vector<int>> binSet;
	binSet.resize(MultiCore::getNumCores());
	for (auto& bin : binSet) {
		bin.resize(binSizes.size(), 0);
	}

	MultiCore::runLambda([this, &binSizes, &binSet](size_t threadNum, size_t numThreads)->bool {
		auto& bins = binSet[threadNum];
		size_t num = numTris();
		for (size_t triIdx = threadNum; triIdx < num; triIdx += numThreads) {
			vector<RayHit> hits;
			if (rayCast(triIdx, hits) != 0) {
				for (const RayHit& hit : hits) {
					if (hit.dist < 0)
						continue;
					for (size_t i = 0; i < binSizes.size(); i++) {
						if (fabs(hit.dist) < binSizes[i]) {
							bins[i]++;
							break;
						}
					}
				}
			}
		}
		return true;
	}, multiCore);

	for (auto& bin : binSet) {
		for (size_t i = 0; i < bins.size(); i++)
			bins[i] += bin[i];
	}
}

void CMesh::merge(CMeshPtr& src, bool destructive)
{
	if (getBBox().contains(src->getBBox())) {
		for (const auto& tri : src->_tris) {
			CVertex pts[3];
			for (int i = 0; i < 3; i++) {
				pts[i] = src->getVert(tri[i]);
			}
			addTriangle(pts[0]._pt, pts[1]._pt, pts[2]._pt);
		}
	} else {
		auto bbox = _triTree.getBounds();
			bbox.merge(src->getBBox());

			CMeshPtr temp = make_shared<CMesh>(*this);
			reset(bbox);

			for (const auto& tri : temp->_tris) {
				CVertex pts[3];
				for (int i = 0; i < 3; i++) {
					pts[i] = temp->getVert(tri[i]);
				}
				addTriangle(pts[0]._pt, pts[1]._pt, pts[2]._pt);
			}

		for (const auto& tri : src->_tris) {
			CVertex pts[3];
			for (int i = 0; i < 3; i++) {
				pts[i] = src->getVert(tri[i]);
			}
			addTriangle(pts[0]._pt, pts[1]._pt, pts[2]._pt);
		}
	}
	if (destructive)
		src = nullptr;
}

void CMesh::merge(vector<CMeshPtr>& src, bool destructive, bool multiCore)
{
#if 1
	while (src.size() > 1) {
		cout << "Num meshes 0: " << src.size() << "\n";
		MultiCore::runLambda([this, &src, destructive](size_t threadNum, size_t numThreads)->bool {
			for (size_t i = threadNum; i < src.size(); i += numThreads) {
				if (i % 2 == 0) {
					size_t j = i + 1;
					if (j < src.size() && src[j]) {
						auto pSrc = src[j];
						assert(src[i]->getBBox().contains(pSrc->getBBox()));
						src[j] = nullptr;

						src[i]->merge(pSrc, destructive);
					}
				}
			}
			return true;
		}, multiCore);

		for (size_t i = 2; i < src.size(); i += 2) {
			src[i / 2] = src[i];
			src[i] = nullptr;
		}

		while (!src.back()) {
			src.pop_back();
		}

		cout << "Num meshes 1: " << src.size() << "\n";
	}

	merge(src.back(), destructive);

#else
	auto bbox = _triTree.getBounds();
	for (const auto& pMesh : src) {
		bbox.merge(pMesh->getBBox());
	}

	CMeshPtr temp = make_shared<CMesh>(*this);
	reset(bbox);

	for (const auto& tri : temp->_tris) {
		CVertex pts[3];
		for (int i = 0; i < 3; i++) {
			pts[i] = temp->getVert(tri[i]);
		}
		addTriangle(pts[0]._pt, pts[1]._pt, pts[2]._pt);
	}

	for (auto& pMesh : src) {
		for (const auto& tri : pMesh->_tris) {
			CVertex pts[3];
			for (int i = 0; i < 3; i++) {
				pts[i] = pMesh->getVert(tri[i]);
			}
			addTriangle(pts[0]._pt, pts[1]._pt, pts[2]._pt);
		}
		pMesh = nullptr;
	}
#endif
}

void CMesh::buildCentroids(bool multiCore) const
{
	if (_centroids.empty()) {
		ScopedSetVal<bool> set(_useCentroidCache, false);
		_centroids.resize(_tris.size());
		MultiCore::runLambda([this](size_t threadNum, size_t numThreads)->bool {
			size_t num = _centroids.size();
			for (size_t triIdx = threadNum; triIdx < num; triIdx += numThreads)
				_centroids[triIdx] = triCentroid(triIdx);
			return true;
		}, multiCore);
	}
}

void CMesh::buildNormals(bool multiCore) const
{
	if (_normals.empty()) {
		ScopedSetVal<bool> set(_useNormalCache, false);
		_normals.resize(_tris.size());
		MultiCore::runLambda([this](size_t threadNum, size_t numThreads)->bool {
			size_t num = _normals.size();
			for (size_t triIdx = threadNum; triIdx < num; triIdx += numThreads)
				_normals[triIdx] = triUnitNormal(triIdx);
			return true;
		}, multiCore);
	}		
}

double CMesh::calSinVertexAngle(size_t triIdx, size_t vertIdx, size_t& oppositeEdgeIdx) const
{
	static double minSinTheta = 1;
	const auto& tri = _tris[triIdx];
	for (int i = 0; i < 3; i++) {
		if (tri[i] == vertIdx) {
			int j = (i + 1) % 3;
			int k = (j + 1) % 3;
			oppositeEdgeIdx = findEdge(tri[j], tri[k]);
			const auto& pt0 = _vertices[tri[i]]._pt;
			const auto& pt1 = _vertices[tri[j]]._pt;
			const auto& pt2 = _vertices[tri[k]]._pt;
			Vector3d v0 = (pt1 - pt0).normalized();
			Vector3d v1 = (pt2 - pt0).normalized();
			double sinTheta = (v0.cross(v1)).norm();
			if (sinTheta < minSinTheta)
				minSinTheta = sinTheta;

			return sinTheta;
		}
	}
	return 1;
}

size_t CMesh::getOtherVertIdx(const CEdge& thisEdge, size_t triIdx) const
{
	const auto& tri = _tris[triIdx];
	int outIdx = 0;
	for (int i = 0; i < 3; i++) {
		if (tri[i] != thisEdge._vertIndex[0] && tri[i] != thisEdge._vertIndex[1]) {
			return tri[i];
		}
	}
	return -1;
}

void CMesh::calCurvatures(double edgeAngleRadians, bool multiCore) const
{
	if (_edgeCurvature.empty()) {
		buildCentroids(multiCore);
		buildNormals(multiCore);
		double sinAngle = sin(edgeAngleRadians);

		_edgeCurvature.resize(_edges.size());
		MultiCore::runLambda([this, sinAngle](size_t threadNum, size_t numThreads)->bool {
			for (size_t edgeIdx = threadNum; edgeIdx < _edgeCurvature.size(); edgeIdx += numThreads) {
				_edgeCurvature[edgeIdx] = calEdgeCurvature(edgeIdx, sinAngle);
			}
			return true;
		}, multiCore);

		vector<int> counts;
		_vertCurvature.clear();
		_vertCurvature.resize(_vertices.size());
		counts.resize(_vertices.size());

		for (size_t edgeIdx = 0; edgeIdx < _edgeCurvature.size(); edgeIdx++) {
			const auto& edge = _edges[edgeIdx];
			double curv = _edgeCurvature[edgeIdx];
			if (curv <= 0)
				continue;

			for (int i = 0; i < 2; i++) {
				size_t vertIdx = edge._vertIndex[i];
				counts[vertIdx]++;
				_vertCurvature[vertIdx] += curv;
			}
		}

		for (size_t vertIdx = 0; vertIdx < _vertices.size(); vertIdx++) {
			if (counts[vertIdx] > 0)
				_vertCurvature[vertIdx] /= counts[vertIdx];
		}
	}
}

double CMesh::calEdgeCurvature(size_t edgeIdx, double sinEdgeAngle) const
{
	const auto& edge = _edges[edgeIdx];
	if (edge._numFaces != 2)
		return 0;

	auto seg = getEdgesLineSeg(edgeIdx);
	const Vector3d& origin = seg._pts[0];
	Vector3d vEdge = seg.calcDir();

	Vector3d norm0 = triUnitNormal(edge._faceIndices[0]);
	Vector3d norm1 = triUnitNormal(edge._faceIndices[1]);

	if (norm0.cross(norm1).norm() > sinEdgeAngle)
		return -1;

	Plane edgePlane(origin, vEdge);
	size_t vertIdx0 = getOtherVertIdx(edge, edge._faceIndices[0]);
	size_t vertIdx1 = getOtherVertIdx(edge, edge._faceIndices[1]);

	// Both edge vertices AND each of the opposite vertices lie on the actual surface
				// Project the points to a plane perpendicular to the edge and fit a circle through it
				// Fit a circle through tree points
	Vector3d triPt0 = edgePlane.projectPoint(_vertices[vertIdx0]._pt);
	Vector3d triPt1 = edgePlane.projectPoint(_vertices[vertIdx1]._pt);
	assert(fabs((triPt0 - origin).dot(vEdge)) < 1.0e-6);
	assert(fabs((triPt1 - origin).dot(vEdge)) < 1.0e-6);

	Vector3d vChord0 = triPt0 - origin;
	Vector3d vChord1 = triPt1 - origin;
	Vector3d midPt0 = origin + 0.5 * (vChord0);
	Vector3d midPt1 = origin + 0.5 * (vChord1);

	Plane midPlane(midPt0, vChord0.normalized());
	Vector3d ctr;
	double dist; // dist is from the center to the mid point of the chord NOT a point on the circle
	if (!midPlane.intersectLine(midPt1, midPt1 + norm1, ctr, dist))
		return 0;

	double radius = (ctr - origin).norm();

	assert(fabs((ctr - origin).dot(vEdge)) < 1.0e-6);
	radius = fabs(radius);

	double curv = 1 / radius;
	return curv;
}

size_t CMesh::findVerts(const BoundingBox& bbox, vector<SearchEntry>& vertIndices, BoxTestType contains) const {
	return _vertTree.find(bbox, vertIndices, contains);
}

size_t CMesh::findEdges(const BoundingBox& bbox, vector<SearchEntry>& edgeIndices, BoxTestType contains) const {
	vector<SearchEntry> allHits;
	if (!_edgeTree.find(bbox, allHits, contains))
		return false;

	for (const auto& hit : allHits) {
		const auto& edge = _edges[hit.getIndex()];
		if (contains == BoxTestType::Intersects) {
			if (bbox.intersects(edge.getSeg(this)))
				edgeIndices.push_back(hit);
		} else {
			int numInBounds = 0;
			for (int i = 0; i < 2; i++) {
				const auto& v = _vertices[edge._vertIndex[i]];
				if (bbox.contains(v._pt))
					numInBounds++;
			}
			if (numInBounds == 2)
				edgeIndices.push_back(hit);
		}
	}

	return !edgeIndices.empty();
}

size_t CMesh::findTris(const BoundingBox& bbox, vector<SearchEntry>& triIndices, BoxTestType contains) const {
	vector<SearchEntry> temp;
	size_t numHits = _triTree.find(bbox, temp, contains);
	if (numHits > 0) {
		for (const auto& triEntry : temp) {
			const auto& tri = getTri(triEntry.getIndex());
			size_t triIdx = triEntry.getIndex();
				
			Vector3d pts[] = {
				getVert(tri[0])._pt,
				getVert(tri[1])._pt,
				getVert(tri[2])._pt,
			};

			if (contains == BoxTestType::Contains && bbox.contains(triEntry.getBBox()))
				triIndices.push_back(triEntry);
			else if (bbox.intersectsTriangle(pts)) {
				triIndices.push_back(triEntry);
			}
		}
		numHits = triIndices.size();
	}
	return numHits;
}

Vector3d CMesh::triCentroid(size_t triIdx) const {
	if (_useCentroidCache && triIdx < _centroids.size())
		return _centroids[triIdx];

	const auto& tri = _tris[triIdx];
	const Vector3d& pt0 = _vertices[tri[0]]._pt;
	const Vector3d& pt1 = _vertices[tri[1]]._pt;
	const Vector3d& pt2 = _vertices[tri[2]]._pt;

	Vector3d ctr = (pt0 + pt1 + pt2) / 3.0;
	return ctr;
}

Vector3d CMesh::triUnitNormal(size_t triIdx) const {
	if (_useNormalCache && triIdx < _normals.size())
		return _normals[triIdx];

	const auto& tri = _tris[triIdx];
	const Vector3d& pt0 = _vertices[tri[0]]._pt;
	const Vector3d& pt1 = _vertices[tri[1]]._pt;
	const Vector3d& pt2 = _vertices[tri[2]]._pt;

	Vector3d v0 = pt1 - pt0;
	Vector3d v1 = pt2 - pt0;
	Vector3d normal = v0.cross(v1).normalized();
	return normal;
}

double CMesh::triArea(size_t triIdx) const
{
	const auto& tri = _tris[triIdx];
	const Vector3d& pt0 = _vertices[tri[0]]._pt;
	const Vector3d& pt1 = _vertices[tri[1]]._pt;
	const Vector3d& pt2 = _vertices[tri[2]]._pt;

	Vector3d v0 = pt1 - pt0;
	Vector3d v1 = pt2 - pt0;
	double area = v0.cross(v1).norm() / 2;
	return area;

}

double CMesh::triAspectRatio(size_t triIdx) const
{
	const double idealL = sqrt(0.75);
	const auto& tri = _tris[triIdx];
	double maxRatio = 0;
	for (size_t i = 0; i < 3; i++) {
		size_t j = (i + 1) % 3;
		size_t k = (j + 1) % 3;

		const auto& pt0 = _vertices[tri[i]]._pt;
		const auto& pt1 = _vertices[tri[j]]._pt;
		const auto& pt2 = _vertices[tri[k]]._pt;

		// calculate the perpendicular distance to the 3rd vertex
		Vector3d v0 = pt1 - pt0;
		double l = v0.norm();
		v0 /= l;
		Vector3d v1 = pt2 - pt0;
		v1 = v1 - v0 * v0.dot(v1);

		double d = v1.norm();
		double ratio = l > d ? l / d : d / l;

		if (ratio > maxRatio)
			maxRatio = ratio;
	}

	return maxRatio / 2;
}

Vector3d CMesh::vertUnitNormal(size_t vertIdx) const
{
	Vector3d result(0, 0, 0);

	const auto& vert = _vertices[vertIdx];
	const auto& tris = vert._faceIndices;
	for (size_t triIdx : tris) {
		result += triUnitNormal(triIdx);
	}

	result.normalize();
	return result;
}


double CMesh::edgeCurvature(size_t edgeIdx) const
{
	if (_edgeCurvature.empty())
		calCurvatures(true);

	if (edgeIdx < _edgeCurvature.size())
		return _edgeCurvature[edgeIdx];
	return 0;
}

double CMesh::edgeLength(size_t edgeIdx) const
{
	const auto& edge = _edges[edgeIdx];
	const Vector3d& pt0 = _vertices[edge._vertIndex[0]]._pt;
	const Vector3d& pt1 = _vertices[edge._vertIndex[1]]._pt;
	return (pt1 - pt0).norm();
}

void CMesh::squeezeSkinnyTriangles(double minAngleDegrees)
{
	for (size_t vertIdx = 0; vertIdx < _vertices.size(); vertIdx++) {
		auto& vert = _vertices[vertIdx];

		if (vert._faceIndices.size() <= 10)
			continue;

		bool done = false;
		while (!done) {
			auto& vertFaces = vert._faceIndices;
			double minSinTheta = sin(minAngleDegrees / 180.0 * M_PI);
			size_t edgeIdx = -1;
			for (size_t faceIdx : vertFaces) {
				size_t oppositeEdgeIdx;
				double sinTheta = calSinVertexAngle(faceIdx, vertIdx, oppositeEdgeIdx);
				if (sinTheta < minSinTheta) {
					minSinTheta = sinTheta;
					edgeIdx = oppositeEdgeIdx;
				}
			}

			if (edgeIdx == -1)
				break;
			squeezeEdge(edgeIdx);
		}
	}
}

bool CMesh::verifyFindAllTris() const {
	bool result = true;
	for (size_t triIdx = 0; triIdx < _tris.size(); triIdx++) {
		BoundingBox triBox = getTriBBox(triIdx);

		vector<SearchEntry> entries;
		_triTree.find(triBox, entries);
		bool found = false;
		for (const auto& entry : entries) {
			found = found || entry.getIndex() == triIdx;
		}
		if (!found) {
			cout << "Error: Could not find newly added tri box.\n";
			result = false;
		}
	}

	for (size_t triIdx = 0; triIdx < _tris.size(); triIdx++) {
		const auto& tri = _tris[triIdx];
		for (int i = 0; i < 3; i++) {
			Vector3d origin = _vertices[tri[i]]._pt;
			origin[0] = -1;
			Ray ray(origin, Vector3d(1, 0, 0));
			vector<size_t> hits;
			_triTree.biDirRayCast(ray, hits);
			bool found = false;
			for (size_t hit : hits) {
				found = found || hit == triIdx;
			}
			if (!found) {
				cout << "Error: Could not find newly added tri box using ray cast. Idx: " << triIdx << ":" << i << "\n";
				result = false;
			}
		}
	}

	return result;
}

const vector<float>& CMesh::getGlTriPoints() const
{
	if (_glTriPoints.size() != 3 * 3 * _tris.size()) {
		_glTriPoints.resize(3 * 3 * _tris.size());
		size_t idx = 0;
		for (size_t triIdx = 0; triIdx < _tris.size(); triIdx++) {
			const auto& vertIndices = _tris[triIdx];
			for (size_t i = 0; i < 3; i++) {
				const auto& pt = _vertices[vertIndices[i]]._pt;
				_glTriPoints[idx++] = (float)pt[0];
				_glTriPoints[idx++] = (float)pt[1];
				_glTriPoints[idx++] = (float)pt[2];
			}
		}
	}
	return _glTriPoints;
}

const vector<float>& CMesh::getGlTriNormals(bool smoothed) const
{
	buildNormals();
	if (_glTriNormals.size() != 3 * 3 * _tris.size()) { // _vertices is a 3 vector, _glTriNormals is floats
		_glTriNormals.resize(3 * 3 * _tris.size());
		size_t idx = 0;
		for (size_t triIdx = 0; triIdx < _tris.size(); triIdx++) {
			const auto& norm = _normals[triIdx];
			for (size_t i = 0; i < 3; i++) {
				_glTriNormals[idx++] = (float)norm[0];
				_glTriNormals[idx++] = (float)norm[1];
				_glTriNormals[idx++] = (float)norm[2];
			}
		}
	}
	return _glTriNormals;
}

const vector<float>& CMesh::getGlTriParams() const
{
	if (_glTriParams.size() != 3 * 2 * _tris.size()) {
		_glTriParams.resize(3 * 2 * _tris.size(), 0);
	}
	return _glTriParams;
}

const vector<unsigned int>& CMesh::getGlTriIndices() const
{
	if (_glTriPoints.empty())
		getGlTriPoints();

	if (_glTriIndices.size() != _glTriPoints.size() / 3) { // _vertices is a 3 vector, _glTriNormals is floats
		_glTriIndices.resize(_glTriPoints.size() / 3);
		for (size_t idx = 0; idx < _glTriIndices.size(); idx++) {
			_glTriIndices[idx] = (unsigned int)idx;
		}
	}
	return _glTriIndices;
}

void CMesh::getGlEdges(std::vector<float>& points, std::vector<unsigned int>& indices, bool multiCore) // size = GlPoints.size() / 3
{
	points.clear();
	indices.clear();

	points.reserve(2 * 3 * _edges.size());
	indices.reserve(2 * _edges.size());

	unsigned int indexCount = 0;
	for (size_t edgeIdx = 0; edgeIdx < _edges.size(); edgeIdx++) {
		float curv = (float)_edgeCurvature[edgeIdx];
		const auto& edge = _edges[edgeIdx];
		for (int i = 0; i < 2; i++) {
			const auto& pt = _vertices[edge._vertIndex[i]]._pt;
			for (int j = 0; j < 3; j++) {
				points.push_back((float)pt[j]);
			}
		}
	}
	points.shrink_to_fit();
	indices.shrink_to_fit();
}

void CMesh::dumpObj(ostream& out) const {
	out << "Vertices\n";
	for (const auto& vert : _vertices) {
		const auto& p = vert._pt;
		out << "v " << p[0] << " " << p[1] << " " << p[2] << "\n";
	}

	out << "tris\n";
	for (const auto& tri : _tris) {
		out << "f " << (tri[0] + 1) << " " << (tri[1] + 1) << " " << (tri[2] + 1) << "\n";
	}
}

void CMesh::dumpModelSharpEdgesObj(ostream& out, double sinAngle) const {
	out << "Vertices\n";
	for (const auto& vert : _vertices) {
		const auto& p = vert._pt;
		out << "v " << p[0] << " " << p[1] << " " << p[2] << "\n";
	}

	out << "edges\n";
	for (size_t i = 0; i < _edges.size(); i++) {
		if (this->isEdgeSharp(i, sinAngle)) {
			const auto& edge = _edges[i];
			out << "l " << (edge._vertIndex[0] + 1) << " " << (edge._vertIndex[1] + 1) << "\n";
		}
	}
}

void CMesh::save(ostream& out) const {
	out << "Mesh version 1\n";

	out << "#Verts: " << _vertices.size() << "\n";
	for (const auto& vert : _vertices)
		vert.save(out);


	out << "#Tris: " << _tris.size() << "\n";
	for (const auto& tri : _tris)
		out << "t " << tri[0] << " " << tri[1] << " " << tri[2] << "\n";

}

bool CMesh::read(istream& in) {
	string str0, str1;
	int version;
	in >> str0 >> str1 >> version;
	if (str0 != "Mesh" || str1 != "version")
		return false;

	size_t numVerts;
	in >> str0 >> numVerts;
	if (str0 != "#Verts:")
		return false;

	BoundingBox bbox;
	_vertices.resize(numVerts);
	for (size_t i = 0; i < numVerts; i++) {
		if (!_vertices[i].read(in))
			return false;
		bbox.merge(BoundingBox(_vertices[i]._pt));
	}

	bbox.grow(SAME_DIST_TOL);

	_vertTree.reset(bbox);
	_edgeTree.reset(bbox);
	_triTree.reset(bbox);

	for (size_t i = 0; i < numVerts; i++) {
		_vertTree.add(BoundingBox(_vertices[i]._pt), i);
	}

	size_t numTris;
	in >> str0 >> numTris;
	if (str0 != "#Tris:")
		return false;

	_tris.resize(numTris);
	for (size_t i = 0; i < numTris; i++) {
		auto& tri = _tris[i];
		in >> str1 >> tri[0] >> tri[1] >> tri[2];
		if (str1 != "t")
			return false;
		for (int j = 0; j < 3; j++) {
			addEdge(tri[j], tri[(j + 1) % 3]);
		}
		BoundingBox triBox;
		triBox.merge(_vertices[tri[0]]._pt);
		triBox.merge(_vertices[tri[1]]._pt);
		triBox.merge(_vertices[tri[2]]._pt);
		_triTree.add(triBox, i);
	}

	return true;
}

bool CMesh::testRemoveTri(size_t idx)
{
	return removeTri(idx);
}

bool CMesh::testSqueezeEdge(size_t idx)
{

	size_t worstVertIdx = -1, maxEdges = 0;
	for (size_t i = 0; i < numVertices(); i++) {
		const auto& vert = getVert(i);
		const auto& vertEdgeIndices = vert._edgeIndices;
		if (vertEdgeIndices.size() > maxEdges) {
			maxEdges = vertEdgeIndices.size();
			worstVertIdx = i;
		}
	}

	map<double, vector<size_t>> aspectRatioToShortEdgeIdxMap; // Using a vector for indices is overkill, but there is a remote chance of a duplicated aspect ratio
	auto& vert = getVert(worstVertIdx);
	auto& vertFaces = vert._faceIndices;

	for (size_t faceIdx : vertFaces) {
		size_t oppositeEdgeIdx;
		double sinTheta = calSinVertexAngle(faceIdx, worstVertIdx, oppositeEdgeIdx);
		auto iter = aspectRatioToShortEdgeIdxMap.find(sinTheta);
		if (iter == aspectRatioToShortEdgeIdxMap.end())
			iter = aspectRatioToShortEdgeIdxMap.insert(make_pair(sinTheta, vector<size_t>())).first;
		iter->second.push_back(oppositeEdgeIdx);
	}

	auto iter = aspectRatioToShortEdgeIdxMap.begin();
	for (size_t i = 0; i < aspectRatioToShortEdgeIdxMap.size(); i++) {
		if (i == idx) {
			auto& pair = *iter;
			const auto& edgeIds = pair.second;
			for (size_t edgeId : edgeIds) {
				squeezeEdge(edgeId);
			}
			break;
		}
		iter++;
	}

	return true;
}


}
