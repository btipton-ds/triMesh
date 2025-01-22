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

#include <cmath>

#include <../../stlReader/include/readWriteStl.h>
#include <tm_lineSegment.h>
#include <tm_ray.h>
#include <triMesh.h>
#include <triMeshPatch.h>
#include <MultiCoreUtil.h>
#include <tm_proxyEdges.h>
#include <tm_proxyTriangles.h>
#include <tm_proxyVertices.h>
#include <tm_ioUtil.h>

using namespace std;

using namespace TriMesh;

namespace
{
	const double g_maxLengthScale = 1 / 1.5;
}
atomic<size_t> CMesh::_statId = 0;

CMesh::CMesh(const CMeshRepoPtr& pRepo)
	: _pRepo(pRepo)
	, _edges(this)
	, _vertices(this)
	, _tris(this)
	, _id(_statId++)
	, _changeNumber(0)
	, _pVertTree(make_shared<SearchTree>())
	, _pEdgeTree(make_shared<SearchTree>())
	, _pTriTree(make_shared<SearchTree>())
{
}

CMesh::CMesh(const Vector3d& min, const Vector3d& max, const CMeshRepoPtr& pRepo)
	: _pRepo(pRepo)
	, _edges(this)
	, _vertices(this)
	, _tris(this)
	, _id(_statId++)
	, _pVertTree(make_shared<SearchTree>(BoundingBox(min, max)))
	, _pEdgeTree(make_shared<SearchTree>(BoundingBox(min, max)))
	, _pTriTree(make_shared<SearchTree>(BoundingBox(min, max)))
{
}

void CMesh::reset(const BoundingBox& bbox) {
	BoundingBox bb(bbox);
	bb.grow(SAME_DIST_TOL);

	_edgeToIdxMap.clear();

	_glTriPoints.clear();
	_glTriNormals.clear();
	_glTriParams.clear();
	_glTriCurvatureColors.clear();
	_glTriIndices.clear();

	_vertices.clear();
	_edges.clear();
	_tris.clear();


	_pVertTree->reset(bb);
	_pEdgeTree->reset(bb);
	_pTriTree->reset(bb);

	// These are cached data and can be reproduced. Marked mutable so they can be accessed from const getters
	_centroids.clear();
	_sharpEdgeIndices.clear();
	_sharpEdgeLoops.clear();
	_normals.clear();
	_edgeCurvature.clear();
	_vertCurvature.clear();
	_triGap.clear();
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
/*
	for (const auto& edge : _edges) {
		out << edge._vertIndex[0] << " " << edge._vertIndex[1] << " " << edge._numFaces << " " << edge._faceIndices[0];
		if (edge._numFaces == 2)
			out << " " << edge._faceIndices[1];
		out << "\n";
	}
*/
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
/*
		if (numFaces != edge._numFaces) {
			return false;
		}

		if (numFaces == 2 && d != edge._faceIndices[1])
			return false;
*/
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
	_pTriTree->dump(out);

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
		for (int i = 0; i < edge.numFaces(_id); i++) {
			if (edge.getTriIdx(_id, i) == triIdx)
				return true;
		}
	}
	return false;
}

bool CMesh::vertReferencesTri(size_t vertIdx, size_t triIdx) const
{
	if (vertIdx < _vertices.size() && triIdx < _tris.size()) {
		auto pFaceIndices = _vertices[vertIdx].getFaceIndices(_id);
		if (pFaceIndices) {
			if (find(pFaceIndices->begin(), pFaceIndices->end(), triIdx) != pFaceIndices->end())
				return true;
		}
		return false;
	}
	return false;
}

bool CMesh::vertReferencesEdge(size_t vertIdx, size_t edgeIdx) const
{
	if (vertIdx < _vertices.size() && edgeIdx < _edges.size()) {
		auto pEdgeIndices = _vertices[vertIdx].getEdgeIndices(_id);
		if (pEdgeIndices) {
			if (find(pEdgeIndices->begin(), pEdgeIndices->end(), edgeIdx) != pEdgeIndices->end())
				return true;
		}
		return false;
	}
	return false;
}

void CMesh::squeezeEdge(size_t idx)
{
	auto& edge = _edges[idx];
	if (edge.numFaces(_id) != 2)
		return;

	Vector3d dir = getEdgesLineSeg(idx).calcDir();

	double minCp[] = { DBL_MAX, DBL_MAX };
	double minDist[] = { DBL_MAX, DBL_MAX };
	for (size_t i = 0; i < 2; i++) {
		const auto& vert = _vertices[edge._vertIndex[i]];
		auto pEdgeIndices = vert.getEdgeIndices(_id);
		if (pEdgeIndices) {
			for (size_t edgeIdx : *pEdgeIndices) {
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

	size_t faceIdx0 = edge.getTriIdx(_id, 0);
	size_t faceIdx1 = edge.getTriIdx(_id, 1);

	// Remove the larger one first. If not, the first one can cause the second to move
	if (faceIdx1 < faceIdx0)
		swap(faceIdx0, faceIdx1);

	removeTri(faceIdx1);
	removeTri(faceIdx0);
	mergeVertices(vertIdxToKeep, vertIdxToRemove);
}

CMeshPtr CMesh::fix(double maxEdgeLength)
{
	const double tol = 1.0e-6;
	const auto& bbox = getBBox();
	CMeshPtr result = make_shared<CMesh>(bbox);
	result->enableOption(OPT_SKIP_DEGEN);
	bool modified = false;
	for (auto idx : _tris) {
		Vector3d pts[] = {
			_vertices[idx[0]]._pt,
			_vertices[idx[1]]._pt,
			_vertices[idx[2]]._pt
		};

		double len[] = {
			(pts[1] - pts[0]).norm(),
			(pts[2] - pts[1]).norm(),
			(pts[0] - pts[2]).norm(),
		};

		size_t shortestIdx = -1;
		double minLen = DBL_MAX, maxLen = -1;
		for (int i = 0; i < 3; i++) {
			if (len[i] < minLen) {
				minLen = len[i];
				shortestIdx = i;
			}
			if (len[i] > maxLen) {
				maxLen = len[i];
			}
		}

		if (maxLen > maxEdgeLength) {
			modified = true;
			Vector3d pt0, pt1, pt2;
			switch (shortestIdx) {
			case 0:
				pt0 = pts[2];
				pt1 = pts[0];
				pt2 = pts[1];
				break;
			case 1:
				pt0 = pts[0];
				pt1 = pts[1];
				pt2 = pts[2];
				break;
			case 2:
				pt0 = pts[1];
				pt1 = pts[2];
				pt2 = pts[0];
				break;
			}

			Vector3d v0 = pt1 - pt0;
			Vector3d v1 = pt2 - pt0;
			Vector3d pt3 = pt0 + 0.5 * v0;
			Vector3d pt4 = pt0 + 0.5 * v1;

			result->addQuad(pt1, pt2, pt4, pt3);
			result->addTriangle(pt0, pt3, pt4);
		}
	}

	if (modified)
		return result;
	else
		return nullptr;
}

bool CMesh::removeTri(size_t triIdx)
{
	bool result = true;
	assert(triIdx < _tris.size());
	
	BoundingBox triBBox = getTriBBox(triIdx);
	_pTriTree->remove(triBBox, triIdx);

	set<size_t> edgesToRemove;
	const auto& tri = _tris[triIdx];
	for (int i = 0; i < 3; i++) {
		int j = (i + 1) % 3;
		size_t vertIdx0 = tri[i];
		size_t vertIdx1 = tri[j];
		size_t edgeIdx = findEdge(vertIdx0, vertIdx1);

		auto& vert0 = _vertices[vertIdx0];
		vert0.removeFaceIndex(_id, triIdx);

		auto& edge = _edges[edgeIdx];
		edge.removeFaceIndex(_id, triIdx);
		if (edge.numFaces(_id) == 0) {
			auto& vert1 = _vertices[vertIdx1];
			vert0.removeEdgeIndex(_id, edgeIdx);
			vert1.removeEdgeIndex(_id, edgeIdx);
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
	_pTriTree->remove(srcBBox, srcIdx);

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
			vert0.changeFaceIndex(_id, srcIdx, triIdx);
			vert0.changeEdgeIndex(_id, srcIdx, triIdx);

			auto& edge = _edges[edgeIdx];
			edge.changeFaceIndex(_id, srcIdx, triIdx);
		}
		BoundingBox bbox = getTriBBox(triIdx);
		_pTriTree->add(bbox, triIdx);
	}

	verifyTris(triIdx);
	return true;
}

bool CMesh::deleteEdgeFromStorage(size_t edgeIdx)
{
	size_t srcIdx = _edges.size() - 1;

	// Must delete from BOTH _edgeTree and _edgeToIdxMap
	BoundingBox bbox = getEdgeBBox(edgeIdx), srcBbox = getEdgeBBox(srcIdx);
	_pEdgeTree->remove(bbox, edgeIdx);
	_edgeToIdxMap.erase(_edges[edgeIdx]);
	_pEdgeTree->remove(srcBbox, srcIdx);
	_edgeToIdxMap.erase(_edges[srcIdx]);

	{
		auto& edge = _edges[edgeIdx];

		size_t vertIdx0 = edge._vertIndex[0];
		size_t vertIdx1 = edge._vertIndex[1];
		auto& vert0 = _vertices[vertIdx0];
		auto& vert1 = _vertices[vertIdx0];

#if FULL_TESTS
		assert(!vert0.containsEdgeIndex(edgeIdx));
		assert(!vert1.containsEdgeIndex(edgeIdx));
		assert(edge._numFaces == 0);
#endif
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
		vert0.changeEdgeIndex(_id, srcIdx, edgeIdx);
		vert1.changeEdgeIndex(_id, srcIdx, edgeIdx);

		// Must add back to BOTH _edgeTree and _edgeToIdxMap
		_pEdgeTree->add(srcBbox, edgeIdx);
		_edgeToIdxMap.insert(make_pair((CEdgeGeo)edge, edgeIdx));
	}

	return true;
}

void CMesh::mergeVertices(size_t vertIdxToKeep, size_t vertIdxToRemove)
{
	auto& vertToRemove = _vertices[vertIdxToRemove];
	auto triIdsToModify = vertToRemove.getFaceIndices(_id);

	for (size_t triIdx : *triIdsToModify) {
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
		vector<size_t> foundTris;
		_pTriTree->find(bbox, foundTris);
		bool found = false;
		for (const auto& idx : foundTris) {
			if (idx == triIdx) {
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
		vector<size_t> foundTris;
		_pVertTree->find(bbox, foundTris);
		bool found = false;
		for (const auto& idx : foundTris) {
			if (idx == vertIdx) {
				found = true;
				break;
			}
		}
		if (!found)
			return false;
	}

	const auto& vert = _vertices[vertIdx];

	for (auto triIdx : *vert.getFaceIndices(_id)) {
		if (!vertReferencesTri(vertIdx, triIdx))
			return false;
	}

	for (auto edgeIdx : *vert.getEdgeIndices(_id)) {
		if (!vertReferencesEdge(vertIdx, edgeIdx))
			return false;
	}

	return true;
}

bool CMesh::verifyEdges(size_t edgeIdx, bool allowEmpty) const
{
	{
		BoundingBox bbox = getEdgeBBox(edgeIdx);
		vector<size_t> foundTris;
		_pEdgeTree->find(bbox, foundTris);
		bool found = false;
		for (const auto& idx : foundTris) {
			if (idx == edgeIdx) {
				found = true;
				break;
			}
		}
		if (!found)
			return false;
	}

	const auto& edge = _edges[edgeIdx];
	if (edge.numFaces(_id) == 0)
		return allowEmpty;

	if (edge.numFaces(_id) > 2)
		return false;

	for (int i = 0; i < 2; i++) {
		if (!vertReferencesEdge(edge._vertIndex[i], edgeIdx))
			return false;
	}

	for (int i = 0; i < edge.numFaces(_id); i++) {
		bool found = false;
		if (!triContainsEdge(edge.getTriIdx(_id, i), edgeIdx))
			return false;
	}

	return true;
}

size_t CMesh::addEdge(size_t vertIdx0, size_t vertIdx1) {
	assert(vertIdx0 < _vertices.size() && vertIdx1 < _vertices.size());

	CEdge edge(vertIdx0, vertIdx1);
	auto iter = _edgeToIdxMap.find(edge);
	if (iter != _edgeToIdxMap.end())
		return iter->second;
	size_t result = _edges.size();
	_edges.push_back(edge);
	_edgeToIdxMap.insert(make_pair((CEdgeGeo)edge, result));

	auto& vert0 = _vertices[vertIdx0];
	auto& vert1 = _vertices[vertIdx1];

	vert0.addEdgeIndex(_id, result);
	vert1.addEdgeIndex(_id, result);

	BoundingBox edgeBox;
	edgeBox.merge(vert0._pt);
	edgeBox.merge(vert1._pt);
	_pEdgeTree->add(edgeBox, result);

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
	if (isEnabled(OPT_SKIP_DEGEN) && areTriPointsDegenerate(tri)) {
		return -1;
	}
	BoundingBox triBox;
	for (int i = 0; i < 3; i++) {
		const auto& vert = _vertices[tri[i]];
		triBox.merge(vert._pt);
	}

	vector<size_t> triIndices;
	_pTriTree->find(triBox, triIndices);
	for (const auto& triIdx : triIndices) {
		if (sameTri(_tris[triIdx], tri))
			return triIdx;
	}

	size_t triIdx = _tris.size();
	_tris.push_back(tri);
	for (int i = 0; i < 3; i++) {
		int j = (i + 1) % 3;
		size_t edgeIdx = addEdge(tri[i], tri[j]); // addEdge adds itself to its vertices

		auto& vert = _vertices[tri[i]];
		vert.addFaceIndex(_id, triIdx);

		auto& edge = _edges[edgeIdx];
		if (_enforceManifold && edge.numFaces(_id) >= 2)
			assert(!"Non manifold mesh");
		edge.addFaceIndex(_id, triIdx);
	}
	triBox.grow(SAME_DIST_TOL);
	_pTriTree->add(triBox, triIdx);

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

bool CMesh::intersectsTri(const LineSegmentd& seg, size_t idx, double tol, RayHitd& hit) const
{
	const auto& tri = _tris[idx];
	const Vector3d* pts[] = {
		&getVert(tri[0])._pt,
		&getVert(tri[1])._pt,
		&getVert(tri[2])._pt,
	};

	if (seg.intersectTri(pts, hit, tol)) {
		hit.triIdx = idx;
		return true;
	}

	return false;
}

bool CMesh::bboxIntersectsTri(const BoundingBox& bbox, size_t idx) const
{
	const auto& tri = _tris[idx];

	return bbox.intersectsOrContains(_vertices[tri[0]]._pt, _vertices[tri[1]]._pt, _vertices[tri[2]]._pt, SAME_DIST_TOL);
}

bool CMesh::bboxIntersectsEdge(const BoundingBox& bbox, size_t idx) const
{
	auto seg = getEdgesLineSeg(idx);
	return bbox.intersectsOrContains(seg, SAME_DIST_TOL, -1);
}

LineSegmentd CMesh::getEdgesLineSeg(size_t edgeIdx) const
{
	const CEdge& edge = _edges[edgeIdx];
	return edge.getSeg(this);
}

bool CMesh::isEdgeSharp(size_t edgeIdx, double sinEdgeAngle) const {
	const CEdge& edge = _edges[edgeIdx];
	if (edge.numFaces(_id) < 2)
		return true;
	const auto seg = getEdgesLineSeg(edgeIdx);
	const Vector3d edgeV = seg.calcDir();
	if (edge.getTriIdx(_id, 0) == edge.getTriIdx(_id, 1)) {
		cout << "Duplicated faceIndex\n";
	}
	const Vector3d norm0 = triUnitNormal(edge.getTriIdx(_id, 0));
	const Vector3d norm1 = triUnitNormal(edge.getTriIdx(_id, 1));

	double sinTheta = norm0.cross(norm1).norm();
	bool isSharp = sinTheta > sinEdgeAngle;
	return isSharp;
}

bool CMesh::createPatches(const vector<size_t>& triIndices, double sinSharpEdgeAngle, vector<PatchPtr>& patches) const
{
	set<size_t> triSet;
	triSet.insert(triIndices.begin(), triIndices.end());

	while (!triSet.empty()) {
		PatchPtr pPatch = make_shared<Patch>();
		vector<size_t> face, stack;
		size_t triIdx = *triSet.begin();
		stack.push_back(triIdx);
		triSet.erase(triIdx);
		while (!stack.empty()) {
			size_t triIdx = stack.back();
			stack.pop_back();
			face.push_back(triIdx);
			const auto& tri = _tris[triIdx];
			for (int i = 0; i < 3; i++) {
				int j = (i + 1) % 3;
				size_t triEdgeIdx = findEdge(CEdge(tri[i], tri[j]));
				const auto& edge = _edges[triEdgeIdx];
				for (int j = 0; j < edge.numFaces(_id); j++) {
					size_t nextTriIdx = edge.getTriIdx(_id, j);
					if (triSet.contains(nextTriIdx)) {
						stack.push_back(nextTriIdx);
						triSet.erase(nextTriIdx);
					}
				}
			}
		}
		if (!face.empty())
			pPatch->addFace(face);

		if (!pPatch->empty())
			patches.push_back(pPatch);
	}

	for (const auto& pPatch : patches)
		pPatch->finishCreation(this, sinSharpEdgeAngle);

	return !patches.empty();
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

size_t CMesh::createSharpEdgeVertexLines(size_t sharpVertIdx, set<size_t>& availEdges, double sharpEdgeAngleRadians, vector<vector<size_t>>& vertIndices) const
{
	const double sinEdgeAngle = sin(sharpEdgeAngleRadians);
	const auto& vert = getVert(sharpVertIdx);
	const auto& vEdges = *vert.getEdgeIndices(_id);
	for (size_t edgeIdx : vEdges) {
		if (availEdges.contains(edgeIdx) && isEdgeSharp(edgeIdx, sinEdgeAngle)) {
			vector<size_t> vertLine;
			vertLine.push_back(sharpVertIdx);
			while (addVertexToEdgeLine(vertLine, availEdges, sinEdgeAngle));
			if (vertLine.size() > 1)
				vertIndices.push_back(vertLine);
		}
	}

	return vertIndices.size();
}

bool CMesh::addVertexToEdgeLine(vector<size_t>& vertLine, set<size_t>& availEdges, double sinEdgeAngle) const
{
	size_t lastIdx = vertLine.back();
	const auto& vert = getVert(lastIdx);
	const auto& edgeIndices = *vert.getEdgeIndices(_id);
	for (size_t edgeIdx : edgeIndices) {
		if (availEdges.contains(edgeIdx) && isEdgeSharp(edgeIdx, sinEdgeAngle)) {
			const auto& edge = getEdge(edgeIdx);
			size_t nextIdx = edge.otherVertIdx(lastIdx);
			if (nextIdx != 0) {
				vertLine.push_back(nextIdx);
				availEdges.erase(edgeIdx);
				return true;
			}
		}
	}

	return false;
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
		if (edge.numFaces(_id) == 1)
			result++;
	}
	return result;
}

size_t CMesh::numBytes() const
{
	size_t result = 0;

	result += _edgeToIdxMap.size() * sizeof(pair<CEdgeGeo, size_t>);
	result += _glTriPoints.capacity() * sizeof(size_t);
	result += _glTriNormals.capacity() * sizeof(size_t);
	result += _glTriParams.capacity() * sizeof(size_t);
	result += _glTriCurvatureColors.capacity() * sizeof(size_t);
	result += _glTriIndices.capacity() * sizeof(unsigned int);

	result += _edges.numBytes();
	result += _vertices.numBytes();
	result += _tris.numBytes();

	result += _pVertTree->numBytes();
	result += _pEdgeTree->numBytes();
	result += _pTriTree->numBytes();

	result += _centroids.capacity() * sizeof(Vector3d);
	result += _normals.capacity() * sizeof(Vector3d);
	result += _sharpEdgeIndices.capacity() * sizeof(size_t);
	result += _sharpEdgeLoops.capacity() * sizeof(vector<size_t>);
	for (const auto& vec : _sharpEdgeLoops) {
		result += vec.capacity() * sizeof(size_t);
	}

	result += _edgeCurvature.capacity() * sizeof(double);
	result += _vertCurvature.capacity() * sizeof(double);
	result += _triGap.capacity() * sizeof(double);

	return result;
}

bool CMesh::isClosed() const {
	return numLaminarEdges() == 0;
}

size_t CMesh::rayCast(const Ray<double>& ray, vector<RayHitd>& hits, bool biDir) const {
	buildNormals();
	vector<size_t> hitIndices;
	if (_pTriTree->biDirRayCast(ray, hitIndices) > 0) {
		for (size_t triIdx2 : hitIndices) {
			const auto& tri = _tris[triIdx2];
			const Vector3d* pts[] = {
				&_vertices[tri[0]]._pt,
				&_vertices[tri[1]]._pt,
				&_vertices[tri[2]]._pt,
			};

			RayHitd hit;
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

size_t CMesh::rayCast(const LineSegmentd& seg, vector<RayHitd>& hits, double tol) const {
	auto segLen = seg.calLength();
	vector<size_t> hitIndices;
	if (_pTriTree->biDirRayCast(seg.getRay(), hitIndices) > 0) {
		for (size_t triIdx2 : hitIndices) {
			const auto& tri = _tris[triIdx2];
			const Vector3d* pts[] = {
				&_vertices[tri[0]]._pt,
				&_vertices[tri[1]]._pt,
				&_vertices[tri[2]]._pt,
			};

			RayHitd hit;
			if (seg.intersectTri(pts, hit, SAME_DIST_TOL)) {
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

size_t CMesh::rayCast(size_t triIdx, vector<RayHitd>& hits, bool biDir) const {
	Vector3d ctr = triCentroid(triIdx);
	Vector3d norm = triUnitNormal(triIdx);
	Ray<double> ray(ctr, norm);

	vector<RayHitd> temp;
	rayCast(ray, temp);

	for (const auto& hit : temp) {
		if (hit.triIdx != triIdx)
			hits.push_back(hit);
	}
	return hits.size();
}

size_t CMesh::addVertex_d(const Vector3d& pt) {
	BoundingBox ptBBox(pt), thisBBox = getBBox();
	ptBBox.grow(SAME_DIST_TOL);
	if (!thisBBox.contains(pt, SAME_DIST_TOL)) {
		std::cout << "CMesh::addVertex box intersects: Error\n";
	}

	std::vector<size_t> results;
	_pVertTree->find(ptBBox, results);
	for (const auto& index : results) {
		if ((_vertices[index]._pt - pt).norm() < SAME_DIST_TOL) {
			return index;
		}
	}
	size_t result = _vertices.size();
	_vertices.push_back(pt);
	_pVertTree->add(ptBBox, result);

#if FULL_TESTS && defined(_DEBUG)
	// Testing
	std::vector<size_t> testResults;
	_pVertTree->find(ptBBox, testResults);
	int numFound = 0;
	for (const auto& index : testResults) {
		if (index == result)
			numFound++;
	}
	if (numFound != 1) {
		std::cout << "CMesh::addVertex numFound: Error. numFound: " << numFound << "\n";
		_pVertTree->find(ptBBox, testResults);
	}
#endif
	return result;
}


size_t CMesh::addTriangle_d(const Vector3d& pt0, const Vector3d& pt1, const Vector3d& pt2, double maxEdgeLength) {
	size_t idx0 = addVertex_d(pt0);
	size_t idx1 = addVertex_d(pt1);
	size_t idx2 = addVertex_d(pt2);

	if (idx0 == idx1 || idx0 == idx2 || idx1 == idx2)
		return stm1;
	return addTriangle(Vector3i(idx0, idx1, idx2));
}

size_t CMesh::addQuad_d(const Vector3d& qpt0, const Vector3d& qpt1, const Vector3d& qpt2, const Vector3d& qpt3, double maxEdgeLength)
{
	Vector3d qp[] = { qpt0, qpt1, qpt2, qpt3 };

	double edgeLen[] = {
		(qp[1] - qp[0]).norm(),
		(qp[2] - qp[1]).norm(),
		(qp[3] - qp[2]).norm(),
		(qp[0] - qp[3]).norm(),
	};

	double maxLen = 0;
	int maxIdx = 0;
	for (int i = 0; i < 4; i++) {
		if (edgeLen[i] > maxLen) {
			maxLen = edgeLen[i];
			maxIdx = i;
		}
	}

	if (maxEdgeLength != -1 && maxLen > maxEdgeLength) {
		Vector3d v0 = qp[1] - qp[0];
		Vector3d v1 = qp[2] - qp[0];
		Vector3d srcNorm = v1.cross(v0);

		int adjIdx = (maxIdx + 2) % 4;
		vector<Vector3d> edgePointsVec[2];
		for (int i = 0; i < 2; i++) {
			auto& ptVec = edgePointsVec[i];
			Vector3d pt0, pt1;
			double len;
			if (i == 0) {
				pt0 = qp[maxIdx];
				pt1 = qp[(maxIdx + 1) % 4];
				len = edgeLen[maxIdx];
			} else {
				pt0 = qp[adjIdx];
				pt1 = qp[(adjIdx + 1) % 4];
				len = edgeLen[adjIdx];
			}
			if (len > maxEdgeLength) {
				const double divLen = maxEdgeLength * g_maxLengthScale;
				size_t numSegs = (size_t)(len / divLen + 1);
				double l = len / numSegs;
				if (l > divLen)
					numSegs++;

				size_t numPts = numSegs + 1;
				if (numPts < 2)
					numPts = 2;
				ptVec.resize(numPts);
				for (size_t j = 0; j < numPts; j++) {
					double t = j / (numPts - 1.0);
					ptVec[j] = LERP(pt0, pt1, t);
				}
			} else {
				ptVec.resize(2);
				ptVec[0] = pt0;
				ptVec[1] = pt1;
			}
		}

		if (edgePointsVec[1].size() > edgePointsVec[0].size())
			std::swap(edgePointsVec[0], edgePointsVec[1]);

		double dist00 = (edgePointsVec[0].front() - edgePointsVec[1].front()).norm();
		double dist01 = (edgePointsVec[0].back() - edgePointsVec[1].front()).norm();
		if (dist01 < dist00) {
			reverse(edgePointsVec[1].begin(), edgePointsVec[1].end());
		}

		addTriangleStrip(edgePointsVec[0], edgePointsVec[1], srcNorm, maxEdgeLength);

		return _tris.size();
	}

	auto diagLen0 = (qpt2 - qpt0).norm();
	auto diagLen1 = (qpt3 - qpt1).norm();

	if (diagLen0 <= diagLen1) {
		addTriangle_d(qpt0, qpt1, qpt2, maxEdgeLength);
		addTriangle_d(qpt0, qpt2, qpt3, maxEdgeLength);
	} else {
		addTriangle_d(qpt1, qpt2, qpt3, maxEdgeLength);
		addTriangle_d(qpt1, qpt3, qpt0, maxEdgeLength);
	}

	return _tris.size();
}

double CMesh::findTriMinimumGap(size_t i) const {
	vector<RayHitd> hits;
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
			vector<RayHitd> hits;
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
			vector<RayHitd> hits;
			if (rayCast(triIdx, hits) != 0) {
				for (const auto& hit : hits) {
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
	if (getBBox().contains(src->getBBox(), SAME_DIST_TOL)) {
		for (const auto& tri : src->_tris) {
			CVertex pts[3];
			for (int i = 0; i < 3; i++) {
				pts[i] = src->getVert(tri[i]);
			}
			addTriangle(pts[0]._pt, pts[1]._pt, pts[2]._pt);
		}
	} else {
		auto bbox = _pTriTree->getBounds();
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
		MultiCore::runLambda([this, &src, destructive](size_t threadNum, size_t numThreads)->bool {
			for (size_t i = threadNum; i < src.size(); i += numThreads) {
				if (i % 2 == 0) {
					size_t j = i + 1;
					if (j < src.size() && src[j]) {
						auto pSrc = src[j];
#if FULL_TESTS
						assert(src[i]->getBBox().contains(pSrc->getBBox()));
#endif
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

	}

	merge(src.back(), destructive);

#else
	auto bbox = _pTriTree->getBounds();
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

void CMesh::calGaps(bool multiCore) const
{
	if (_triGap.empty()) {
		_triGap.resize(_tris.size());
		MultiCore::runLambda([this](size_t threadNum, size_t numThreads)->bool {
			for (size_t idx = threadNum; idx < _tris.size(); idx += numThreads) {
				double t = findTriMinimumGap(idx);
				if (t < 0)
					t = 0;
				_triGap[idx] = t;
			}
			return true;
		}, multiCore);
	}
}

double CMesh::calEdgeCurvature(size_t edgeIdx, double sinEdgeAngle) const
{
	const auto& edge = _edges[edgeIdx];
	if (edge.numFaces(_id) != 2)
		return 0;

	auto seg = getEdgesLineSeg(edgeIdx);
	const Vector3d& origin = seg._pts[0];
	Vector3d vEdge = seg.calcDir();

	Vector3d norm0 = triUnitNormal(edge.getTriIdx(_id, 0));
	Vector3d norm1 = triUnitNormal(edge.getTriIdx(_id, 1));

	double magCp = norm0.cross(norm1).norm();
	if (magCp > sinEdgeAngle)
		return -1;
	if (magCp < 1.0e-6)
		return 0;

	Planed edgePlane(origin, vEdge);
	size_t vertIdx0 = getOtherVertIdx(edge, edge.getTriIdx(_id, 0));
	size_t vertIdx1 = getOtherVertIdx(edge, edge.getTriIdx(_id, 1));

	// Both edge vertices AND each of the opposite vertices lie on the actual surface
				// Project the points to a plane perpendicular to the edge and fit a circle through it
				// Fit a circle through tree points
	Vector3d triPt0 = edgePlane.projectPoint(_vertices[vertIdx0]._pt);
	Vector3d triPt1 = edgePlane.projectPoint(_vertices[vertIdx1]._pt);

#if FULL_TESTS
	assert(fabs((triPt0 - origin).dot(vEdge)) < 1.0e-6);
	assert(fabs((triPt1 - origin).dot(vEdge)) < 1.0e-6);
#endif

	Vector3d vChord0 = triPt0 - origin;
	Vector3d vChord1 = triPt1 - origin;
	Vector3d midPt0 = origin + 0.5 * (vChord0);
	Vector3d midPt1 = origin + 0.5 * (vChord1);

	Planed midPlane(midPt0, vChord0.normalized());
	RayHitd hit; // dist is from the center to the mid point of the chord NOT a point on the circle
	if (!midPlane.intersectLine(midPt1, midPt1 + norm1, hit, SAME_DIST_TOL))
		return 0;

	double radius = (hit.hitPt - origin).norm();

#if FULL_TESTS
	assert(fabs((hit.hitPt - origin).dot(vEdge)) < 1.0e-6);
#endif
	radius = fabs(radius);

	double curv = 1 / radius;
	return curv;
}

size_t CMesh::findVerts(const BoundingBox& bbox, vector<SearchEntry>& vertIndices, BoxTestType contains) const {
	return _pVertTree->find(bbox, vertIndices, contains);
}

size_t CMesh::findVerts(const BoundingBox& bbox, vector<size_t>& vertIndices, BoxTestType contains) const {
	return _pVertTree->find(bbox, vertIndices, contains);
}

size_t CMesh::findEdges(const BoundingBox& bbox, vector<SearchEntry>& edgeIndices, BoxTestType contains) const {
	vector<SearchEntry> allHits;

	if (_pEdgeTree->find(bbox, allHits, contains))
		return processFoundEdges(allHits, bbox, edgeIndices, contains);

	return false;
}

size_t CMesh::processFoundEdges(const vector<SearchEntry>& allHits, const BoundingBox& bbox, vector<SearchEntry>& edgeIndices, BoxTestType contains) const
{
	for (const auto& hit : allHits) {
		const auto& edge = _edges[hit.getIndex()];
		if (contains == BoxTestType::Intersects) {
			if (bbox.intersectsOrContains(edge.getSeg(this), SAME_DIST_TOL, -1))
				edgeIndices.push_back(hit);
		}
		else {
			int numInBounds = 0;
			for (int i = 0; i < 2; i++) {
				const auto& v = _vertices[edge._vertIndex[i]];
				if (bbox.contains(v._pt, SAME_DIST_TOL))
					numInBounds++;
			}
			if (numInBounds == 2)
				edgeIndices.push_back(hit);
		}
	}

	return !edgeIndices.empty();
}

size_t CMesh::findEdges(const BoundingBox& bbox, vector<size_t>& edgeIndices, BoxTestType contains) const {
	vector<size_t> allHits;
	if (_pEdgeTree->find(bbox, allHits, contains))
		return processFoundEdges(allHits, bbox, edgeIndices, contains);

	return false;
}

size_t CMesh::processFoundEdges(const vector<size_t>& allHits, const BoundingBox& bbox, vector<size_t>& edgeIndices, BoxTestType contains) const
{

	for (const auto& hit : allHits) {
		const auto& edge = _edges[hit];
		if (contains == BoxTestType::Intersects) {
			if (bbox.intersectsOrContains(edge.getSeg(this), SAME_DIST_TOL, -1))
				edgeIndices.push_back(hit);
		}
		else {
			int numInBounds = 0;
			for (int i = 0; i < 2; i++) {
				const auto& v = _vertices[edge._vertIndex[i]];
				if (bbox.contains(v._pt, SAME_DIST_TOL))
					numInBounds++;
			}
			if (numInBounds == 2)
				edgeIndices.push_back(hit);
		}
	}

	return !edgeIndices.empty();
}

size_t CMesh::findTris(const BoundingBox& bbox, vector<SearchEntry>& triIndices, BoxTestType contains) const {
	vector<SearchEntry> allHits;

	if (_pTriTree->find(bbox, allHits, contains) > 0) {
		return processFoundTris(allHits, bbox, triIndices, contains);
	}

	return false;
}

size_t CMesh::processFoundTris(const vector<SearchEntry>& allHits, const BoundingBox& bbox, vector<SearchEntry>& triIndices, BoxTestType contains) const
{
	bool useContains = contains == BoxTestType::Contains;
	for (const auto& triEntry : allHits) {
		size_t idx = triEntry.getIndex();
		bool useEntry = useContains ? bbox.contains(triEntry.getBBox(), SAME_DIST_TOL) : bboxIntersectsTri(bbox, idx);
		if (useEntry && bboxIntersectsTri(bbox, idx)) {
			triIndices.push_back(triEntry);
		}
	}

	return triIndices.size();
}

size_t CMesh::findTris(const BoundingBox& bbox, vector<size_t>& triIndices, BoxTestType contains) const {
	vector<size_t> allHits;
	if (_pTriTree->find(bbox, allHits, contains) > 0) {
		return processFoundTris(allHits, bbox, triIndices, contains);
	}

	return false;
}

size_t CMesh::processFoundTris(const vector<size_t>& allHits, const BoundingBox& bbox, vector<size_t>& triIndices, BoxTestType contains) const
{
	bool useContains = contains == BoxTestType::Contains;
	triIndices.reserve(triIndices.size() + allHits.size());
	for (const auto& idx : allHits) {
		bool useEntry = useContains ? bbox.contains(getTriBBox(idx), SAME_DIST_TOL) : bboxIntersectsTri(bbox, idx);
		if (useEntry) {
			triIndices.push_back(idx);
		}
	}

	return triIndices.size();
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

Planed CMesh::triPlane(size_t triIdx) const
{
	Vector3d ctr = triCentroid(triIdx);
	Vector3d n = triUnitNormal(triIdx);
	return Planed(ctr, n);
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

double CMesh::triGap(size_t triIdx) const
{
	if (_triGap.empty()) {
		double t = findTriMinimumGap(triIdx);
		if (t < 0)
			t = 0;
		return t;
	}

	if (_triGap[triIdx] == -1) {
		double t = findTriMinimumGap(triIdx);
		if (t < 0)
			t = 0;
		_triGap[triIdx] = t;
	}

	return _triGap[triIdx];
}

Vector3d CMesh::vertUnitNormal(size_t vertIdx) const
{
	Vector3d result(0, 0, 0);

	const auto& vert = _vertices[vertIdx];
	const auto& tris = *vert.getFaceIndices(_id);
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

double CMesh::triCurvature(size_t triIdx) const
{
	const auto& tri = _tris[triIdx];

	double result = 0;
	int count = 0;
	for (int i = 0; i < 3; i++) {
		double c = _vertCurvature[tri[i]];
		if (c > 0) {
			result += c;
			count++;
		}
	}
	if (count > 0)
		return result / count;
	return 0;
}

double CMesh::edgeLength(size_t edgeIdx) const
{
	const auto& edge = _edges[edgeIdx];
	const Vector3d& pt0 = _vertices[edge._vertIndex[0]]._pt;
	const Vector3d& pt1 = _vertices[edge._vertIndex[1]]._pt;
	return (pt1 - pt0).norm();
}

void CMesh::splitLongTris(double maxEdgeLength)
{
	// capture all the points
	vector<Vector3d> points;
	for (const auto& tri : _tris) {
		for (int i = 0; i < 3; i++)
			points.push_back(getVert(tri[i])._pt);
	}

	reset(getBBox());

	size_t numTris = points.size() / 3, vertIdx = 0;
	for (size_t triIdx = 0; triIdx < numTris; triIdx++) {
		Vector3d triPts[] = {
			points[vertIdx++],
			points[vertIdx++],
			points[vertIdx++],
		};

		Vector3d v0 = triPts[1] - triPts[0];
		Vector3d v1 = triPts[2] - triPts[0];
		Vector3d srcNorm = v1.cross(v0);
		addTriangle_d(triPts[0], triPts[1], triPts[2], srcNorm, maxEdgeLength);
	}
}

void CMesh::addTriangle_d(const Vector3d& tpt0, const Vector3d& tpt1, const Vector3d& tpt2, const Vector3d& srcNorm, double maxEdgeLength)
{
	const double paramTol = 1.0e-9;
	Vector3d pts[] = { tpt0, tpt1, tpt2 };
	int numLong = 0;
	double lengths[3];
	for (int i = 0; i < 3; i++) {
		int j = (i + 1) % 3;

		const auto& pt0 = pts[i];
		const auto& pt1 = pts[j];
		Vector3d v = pt1 - pt0;
		lengths[i] = v.norm();
		if (lengths[i] > maxEdgeLength)
			numLong++;
	}

	// reverse the normal
	Vector3d v0 = pts[1] - pts[0];
	Vector3d v1 = pts[2] - pts[0];
	Vector3d norm = v1.cross(v0);
	if (norm.dot(srcNorm) < 0)
		swap(pts[1], pts[2]);

	if (numLong == 0) {
		// No trimming required. Put the triangle back in the list.

		addTriangle(pts);
		return;
	}

	// find the vertex between the two longest legs of the triangle
	double max = -1;
	int nearIdx = -1, farIdx, midIdx;
	for (int i = 0; i < 3; i++) {
		int j = (i + 1) % 3;
		int k = (i + 2) % 3;

		double l0 = (pts[j] - pts[i]).norm();
		double l1 = (pts[k] - pts[i]).norm();
		double l = l0 + l1;
		if (l > max) {
			max = l;
			nearIdx = i;
			if (l0 > l1) {
				farIdx = j;
				midIdx = k;
			} else {
				farIdx = k;
				midIdx = j;
			}
		}
	}

	std::vector<Vector3d> edgePoints[2];

	const auto& ptNear = pts[nearIdx];
	const auto& ptFar = pts[farIdx];
	const auto& ptMid = pts[midIdx];
	v0 = ptFar - ptNear;
	double len = v0.norm();
	v0 /= len;

	if (len > maxEdgeLength) {
		const double divLen = maxEdgeLength * g_maxLengthScale;
		int numSegs = (int)(len / divLen + 1);
		if (numSegs < 1)
			numSegs = 1;
		double l2 = len / numSegs;
		if (l2 > divLen)
			numSegs++;
		int numPts = numSegs + 1;
		assert(numPts >= 2);

		for (size_t i = 0; i < numPts; i++) {
			double t = i / (numPts - 1.0);
			Vector3d pt = LERP(ptNear, ptFar, t);
			edgePoints[0].push_back(pt);
		}
	}

	Vector3d v = ptMid - ptNear;
	double l = v.norm();
	if (l > maxEdgeLength) {
		const double divLen = maxEdgeLength * g_maxLengthScale;
		int numSegs = (int)(l / divLen + 1);
		if (numSegs < 1)
			numSegs = 1;
		double l2 = l / numSegs;
		if (l2 > divLen)
			numSegs++;
		int numPts = numSegs + 1;
		assert(numPts >= 2);
		for (size_t i = 0; i < numPts; i++) {
			double t = i / (numPts - 1.0);
			Vector3d pt = LERP(ptNear, ptMid, t);
			edgePoints[1].push_back(pt);
		}
	} else {
		edgePoints[1].push_back(ptNear);
		edgePoints[1].push_back(ptMid);
	}

	assert(edgePoints[0].size() >= edgePoints[1].size());
	if (edgePoints[0].size() != edgePoints[1].size()) {
		// make the sample points topologically Isosceles - two legs have equal number divisions, but not necessarily equal length. 
		edgePoints[0].resize(edgePoints[1].size());

		// tesselate a quad strip
		addTriangleStrip(edgePoints[0], edgePoints[1], srcNorm, maxEdgeLength);
		//tesselate the remaining triangle by the same rule
		auto ptEnd = edgePoints[0].back();

		// Reduce recursive storage storage
		edgePoints[0].clear();
		edgePoints[1].clear();
		addTriangle_d(ptFar, ptMid, ptEnd, srcNorm, maxEdgeLength);
	} else 
		addTriangleStrip(edgePoints[0], edgePoints[1], srcNorm, maxEdgeLength);
}


struct LocalEdge
{
	inline LocalEdge(size_t idx0, size_t idx1)
		: _idx0(idx0)
		, _idx1(idx1)
	{
	}
	size_t _idx0, _idx1;
};
	
void CMesh::addTriangleStrip(std::vector<Vector3d>& pts0, std::vector<Vector3d>& pts1, const Vector3d& srcNorm, double maxEdgeLength)
{
	if (pts0.empty() || pts1.empty())
		return;
	// These two vectors define two legs of a quad or triangle

	double len[] = {
		(pts0.front() - pts1.front()).norm(),
		(pts0.front() - pts1.back()).norm(),
		(pts0.back() - pts1.front()).norm(),
		(pts0.back() - pts1.back()).norm() 
	};

	double minLen = DBL_MAX;
	int shortIdx = 4;
	for (int i = 0; i < 4; i++) {
		if (len[i] < minLen) {
			shortIdx = i;
			minLen = len[i];
		}
	}

	size_t idx0 = 1, idx1 = 1;
	LocalEdge lastCrossEdge(0, 0);

	vector<Vector3d> lastRowPts;
	bool done = false;
	do {
		Vector3d triPts[] = {
			pts0[lastCrossEdge._idx0],  // idx0
			pts0[idx0],							// idx0
			pts1[idx1],							// idx1
			pts1[lastCrossEdge._idx1]	// idx1
		};

		int triPtSide[] = { 0, 0, 1, 1 };
		size_t triPtIndices[] = { lastCrossEdge._idx0, idx0, idx1, lastCrossEdge._idx1 };

		double edgeLen[] = {
			(triPts[1] - triPts[0]).norm(),
			(triPts[2] - triPts[1]).norm(),
			(triPts[3] - triPts[2]).norm(),
			(triPts[0] - triPts[3]).norm(),
		};

		vector<int> edges;
		int skipSide = -1;
		for (int i = 0; i < 4; i++) {
			if (edgeLen[i] >= SAME_DIST_TOL)
				edges.push_back(i);
			else
				skipSide = triPtSide[i];
		}

		size_t maxIdx0, maxIdx1;

		switch (edges.size()) {
		case 3: {
			Vector3d v0 = (triPts[edges[1]] - triPts[edges[0]]);
			Vector3d v1 = (triPts[edges[2]] - triPts[edges[0]]);
			Vector3d norm = v1.cross(v0);
			if (norm.dot(srcNorm) < 0)
				swap(edges[1], edges[2]);
			addTriangle_d(triPts[edges[0]], triPts[edges[1]], triPts[edges[2]], srcNorm, maxEdgeLength);
			break;
		}
		case 4: {
			Vector3d v0 = (triPts[edges[1]] - triPts[edges[0]]);
			Vector3d v1 = (triPts[edges[2]] - triPts[edges[0]]);
			Vector3d norm = v1.cross(v0);
			if (norm.dot(srcNorm) < 0) {
				swap(edges[0], edges[1]);
				swap(edges[2], edges[3]);
			}

			addQuad_d(triPts[edges[0]], triPts[edges[1]], triPts[edges[2]], triPts[edges[3]], maxEdgeLength);
			break;
		}
		default:
			return;
		}

		maxIdx0 = maxIdx1 = -1;
		for (int ptIdx : edges) {
			auto tmpIdx = triPtIndices[ptIdx];
			if (triPtSide[ptIdx] == 0) {
				if (maxIdx0 == -1 || tmpIdx > maxIdx0)
					maxIdx0 = tmpIdx;
			}
			else {
				if (maxIdx1 == -1 || tmpIdx > maxIdx1)
					maxIdx1 = tmpIdx;
			}
		}

		lastCrossEdge = LocalEdge(maxIdx0, maxIdx1);
		idx0 = lastCrossEdge._idx0 + 1;
		idx1 = lastCrossEdge._idx1 + 1;

		if (idx0 >= pts0.size())
			idx0 = pts0.size() - 1;

		if (idx1 >= pts1.size())
			idx1 = pts1.size() - 1;

		done = idx0 == lastCrossEdge._idx0 && idx1 == lastCrossEdge._idx1;
	} while (!done);
}

void CMesh::squeezeSkinnyTriangles(double minAngleDegrees)
{
	for (size_t vertIdx = 0; vertIdx < _vertices.size(); vertIdx++) {
		auto& vert = _vertices[vertIdx];

		const auto& vertFaces = *vert.getFaceIndices(_id);
		if (vertFaces.size() <= 10)
			continue;

		bool done = false;
		while (!done) {
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

bool CMesh::isVertConvex(size_t vIdx, bool& isConvex, LineSegmentd& axis) const
{
	// If the point lies on a plane, the function returns false
	// Otherwise, if the point is "above" the perimeter isConvex is set true
	// Otherwise isConvex is set false;
	Vector3d tipPt = getVert(vIdx)._pt;

	Vector3d perimeterCtr(0, 0, 0);
	double perimeter = 0;
	const auto& vert = getVert(vIdx);
	const auto& triIndices = *vert.getFaceIndices(_id);
	for (size_t triIdx : triIndices) {
		const auto& tri = getTri(triIdx);
		bool first = true;
		Vector3d pt0, pt1;
		for (int i = 0; i < 3; i++) {
			if (tri[i] != vIdx) {
				if (first) {
					pt0 = getVert(tri[i])._pt;
					first = false;
				} else {
					pt1 = getVert(tri[i])._pt;
					break;
				}
			}
		}

		Vector3d vSeg = pt1 - pt0;
		Vector3d oppCenter = (pt0 + pt1) * 0.5;
		Vector3d v = tipPt - oppCenter;
		double l = v.norm();
		if (l < SAME_DIST_TOL)
			return false;
		v / l;

		// Make vSeg perpendicular to the line between the center and the tip.
		vSeg = vSeg - vSeg.dot(v) * v;
		double segLen = vSeg.norm();;
		perimeter += segLen;

		perimeterCtr += segLen * oppCenter;
	}
	perimeterCtr /= perimeter;

	Vector3d dir = tipPt - perimeterCtr;
	dir.normalize();

	double avgDp = 0;
	for (size_t triIdx : triIndices) {
		Vector3d n = triUnitNormal(triIdx);
		auto dp = n.dot(dir);
		avgDp += dp;
	}

	axis = LineSegmentd(perimeterCtr, tipPt);
	isConvex = avgDp >= 0;
	return true;
}

bool CMesh::verifyFindAllTris() const {
	bool result = true;
	for (size_t triIdx = 0; triIdx < _tris.size(); triIdx++) {
		BoundingBox triBox = getTriBBox(triIdx);

		vector<size_t> entries;
		_pTriTree->find(triBox, entries);
		bool found = false;
		for (const auto& idx : entries) {
			found = found || idx == triIdx;
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
			Ray<double> ray(origin, Vector3d(1, 0, 0));
			vector<size_t> hits;
			_pTriTree->biDirRayCast(ray, hits);
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

size_t CMesh::getSTLPoints(std::vector<Vector3f>& pts) const
{
	pts.clear();
	pts.reserve(_tris.size() * 3);
	for (const auto& tri : _tris) {
		for (size_t i = 0; i < 3; i++) {
			const auto& pt = getVert(tri[i])._pt;
			pts.push_back(Vector3f((float)pt[0], (float)pt[1], (float)pt[2]));
		}
	}
	return pts.size();
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

void CMesh::getGlEdges(vector<float>& points, vector<unsigned int>& indices) // size = GlPoints.size() / 3
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

void CMesh::write(ostream& out) const {
	uint8_t version = 0;
	out.write((char*)&version, sizeof(version));

	out.write((char*)&_enforceManifold, sizeof(_enforceManifold));

	IoUtil::writeObj(out, _vertices, _id);
	IoUtil::writeObj(out, _edges, _id);
	IoUtil::write(out, _tris);
}

bool CMesh::read(istream& in) {
	uint8_t version = -1;
	in.read((char*)&version, sizeof(version));

	in.read((char*)&_enforceManifold, sizeof(_enforceManifold));

	IoUtil::readObj(in, _vertices, _id);
	IoUtil::readObj(in, _edges, _id);
	IoUtil::read(in, _tris);

	BoundingBox bbox;
	for (size_t i = 0; i < _vertices.size(); i++) {
		BoundingBox vertBbox = getVertBBox(i);
		bbox.merge(vertBbox);
	}

	_pVertTree = make_shared<SearchTree>(bbox);
	for (size_t i = 0; i < _vertices.size(); i++) {
		BoundingBox vertBbox = getVertBBox(i);
		_pVertTree->add(vertBbox, i);
	}

	_pEdgeTree = make_shared<SearchTree>(bbox);
	for (size_t i = 0; i < _edges.size(); i++) {
		const auto& edge = _edges[i];
		_edgeToIdxMap.insert(make_pair((CEdgeGeo)edge, i));
		BoundingBox edgeBbox = getEdgeBBox(i);
		_pEdgeTree->add(edgeBbox, i);
	}

	_pTriTree = make_shared<SearchTree>(bbox);
	for (size_t i = 0; i < _tris.size(); i++) {
		BoundingBox triBbox = getTriBBox(i);
		_pTriTree->add(triBbox, i);
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
		const auto& vertEdgeIndices = *vert.getEdgeIndices(_id);
		if (vertEdgeIndices.size() > maxEdges) {
			maxEdges = vertEdgeIndices.size();
			worstVertIdx = i;
		}
	}

	map<double, vector<size_t>> aspectRatioToShortEdgeIdxMap; // Using a vector for indices is overkill, but there is a remote chance of a duplicated aspect ratio
	auto& vert = getVert(worstVertIdx);
	auto& vertFaces = *vert.getFaceIndices(_id);

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

