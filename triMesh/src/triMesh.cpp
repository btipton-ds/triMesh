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

#include <tm_defines.h>

#include <algorithm>
#include <fstream>
#include <iomanip>

#include <triMesh.h>
#include <MultiCoreUtil.h>

using namespace std;

namespace
{
namespace
{

inline string fromWString(const wstring& str)
{
	return string(str.begin(), str.end());
}

}

}

namespace TriMesh {

	long _statId = 0;

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
		wofstream out(fromWString(filename));
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
			out << edge._vertIndex[0] << " " << edge._vertIndex[1] << " " << edge._numFaces << " " << edge._faceIndex[0];
			if (edge._numFaces == 2)
				out << " " << edge._faceIndex[1];
			out << "\n";
		}
	}

	bool CMesh::compareDumpedTris(const wstring& filename) const
	{
		wifstream in(fromWString(filename));
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

			if (numFaces == 2 && d != edge._faceIndex[1])
				return false;
		}
		return true;
	}

	void CMesh::dumpTree(const wstring& filename) const
	{
		wofstream out(fromWString(filename));
		out << setprecision(15);
		_triTree.dump(out);

	}

	bool CMesh::compareDumpedTree(const wstring& filename) const
	{
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

	size_t CMesh::addTriangle(const Vector3i& tri) {
		size_t triIdx = _tris.size();
		_tris.push_back(tri);
		BoundingBox triBox;
		for (int i = 0; i < 3; i++) {
			int j = (i + 1) % 3;
			size_t edgeIdx = addEdge(tri[i], tri[j]);

			auto& vert = _vertices[tri[i]];
			vert.addFaceIndex(triIdx);

			_edges[edgeIdx].addFace(triIdx);
			triBox.merge(_vertices[tri[i]]._pt);
		}
		triBox.grow(SAME_DIST_TOL);
		_triTree.add(triBox, triIdx);

		return triIdx;
	}

	LineSegment CMesh::getEdgesLineSeg(size_t edgeIdx) const {

		/*TODO, not finished, may not be needed*/
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
		if (edge._faceIndex[0] == edge._faceIndex[1]) {
			cout << "Duplicated faceIndex\n";
		}
		const Vector3d norm0 = triUnitNormal(edge._faceIndex[0]);
		const Vector3d norm1 = triUnitNormal(edge._faceIndex[1]);

		Vector3d cp = norm0.cross(norm1);
		double edgeCross = fabs(cp.dot(edgeV));
		bool isSharp = edgeCross > sinEdgeAngle;
		return isSharp;
	}

	const vector<size_t>& CMesh::getSharpEdgeIndices(double edgeAngleRadians)
	{
		if (edgeAngleRadians < 1.0e-6) {
			return _sharpEdgeIndices;
		}

		double sinEdgeAngle = sin(edgeAngleRadians);
		buildNormals();
		_sharpEdgeIndices.clear();
		_sharpEdgeIndices.reserve(_edges.size());

		for (size_t i = 0; i < _edges.size(); i++) {
			if (isEdgeSharp(i, sinEdgeAngle)) {
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

	size_t CMesh::rayCast(const LineSegment& seg, vector<RayHit>& hits) const {
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
					if ((hit.dist > 0) && (hit.dist <= segLen)) {
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

	void CMesh::changed()
	{
		_changeNumber++;
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

		MultiCore::runLambda([this, tol, &minGapVec](size_t threadNum, size_t numThreads) {
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

		MultiCore::runLambda([this, &binSizes, &binSet](size_t threadNum, size_t numThreads) {
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
			MultiCore::runLambda([this, &src, destructive](size_t threadNum, size_t numThreads) {
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
			MultiCore::runLambda([this](size_t threadNum, size_t numThreads) {
				size_t num = _centroids.size();
				for (size_t triIdx = threadNum; triIdx < num; triIdx += numThreads)
					_centroids[triIdx] = triCentroid(triIdx);
			}, multiCore);
		}
	}

	void CMesh::buildNormals(bool multiCore) const
	{
		if (_normals.empty()) {
			ScopedSetVal<bool> set(_useNormalCache, false);
			_normals.resize(_tris.size());
			MultiCore::runLambda([this](size_t threadNum, size_t numThreads) {
				size_t num = _normals.size();
				for (size_t triIdx = threadNum; triIdx < num; triIdx += numThreads)
					_normals[triIdx] = triUnitNormal(triIdx);
			}, multiCore);
		}		
	}

	size_t CMesh::findVerts(const BoundingBox& bbox, vector<size_t>& vertIndices, BoxTestType contains) const {
		return _vertTree.find(bbox, vertIndices, contains);
	}

	size_t CMesh::findEdges(const BoundingBox& bbox, vector<size_t>& edgeIndices, BoxTestType contains) const {
		return _edgeTree.find(bbox, edgeIndices, contains);
	}

	size_t CMesh::findTris(const BoundingBox& bbox, vector<size_t>& triIndices, BoxTestType contains) const {
		return _triTree.find(bbox, triIndices, contains);
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

	bool CMesh::verifyFindAllTris() const {
		bool result = true;
		for (size_t triIdx = 0; triIdx < _tris.size(); triIdx++) {
			const auto& tri = _tris[triIdx];
			BoundingBox triBox;
			for (int i = 0; i < 3; i++) {
				const auto& v = _vertices[tri[i]];
				triBox.merge(v._pt);
			}

			vector<size_t> hits;
			_triTree.find(triBox, hits);
			bool found = false;
			for (size_t hit : hits) {
				found = found || hit == triIdx;
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

	const vector<float>& CMesh::getGlPoints()
	{
		if (_glPoints.size() != 3 * 3 * _tris.size()) {
			_glPoints.resize(3 * 3 * _tris.size());
			size_t idx = 0;
			for (size_t triIdx = 0; triIdx < _tris.size(); triIdx++) {
				const auto& vertIndices = _tris[triIdx];
				for (size_t i = 0; i < 3; i++) {
					const auto& pt = _vertices[vertIndices[i]]._pt;
					_glPoints[idx++] = (float)pt[0];
					_glPoints[idx++] = (float)pt[1];
					_glPoints[idx++] = (float)pt[2];
				}
			}
		}
		return _glPoints;
	}

	const vector<float>& CMesh::getGlNormals(bool smoothed)
	{
		buildNormals();
		if (_glNormals.size() != 3 * 3 * _tris.size()) { // _vertices is a 3 vector, _glNormals is floats
			_glNormals.resize(3 * 3 * _tris.size());
			size_t idx = 0;
			for (size_t triIdx = 0; triIdx < _tris.size(); triIdx++) {
				const auto& norm = _normals[triIdx];
				for (size_t i = 0; i < 3; i++) {
					_glNormals[idx++] = (float)norm[0];
					_glNormals[idx++] = (float)norm[1];
					_glNormals[idx++] = (float)norm[2];
				}
			}
		}
		return _glNormals;
	}

	const vector<float>& CMesh::getGlParams()
	{
		if (_glParams.size() != 3 * 2 * _tris.size()) {
			_glParams.resize(3 * 2 * _tris.size(), 0);
		}
		return _glParams;
	}

	const vector<unsigned int>& CMesh::getGlFaceIndices()
	{
		if (_glPoints.empty())
			getGlPoints();

		if (_glTriIndices.size() != _glPoints.size() / 3) { // _vertices is a 3 vector, _glNormals is floats
			_glTriIndices.resize(_glPoints.size() / 3);
			for (size_t idx = 0; idx < _glTriIndices.size(); idx++) {
				_glTriIndices[idx] = (unsigned int)idx;
			}
		}
		return _glTriIndices;
	}

	const vector<unsigned int>& CMesh::getGlEdgeIndices()
	{
		if (_glEdgeIndices.size() != 3 * 3 * _tris.size()) { // _vertices is a 3 vector, _glNormals is floats
			_glEdgeIndices.resize(3 * 3 * _edges.size());
			size_t idx = 0;
			for (size_t triIdx = 0; triIdx < _tris.size(); triIdx++) {
				for (size_t i = 0; i < 3; i++) {
					_glEdgeIndices[idx] = (unsigned int)idx++;
					_glEdgeIndices[idx] = (unsigned int)idx++;
					_glEdgeIndices[idx] = (unsigned int)idx++;
				}
			}
		}
		return _glEdgeIndices;
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
}
