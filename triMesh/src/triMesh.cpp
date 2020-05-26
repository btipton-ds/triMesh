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

#include <defines.h>

#include <algorithm>

#include <triMesh.h>

using namespace std;

namespace TriMesh {

	CMesh::CMesh() {
	}

	CMesh::CMesh(const Vector3d& min, const Vector3d& max)
		: _vertTree(BoundingBox(min, max))
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

			_edges[edgeIdx].addFace(triIdx);
			triBox.merge(_vertices[tri[i]]._pt);
		}
		triBox.grow(SAME_DIST_TOL);
		_triTree.add(triBox, triIdx);

		return triIdx;
	}

	LineSegment CMesh::getEdgesLineSeg(size_t edgeIdx) const {
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
		const Vector3d norm0 = triUnitNormal(edge._faceIndex[0]);
		const Vector3d norm1 = triUnitNormal(edge._faceIndex[1]);

		Vector3d cp = norm0.cross(norm1);
		double edgeCross = cp.dot(edgeV);
		return fabs(edgeCross) > sinEdgeAngle;
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

	size_t CMesh::biDirRayCast(const Ray& ray, std::vector<RayHit>& hits) const {
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
					hit.triIdx = triIdx2;
					hits.push_back(hit);
				}
			}
		}
		sort(hits.begin(), hits.end());
		return hits.size();
	}

	size_t CMesh::biDirRayCast(const LineSegment& seg, std::vector<RayHit>& hits) const {
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
					hit.triIdx = triIdx2;
					hits.push_back(hit);
				}
			}
		}
		sort(hits.begin(), hits.end());
		return hits.size();
	}

	size_t CMesh::biDirRayCast(size_t triIdx, std::vector<RayHit>& hits) const {
		Vector3d ctr = triCentroid(triIdx);
		Vector3d norm = triUnitNormal(triIdx);
		Ray ray(ctr, norm);

		vector<RayHit> temp;
		biDirRayCast(ray, temp);

		for (const auto& hit : temp) {
			if (hit.triIdx != triIdx)
				hits.push_back(hit);
		}
		return hits.size();
	}

	double CMesh::findTriMinimumGap(size_t i) const {
		vector<RayHit> hits;
		if (biDirRayCast(i, hits) == 0)
			return FLT_MAX;
		else
			return fabs(hits[0].dist);
	}

	double CMesh::findMinGap() const {
		double minGap = FLT_MAX;
		for (size_t i = 0; i < _tris.size(); i++) {
			vector<RayHit> hits;
			if (biDirRayCast(i, hits) > 0) {
				double d = fabs(hits[0].dist);
				if (d < minGap)
					minGap = d;
			}
		}

		return minGap;
	}

	void CMesh::getGapHistogram(const std::vector<double>& binSizes, std::vector<size_t>& bins) const {
		for (size_t i = 0; i < _tris.size(); i++) {
			vector<RayHit> hits;
			biDirRayCast(i, hits);

			for (const auto& hit : hits) {
				for (size_t i = 0; i < binSizes.size(); i++) {
					if (fabs(hit.dist) < binSizes[i]) {
						bins[i]++;
						break;
					}
				}
			}
		}
	}

	size_t CMesh::findVerts(const BoundingBox& bbox, std::vector<size_t>& vertIndices) const {
		return _vertTree.find(bbox, vertIndices);
	}

	size_t CMesh::findEdges(const BoundingBox& bbox, std::vector<size_t>& edgeIndices) const {
		return _edgeTree.find(bbox, edgeIndices);
	}

	size_t CMesh::findTris(const BoundingBox& bbox, std::vector<size_t>& triIndices) const {
		return _triTree.find(bbox, triIndices);
	}

	Vector3d CMesh::triCentroid(size_t triIdx) const {
		const auto& tri = _tris[triIdx];
		const Vector3d& pt0 = _vertices[tri[0]]._pt;
		const Vector3d& pt1 = _vertices[tri[1]]._pt;
		const Vector3d& pt2 = _vertices[tri[2]]._pt;

		Vector3d ctr = (pt0 + pt1 + pt2) / 3.0;
		return ctr;
	}

	Vector3d CMesh::triUnitNormal(size_t triIdx) const {
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

	void CMesh::dumpVertices(ostream& out) const {
		out << "Vertices\n";
		for (const auto& vert : _vertices) {
			vert.dump(out);
		}
	}

	void CMesh::dumpEdges(ostream& out) const {
		out << "Edge\n";
		for (const auto& edge : _edges) {
			edge.dump(out);
		}
	}

	void CMesh::dumpTris(ostream& out) const {
		for (const auto& p : _tris) {
			out << p[0] << ", " << p[1] << ", " << p[2] << "\n";
		}
	}

	void CMesh::dumpObj(std::ostream& out) const {
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

	void CMesh::dumpModelSharpEdgesObj(std::ostream& out, double sinAngle) const {
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
