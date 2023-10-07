#pragma once

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
#include <map>
#include <vector>
#include <iostream>

#include <tm_math.h>
#include <tm_spatialSearch.h>
#include <tm_edge.h>
#include <tm_vertex.h>

namespace TriMesh {

	template<typename T>
	class ScopedSetVal {
	public:
		ScopedSetVal(T& variable, T newVal);
		~ScopedSetVal();
	private:
		T _oldVal;
		T& _variable;
	};

	class CMesh {
	public:
		using BoundingBox = CBoundingBox3Dd;
		using SearchTree = CSpatialSearchST<BoundingBox>;

		CMesh();
		CMesh(const Vector3d& min, const Vector3d& max);
		void reset(const BoundingBox& bbox);

		long getId() const;
		int getChangeNumber() const;

		void save(std::ostream& out) const;
		bool read(std::istream& in);

		void dumpTris(const std::wstring& filename) const;
		bool compareDumpedTris(const std::wstring& filename) const;
		void dumpTree(const std::wstring& filename) const;
		bool compareDumpedTree(const std::wstring& filename) const;

		size_t addEdge(size_t vertIdx0, size_t vertIdx1);
		size_t addTriangle(const Vector3i& tri);

		template<class POINT_TYPE>
		size_t addTriangle(const POINT_TYPE pts[3]);
		template<class POINT_TYPE>
		size_t addTriangle(const POINT_TYPE& pt0, const POINT_TYPE& pt1, const POINT_TYPE& pt2);
		template<class POINT_TYPE>
		size_t addVertex(const POINT_TYPE& ptUnk);

		const BoundingBox& getBBox() const {
			return _vertTree.getBounds();
		}
		LineSegment getEdgesLineSeg(size_t edgeIdx) const;
		bool isEdgeSharp(size_t edgeIdx, double sinEdgeAngle) const;

		size_t numVertices() const;
		size_t numEdges() const;
		size_t numTris() const;
		size_t numLaminarEdges() const;

		const CVertex& getVert(size_t idx) const;
		const Vector3i& getTri(size_t idx) const;
		const CEdge& getEdge(size_t idx) const;

		bool isClosed() const;
		double findMinGap(double tol = 0.0001, bool multiCore = true) const;
		void getGapHistogram(const std::vector<double>& binSizes, std::vector<size_t>& bins, bool multiCore = true) const;
		size_t biDirRayCast(size_t triIdx, std::vector<RayHit>& hits) const;
		size_t biDirRayCast(const Ray& ray, std::vector<RayHit>& hits) const;
		size_t biDirRayCast(const LineSegment& seg, std::vector<RayHit>& hits) const;

		size_t findVerts(const BoundingBox& bbox, std::vector<size_t>& vertIndices) const;
		size_t findEdges(const BoundingBox& bbox, std::vector<size_t>& edgeIndices) const;
		size_t findTris(const BoundingBox& bbox, std::vector<size_t>& triIndices) const;

		Vector3d triCentroid(size_t triIdx) const;
		Vector3d triUnitNormal(size_t triIdx) const;

		void buildCentroids(bool multiCore = true) const;
		void buildNormals(bool multiCore = true) const;

		bool verifyFindAllTris() const;

		void dumpObj(std::ostream& out) const;
		void dumpModelSharpEdgesObj(std::ostream& out, double sinAngle) const;

		const std::vector<float>& getGlPoints();
		const std::vector<float>& getGlNormals(bool smoothed);
		const std::vector<float>& getGlParams();
		const std::vector<unsigned int>& getGlFaceIndices();
		const std::vector<unsigned int>& getGlEdgeIndices();

	private:
		void changed();
		double findTriMinimumGap(size_t i) const;

		const long _id;
		mutable int _changeNumber;
		std::vector<CVertex> _vertices;
		SearchTree _vertTree;

		std::vector<CEdge> _edges;
		SearchTree _edgeTree;
		std::map<CEdge, size_t> _edgeToIdxMap;

		std::vector<Vector3i> _tris;

		mutable bool _useCentroidCache = true;
		mutable std::vector<Vector3d> _centroids;

		std::vector<float> _glPoints, _glNormals, _glParams;
		std::vector<unsigned int> _glTriIndices, _glEdgeIndices;
		mutable bool _useNormalCache = true;
		mutable std::vector<Vector3d> _normals;
		SearchTree _triTree;
	};

	using CMeshPtr = std::shared_ptr<CMesh>;

	template<class POINT_TYPE>
	size_t CMesh::addTriangle(const POINT_TYPE pts[3]) {
		size_t indices[3];
		for (int i = 0; i < 3; i++)
			indices[i] = addVertex(pts[i]);

		if (indices[0] == indices[1] || indices[0] == indices[2] || indices[1] == indices[2])
			return stm1;
		return addTriangle(Vector3i(indices[0], indices[1], indices[2]));
	}

	template<class POINT_TYPE>
	size_t CMesh::addTriangle(const POINT_TYPE& pt0, const POINT_TYPE& pt1, const POINT_TYPE& pt2) {
		POINT_TYPE pts[] = { pt0, pt1, pt2 };

		return addTriangle(pts);
	}

	template<class POINT_TYPE>
	size_t CMesh::addVertex(const POINT_TYPE& ptUnk) {
		Vector3d pt(ptUnk);
		BoundingBox box(pt);
		box.grow(SAME_DIST_TOL);
		if (!box.intersects(BoundingBox(pt))) {
			std::cout << "Error\n";
		}
		auto results = _vertTree.find(box);
		for (const auto& index : results) {
			if ((_vertices[index]._pt - ptUnk).norm() < SAME_DIST_TOL) {
				return index;
			}
		}
		size_t result = _vertices.size();
		_vertices.push_back(CVertex(ptUnk));
		_vertTree.add(box, result);

		// Testing
		auto testResults = _vertTree.find(BoundingBox(pt));
		int numFound = 0;
		for (const auto& e : testResults) {
			if (e == result)
				numFound++;
		}
		if (numFound != 1) {
			std::cout << "Error\n";
			testResults = _vertTree.find(BoundingBox(pt));
		}
		return result;
	}

	inline long CMesh::getId() const
	{
		return _id;
	}

	inline int CMesh::getChangeNumber() const
	{
		return _changeNumber;
	}

	inline const CVertex& CMesh::getVert(size_t idx) const {
		return _vertices[idx];
	}

	inline const Vector3i& CMesh::getTri(size_t idx) const {
		return _tris[idx];
	}

	inline const CEdge& CMesh::getEdge(size_t idx) const {
		return _edges[idx];
	}

	template<typename T>
	inline ScopedSetVal<T>::ScopedSetVal(T& variable, T newVal)
		: _variable(variable)
	{
		_oldVal = _variable;
		_variable = newVal;
	}

	template<typename T>
	inline ScopedSetVal<T>::~ScopedSetVal()
	{
		_variable = _oldVal;
	}

}
