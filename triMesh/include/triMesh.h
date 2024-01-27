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

	class CMesh;
	using CMeshPtr = std::shared_ptr<CMesh>;

	class CMesh {
	public:
		using BoundingBox = CBoundingBox3Dd;
		using SearchTree = CSpatialSearchST<BoundingBox>;
		using BoxTestType = SearchTree::BoxTestType;

		CMesh();
		CMesh(const BoundingBox& bbox);
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

		template<class POINT_TYPE>
		size_t addQuad(const POINT_TYPE& pt0, const POINT_TYPE& pt1, const POINT_TYPE& pt2, const POINT_TYPE& pt3);

		// takes a vector of 8 points in {
		// 0,3,2,1
		// 4,5,6,7 } order
		template<class POINT_TYPE>
		size_t addRectPrism(const std::vector<POINT_TYPE>& pts);

		const BoundingBox& getBBox() const {
			return _vertTree.getBounds();
		}
		LineSegment getEdgesLineSeg(size_t edgeIdx) const;
		bool isEdgeSharp(size_t edgeIdx, double sinEdgeAngle) const;

		const std::vector<size_t>& getSharpEdgeIndices(double edgeAngleRadians = 0);

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
		size_t rayCast(size_t triIdx, std::vector<RayHit>& hits, bool biDir = true) const;
		size_t rayCast(const Ray& ray, std::vector<RayHit>& hits, bool biDir = true) const;
		size_t rayCast(const LineSegment& seg, std::vector<RayHit>& hits, double tol = 1.0e-6) const;

		size_t findVerts(const BoundingBox& bbox, std::vector<size_t>& vertIndices, BoxTestType contains = BoxTestType::Intersects) const;
		size_t findEdges(const BoundingBox& bbox, std::vector<size_t>& edgeIndices, BoxTestType contains = BoxTestType::Intersects) const;
		size_t findTris(const BoundingBox& bbox, std::vector<size_t>& triIndices, BoxTestType contains = BoxTestType::Intersects) const;

		Vector3d triCentroid(size_t triIdx) const;
		Vector3d triUnitNormal(size_t triIdx) const;
		Vector3d vertUnitNormal(size_t vertIdx) const;
		double edgeCurvature(size_t edgeIdx) const;

		void merge(CMeshPtr& src, bool destructive);
		void merge(std::vector<CMeshPtr>& src, bool destructive, bool multiCore = true);

		void buildCentroids(bool multiCore = true) const;
		void buildNormals(bool multiCore = true) const;
		void calCurvatures(double edgeAngleRadians, bool multiCore = true) const;

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
		double calEdgeCurvature(size_t edgeIdx) const;
		double calVertCurvature(size_t vertIdx) const;

		const long _id;
		mutable int _changeNumber;
		std::vector<CVertex> _vertices;
		SearchTree _vertTree;

		std::vector<CEdge> _edges;
		std::vector<size_t> _sharpEdgeIndices;
		SearchTree _edgeTree;
		std::map<CEdge, size_t> _edgeToIdxMap;

		std::vector<Vector3i> _tris;

		mutable bool _useCentroidCache = true;
		mutable std::vector<Vector3d> _centroids;

		std::vector<float> _glPoints, _glNormals, _glParams;
		std::vector<unsigned int> _glTriIndices, _glEdgeIndices;
		mutable bool _useNormalCache = true;
		mutable std::vector<Vector3d> _normals;
		mutable std::vector<double> _edgeCurvature;
		SearchTree _triTree;
	};

	using CMeshPtr = std::shared_ptr<CMesh>;

	inline CMesh::CMesh(const BoundingBox& bbox)
		: CMesh(bbox.getMin(), bbox.getMax())
	{
	}

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
		BoundingBox ptBBox(pt), thisBBox = getBBox();
		ptBBox.grow(SAME_DIST_TOL);
		if (!thisBBox.contains(pt)) {
			std::cout << "CMesh::addVertex box intersects: Error\n";
		}
		auto results = _vertTree.find(ptBBox);
		for (const auto& index : results) {
			if ((_vertices[index]._pt - ptUnk).norm() < SAME_DIST_TOL) {
				return index;
			}
		}
		size_t result = _vertices.size();
		_vertices.push_back(CVertex(ptUnk));
		_vertTree.add(ptBBox, result);

#ifdef _DEBUG
		// Testing
		auto testResults = _vertTree.find(ptBBox);
		int numFound = 0;
		for (const auto& e : testResults) {
			if (e == result)
				numFound++;
		}
		if (numFound != 1) {
			std::cout << "CMesh::addVertex numFound: Error. numFound: " << numFound << "\n";
			testResults = _vertTree.find(ptBBox);
		}
#endif
		return result;
	}

	template<class POINT_TYPE>
	size_t CMesh::addQuad(const POINT_TYPE& pt0, const POINT_TYPE& pt1, const POINT_TYPE& pt2, const POINT_TYPE& pt3)
	{
		addTriangle(pt0, pt1, pt2);
		addTriangle(pt0, pt2, pt3);

		return _tris.size();
	}

	template<class POINT_TYPE>
	size_t CMesh::addRectPrism(const std::vector<POINT_TYPE>& pts)
	{
		if (pts.size() != 8)
			return -1;

		// add bottom and top
		addQuad(pts[0], pts[3], pts[2], pts[1]);
		addQuad(pts[4], pts[5], pts[6], pts[7]);

		// add left and right
		addQuad(pts[0], pts[4], pts[7], pts[3]);
		addQuad(pts[1], pts[2], pts[6], pts[5]);

		// add front and back
		addQuad(pts[0], pts[1], pts[5], pts[4]);
		addQuad(pts[2], pts[3], pts[7], pts[6]);

		return _tris.size();
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
