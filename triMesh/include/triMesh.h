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
	using CMeshConstPtr = std::shared_ptr<const CMesh>;

	class CMesh : public std::enable_shared_from_this<CMesh> {
	public:
		using SearchTree = CSpatialSearchST<double>;
		using SearchTreePtr = std::shared_ptr<SearchTree>;
		using SearchTreeConstPtr = std::shared_ptr<const SearchTree>;
		using BoundingBox = SearchTree::BOX_TYPE;
		using BoxTestType = SearchTree::BoxTestType;
		using SearchEntry = SearchTree::Entry;

		CMesh();
		CMesh(const BoundingBox& bbox);
		CMesh(const Vector3d& min, const Vector3d& max);
		void reset(const BoundingBox& bbox);
		void setEnforceManifold(bool val);
		bool enforceManifold() const;

		size_t getId() const;
		size_t getChangeNumber() const;
		void changed();

		void write(std::ostream& out) const;
		bool read(std::istream& in);

		void dumpTris(const std::wstring& filename) const;
		bool compareDumpedTris(const std::wstring& filename) const;
		void dumpTree(const std::wstring& filename) const;
		bool compareDumpedTree(const std::wstring& filename) const;

		size_t addEdge(size_t vertIdx0, size_t vertIdx1);
		size_t addTriangle(const Vector3i& tri);
		size_t findEdge(size_t vertIdx0, size_t vertIdx1) const;
		size_t findEdge(const CEdge& edge) const;

		template<class POINT_TYPE>
		size_t addTriangle(const POINT_TYPE pts[3]);
		template<class POINT_TYPE>
		size_t addTriangle(const POINT_TYPE& pt0, const POINT_TYPE& pt1, const POINT_TYPE& pt2);
		template<class POINT_TYPE>
		size_t addVertex(const POINT_TYPE& ptUnk);

		template<class POINT_TYPE>
		size_t addQuad(const POINT_TYPE& pt0, const POINT_TYPE& pt1, const POINT_TYPE& pt2, const POINT_TYPE& pt3);

		void squeezeSkinnyTriangles(double minAngleDegrees);
		void squeezeEdge(size_t idx);

		// takes a vector of 8 points in {
		// 0,3,2,1
		// 4,5,6,7 } order
		template<class POINT_TYPE>
		size_t addRectPrism(const std::vector<POINT_TYPE>& pts);

		const BoundingBox& getBBox() const {
			return _pVertTree->getBounds();
		}

		BoundingBox getTriBBox(size_t triIdx) const;
		BoundingBox getEdgeBBox(size_t edgeIdx) const;
		BoundingBox getVertBBox(size_t vertIdx) const;
		bool bboxIntersectsTri(const BoundingBox& bbox, size_t idx) const;
		bool bboxIntersectsEdge(const BoundingBox& bbox, size_t idx) const;
		LineSegment<double> getEdgesLineSeg(size_t edgeIdx) const;
		bool isEdgeSharp(size_t edgeIdx, double sinEdgeAngle) const;

		const std::vector<size_t>& getSharpEdgeIndices(double edgeAngleRadians = 0) const;
		size_t createSharpEdgeVertexLines(size_t sharpVertIdx, std::set<size_t>& availEdges, double sharpEdgeAngleRadians, std::vector<std::vector<size_t>>& vertIndices) const;

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
		size_t rayCast(size_t triIdx, std::vector<RayHit<double>>& hits, bool biDir = true) const;
		size_t rayCast(const Ray<double>& ray, std::vector<RayHit<double>>& hits, bool biDir = true) const;
		size_t rayCast(const LineSegment<double>& seg, std::vector<RayHit<double>>& hits, double tol = 1.0e-6) const;

		size_t findVerts(const BoundingBox& bbox, std::vector<SearchEntry>& vertIndices, BoxTestType contains = BoxTestType::Intersects) const;
		size_t findVerts(const BoundingBox& bbox, std::vector<size_t>& vertIndices, BoxTestType contains = BoxTestType::Intersects) const;

		size_t findEdges(const BoundingBox& bbox, std::vector<SearchEntry>& edgeIndices, BoxTestType contains = BoxTestType::Intersects) const;
		size_t findEdges(const BoundingBox& bbox, std::vector<size_t>& edgeIndices, BoxTestType contains = BoxTestType::Intersects) const;

		size_t findTris(const BoundingBox& bbox, std::vector<SearchEntry>& triIndices, BoxTestType contains = BoxTestType::Intersects) const;
		size_t findTris(const BoundingBox& bbox, std::vector<size_t>& triIndices, BoxTestType contains = BoxTestType::Intersects) const;

		Vector3d triCentroid(size_t triIdx) const;
		Vector3d triUnitNormal(size_t triIdx) const;
		double triArea(size_t triIdx) const;
		double triAspectRatio(size_t triIdx) const;
		double triGap(size_t triIdx) const;

		Vector3d vertUnitNormal(size_t vertIdx) const;
		double edgeCurvature(size_t edgeIdx) const;
		double edgeLength(size_t edgeIdx) const;
		double triCurvature(size_t triIdx) const;

		void merge(CMeshPtr& src, bool destructive);
		void merge(std::vector<CMeshPtr>& src, bool destructive, bool multiCore = true);

		void buildCentroids(bool multiCore = true) const;
		void buildNormals(bool multiCore = true) const;
		void calCurvatures(double edgeAngleRadians, bool multiCore = true) const;
		void calGaps(bool multiCore = true) const;

		bool verifyFindAllTris() const;

		void dumpObj(std::ostream& out) const;
		void dumpModelSharpEdgesObj(std::ostream& out, double sinAngle) const;

		const std::vector<float>& getGlTriPoints() const;
		const std::vector<float>& getGlTriNormals(bool smoothed) const;
		const std::vector<float>& getGlTriParams() const;
		template<typename LAMBDA>
		const std::vector<float>& getGlTriCurvatureColors(LAMBDA curvatureToColorFunc) const; // size = GlPoints.size() / 3
		const std::vector<unsigned int>& getGlTriIndices() const;

		// If all is true, get every edge. If false, only get sharp and curved edges.

		void getGlEdges(std::vector<float>& points, std::vector<unsigned int>& indices);

		template<typename LAMBDA>
		void getGlEdges(LAMBDA cuvatureToColorFunc, bool includeSmooth, std::vector<float>& points, std::vector<float>& color, std::vector<unsigned int>& indices);

		bool testSqueezeEdge(size_t idx);
		bool testRemoveTri(size_t idx);

		bool verifyTopology(bool allowEmptyEdges) const;

		size_t processFoundEdges(const std::vector<SearchEntry>& allHits, const BoundingBox& bbox, std::vector<SearchEntry>& edgeIndices, BoxTestType contains = BoxTestType::Intersects) const;
		size_t processFoundEdges(const std::vector<size_t>& allHits, const BoundingBox& bbox, std::vector<size_t>& edgeIndices, BoxTestType contains = BoxTestType::Intersects) const;
		size_t processFoundTris(const std::vector<SearchEntry>& allHits, const BoundingBox& bbox, std::vector<SearchEntry>& triIndices, BoxTestType contains = BoxTestType::Intersects) const;
		size_t processFoundTris(const std::vector<size_t>& allHits, const BoundingBox& bbox, std::vector<size_t>& triIndices, BoxTestType contains = BoxTestType::Intersects) const;

	private:
		static bool sameTri(const Vector3i& tri0, const Vector3i& tri1);

		double findTriMinimumGap(size_t i) const;
		double calEdgeCurvature(size_t edgeIdx, double sinEdgeAngle) const;
		double calSinVertexAngle(size_t triIdx, size_t vertIdx, size_t& oppositeEdgeIdx) const;
		size_t getOtherVertIdx(const CEdge& thisEdge, size_t triIdx) const;
		bool removeTri(size_t triIdx);
		bool deleteTriFromStorage(size_t edgeIdx);
		bool deleteEdgeFromStorage(size_t edgeIdx);
		void mergeVertices(size_t vertIdxToKeep, size_t vertIdxToRemove);
		bool addVertexToEdgeLine(std::vector<size_t>& vertLine, std::set<size_t>& availEdges, double sinEdgeAngle) const;

		bool triContainsVertex(size_t triIdx, size_t vertIdx) const;
		bool triContainsEdge(size_t triIdx, size_t edgeIdx) const;
		bool edgeContainsVert(size_t edgeIdx, size_t vertIdx) const;
		bool edgeReferencesTri(size_t edgeIdx, size_t triIdx) const;
		bool vertReferencesTri(size_t vertIdx, size_t triIdx) const;
		bool vertReferencesEdge(size_t vertIdx, size_t edgeIdx) const;

		bool verifyTris(size_t triIdx) const;
		bool verifyVerts(size_t vertIdx) const;
		bool verifyEdges(size_t edgeIdx, bool allowEmpty) const;

		bool _enforceManifold = true;
		static std::atomic<size_t> _statId;
		const size_t _id;
		size_t _changeNumber = 0;
		std::vector<CVertex> _vertices;
		SearchTreePtr _pVertTree;

		std::vector<CEdge> _edges;
		SearchTreePtr _pEdgeTree;
		std::map<CEdge, size_t> _edgeToIdxMap;

		std::vector<Vector3i> _tris;

		mutable std::vector<float> _glTriPoints, _glTriNormals, _glTriParams, _glTriCurvatureColors;
		mutable std::vector<unsigned int> _glTriIndices;
		SearchTreePtr _pTriTree;

		// These are cached data and can be reproduced. Marked mutable so they can be accessed from const getters
		mutable bool _useCentroidCache = true;
		mutable std::vector<Vector3d> _centroids;
		mutable std::vector<size_t> _sharpEdgeIndices;
		mutable std::vector<std::vector<size_t>> _sharpEdgeLoops;
		mutable bool _useNormalCache = true;
		mutable std::vector<Vector3d> _normals;
		mutable std::vector<double> _edgeCurvature, _vertCurvature, _triGap;

	};

	using CMeshPtr = std::shared_ptr<CMesh>;

	inline CMesh::CMesh(const BoundingBox& bbox)
		: CMesh(bbox.getMin(), bbox.getMax())
	{
	}

	inline void CMesh::setEnforceManifold(bool val)
	{
		_enforceManifold = val;
	}

	inline bool CMesh::enforceManifold() const
	{
		return _enforceManifold;
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

		std::vector<size_t> results;
		_pVertTree->find(ptBBox, results);
		for (const auto& index : results) {
			if ((_vertices[index]._pt - ptUnk).norm() < SAME_DIST_TOL) {
				return index;
			}
		}
		size_t result = _vertices.size();
		_vertices.push_back(CVertex(ptUnk));
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

	inline size_t CMesh::getId() const
	{
		return _id;
	}

	inline size_t CMesh::getChangeNumber() const
	{
		return _changeNumber;
	}

	inline void CMesh::changed()
	{
		_changeNumber++;
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

	template<typename LAMBDA>
	const std::vector<float>& CMesh::getGlTriCurvatureColors(LAMBDA curvatureToColorFunc) const // size = GlPoints.size() / 3
	{
		size_t requiredSize = 3 * 3 * _tris.size();
		if (_vertCurvature.empty()) {
			_glTriCurvatureColors.clear();
		} else if (_glTriCurvatureColors.size() == requiredSize)
			return _glTriCurvatureColors;
		else {
			_glTriCurvatureColors.clear();
			_glTriCurvatureColors.reserve(requiredSize);

			for (size_t triIdx = 0; triIdx < _tris.size(); triIdx++) {
				const auto& tri = _tris[triIdx];
				for (int i = 0; i < 3; i++) {
					auto curv = _vertCurvature[tri[i]];
					float rgb[3];
					if (curvatureToColorFunc(curv, rgb)) {
						_glTriCurvatureColors.push_back(rgb[0]);
						_glTriCurvatureColors.push_back(rgb[1]);
						_glTriCurvatureColors.push_back(rgb[2]);
					}
				}
			}
		}

		return _glTriCurvatureColors;
	}

	template<typename LAMBDA>
	void CMesh::getGlEdges(LAMBDA curvatureToColorFunc, bool includeSmooth, std::vector<float>& points, std::vector<float>& colors, std::vector<unsigned int>& indices) // size = GlPoints.size() / 3
	{
		points.clear();
		colors.clear();
		indices.clear();

		points.reserve(2 * 3 * _edges.size());
		indices.reserve(2 * _edges.size());

		unsigned int indexCount = 0;
		for (size_t edgeIdx = 0; edgeIdx < _edges.size(); edgeIdx++) {
			float curv = (float)_edgeCurvature[edgeIdx];
			if (!includeSmooth && fabs(curv) < 0.1)
				continue;

			float rgb[3];
			if (curvatureToColorFunc(curv, rgb)) {
				indices.push_back(indexCount++);
				indices.push_back(indexCount++);

				const auto& edge = _edges[edgeIdx];
				for (int i = 0; i < 2; i++) {
					const auto& pt = _vertices[edge._vertIndex[i]]._pt;
					for (int j = 0; j < 3; j++) {
						points.push_back((float)pt[j]);
						colors.push_back(rgb[j]);
					}
				}
			}
		}
		points.shrink_to_fit();
		colors.shrink_to_fit();
		indices.shrink_to_fit();
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
