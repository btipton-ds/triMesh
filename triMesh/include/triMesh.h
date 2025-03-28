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
#include <tm_ray.h>
#include <tm_vertex.h>

/*
	All vertices and triangles are stored in a CMeshRepo. If a shared repo is desired, create one and pass it as a parameter. If not
	a default repo is created automiatically.

	The repo unifies all vertex and triangle indices across all meshes.

	TODO - 
	move edges to the repo
	Fix back pointers so tris and edges have separate entries for each mesh
	Add file io
*/

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

	class Patch;
	using PatchPtr = std::shared_ptr<Patch>;

	class CMesh : public std::enable_shared_from_this<CMesh> {
	public:
		using SearchTree = CSpatialSearchST<double>;
		using SearchTreePtr = std::shared_ptr<SearchTree>;
		using SearchTreeConstPtr = std::shared_ptr<const SearchTree>;
		using BoundingBox = SearchTree::BOX_TYPE;
		using BoxTestType = SearchTree::BoxTestType;
		using SearchEntry = SearchTree::Entry;

		enum Options {
			OPT_NONE = 0,
			OPT_SKIP_DEGEN = 1, // Bit number, so power of 2
			// OPT_NEXT = 2,
			// OPT_NEXT = 4, etc
		};

		CMesh();
		CMesh(const BoundingBox& bbox);
		CMesh(const Vector3d& min, const Vector3d& max);

		void reset(const BoundingBox& bbox);
		void setEnforceManifold(bool val);
		bool enforceManifold() const;
		void enableOption(Options opt);
		void disableOption(Options opt);
		bool isEnabled(Options opt) const;

		size_t getId() const;
		size_t getChangeNumber() const;
		void changed();

		// Repo should only be written and read once. If writeRepo/readRepo is false
		// you must write the repos yourself.
		// If each mesh has its own repo, set writeRepo/readRepo true and it will be written with the mesh.
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

		template<class T>
		size_t addTriangle(const Vector3<T> pts[3], T maxEdgeLength = -1);
		template<class T>
		size_t addTriangle(const Vector3<T>& pt0, const Vector3<T>& pt1, const Vector3<T>& pt2, T maxEdgeLength = -1);
		template<class T>
		size_t addVertex(const Vector3<T>& ptUnk);

		template<class T>
		size_t addQuad(const Vector3<T>& pt0, const Vector3<T>& pt1, const Vector3<T>& pt2, const Vector3<T>& pt3, T maxEdgeLength = -1);
		template<class T>
		size_t addQuad(const Vector3<T> pts[4], T maxEdgeLength = -1);

		void splitLongTris(double maxEdgeLength);
		CMeshConstPtr splitWithPlane(const Planed& splitPlane, double tol = sameDistTol<double>()) const;
		void squeezeSkinnyTriangles(double minAngleDegrees);
		void squeezeEdge(size_t idx);
		CMeshPtr fix(double maxEdgeLength);

		// takes a vector of 8 points in {
		// 0,3,2,1
		// 4,5,6,7 } order
		template<class T>
		size_t addRectPrism(const std::vector<Vector3<T>>& pts);

		BoundingBox getBBox() const;

		BoundingBox getTriBBox(size_t triIdx) const;
		BoundingBox getEdgeBBox(size_t edgeIdx) const;
		BoundingBox getVertBBox(size_t vertIdx) const;
		bool intersectsTri(const LineSegmentd& seg, size_t idx, double tol, RayHitd& hit) const;
		bool bboxIntersectsTri(const BoundingBox& bbox, size_t idx) const;
		bool bboxIntersectsEdge(const BoundingBox& bbox, size_t idx) const;
		LineSegmentd getEdgesLineSeg(size_t edgeIdx) const;
		bool isEdgeSharp(size_t edgeIdx, double sinEdgeAngle) const;
		bool createPatches(const std::vector<size_t>& triIndices, double sinSharpEdgeAngle, std::vector<PatchPtr>& patches) const;

		const std::vector<size_t>& getSharpEdgeIndices(double edgeAngleRadians = 0) const;
		size_t createSharpEdgeVertexLines(size_t sharpVertIdx, std::set<size_t>& availEdges, double sharpEdgeAngleRadians, std::vector<std::vector<size_t>>& vertIndices) const;

		size_t numVertices() const;
		size_t numEdges() const;
		size_t numTris() const;
		size_t numLaminarEdges() const;
		size_t numBytes() const;

		CVertex& getVert(size_t idx);
		const CVertex& getVert(size_t idx) const;
		const Vector3i& getTri(size_t idx) const;
		const CEdge& getEdge(size_t idx) const;

		bool isClosed() const;
		double findMinGap(double tol = 0.0001, bool multiCore = true) const;
		void getGapHistogram(const std::vector<double>& binSizes, std::vector<size_t>& bins, bool multiCore = true) const;
		size_t rayCast(size_t triIdx, std::vector<RayHitd>& hits, bool biDir = true) const;
		size_t rayCast(const Rayd& ray, std::vector<RayHitd>& hits, bool biDir = true) const;
		size_t rayCast(const LineSegmentd& seg, std::vector<RayHitd>& hits, double tol = 1.0e-6) const;

		size_t findVerts(const BoundingBox& bbox, std::vector<SearchEntry>& vertIndices, BoxTestType contains = BoxTestType::IntersectsOrContains) const;
		size_t findVerts(const BoundingBox& bbox, std::vector<size_t>& vertIndices, BoxTestType contains = BoxTestType::IntersectsOrContains) const;

		size_t findEdges(const BoundingBox& bbox, std::vector<SearchEntry>& edgeIndices, BoxTestType contains = BoxTestType::IntersectsOrContains) const;
		size_t findEdges(const BoundingBox& bbox, std::vector<size_t>& edgeIndices, BoxTestType contains = BoxTestType::IntersectsOrContains) const;

		size_t findTris(const BoundingBox& bbox, std::vector<SearchEntry>& triIndices, BoxTestType contains = BoxTestType::IntersectsOrContains) const;
		size_t findTris(const BoundingBox& bbox, std::vector<size_t>& triIndices, BoxTestType contains = BoxTestType::IntersectsOrContains) const;

		Vector3d triCentroid(size_t triIdx) const;
		Vector3d triUnitNormal(size_t triIdx) const;
		Planed triPlane(size_t triIdx) const;
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
		bool isVertConvex(size_t vIdx, bool& isConvex, LineSegmentd& axis) const;

		void dumpObj(std::ostream& out) const;
		void dumpModelSharpEdgesObj(std::ostream& out, double sinAngle) const;

		size_t getSTLPoints(std::vector<Vector3f>& pts) const;
		const std::vector<float>& getGlTriPoints() const;
		const std::vector<float>& getGlTriNormals(bool smoothed) const;
		const std::vector<float>& getGlTriParams() const;
		template<typename LAMBDA>
		const std::vector<float>& getGlTriCurvatureColors(LAMBDA curvatureToColorFunc) const; // size = GlPoints.size() / 3
		const std::vector<unsigned int>& getGlTriIndices() const;

		// If all is true, get every edge. If false, only get sharp and curved edges.

		void getGlEdges(std::vector<float>& points, std::vector<unsigned int>& indices);

		template<typename LAMBDA>
		void getGlEdges(LAMBDA curvatureToColorFunc, bool includeSmooth, std::vector<float>& points, std::vector<float>& colors,
			double sinSharpAngle, std::vector<unsigned int>& sharpIndices, std::vector<unsigned int>& smoothIndices);

		bool testSqueezeEdge(size_t idx);
		bool testRemoveTri(size_t idx);

		bool verifyTopology(bool allowEmptyEdges) const;

		size_t processFoundEdges(const std::vector<SearchEntry>& allHits, const BoundingBox& bbox, std::vector<SearchEntry>& edgeIndices, BoxTestType contains = BoxTestType::Intersects) const;
		size_t processFoundEdges(const std::vector<size_t>& allHits, const BoundingBox& bbox, std::vector<size_t>& edgeIndices, BoxTestType contains = BoxTestType::Intersects) const;
		size_t processFoundTris(const std::vector<SearchEntry>& allHits, const BoundingBox& bbox, std::vector<SearchEntry>& triIndices, BoxTestType contains = BoxTestType::Intersects) const;
		size_t processFoundTris(const std::vector<size_t>& allHits, const BoundingBox& bbox, std::vector<size_t>& triIndices, BoxTestType contains = BoxTestType::Intersects) const;

		void clearSearchTrees();

	private:
		static bool sameTri(const Vector3i& tri0, const Vector3i& tri1);
		static bool areTriPointsDegenerate(const Vector3i& tri);

		size_t addVertex_d(const Vector3d& ptUnk);
		size_t addTriangle_d(const Vector3d& pt0, const Vector3d& pt1, const Vector3d& pt2, double maxEdgeLength);
		size_t addQuad_d(const Vector3d& pt0, const Vector3d& pt1, const Vector3d& pt2, const Vector3d& pt3, double maxEdgeLength);

		double findTriMinimumGap(size_t i) const;
		double calEdgeCurvature(size_t edgeIdx, double sinEdgeAngle) const;
		double calSinVertexAngle(size_t triIdx, size_t vertIdx, size_t& oppositeEdgeIdx) const;
		size_t getOtherVertIdx(const CEdge& thisEdge, size_t triIdx) const;
		bool removeTri(size_t triIdx);
		bool deleteTriFromStorage(size_t edgeIdx);
		bool deleteEdgeFromStorage(size_t edgeIdx);
		void mergeVertices(size_t vertIdxToKeep, size_t vertIdxToRemove);
		bool addVertexToEdgeLine(std::vector<size_t>& vertLine, std::set<size_t>& availEdges, double sinEdgeAngle) const;

		void addTriangle_d(const Vector3d& pt0, const Vector3d& pt1, const Vector3d& pt2, const Vector3d& srcNorm, double maxEdgeLength);
		void addTriangleStrip(std::vector<Vector3d>& pts0, std::vector<Vector3d>& pts1, const Vector3d& norm, double maxEdgeLength);

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
		size_t _options = 0;
		static std::atomic<size_t> _statId;
		const size_t _id;
		size_t _changeNumber = 0;

		std::map<CEdgeGeo, size_t> _edgeToIdxMap;

		mutable std::vector<float> _glTriPoints, _glTriNormals, _glTriParams, _glTriCurvatureColors;
		mutable std::vector<unsigned int> _glTriIndices;

		std::vector<CEdge> _edges;
		std::vector<CVertex> _vertices;
		std::vector<Vector3i> _tris;

		SearchTreePtr _pVertTree;
		SearchTreePtr _pEdgeTree;
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

	/*
	inline const CMeshRepoPtr& CMesh::getRepo() const
	{
		return _pRepo;
	}
	*/

	inline void CMesh::setEnforceManifold(bool val)
	{
		_enforceManifold = val;
	}

	inline bool CMesh::enforceManifold() const
	{
		return _enforceManifold;
	}

	inline void CMesh::enableOption(Options opt)
	{
		size_t bit = (size_t)opt;
		_options = _options | bit;
	}

	inline void CMesh::disableOption(Options opt)
	{
		size_t bit = (size_t)opt;
		_options = _options & ~bit;
	}

	inline bool CMesh::isEnabled(Options opt) const
	{
		size_t bit = (size_t)opt;
		return (_options & bit) == bit;
	}

	inline bool CMesh::areTriPointsDegenerate(const Vector3i& tri)
	{
		for (int i = 0; i < 3; i++) {
			for (int j = i + 1; j < 3; j++) {
				if (tri[i] == tri[j])
					return true;
			}
		}
		return false;
	}

	template<class T>
	inline size_t CMesh::addVertex(const Vector3<T>& ptUnk) {
		return addVertex_d(Vector3d(ptUnk));
	}

	template<class T>
	inline size_t CMesh::addTriangle(const Vector3<T> pts[3], T maxEdgeLength) {
		return addTriangle_d(pts[0], pts[1], pts[2], maxEdgeLength);
	}

	template<class T>
	inline size_t CMesh::addTriangle(const Vector3<T>& pt0, const Vector3<T>& pt1, const Vector3<T>& pt2, T maxEdgeLength) {
		return addTriangle_d(pt0, pt1, pt2, maxEdgeLength);
	}

	template<class T>
	inline size_t CMesh::addQuad(const Vector3<T> pts[4], T maxEdgeLength)
	{
		return addQuad_d(pts[0], pts[1], pts[2], pts[3], maxEdgeLength);
	}

	template<class T>
	inline size_t CMesh::addQuad(const Vector3<T>& pt0, const Vector3<T>& pt1, const Vector3<T>& pt2, const Vector3<T>& pt3, T maxEdgeLength)
	{
		return addQuad_d(pt0, pt1, pt2, pt3, maxEdgeLength);
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

	inline CVertex& CMesh::getVert(size_t idx) {
		return _vertices[idx];
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
