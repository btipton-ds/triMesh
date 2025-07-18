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
#include <vector>
#include <memory>

#include <tm_math.h>
#include <tm_boundingBox.h>

template <class SCALAR_TYPE, class INDEX_TYPE, int ENTRY_LIMIT>
class CSpatialSearchBase;

template <class SCALAR_TYPE, class INDEX_TYPE, int ENTRY_LIMIT>
using CSpatialSearchBasePtr = std::shared_ptr<CSpatialSearchBase<SCALAR_TYPE, INDEX_TYPE, ENTRY_LIMIT>>;

template <class SCALAR_TYPE, class INDEX_TYPE, int ENTRY_LIMIT>
using CSpatialSearchBaseConstPtr = std::shared_ptr<const CSpatialSearchBase<SCALAR_TYPE, INDEX_TYPE, ENTRY_LIMIT>>;

template <class SCALAR_TYPE, class INDEX_TYPE, int ENTRY_LIMIT>
class CSpatialSearchBase : public std::enable_shared_from_this<CSpatialSearchBase<SCALAR_TYPE, INDEX_TYPE, ENTRY_LIMIT>> {
public:
	using BOX_TYPE = CBoundingBox3D<SCALAR_TYPE>;
	using SpatialSearchBasePtr = CSpatialSearchBasePtr<SCALAR_TYPE, INDEX_TYPE, ENTRY_LIMIT>;
	using SpatialSearchBaseConstPtr = CSpatialSearchBaseConstPtr<SCALAR_TYPE, INDEX_TYPE, ENTRY_LIMIT>;

	enum class BoxTestType {
		Contains, Intersects, IntersectsOrContains
	};

	struct Entry {
		Entry() = default;
		Entry(const Entry& src) = default;
		Entry(const BOX_TYPE& box, const INDEX_TYPE& idx);

		const INDEX_TYPE& getIndex() const;
		const BOX_TYPE& getBBox() const;
		bool operator < (const Entry& rhs) const;

	private:
		INDEX_TYPE _index = {};
		BOX_TYPE _bbox = {};
	};

	class Refiner {
	public:
		virtual bool entryIntersects(const BOX_TYPE& bbox, const Entry& entry) const = 0;
	};

	CSpatialSearchBase(const BOX_TYPE& bbox = BOX_TYPE(), int axis = 0);
	virtual ~CSpatialSearchBase();

	void reset(const BOX_TYPE& bbox);
	void clear();
	size_t numInTree() const;
	size_t numBytes() const;
	const BOX_TYPE& getBounds() const;

	size_t count(const BOX_TYPE& bbox, const Refiner* pRefiner, BoxTestType contains = BoxTestType::IntersectsOrContains) const;
	size_t find(const BOX_TYPE& bbox, const Refiner* pRefiner, std::vector<Entry>& result, BoxTestType contains = BoxTestType::IntersectsOrContains) const;
	size_t find(const BOX_TYPE& bbox, const Refiner* pRefiner, std::vector<INDEX_TYPE>& result, BoxTestType contains = BoxTestType::IntersectsOrContains) const;
	size_t biDirRayCast(const Ray<SCALAR_TYPE>& ray, std::vector<INDEX_TYPE>& hits) const;

	template<class FUNC>
	void traverse(const BOX_TYPE& bbox, const FUNC& func, BoxTestType contains = BoxTestType::IntersectsOrContains) const;
	template<class FUNC>
	void biDirRayCastTraverse(const Ray<SCALAR_TYPE>& ray, const FUNC& func) const;

	SpatialSearchBaseConstPtr getSubTree(const BOX_TYPE& bbox, const Refiner* pRefiner, BoxTestType testType = BoxTestType::IntersectsOrContains) const;

	bool add(const BOX_TYPE& bbox, const INDEX_TYPE& index);

	bool remove(const Entry& newEntry);
	bool remove(const BOX_TYPE& bbox, const INDEX_TYPE& index);

	void biDirRayCastRecursive(const Ray<SCALAR_TYPE>& ray, std::vector<INDEX_TYPE>& hits) const;

	void dump(std::wostream& out, size_t depth = 0) const;

private:
	struct Contents {
		BOX_TYPE _bbox;
		std::vector<Entry> _vals;
	};
	bool add(const Entry& newEntry, int depth);
	void copyTreeToReducedTree(const BOX_TYPE& smallerBbox, const Refiner* pRefiner, SpatialSearchBasePtr& dst, BoxTestType testType) const;

	void addToContents(const Entry& newEntry);
	void split(int depth);

	void setSubContents(const BOX_TYPE& smallerBbox, const Refiner* pRefiner, const CSpatialSearchBase* pSrc, BoxTestType testType);

	static bool containsBbox(const BOX_TYPE& bbox, const BOX_TYPE& otherBbox, BoxTestType testType);
	static bool containsEntry(const BOX_TYPE& bbox, const Entry& entry, const Refiner* pRefiner, BoxTestType testType);

	size_t _numInTree = 0;
	size_t _id = 0;
	BOX_TYPE _bbox;
	int _axis = 0;
	std::shared_ptr<Contents> _pContents;
	std::shared_ptr<CSpatialSearchBase> _pLeft, _pRight;
};

template <class SCALAR_TYPE, class INDEX_TYPE, int ENTRY_LIMIT>
inline size_t CSpatialSearchBase<SCALAR_TYPE, INDEX_TYPE, ENTRY_LIMIT>::numInTree() const {
	return _numInTree;
}

template <class SCALAR_TYPE, class INDEX_TYPE, int ENTRY_LIMIT>
const typename CSpatialSearchBase<SCALAR_TYPE, INDEX_TYPE, ENTRY_LIMIT>::BOX_TYPE& CSpatialSearchBase<SCALAR_TYPE, INDEX_TYPE, ENTRY_LIMIT>::getBounds() const {
	return _bbox;
}

template <typename SCALAR_TYPE, typename INDEX_TYPE>
using CSpatialSearch = CSpatialSearchBase<SCALAR_TYPE, INDEX_TYPE, 25>;

template <typename SCALAR_TYPE>
using CSpatialSearchST = CSpatialSearch<SCALAR_TYPE, size_t>;

using CSpatialSearchSTd = CSpatialSearchST<double>;
using CSpatialSearchSTf = CSpatialSearchST<float>;

//#include <tm_spatialSearch.hpp>
