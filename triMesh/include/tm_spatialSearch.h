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

template <class BOX_TYPE, class INDEX_TYPE, int ENTRY_LIMIT>
class CSpatialSearchBase {
public:
	enum class BoxTestType {
		Contains, Intersects
	};

	struct Entry {
		Entry() = default;
		Entry(const Entry& src) = default;

		inline Entry(const BOX_TYPE& box, const INDEX_TYPE& idx)
			: _index(idx)
			, _bbox(box) {

		}

		inline const INDEX_TYPE& getIndex() const
		{
			return _index;
		}

		inline const BOX_TYPE& getBBox() const
		{
			return _bbox;
		}

	private:
		INDEX_TYPE _index = {};
		BOX_TYPE _bbox = {};
	};

	CSpatialSearchBase(const BOX_TYPE& bbox = BOX_TYPE(), int axis = 0);

	void reset(const BOX_TYPE& bbox);
	void clear();
	size_t numInTree() const;
	const BOX_TYPE& getBounds() const;

	std::vector<Entry> find(const BOX_TYPE& bbox, BoxTestType contains = BoxTestType::Intersects) const;
	size_t find(const BOX_TYPE& bbox, std::vector<Entry>& result, BoxTestType contains = BoxTestType::Intersects) const;
	size_t biDirRayCast(const Ray& ray, std::vector<INDEX_TYPE>& hits) const;

	bool add(const BOX_TYPE& bbox, const INDEX_TYPE& index);

	bool remove(const Entry& newEntry);
	bool remove(const BOX_TYPE& bbox, const INDEX_TYPE& index);

	void biDirRayCastRecursive(const Ray& ray, std::vector<INDEX_TYPE>& hits) const;

	void dump(std::wostream& out, size_t depth = 0) const;

private:
	bool add(const Entry& newEntry, int depth);
	void split(int depth);
	static bool boxesMatch(const BOX_TYPE& lhs, const BOX_TYPE& rhs, BoxTestType testType);

	size_t _numInTree = 0;
	BOX_TYPE _bbox;
	int _axis = 0;
	std::vector<Entry> _contents;
	std::shared_ptr<CSpatialSearchBase> _left, _right;
};

template <class BOX_TYPE, class INDEX_TYPE, int ENTRY_LIMIT>
inline size_t CSpatialSearchBase<BOX_TYPE, INDEX_TYPE, ENTRY_LIMIT>::numInTree() const {
	return _numInTree;
}

template <class BOX_TYPE, class INDEX_TYPE, int ENTRY_LIMIT>
const BOX_TYPE& CSpatialSearchBase<BOX_TYPE, INDEX_TYPE, ENTRY_LIMIT>::getBounds() const {
	return _bbox;
}

template <typename BOX_TYPE, typename INDEX_TYPE>
using CSpatialSearch = CSpatialSearchBase<BOX_TYPE, INDEX_TYPE, 25>;

template <typename BOX_TYPE>
using CSpatialSearchST = CSpatialSearch<BOX_TYPE, size_t>;

using CSpatialSearchSTd = CSpatialSearchST<CBoundingBox3D<double>>;
using CSpatialSearchSTf = CSpatialSearchST<CBoundingBox3D<float>>;

#include <tm_spatialSearch.hpp>
