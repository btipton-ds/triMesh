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

using namespace std;

#include <tm_defines.h>
#include <iostream>
#include <thread>
#include <tm_spatialSearch.h>

#define CSSB_TMPL template <class SCALAR_TYPE, class INDEX_TYPE, int ENTRY_LIMIT>
#define CSSB_DCL CSpatialSearchBase<SCALAR_TYPE, INDEX_TYPE, ENTRY_LIMIT>
#define CSSB_PTR_DCL CSpatialSearchBasePtr<SCALAR_TYPE, INDEX_TYPE, ENTRY_LIMIT>

#define DO_SPATIAL_SEARCH_TREE_VERIFICATION 0

CSSB_TMPL
inline CSSB_DCL::Entry::Entry(const BOX_TYPE& box, const INDEX_TYPE& idx)
	: _index(idx)
	, _bbox(box) {

}

CSSB_TMPL
inline const INDEX_TYPE& CSSB_DCL::Entry::getIndex() const
{
	return _index;
}

CSSB_TMPL
inline const typename CSSB_DCL::BOX_TYPE& CSSB_DCL::Entry::getBBox() const
{
	return _bbox;
}

CSSB_TMPL
inline bool CSSB_DCL::Entry::operator < (const Entry& rhs) const
{
	return _index < rhs._index;
}

CSSB_TMPL
CSSB_DCL::CSpatialSearchBase(const BOX_TYPE& bbox, int axis)
	: enable_shared_from_this<CSpatialSearchBase>()
	, _bbox(bbox)
	, _axis(axis)
{
	static atomic<size_t> s_count;
	_id = s_count++;
}

CSSB_TMPL
CSSB_DCL::~CSpatialSearchBase()
{
	clear();
}

CSSB_TMPL
void CSSB_DCL::reset(const BOX_TYPE& bbox) {
	clear();
	_bbox = bbox;
}

CSSB_TMPL
void CSSB_DCL::clear() {
	_numInTree = 0;
	_axis = 0;
	_pLeft = _pRight = nullptr;
	_pContents = nullptr;
	_numInTree = 0;
}

CSSB_TMPL
size_t CSSB_DCL::numBytes() const
{
	size_t result = sizeof(CSpatialSearchBase<SCALAR_TYPE, INDEX_TYPE, ENTRY_LIMIT>);

	if (_pContents)
		result += _pContents->_vals.capacity() * sizeof(Entry);
	if (_pLeft)
		result += _pLeft->numBytes();
	if (_pRight)
		result += _pRight->numBytes();
	return result;
}

CSSB_TMPL
size_t CSSB_DCL::find(const BOX_TYPE& bbox, const Refiner* pRefiner, vector<Entry>& result, BoxTestType testType) const {
	if (containsBbox(_bbox, bbox, testType)) {
		if (_pContents && containsBbox(_pContents->_bbox, bbox, testType)) {
			for (const auto& entry : _pContents->_vals) {
				if (containsEntry(bbox, entry, pRefiner, testType)) {
					result.push_back(entry);
				}
			}
		}
		if (_pLeft)
			_pLeft->find(bbox, pRefiner, result, testType);
		if (_pRight)
			_pRight->find(bbox, pRefiner, result, testType);
	}
	return result.size();
}

CSSB_TMPL
size_t CSSB_DCL::count(const BOX_TYPE& bbox, const Refiner* pRefiner, BoxTestType testType) const {
	size_t result = 0;
	if (containsBbox(_bbox, bbox, testType)) {
		if (_pContents && containsBbox(_pContents->_bbox, bbox, testType)) {
			for (const auto& entry : _pContents->_vals) {
				if (containsEntry(bbox, entry, pRefiner, testType)) {
					result++;
				}
			}
		}
		if (_pLeft)
			result += _pLeft->count(bbox, pRefiner, testType);
		if (_pRight)
			result += _pRight->count(bbox, pRefiner, testType);
	}
	return result;
}

CSSB_TMPL
size_t CSSB_DCL::find(const BOX_TYPE& bbox, const Refiner* pRefiner, vector<INDEX_TYPE>& result, BoxTestType testType) const {
	if (containsBbox(_bbox, bbox, testType)) {
		if (_pContents && containsBbox(_pContents->_bbox, bbox, testType)) {
			for (const auto& entry : _pContents->_vals) {
				if (containsEntry(bbox, entry, pRefiner, testType)) {
					result.push_back(entry.getIndex());
				}
			}
		}
		if (_pLeft)
			_pLeft->find(bbox, pRefiner, result, testType);
		if (_pRight)
			_pRight->find(bbox, pRefiner, result, testType);
	}
	return result.size();
}

CSSB_TMPL
size_t CSSB_DCL::biDirRayCast(const Ray<SCALAR_TYPE>& ray, vector<INDEX_TYPE>& hits) const {
	hits.clear();

	biDirRayCastRecursive(ray, hits);

	return hits.size();
}

CSSB_TMPL
template<class FUNC>
void CSSB_DCL::traverse(const BOX_TYPE& bbox, const FUNC& func, BoxTestType testType) const {
	if (containsBbox(_bbox, bbox, testType)) {
		if (_pContents && containsBbox(_pContents->_bbox, bbox, testType)) {
			for (const auto& entry : _pContents->_vals) {
				if (containsEntry(bbox, entry, nullptr, testType)) {
					if (!func(entry.getIndex()))
						return;
				}
			}
		}
		if (_pLeft)
			_pLeft->traverse(bbox, func, testType);
		if (_pRight)
			_pRight->traverse(bbox, func, testType);
	}
}

CSSB_TMPL
template<class FUNC>
void CSSB_DCL::biDirRayCastTraverse(const Ray<SCALAR_TYPE>& ray, const FUNC& func) const {
	if (_bbox.intersects(ray, (SCALAR_TYPE)SAME_DIST_TOL)) {
		if (_pContents && _pContents->_bbox.intersects(ray, (SCALAR_TYPE)SAME_DIST_TOL)) {
			for (const auto& entry : _pContents->_vals) {
				if (entry.getBBox().intersects(ray, (SCALAR_TYPE)SAME_DIST_TOL)) {
					if (!func(ray, entry.getIndex()))
						return;
				}
			}
		}
		if (_pLeft)
			_pLeft->biDirRayCastTraverse(ray, func);
		if (_pRight)
			_pRight->biDirRayCastTraverse(ray, func);
	}
}

CSSB_TMPL
typename CSSB_DCL::SpatialSearchBaseConstPtr CSSB_DCL::getSubTree(const BOX_TYPE& bbox, const Refiner* pRefiner, BoxTestType testType) const
{
#if 0
	return this->shared_from_this();
#else
	vector<INDEX_TYPE> entries;
	if (find(bbox, pRefiner, entries, testType)) {
#if !(DO_SPATIAL_SEARCH_TREE_VERIFICATION && defined(_DEBUG))
		entries.clear();
#endif
		shared_ptr<CSpatialSearchBase> result = make_shared<CSpatialSearchBase>(_bbox, _axis);
		copyTreeToReducedTree(bbox, pRefiner, result, testType);

#if DO_SPATIAL_SEARCH_TREE_VERIFICATION && defined(_DEBUG)
		vector<INDEX_TYPE> entries1;
		if (result) {
			if (result->find(bbox, refineFunc, entries1)) {
				if (entries.size() == entries1.size()) {
					set<INDEX_TYPE> test;
					test.insert(entries1.begin(), entries1.end());
					for (const auto& entry : entries) {
						if (!test.contains(entry)) {
							assert(!"Subtree search contents did not match source tree search contents.");
						}
					}
				}
				else {
					assert(!"Subtree search size did not match source tree search size.");
				}
			}
			else {
				assert(!"Subtree search result did not match source tree search result.");
			}
		}
#endif // _DEBUG
		return result;
	}
	return nullptr;
#endif
}

CSSB_TMPL
void CSSB_DCL::copyTreeToReducedTree(const BOX_TYPE& smallerBbox, const Refiner* pRefiner, SpatialSearchBasePtr& dst, BoxTestType testType) const
{
	dst->setSubContents(smallerBbox, pRefiner, this, testType);

	if (_pLeft) {
		size_t n = _pLeft->count(smallerBbox, pRefiner, testType);
		if (n > 0) {
			dst->_pLeft = make_shared<CSpatialSearchBase>(_pLeft->_bbox, _pLeft->_axis);
			_pLeft->copyTreeToReducedTree(smallerBbox, pRefiner, dst->_pLeft, testType);
			assert(dst->_pLeft->count(smallerBbox, pRefiner, testType) == n);
			dst->_numInTree += dst->_pLeft->_numInTree;
		}
	}

	if (_pRight) {
		size_t n = _pRight->count(smallerBbox, pRefiner, testType);
		if (n > 0) {
			dst->_pRight = make_shared<CSpatialSearchBase>(_pRight->_bbox, _pRight->_axis);
			_pRight->copyTreeToReducedTree(smallerBbox, pRefiner, dst->_pRight, testType);
			assert(dst->_pRight->count(smallerBbox, pRefiner, testType) == n);
			dst->_numInTree += dst->_pRight->_numInTree;
		}
	}
}

CSSB_TMPL
bool CSSB_DCL::add(const Entry& newEntry, int depth) {
	if (!_bbox.contains(newEntry.getBBox(), (SCALAR_TYPE)SAME_DIST_TOL))
		return false;

	if (_pContents) {
		for (const auto& curEntry : _pContents->_vals) {
			if (curEntry.getIndex() == newEntry.getIndex() && curEntry.getBBox().contains(newEntry.getBBox(), (SCALAR_TYPE)SAME_DIST_TOL)) {
				assert(!"duplicate");
				return false;
			}
		}
	}

	if (_pLeft && _pLeft->add(newEntry, depth + 1)) {
		_numInTree++;
		return true;
	} if (_pRight && _pRight->add(newEntry, depth + 1)) {
		_numInTree++;
		return true;
	}

	addToContents(newEntry);
	_numInTree++;

	// Split node and add to correct node
	if (!_pLeft && _pContents->_vals.size() > ENTRY_LIMIT)
		split(depth);

	return true;

}

CSSB_TMPL
void CSSB_DCL::setSubContents(const BOX_TYPE& smallerBbox, const Refiner* pRefiner, const CSpatialSearchBase* pSrc, BoxTestType testType)
{
	const auto tol = (SCALAR_TYPE)SAME_DIST_TOL;

	if (!pSrc || !pSrc->_pContents)
		return;

	if (containsBbox(pSrc->_pContents->_bbox, smallerBbox, testType)) {
		size_t n = 0;
		for (auto& entry : pSrc->_pContents->_vals) {
			if (containsEntry(smallerBbox, entry, pRefiner, testType)) {
				n++;
			}
		}

		if (n != 0) {
			_pContents = pSrc->_pContents;
			_numInTree += _pContents->_vals.size();
		}
	}

}

CSSB_TMPL
void CSSB_DCL::addToContents(const Entry& newEntry)
{
	if (!_pContents)
		_pContents = make_shared<Contents>();

	_pContents->_vals.push_back(newEntry);

	// Need to clip the newBox to the limits of the current box or ray casting will return false positives.
	BOX_TYPE newBox(_bbox);
	newBox.merge(newEntry.getBBox());
	auto& oldMin = newBox.getMin();
	auto& oldMax = newBox.getMax();

	auto minPt = oldMin;
	auto maxPt = oldMax;
	for (int i = 0; i < 3; i++) {
		if (minPt[i] < oldMin[i])
			minPt[i] = oldMin[i];
		if (maxPt[i] > oldMax[i])
			maxPt[i] = oldMax[i];
	}
	newBox = BOX_TYPE(minPt, maxPt);
	_pContents->_bbox.merge(newBox);
}

CSSB_TMPL
bool CSSB_DCL::remove(const Entry& newEntry) {
	if (_bbox.contains(newEntry.getBBox(), (SCALAR_TYPE)SAME_DIST_TOL)) {
		if (_pContents) {
			for (size_t idx = 0; idx < _pContents->_vals.size(); idx++) {
				const auto& curEntry = _pContents->_vals[idx];
				if (curEntry.getIndex() == newEntry.getIndex() && curEntry.getBBox().contains(newEntry.getBBox(), (SCALAR_TYPE)SAME_DIST_TOL)) {
					_pContents->_vals.erase(_pContents->_vals.begin() + idx);
					_numInTree--;
					return true;
				}
			}
		}

		if (_pLeft && _pLeft->remove(newEntry)) {
			// TODO prune dead branch
			_numInTree--;
			return true;
		} if (_pRight && _pRight->remove(newEntry)) {
			// TODO prune dead branch
			_numInTree--;
			return true;
		}
	}
	return false;
}

CSSB_TMPL
bool CSSB_DCL::add(const BOX_TYPE& bbox, const INDEX_TYPE& index) {
	return add(Entry(bbox, index), 0);
}

CSSB_TMPL
bool CSSB_DCL::remove(const BOX_TYPE& bbox, const INDEX_TYPE& index) {
	return remove(Entry(bbox, index));
}

CSSB_TMPL
void CSSB_DCL::biDirRayCastRecursive(const Ray<SCALAR_TYPE>& ray, vector<INDEX_TYPE>& hits) const {
	if (_bbox.intersects(ray, (SCALAR_TYPE)SAME_DIST_TOL)) {
		if (_pContents && _pContents->_bbox.intersects(ray, (SCALAR_TYPE)SAME_DIST_TOL)) {
			for (const auto& entry : _pContents->_vals) {
				if (entry.getBBox().intersects(ray, (SCALAR_TYPE)SAME_DIST_TOL)) {
					hits.push_back(entry.getIndex());
				}
			}
		}
		if (_pLeft)
			_pLeft->biDirRayCastRecursive(ray, hits);
		if (_pRight)
			_pRight->biDirRayCastRecursive(ray, hits);
	}
}

CSSB_TMPL
void CSSB_DCL::dump(wostream& out, size_t depth) const
{
	wstring pad = L"";
	for (size_t i = 0; i < depth; i++)
		pad += L"   ";
	wstring axisStr = L"XAxis";
	if (_axis == 1)
		axisStr = L"YAxis";
	else if (_axis == 2)
		axisStr = L"ZAxis";
	out << pad << "Depth: " << depth << "\n";
	out << pad << _bbox.getMin()[0] << " " << _bbox.getMin()[1] << " " << _bbox.getMin()[2] << "\n";
	out << pad << _bbox.getMax()[0] << " " << _bbox.getMax()[1] << " " << _bbox.getMax()[2] << "\n";
	if (_pContents) {
		out << pad << axisStr << " " << _pContents->_vals.size() << "\n";
		for (const auto& entry : _pContents->_vals) {
			out << pad << "  " << entry.getIndex() << "\n";
			out << pad << "  " << entry.getBBox().getMin()[0] << " " << entry.getBBox().getMin()[1] << " " << entry.getBBox().getMin()[2] << "\n";
			out << pad << "  " << entry.getBBox().getMax()[0] << " " << entry.getBBox().getMax()[1] << " " << entry.getBBox().getMax()[2] << "\n";
		}
	}
	if (_pLeft)
		_pLeft->dump(out, depth + 1);
	if (_pRight)
		_pRight->dump(out, depth + 1);
}

CSSB_TMPL
void CSSB_DCL::split(int depth) {
	BOX_TYPE leftBBox, rightBBox;
	_bbox.split(_axis, leftBBox, rightBBox, (SCALAR_TYPE)0.10);
	int nextAxis = (_axis + 1) % 3;
	_pLeft = make_shared<CSpatialSearchBase>(leftBBox, nextAxis);
	_pRight = make_shared<CSpatialSearchBase>(rightBBox, nextAxis);
	assert(_pContents);
	const vector<Entry> temp(_pContents->_vals);
	_pContents = nullptr;

	for (const auto& entry : temp) {
		if (leftBBox.contains(entry.getBBox(), (SCALAR_TYPE)SAME_DIST_TOL))
			_pLeft->add(entry, depth + 1);
		else if (rightBBox.contains(entry.getBBox(), (SCALAR_TYPE)SAME_DIST_TOL))
			_pRight->add(entry, depth + 1);
		else {
			addToContents(entry);
		}
	}

}

CSSB_TMPL
inline bool CSSB_DCL::containsBbox(const BOX_TYPE& bbox, const BOX_TYPE& otherBbox, BoxTestType testType)
{
	bool result = false;
	switch (testType) {
		default:
		case BoxTestType::IntersectsOrContains:
			result = bbox.intersectsOrContains(otherBbox, (SCALAR_TYPE)SAME_DIST_TOL);
			break;
		case BoxTestType::Contains:
			result = bbox.contains(otherBbox, (SCALAR_TYPE)SAME_DIST_TOL);
			break;
		case BoxTestType::Intersects:
			result = bbox.intersects(otherBbox, (SCALAR_TYPE)SAME_DIST_TOL);
			break;
	}

	return result;
}

CSSB_TMPL
inline bool CSSB_DCL::containsEntry(const BOX_TYPE& bbox, const Entry& entry, const Refiner* pRefiner, BoxTestType testType)
{
	const auto& otherBbox = entry.getBBox();

	bool result = false;
	switch (testType) {
	default:
	case BoxTestType::IntersectsOrContains:
		result = bbox.intersectsOrContains(otherBbox, (SCALAR_TYPE)SAME_DIST_TOL);
		break;
	case BoxTestType::Contains:
		result = bbox.contains(otherBbox, (SCALAR_TYPE)SAME_DIST_TOL);
		break;
	case BoxTestType::Intersects:
		result = bbox.intersects(otherBbox, (SCALAR_TYPE)SAME_DIST_TOL);
		break;
	}

	return result && (!pRefiner || pRefiner->entryIntersects(bbox, entry));
}

