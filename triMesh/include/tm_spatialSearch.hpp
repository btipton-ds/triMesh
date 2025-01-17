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

#include <iostream>
#include <thread>
#include <tm_spatialSearch.h>

#define CSSB_TMPL template <class SCALAR_TYPE, class INDEX_TYPE, int ENTRY_LIMIT>
#define CSSB_DCL CSpatialSearchBase<SCALAR_TYPE, INDEX_TYPE, ENTRY_LIMIT>
#define CSSB_PTR_DCL CSpatialSearchBasePtr<SCALAR_TYPE, INDEX_TYPE, ENTRY_LIMIT>


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
	_left = _right = nullptr;
	_contents.clear();
	_numInTree = 0;
}

CSSB_TMPL
size_t CSSB_DCL::numBytes() const
{
	size_t result = sizeof(CSpatialSearchBase<SCALAR_TYPE, INDEX_TYPE, ENTRY_LIMIT>);

	result += _contents.capacity() + sizeof(Entry);
	if (_left)
		result += _left->numBytes();
	if (_right)
		result += _right->numBytes();
	return result;
}

CSSB_TMPL
size_t CSSB_DCL::find(const BOX_TYPE& bbox, vector<Entry>& result, BoxTestType testType) const {
	if (boxesMatch(_bbox, bbox, testType)) {
		for (const auto& entry : _contents) {
			const auto& bb = entry.getBBox();
			if (boxesMatch(_bbox, bbox, testType)) {
				result.push_back(entry);
			}
		}
		if (_left)
			_left->find(bbox, result, testType);
		if (_right)
			_right->find(bbox, result, testType);
	}
	return result.size();
}

CSSB_TMPL
size_t CSSB_DCL::find(const BOX_TYPE& bbox, vector<INDEX_TYPE>& result, BoxTestType testType) const {
	if (boxesMatch(_bbox, bbox, testType)) {
		for (const auto& entry : _contents) {
			const auto& bb = entry.getBBox();
			if (boxesMatch(bb, bbox, testType)) {
				result.push_back(entry.getIndex());
			}
		}
		if (_left)
			_left->find(bbox, result, testType);
		if (_right)
			_right->find(bbox, result, testType);
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
typename CSSB_DCL::SpatialSearchBasePtr CSSB_DCL::getSubTree(const BOX_TYPE& bbox) const
{
#if 0
	return shared_from_this();
#else
	bool useLeft = _left && (bbox.intersectsOrContains(_left->_bbox, (SCALAR_TYPE)SAME_DIST_TOL));
	bool useRight = _right && (bbox.intersectsOrContains(_right->_bbox, (SCALAR_TYPE)SAME_DIST_TOL));
	if (useLeft && useRight)
		return enable_shared_from_this<CSpatialSearchBase<SCALAR_TYPE, INDEX_TYPE, ENTRY_LIMIT>>::shared_from_this();

	for (const auto& entry : _contents) {
		if (bbox.intersectsOrContains(entry.getBBox(), (SCALAR_TYPE)SAME_DIST_TOL)) {
			return enable_shared_from_this<CSpatialSearchBase<SCALAR_TYPE, INDEX_TYPE, ENTRY_LIMIT>>::shared_from_this();
		}
	}

	if (useLeft)
		return _left->getSubTree(bbox);
	else if (useRight)
		return _right->getSubTree(bbox);

	return nullptr;
#endif
}

CSSB_TMPL
bool CSSB_DCL::add(const Entry& newEntry, int depth) {
	if (!_bbox.contains(newEntry.getBBox(), (SCALAR_TYPE)SAME_DIST_TOL))
		return false;

	for (const auto& curEntry : _contents) {
		if (curEntry.getIndex() == newEntry.getIndex() && curEntry.getBBox().contains(newEntry.getBBox(), (SCALAR_TYPE)SAME_DIST_TOL)) {
			assert(!"duplicate");
			return false;
		}
	}

	if (_left && _left->add(newEntry, depth + 1)) {
		_numInTree++;
		return true;
	} if (_right && _right->add(newEntry, depth + 1)) {
		_numInTree++;
		return true;
	}

	_contents.push_back(newEntry);
	_numInTree++;

	// Split node and add to correct node
	if (!_left && _contents.size() > ENTRY_LIMIT)
		split(depth);

	return true;

}

CSSB_TMPL
bool CSSB_DCL::remove(const Entry& newEntry) {
	if (_bbox.contains(newEntry.getBBox(), (SCALAR_TYPE)SAME_DIST_TOL)) {
		for (size_t idx = 0; idx < _contents.size(); idx++) {
			const auto& curEntry = _contents[idx];
			if (curEntry.getIndex() == newEntry.getIndex() && curEntry.getBBox().contains(newEntry.getBBox(), (SCALAR_TYPE)SAME_DIST_TOL)) {
				_contents.erase(_contents.begin() + idx);
				_numInTree--;
				return true;
			}
		}

		if (_left && _left->remove(newEntry)) {
			// TODO prune dead branch
			_numInTree--;
			return true;
		} if (_right && _right->remove(newEntry)) {
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
		for (const auto& entry : _contents) {
			if (entry.getBBox().intersects(ray, (SCALAR_TYPE)SAME_DIST_TOL)) {
				hits.push_back(entry.getIndex());
			}
		}
		if (_left)
			_left->biDirRayCastRecursive(ray, hits);
		if (_right)
			_right->biDirRayCastRecursive(ray, hits);
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
	out << pad << axisStr << " " << _contents.size() << "\n";
	for (const auto& entry : _contents) {
		out << pad << "  " << entry.getIndex() << "\n";
		out << pad << "  " << entry.getBBox().getMin()[0] << " " << entry.getBBox().getMin()[1] << " " << entry.getBBox().getMin()[2] << "\n";
		out << pad << "  " << entry.getBBox().getMax()[0] << " " << entry.getBBox().getMax()[1] << " " << entry.getBBox().getMax()[2] << "\n";
	}
	if (_left)
		_left->dump(out, depth + 1);
	if (_right)
		_right->dump(out, depth + 1);
}

CSSB_TMPL
void CSSB_DCL::split(int depth) {
	BOX_TYPE leftBBox, rightBBox;
	_bbox.split(_axis, leftBBox, rightBBox, (SCALAR_TYPE)0.10);
	int nextAxis = (_axis + 1) % 3;
	_left = new CSpatialSearchBase(leftBBox, nextAxis);
	_right = new CSpatialSearchBase(rightBBox, nextAxis);

	const vector<Entry> temp(_contents);
	_contents.clear();

	for (const auto& entry : temp) {
		if (leftBBox.contains(entry.getBBox(), (SCALAR_TYPE)SAME_DIST_TOL))
			_left->add(entry, depth + 1);
		else if (rightBBox.contains(entry.getBBox(), (SCALAR_TYPE)SAME_DIST_TOL))
			_right->add(entry, depth + 1);
		else
			_contents.push_back(entry);
	}

}

CSSB_TMPL
inline bool CSSB_DCL::boxesMatch(const BOX_TYPE& lhs, const BOX_TYPE& rhs, BoxTestType testType)
{
	if (testType == BoxTestType::Contains)
		return lhs.contains(rhs, (SCALAR_TYPE)SAME_DIST_TOL);
	else
		return lhs.intersectsOrContains(rhs, (SCALAR_TYPE)SAME_DIST_TOL);
}
