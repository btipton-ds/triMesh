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

#include <iostream>
#include <tm_spatialSearch.h>

#define CSSB_TMPL template <class BOX_TYPE, class INDEX_TYPE, int ENTRY_LIMIT>
#define CSSB_DCL CSpatialSearchBase<BOX_TYPE, INDEX_TYPE, ENTRY_LIMIT>

CSSB_TMPL
CSSB_DCL::CSpatialSearchBase(const BOX_TYPE& bbox, int axis)
	: _bbox(bbox)
	, _axis(axis)
{
}

CSSB_TMPL
void CSSB_DCL::reset(const BOX_TYPE& bbox) {
	clear();
	_bbox = bbox;
}

CSSB_TMPL
void CSSB_DCL::clear() {
	_axis = 0;
	_left = _right = nullptr;
	_contents.clear(); 
	_numInTree = 0;
}

CSSB_TMPL
std::vector<INDEX_TYPE> CSSB_DCL::find(const BOX_TYPE& bbox, bool contains) const {
	std::vector<INDEX_TYPE> result;
	find(bbox, result, contains);
	return result;
}

CSSB_TMPL
size_t CSSB_DCL::find(const BOX_TYPE& bbox, std::vector<INDEX_TYPE>& result, bool contains) const {
	if (contains ? _bbox.contains(bbox) : _bbox.intersects(bbox)) {
		for (const auto& entry : _contents) {
			const auto& bb = entry._bbox;
			if (contains ? bb.contains(bbox) : bb.intersects(bbox)) {
				result.push_back(entry._index);
			}
		}
		if (_left)
			_left->find(bbox, result, contains);
		if (_right)
			_right->find(bbox, result, contains);
	}
	return result.size();
}

CSSB_TMPL
size_t CSSB_DCL::biDirRayCast(const Ray& ray, std::vector<INDEX_TYPE>& hits) const {
	hits.clear();

	biDirRayCastRecursive(ray, hits);

	return hits.size();
}

CSSB_TMPL
bool CSSB_DCL::add(const Entry& newEntry, int depth) {
	if (!_bbox.contains(newEntry._bbox))
		return false;

	for (const auto& curEntry : _contents) {
		if (curEntry._index == newEntry._index && curEntry._bbox.contains(newEntry._bbox)) {
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
	if (_bbox.contains(newEntry._bbox)) {
		for (size_t idx = 0; idx < _contents.size(); idx++) {
			const auto& curEntry = _contents[idx];
			if (curEntry._index == newEntry._index && curEntry._bbox.contains(newEntry._bbox)) {
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
void CSSB_DCL::biDirRayCastRecursive(const Ray& ray, std::vector<INDEX_TYPE>& hits) const {
	if (_bbox.intersects(ray)) {
		for (const auto& entry : _contents) {
			if (entry._bbox.intersects(ray)) {
				hits.push_back(entry._index);
			}
		}
		if (_left)
			_left->biDirRayCastRecursive(ray, hits);
		if (_right)
			_right->biDirRayCastRecursive(ray, hits);
	}
}

CSSB_TMPL
void CSSB_DCL::dump(std::wostream& out, size_t depth) const
{
	std::wstring pad = L"";
	for (size_t i = 0; i < depth; i++)
		pad += L"   ";
	std::wstring axisStr = L"XAxis";
	if (_axis == 1)
		axisStr = L"YAxis";
	else if (_axis == 2)
		axisStr = L"ZAxis";
	out << pad << "Depth: " << depth << "\n";
	out << pad << _bbox.getMin()[0] << " " << _bbox.getMin()[1] << " " << _bbox.getMin()[2] << "\n";
	out << pad << _bbox.getMax()[0] << " " << _bbox.getMax()[1] << " " << _bbox.getMax()[2] << "\n";
	out << pad << axisStr << " " << _contents.size() << "\n";
	for (const auto& entry : _contents) {
		out << pad << "  " << entry._index << "\n";
		out << pad << "  " << entry._bbox.getMin()[0] << " " << entry._bbox.getMin()[1] << " " << entry._bbox.getMin()[2] << "\n";
		out << pad << "  " << entry._bbox.getMax()[0] << " " << entry._bbox.getMax()[1] << " " << entry._bbox.getMax()[2] << "\n";
	}
	if (_left)
		_left->dump(out, depth + 1);
	if (_right)
		_right->dump(out, depth + 1);
}

CSSB_TMPL
void CSSB_DCL::split(int depth) {
	BOX_TYPE leftBBox, rightBBox;
	_bbox.split(_axis, leftBBox, rightBBox, 0.10);
	int nextAxis = (_axis + 1) % 3;
	_left = std::make_shared<CSpatialSearchBase>(leftBBox, nextAxis);
	_right = std::make_shared<CSpatialSearchBase>(rightBBox, nextAxis);

	const std::vector<Entry> temp(_contents);
	_contents.clear();

	for (const auto& entry : temp) {
		if (leftBBox.contains(entry._bbox))
			_left->add(entry, depth + 1);
		else if (rightBBox.contains(entry._bbox))
			_right->add(entry, depth + 1);
		else
			_contents.push_back(entry);
	}

}
