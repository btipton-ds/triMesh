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

#include <iomanip>

#include <tm_ioUtil.h>
#include <tm_vertex.h>

using namespace std;
using namespace TriMesh;

size_t CVertex::numBytes() const
{
	size_t result = sizeof(CVertex);

	result += _faceIndices.size() * sizeof(size_t);
	result += _edgeIndices.size() * sizeof(size_t);
	return result;
}

void CVertex::write(ostream& out) const {
	uint8_t version = 0;
	out.write((char*) &version, sizeof(version));

	writeVector3(out, _pt);
	IoUtil::write(out, _edgeIndices);
	IoUtil::write(out, _faceIndices);
}

bool CVertex::read(istream& in) {
	uint8_t version = -1;
	in.read((char*)&version, sizeof(version));

	readVector3(in, _pt);
	IoUtil::read(in, _edgeIndices);
	IoUtil::read(in, _faceIndices);

	return true;
}

const std::vector<size_t>& CVertex::getEdgeIndices() const
{
	return _edgeIndices;
}

bool CVertex::containsEdgeIndex(size_t index) const
{
	auto iter = find(_edgeIndices.begin(), _edgeIndices.end(), index);
	return iter != _edgeIndices.end();
}

void CVertex::addEdgeIndex(size_t index)
{
	if (!containsEdgeIndex(index))
		_edgeIndices.push_back(index);
}

void CVertex::removeEdgeIndex(size_t index)
{
	auto iter = find(_edgeIndices.begin(), _edgeIndices.end(), index);
	if (iter != _edgeIndices.end())
		_edgeIndices.erase(iter);
}

void CVertex::changeEdgeIndex(size_t oldEdgeIdx, size_t newEdgeIdx)
{
	for (size_t i = 0; i < _edgeIndices.size(); i++) {
		if (_edgeIndices[i] == oldEdgeIdx) {
			_edgeIndices[i] = newEdgeIdx;
			break;
		}
	}
}

const std::vector<size_t>& CVertex::getFaceIndices() const
{
	return _faceIndices;
}

bool CVertex::containsFaceIndex(size_t index) const
{
	auto iter = find(_faceIndices.begin(), _faceIndices.end(), index);
	return iter != _faceIndices.end();
}

void CVertex::addFaceIndex(size_t index)
{
	if (!containsFaceIndex(index))
		_faceIndices.push_back(index);
}

void CVertex::removeFaceIndex(size_t index)
{
	auto iter = find(_faceIndices.begin(), _faceIndices.end(), index);
	if (iter != _faceIndices.end())
		_faceIndices.erase(iter);
}

void CVertex::changeFaceIndex(size_t oldFaceIdx, size_t newFaceIdx)
{
	for (size_t i = 0; i < _faceIndices.size(); i++) {
		if (_faceIndices[i] == oldFaceIdx) {
			_faceIndices[i] = newFaceIdx;
			break;
		}
	}
}
