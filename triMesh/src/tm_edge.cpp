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

#include <iostream>
#include <tm_edge.h>
#include <tm_lineSegment.h>
#include <triMesh.h>

using namespace std;
using namespace TriMesh;

LineSegmentd CEdgeGeo::getSeg(const CMesh* pMesh) const
{
	Vector3d pt0 = pMesh->getVert(_vertIndex[0])._pt;
	Vector3d pt1 = pMesh->getVert(_vertIndex[1])._pt;
	return LineSegmentd(pt0, pt1);
}

LineSegmentd CEdgeGeo::getSeg(const CMeshPtr& pMesh) const
{
	Vector3d pt0 = pMesh->getVert(_vertIndex[0])._pt;
	Vector3d pt1 = pMesh->getVert(_vertIndex[1])._pt;
	return LineSegmentd(pt0, pt1);
}

CEdgeGeo::CEdgeGeo(size_t vertIdx0, size_t vertIdx1)
{
	if (vertIdx0 <= vertIdx1) {
		_vertIndex[0] = vertIdx0;
		_vertIndex[1] = vertIdx1;
	}
	else {
		_vertIndex[0] = vertIdx1;
		_vertIndex[1] = vertIdx0;
	}
}

bool CEdgeGeo::operator < (const CEdgeGeo& rhs) const {
	for (int i = 0; i < 2; i++) {
		if (_vertIndex[i] < rhs._vertIndex[i])
			return true;
		else if (_vertIndex[i] > rhs._vertIndex[i])
			return false;
	}
	return false;
}

bool CEdgeGeo::operator == (const CEdgeGeo& rhs) const {
	return _vertIndex[0] == rhs._vertIndex[0] && _vertIndex[1] == rhs._vertIndex[1];
}

CEdge::CEdge(size_t vertIdx0, size_t vertIdx1)
	: CEdgeGeo(vertIdx0, vertIdx1)
{
}

size_t CEdge::numBytes() const
{
	size_t result = sizeof(CEdge);

	return result;
}

int CEdge::numFaces() const
{
	return _numFaces;
}

size_t CEdge::getTriIdx(int idx) const
{
	if (idx < _numFaces)
		return _faceIndices[idx];

	return -1;
}

bool CEdge::isAttachedToFace(size_t faceIdx) const
{
	for (int i = 0; i < _numFaces; i++) {
		if (_faceIndices[i] == faceIdx)
			return true;
	}
	return false;
}

void CEdge::write(std::ostream& out) const
{
	uint8_t version = 0;
	out.write((char*) &version, sizeof(version));

	out.write((char*)&_vertIndex[0], sizeof(size_t));
	out.write((char*)&_vertIndex[1], sizeof(size_t));

	out.write((char*)&_numFaces, sizeof(int));
	out.write((char*)_faceIndices, _numFaces * sizeof(size_t));
}

bool CEdge::read(std::istream& in)
{
	uint8_t version = -1;
	in.read((char*)&version, sizeof(version));

	in.read((char*)&_vertIndex[0], sizeof(size_t));
	in.read((char*)&_vertIndex[1], sizeof(size_t));

	in.read((char*)&_numFaces, sizeof(int));
	in.read((char*)_faceIndices, _numFaces * sizeof(size_t));

	return true;
}

void CEdge::addFaceIndex(size_t faceIdx) {
	for (int i = 0; i < _numFaces; i++) {
		if (_faceIndices[i] == faceIdx)
			return;
	}

	if (_numFaces < TRI_MESH_MAX_EDGE_CONNECTED_FACES) {
		_faceIndices[_numFaces++] = faceIdx;
	}
}

void CEdge::removeFaceIndex(size_t faceIdx)
{
	for (int i = 0; i < _numFaces; i++) {
		if (_faceIndices[i] == faceIdx) {
			if (i == 0) {
				_faceIndices[i] = _faceIndices[i + 1];
			}
			_numFaces--;
			_faceIndices[1] = -1;
			if (_numFaces == 0)
				_faceIndices[0] = -1;
			return;
		}
	}
}

void CEdge::changeFaceIndex(size_t oldFaceIdx, size_t newFaceIdx)
{
	for (int i = 0; i < _numFaces; i++) {
		if (_faceIndices[i] == oldFaceIdx) {
			_faceIndices[i] = newFaceIdx;
			return;
		}
	}
}

void CEdge::dump(std::ostream& out) const {
	out << "edge { verts:(" << _vertIndex[0] << ", " << _vertIndex[1] << "), faces(";
/*
	for (int i = 0; i < _numFaces; i++) {
		out << _faceIndices[i] << ",";
	}
*/
	out << ") }\n";
}
