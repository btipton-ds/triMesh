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

CEdge::TopolEntry* CEdge::getTopol(size_t meshId)
{
	auto mIter = _meshTopol.find(meshId);
	if (mIter != _meshTopol.end()) {
		return mIter->second.get();
	}
	return nullptr;
}

const CEdge::TopolEntry* CEdge::getTopol(size_t meshId) const
{
	auto mIter = _meshTopol.find(meshId);
	if (mIter != _meshTopol.end()) {
		return mIter->second.get();
	}
	return nullptr;
}

bool CEdge::isAttachedToFace(size_t meshId, size_t faceIdx) const
{
	auto mIter = _meshTopol.find(meshId);
	if (mIter != _meshTopol.end()) {
		const auto r = *mIter->second;
		for (int i = 0; i < r._numFaces; i++) {
			if (r._faceIndices[i] == faceIdx)
				return true;
		}
	}
	return false;
}

void CEdge::write(std::ostream& out) const
{
	uint8_t version = 0;
	out.write((char*) &version, sizeof(version));

	out.write((char*)&_vertIndex[0], sizeof(size_t));
	out.write((char*)&_vertIndex[1], sizeof(size_t));

	size_t n = _meshTopol.size();
	out.write((char*)&n, sizeof(n));
	for (const auto& rec : _meshTopol) {
		size_t meshId = rec.first;
		out.write((char*)&meshId, sizeof(size_t));

		out.write((char*)&rec.second->_numFaces, sizeof(int));
		for (size_t i = 0; i < rec.second->_numFaces; i++)
			out.write((char*)&rec.second->_faceIndices[i], sizeof(size_t));
	}
}

bool CEdge::read(std::istream& in)
{
	uint8_t version = -1;
	in.read((char*)&version, sizeof(version));

	in.read((char*)&_vertIndex[0], sizeof(size_t));
	in.read((char*)&_vertIndex[1], sizeof(size_t));
	size_t n;
	in.read((char*)&n, sizeof(n));
	for (size_t i = 0; i < n; i++) {
		size_t meshId;
		in.read((char*)&meshId, sizeof(size_t));
		auto p = make_shared<TopolEntry>();
		_meshTopol.insert(make_pair(meshId, p));

		in.read((char*)&p->_numFaces, sizeof(int));
		for (size_t i = 0; i < p->_numFaces; i++)
			in.read((char*)&p->_faceIndices[i], sizeof(size_t));
	}
	return true;
}

void CEdge::addFaceIndex(size_t meshId, size_t faceIdx) {
	auto mIter = _meshTopol.find(meshId);
	if (mIter == _meshTopol.end())
		mIter = _meshTopol.insert(make_pair(meshId, make_shared<TopolEntry>())).first;
	auto& r = *mIter->second;
	for (int i = 0; i < r._numFaces; i++) {
		if (r._faceIndices[i] == faceIdx)
			return;
	}
	if (r._numFaces < 2) {
		r._faceIndices[r._numFaces++] = faceIdx;
	}
}

void CEdge::removeFaceIndex(size_t meshId, size_t faceIdx)
{
	auto mIter = _meshTopol.find(meshId);
	if (mIter != _meshTopol.end()) {
		auto& r = *mIter->second;
		for (int i = 0; i < r._numFaces; i++) {
			if (r._faceIndices[i] == faceIdx) {
				if (i == 0) {
					r._faceIndices[i] = r._faceIndices[i + 1];
				}
				r._numFaces--;
				r._faceIndices[1] = -1;
				if (r._numFaces == 0)
					r._faceIndices[0] = -1;
				return;
			}
		}
	}
}

void CEdge::changeFaceIndex(size_t meshId, size_t oldFaceIdx, size_t newFaceIdx)
{
	auto mIter = _meshTopol.find(meshId);
	if (mIter != _meshTopol.end()) {
		auto& r = *mIter->second;
		for (int i = 0; i < r._numFaces; i++) {
			if (r._faceIndices[i] == oldFaceIdx) {
				r._faceIndices[i] = newFaceIdx;
				return;
			}
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
