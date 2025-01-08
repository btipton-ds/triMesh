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

size_t CEdge::numBytes() const
{
	size_t result = sizeof(CEdge);

	result += _meshTopol.size() * sizeof(pair<size_t, TopolEntry>);

	return result;
}

int CEdge::numFaces(size_t meshId) const
{
	auto pTopol = _meshTopol.find(meshId);
	if (pTopol)
		return pTopol->_numFaces;

	return 0;
}

size_t CEdge::getTriIdx(size_t meshId, int idx) const
{
	auto pTopol = _meshTopol.find(meshId);
	if (pTopol && idx < pTopol->_numFaces)
		return pTopol->_faceIndices[idx];

	return -1;
}

bool CEdge::isAttachedToFace(size_t meshId, size_t faceIdx) const
{
	auto pTopol = _meshTopol.find(meshId);
	if (pTopol) {
		const auto& r = *pTopol;
		for (int i = 0; i < r._numFaces; i++) {
			if (r._faceIndices[i] == faceIdx)
				return true;
		}
	}
	return false;
}

void CEdge::write(std::ostream& out, size_t meshId) const
{
	uint8_t version = 0;
	out.write((char*) &version, sizeof(version));

	out.write((char*)&_vertIndex[0], sizeof(size_t));
	out.write((char*)&_vertIndex[1], sizeof(size_t));

	auto pTopol = _meshTopol.find(meshId);
	if (pTopol) {

		auto& rec = *pTopol;
		out.write((char*) &rec._numFaces, sizeof(int));
		out.write((char*) rec._faceIndices, rec._numFaces * sizeof(size_t));
	}
}

bool CEdge::read(std::istream& in, size_t meshId)
{
	uint8_t version = -1;
	in.read((char*)&version, sizeof(version));

	in.read((char*)&_vertIndex[0], sizeof(size_t));
	in.read((char*)&_vertIndex[1], sizeof(size_t));

	auto pTopol = _meshTopol.find(meshId);
	if (!pTopol)
		pTopol = _meshTopol.insert(meshId);

	auto& rec = *pTopol;
	in.read((char*)&rec._numFaces, sizeof(int));
	in.read((char*)rec._faceIndices, rec._numFaces * sizeof(size_t));

	return true;
}

void CEdge::addFaceIndex(size_t meshId, size_t faceIdx) {
	auto pTopol = _meshTopol.find(meshId);
	if (!pTopol)
		pTopol = _meshTopol.insert(meshId);
	auto& r = *pTopol;
	for (int i = 0; i < r._numFaces; i++) {
		if (r._faceIndices[i] == faceIdx)
			return;
	}
	if (r._numFaces < TRI_MESH_MAX_EDGE_CONNECTED_FACES) {
		r._faceIndices[r._numFaces++] = faceIdx;
	}
}

void CEdge::removeFaceIndex(size_t meshId, size_t faceIdx)
{
	auto pTopol = _meshTopol.find(meshId);
	if (pTopol) {
		auto& r = *pTopol;
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
	auto pTopol = _meshTopol.find(meshId);
	if (pTopol) {
		auto& r = *pTopol;
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

CEdge::Topology::Topology(const Topology& src)
{
	for (const TopolEntry* p : src._data) {
		_data.push_back(new TopolEntry(*p));
	}
}

CEdge::Topology::~Topology()
{
	for (size_t i = 0; i < _data.size(); i++) {
		delete _data[i];
		_data[i] = nullptr;
	}
}

CEdge::Topology& CEdge::Topology::operator =(const Topology& src)
{
	for (size_t i = 0; i < _data.size(); i++) {
		delete _data[i];
		_data[i] = 0;
	}
	_data.clear();

	for (const TopolEntry* p : src._data) {
		_data.push_back(new TopolEntry(*p));
	}

	return *this;
}

size_t CEdge::Topology::size() const
{
	return _data.size();
}

size_t CEdge::Topology::capacity() const
{
	return _data.capacity();
}

size_t CEdge::Topology::numBytes() const
{
	size_t result = 0;

	result += _data.capacity() * sizeof(TopolEntry);

	return result;
}

CEdge::TopolEntry* CEdge::Topology::insert(size_t meshId)
{

	TopolEntry* pEntry = find(meshId);
	if (pEntry)
		return pEntry;
	pEntry = new TopolEntry;
	pEntry->_meshId = meshId;
	_data.push_back(pEntry);
	std::sort(_data.begin(), _data.end(), [](const TopolEntry* pLhs, const TopolEntry* pRhs)->bool {
		return pLhs->_meshId < pRhs->_meshId;
	});

	return pEntry;
}

const CEdge::TopolEntry* CEdge::Topology::find(size_t meshId) const
{
	size_t min = 0, max = _data.size() - 1, idx = _data.size() / 2;
	while (idx < _data.size()) {
		if (min == max) {
			if (_data[idx]->_meshId == meshId)
				return _data[idx];
			break;
		} else if (_data[idx]->_meshId < meshId) {
			max = idx;
			idx = (min + max) / 2;
		} else if (_data[idx]->_meshId > meshId) {
			min = idx;
			idx = (min + max) / 2;
		}
	}

	return nullptr;
}

CEdge::TopolEntry* CEdge::Topology::find(size_t meshId)
{
	size_t min = 0, max = _data.size() - 1, idx = _data.size() / 2;
	while (idx < _data.size()) {
		if (min == max) {
			if (_data[idx]->_meshId == meshId)
				return _data[idx];
			break;
		} else if (_data[idx]->_meshId < meshId) {
			max = idx;
			idx = (min + max) / 2;
		} else if (_data[idx]->_meshId > meshId) {
			min = idx;
			idx = (min + max) / 2;
		}
	}
	return nullptr;
}
