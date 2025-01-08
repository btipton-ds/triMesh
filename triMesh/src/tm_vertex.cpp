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

	result += _meshTopol.numBytes();

	return result;
}

void CVertex::write(ostream& out, size_t meshId) const {
	uint8_t version = 0;
	out.write((char*) &version, sizeof(version));

	writeVector3(out, _pt);

	auto pTopol = _meshTopol.find(meshId);
	if (pTopol) {
		auto& rec = *pTopol;
		IoUtil::write(out, rec._edgeIndices);
		IoUtil::write(out, rec._faceIndices);
	}
}

bool CVertex::read(istream& in, size_t meshId) {
	uint8_t version = -1;
	in.read((char*)&version, sizeof(version));

	readVector3(in, _pt);

	auto pTopol = _meshTopol.find(meshId);
	if (!pTopol)
		pTopol = _meshTopol.insert(meshId);
	auto& rec = *pTopol;
	IoUtil::read(in, rec._edgeIndices);
	IoUtil::read(in, rec._faceIndices);

	return true;
}

const std::vector<size_t>* CVertex::getEdgeIndices(size_t meshId) const
{
	auto pTopol = _meshTopol.find(meshId);
	if (pTopol) {
		const auto& edgeIndices = pTopol->_edgeIndices;
		return &edgeIndices;
	}
	return nullptr;
}

bool CVertex::containsEdgeIndex(size_t meshId, size_t index) const
{
	auto pTopol = _meshTopol.find(meshId);
	if (pTopol) {
		const auto& edgeIndices = pTopol->_edgeIndices;
		auto iter = find(edgeIndices.begin(), edgeIndices.end(), index);
		return iter != edgeIndices.end();
	}
	return false;
}

void CVertex::addEdgeIndex(size_t meshId, size_t index)
{
	auto pTopol = _meshTopol.find(meshId);
	if (!pTopol)
		pTopol = _meshTopol.insert(meshId);

	auto& edgeIndices = pTopol->_edgeIndices;
	auto iter = find(edgeIndices.begin(), edgeIndices.end(), index);
	if (iter == edgeIndices.end())
		edgeIndices.push_back(index);
}

void CVertex::removeEdgeIndex(size_t meshId, size_t index)
{
	auto pTopol = _meshTopol.find(meshId);
	if (pTopol) {
		auto& edgeIndices = pTopol->_edgeIndices;
		auto iter = find(edgeIndices.begin(), edgeIndices.end(), index);
		if (iter != edgeIndices.end())
			edgeIndices.erase(iter);
	}
}

void CVertex::changeEdgeIndex(size_t meshId, size_t oldEdgeIdx, size_t newEdgeIdx)
{
	auto pTopol = _meshTopol.find(meshId);
	if (!pTopol) {
		auto& edgeIndices = pTopol->_edgeIndices;
		for (size_t i = 0; i < edgeIndices.size(); i++) {
			if (edgeIndices[i] == oldEdgeIdx) {
				edgeIndices[i] = newEdgeIdx;
				break;
			}
		}
	}
}

const std::vector<size_t>* CVertex::getFaceIndices(size_t meshId) const
{
	auto pTopol = _meshTopol.find(meshId);
	if (!pTopol) {
		const auto& faceIndices = pTopol->_faceIndices;
		return &faceIndices;
	}
	return nullptr;
}

bool CVertex::containsFaceIndex(size_t meshId, size_t index) const
{
	auto pTopol = _meshTopol.find(meshId);
	if (!pTopol) {
		const auto& faceIndices = pTopol->_faceIndices;
		auto iter = find(faceIndices.begin(), faceIndices.end(), index);
		return iter != faceIndices.end();
	}
	return false;
}

void CVertex::addFaceIndex(size_t meshId, size_t index)
{
	auto pTopol = _meshTopol.find(meshId);
	if (!pTopol) 
		pTopol = _meshTopol.insert(meshId);

	auto& faceIndices = pTopol->_faceIndices;
	auto iter = find(faceIndices.begin(), faceIndices.end(), index);
	if (iter == faceIndices.end())
		faceIndices.push_back(index);
}

void CVertex::removeFaceIndex(size_t meshId, size_t index)
{
	auto pTopol = _meshTopol.find(meshId);
	if (pTopol) {
		auto& faceIndices = pTopol->_faceIndices;
		auto iter = find(faceIndices.begin(), faceIndices.end(), index);
		if (iter != faceIndices.end())
			faceIndices.erase(iter);
	}
}

void CVertex::changeFaceIndex(size_t meshId, size_t oldFaceIdx, size_t newFaceIdx)
{
	auto pTopol = _meshTopol.find(meshId);
	if (pTopol) {
		auto& faceIndices = pTopol->_faceIndices;
		for (size_t i = 0; i < faceIndices.size(); i++) {
			if (faceIndices[i] == oldFaceIdx) {
				faceIndices[i] = newFaceIdx;
				break;
			}
		}
	}
}

CVertex::Topology::~Topology()
{
	for (size_t i = 0; i < _data.size(); i++) {
		delete _data[i];
		_data[i] = nullptr;
	}
}

CVertex::Topology::Topology(const Topology& src)
{
	for (const TopolEntry* p : src._data) {
		_data.push_back(new TopolEntry(*p));
	}
}

CVertex::Topology& CVertex::Topology::operator =(const Topology& src)
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

size_t CVertex::Topology::size() const
{
	return _data.size();
}

size_t CVertex::Topology::capacity() const
{
	return _data.capacity();
}

size_t CVertex::Topology::numBytes() const
{
	size_t result = 0;

	result += _data.capacity() * sizeof(TopolEntry);
	for (size_t i = 0; i < _data.size(); i++) {
		const auto& rec = *_data[i];
		result += rec._edgeIndices.capacity() * sizeof(size_t);
		result += rec._faceIndices.capacity() * sizeof(size_t);
	}

	return result;
}

CVertex::TopolEntry* CVertex::Topology::insert(size_t meshId)
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

const CVertex::TopolEntry* CVertex::Topology::find(size_t meshId) const
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
		}
		else if (_data[idx]->_meshId > meshId) {
			min = idx;
			idx = (min + max) / 2;
		}
	}

	return nullptr;
}

CVertex::TopolEntry* CVertex::Topology::find(size_t meshId)
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
		}
		else if (_data[idx]->_meshId > meshId) {
			min = idx;
			idx = (min + max) / 2;
		}
	}
	return nullptr;
}

