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

void CVertex::write(ostream& out) const {
	uint8_t version = 0;
	out.write((char*) &version, sizeof(version));

	writeVector3(out, _pt);

	size_t n = _meshTopol.size();
	out.write((char*)&n, sizeof(n));
	for (const auto& pair : _meshTopol) {
		size_t meshId = pair.first;
		out.write((char*)&meshId, sizeof(size_t));
		auto p = pair.second;
		IoUtil::write(out, p->_edgeIndices);
		IoUtil::write(out, p->_faceIndices);
	}
}

bool CVertex::read(istream& in) {
	uint8_t version = -1;
	in.read((char*)&version, sizeof(version));

	readVector3(in, _pt);

	size_t n;
	in.read((char*)&n, sizeof(n));
	for (size_t i = 0; i < n; i++) {
		size_t meshId;
		in.read((char*)&meshId, sizeof(size_t));
		auto p = make_shared<TopolEntry>();
		_meshTopol.insert(make_pair(meshId, p));

		IoUtil::read(in, p->_edgeIndices);
		IoUtil::read(in, p->_faceIndices);
	}
	return true;
}

const std::vector<size_t>* CVertex::getEdgeIndices(size_t meshId) const
{
	auto mIter = _meshTopol.find(meshId);
	if (mIter != _meshTopol.end()) {
		const auto& edgeIndices = mIter->second->_edgeIndices;
		return &edgeIndices;
	}
	return nullptr;
}

bool CVertex::containsEdgeIndex(size_t meshId, size_t index) const
{
	auto mIter = _meshTopol.find(meshId);
	if (mIter != _meshTopol.end()) {
		const auto& edgeIndices = mIter->second->_edgeIndices;
		auto iter = find(edgeIndices.begin(), edgeIndices.end(), index);
		return iter != edgeIndices.end();
	}
	return false;
}

void CVertex::addEdgeIndex(size_t meshId, size_t index)
{
	auto mIter = _meshTopol.find(meshId);
	if (mIter == _meshTopol.end())
		mIter = _meshTopol.insert(make_pair(meshId, make_shared<TopolEntry>())).first;

	auto& edgeIndices = mIter->second->_edgeIndices;
	auto iter = find(edgeIndices.begin(), edgeIndices.end(), index);
	if (iter == edgeIndices.end())
		edgeIndices.push_back(index);
}

void CVertex::removeEdgeIndex(size_t meshId, size_t index)
{
	auto mIter = _meshTopol.find(meshId);
	if (mIter != _meshTopol.end()) {
		auto& edgeIndices = mIter->second->_edgeIndices;
		auto iter = find(edgeIndices.begin(), edgeIndices.end(), index);
		if (iter != edgeIndices.end())
			edgeIndices.erase(iter);
	}
}

void CVertex::changeEdgeIndex(size_t meshId, size_t oldEdgeIdx, size_t newEdgeIdx)
{
	auto mIter = _meshTopol.find(meshId);
	if (mIter != _meshTopol.end()) {
		auto& edgeIndices = mIter->second->_edgeIndices;
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
	auto mIter = _meshTopol.find(meshId);
	if (mIter != _meshTopol.end()) {
		const auto& faceIndices = mIter->second->_faceIndices;
		return &faceIndices;
	}
	return nullptr;
}

bool CVertex::containsFaceIndex(size_t meshId, size_t index) const
{
	auto mIter = _meshTopol.find(meshId);
	if (mIter != _meshTopol.end()) {
		const auto& faceIndices = mIter->second->_faceIndices;
		auto iter = find(faceIndices.begin(), faceIndices.end(), index);
		return iter != faceIndices.end();
	}
	return false;
}

void CVertex::addFaceIndex(size_t meshId, size_t index)
{
	auto mIter = _meshTopol.find(meshId);
	if (mIter == _meshTopol.end())
		mIter = _meshTopol.insert(make_pair(meshId, make_shared<TopolEntry>())).first;

	auto& faceIndices = mIter->second->_faceIndices;
	auto iter = find(faceIndices.begin(), faceIndices.end(), index);
	if (iter == faceIndices.end())
		faceIndices.push_back(index);
}

void CVertex::removeFaceIndex(size_t meshId, size_t index)
{
	auto mIter = _meshTopol.find(meshId);
	if (mIter != _meshTopol.end()) {
		auto& faceIndices = mIter->second->_faceIndices;
		auto iter = find(faceIndices.begin(), faceIndices.end(), index);
		if (iter != faceIndices.end())
			faceIndices.erase(iter);
	}
}

void CVertex::changeFaceIndex(size_t meshId, size_t oldFaceIdx, size_t newFaceIdx)
{
	auto mIter = _meshTopol.find(meshId);
	if (mIter != _meshTopol.end()) {
		auto& faceIndices = mIter->second->_faceIndices;
		for (size_t i = 0; i < faceIndices.size(); i++) {
			if (faceIndices[i] == oldFaceIdx) {
				faceIndices[i] = newFaceIdx;
				break;
			}
		}
	}
}
