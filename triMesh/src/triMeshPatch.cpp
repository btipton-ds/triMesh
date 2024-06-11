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

#include <list>
#include <tm_defines.h>

#include <algorithm>
#include <fstream>
#include <iomanip>

#include <cmath>

#include <../../stlReader/include/readStl.h>
#include <tm_ioUtil.h>
#include <tm_lineSegment.h>
#include <tm_ray.h>
#include <triMesh.h>
#include <triMeshPatch.h>
#include <MultiCoreUtil.h>

using namespace std;

using namespace TriMesh;

bool Patch::empty() const
{
	return _faces.empty();
}

void Patch::addFace(const vector<size_t>& face)
{
	_faces.push_back(face);
}

const vector<vector<size_t>>& Patch::getSharpEdgeChains() const
{
	return _sharpEdgeChains;
}

const vector<vector<size_t>>& Patch::getFaces() const
{
	return _faces;
}

void Patch::finishCreation(const CMesh* pMesh, double sinSharpEdgeAngle)
{
	if (_faces.empty())
		return;

	set<size_t> triSet, sharpEdgeSet;
	for (auto& face : _faces) {
		for (size_t triIdx : face) {
			triSet.insert(triIdx);
			const auto& tri = pMesh->getTri(triIdx);
			for (int i = 0; i < 3; i++) {
				int j = (i + 1) % 3;
				size_t edgeIdx = pMesh->findEdge(CEdge(tri[i], tri[j]));
				if (pMesh->isEdgeSharp(edgeIdx, sinSharpEdgeAngle))
					sharpEdgeSet.insert(edgeIdx);
			}
		}
		face.clear();
	}
	_faces.clear();
	_sharpEdgeChains.clear();

	while (!triSet.empty()) {
		vector<size_t> face;
		vector<size_t> stack;
		size_t triIdx = *triSet.begin();
		stack.push_back(triIdx);
		triSet.erase(triIdx);
		while (!stack.empty()) {
			size_t triIdx = stack.back();
			stack.pop_back();
			face.push_back(triIdx);
			const auto& tri = pMesh->getTri(triIdx);
			for (int i = 0; i < 3; i++) {
				int j = (i + 1) % 3;
				size_t triEdgeIdx = pMesh->findEdge(CEdge(tri[i], tri[j]));
				if (!sharpEdgeSet.contains(triEdgeIdx)) {
					const auto& edge = pMesh->getEdge(triEdgeIdx);
					for (int j = 0; j < edge._numFaces; j++) {
						size_t nextTriIdx = edge._faceIndices[j];
						if (triSet.contains(nextTriIdx)) {
							stack.push_back(nextTriIdx);
							triSet.erase(nextTriIdx);
						}
					}
				}
			}
		}

		if (!face.empty()) {
			_faces.push_back(face);
		}
	}

	while (!sharpEdgeSet.empty()) {
		vector<size_t> vertChain;
		size_t edgeIdx = *sharpEdgeSet.begin();
		sharpEdgeSet.erase(edgeIdx);
		const auto& edge = pMesh->getEdge(edgeIdx);
		vertChain.push_back(edge._vertIndex[0]);
		vertChain.push_back(edge._vertIndex[1]);

		bool found = true;
		while (found) {
			found = false;
			for (size_t edgeIdx : sharpEdgeSet) {
				const auto& edge = pMesh->getEdge(edgeIdx);
				size_t nextIdx = edge.otherVertIdx(vertChain.back());
				if (nextIdx != -1) {
					vertChain.insert(vertChain.end(), nextIdx);
					sharpEdgeSet.erase(edgeIdx);
					found = true;
					break;
				}

				nextIdx = edge.otherVertIdx(vertChain.front());
				if (nextIdx != -1) {
					vertChain.insert(vertChain.begin(), nextIdx);
					sharpEdgeSet.erase(edgeIdx);
					found = true;
					break;
				}
			}
		}

		vector<size_t> edgeChain;
		if (!vertChain.empty()) {
			size_t vertIdx0 = vertChain.back();
			vertChain.pop_back();
			while (!vertChain.empty()) {
				size_t vertIdx1 = vertChain.back();
				vertChain.pop_back();
				size_t edgeIdx = pMesh->findEdge(CEdge(vertIdx0, vertIdx1));
				assert(edgeIdx != -1);
				if (edgeIdx != -1)
					edgeChain.push_back(edgeIdx);

				vertIdx0 = vertIdx1;
			}
			if (!edgeChain.empty())
				_sharpEdgeChains.insert(_sharpEdgeChains.end(), edgeChain);
		}
	}
}
