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

#include <tm_defines.h>

#include <memory>
#include <vector>
#include <map>
#include <ostream>

#include <tm_math.h>

template<class T>
struct LineSegment;
using LineSegmentd = LineSegment<double>;

namespace TriMesh {

class CMesh;
using CMeshPtr = std::shared_ptr<CMesh>;

// need to split this into the raw edge and the connectivity for the search tree.
class CEdgeGeo {
public:
	CEdgeGeo(size_t vertIdx0 = stm1, size_t vertIdx1 = stm1);
	bool operator < (const CEdgeGeo& rhs) const;
	bool operator == (const CEdgeGeo& rhs) const;
	size_t otherVertIdx(size_t vertIndex) const;

	LineSegmentd getSeg(const CMesh* pMesh) const;
	LineSegmentd getSeg(const CMeshPtr& pMesh) const;

	size_t _vertIndex[2];
};

class CEdge : public CEdgeGeo {
public:
	struct TopolEntry {
		int _numFaces = 0;
		size_t _faceIndices[2] = { stm1, stm1 };
	};

	CEdge(size_t vertIdx0 = stm1, size_t vertIdx1 = stm1);

	TopolEntry* getTopol(size_t meshId);
	const TopolEntry* getTopol(size_t meshId) const;

	bool isAttachedToFace(size_t meshId, size_t faceIdx) const;
	void addFaceIndex(size_t meshId, size_t faceIdx);
	void removeFaceIndex(size_t meshId, size_t faceIdx);
	void changeFaceIndex(size_t meshId, size_t oldFaceIdx, size_t newFaceIdx);

	void write(std::ostream& out, size_t meshId) const;
	bool read(std::istream& in, size_t meshId);

	void dump(std::ostream& out) const;

	std::map<size_t, TopolEntry> _meshTopol;
};

inline size_t CEdgeGeo::otherVertIdx(size_t vertIndex) const
{
	if (vertIndex == _vertIndex[0])
		return _vertIndex[1];
	else if (vertIndex == _vertIndex[1])
		return _vertIndex[0];
	return -1;
}

}
