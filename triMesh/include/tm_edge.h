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

#include <ostream>
#include <vector>

#include <tm_math.h>

namespace TriMesh {

	class CEdge {
	public:

		CEdge(size_t vertIdx0 = stm1, size_t vertIdx1 = stm1);
		bool operator < (const CEdge& rhs) const;
		bool operator == (const CEdge& rhs) const;
		void addFace(size_t faceIdx);

		void dump(std::ostream& out) const;

		size_t _vertIndex[2];
		int _numFaces;
		size_t _faceIndex[2];
	};


	inline CEdge::CEdge(size_t vertIdx0, size_t vertIdx1)
		: _numFaces(0)
	{
		_faceIndex[0] = _faceIndex[1] = stm1;
		if (vertIdx0 <= vertIdx1) {
			_vertIndex[0] = vertIdx0;
			_vertIndex[1] = vertIdx1;
		}
		else {
			_vertIndex[0] = vertIdx1;
			_vertIndex[1] = vertIdx0;
		}
	}

	inline bool CEdge::operator < (const CEdge& rhs) const {
		for (int i = 0; i < 2; i++) {
			if (_vertIndex[i] < rhs._vertIndex[i])
				return true;
			else if (_vertIndex[i] > rhs._vertIndex[i])
				return false;
		}
		return false;
	}

	inline bool CEdge::operator == (const CEdge& rhs) const {
		return _vertIndex[0] == rhs._vertIndex[0] && _vertIndex[1] == rhs._vertIndex[1];
	}

	inline void CEdge::addFace(size_t faceIdx) {
		for (int i = 0; i < _numFaces; i++) {
			if (_faceIndex[i] == faceIdx)
				return;
		}
		if (_numFaces < 2) {
			_faceIndex[_numFaces++] = faceIdx;
		}
		else {
			// Log error
		}
	}

	inline void CEdge::dump(std::ostream& out) const {
		out << "edge { verts:(" << _vertIndex[0] << ", " << _vertIndex[1] << "), faces(";
		for (int i = 0; i < _numFaces; i++) {
			out << _faceIndex[i] << ",";
		}
		out << ") }\n";
	}

}
