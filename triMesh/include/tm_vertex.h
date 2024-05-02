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
#include <vector>
#include <ostream>
#include <iomanip>

#include <tm_math.h>

namespace TriMesh {

struct CVertex {
	CVertex();
	CVertex(const CVertex& src) = default;
	CVertex(const Vector3d& pt);
	void write(std::ostream& out) const;
	bool read(std::istream& in);

	bool containsEdgeIndex(size_t index) const;
	void addEdgeIndex(size_t index);
	void removeEdgeIndex(size_t index);
	void changeEdgeIndex(size_t oldEdgeIdx, size_t newEdgeIdx);

	bool containsFaceIndex(size_t index) const;
	void addFaceIndex(size_t index);
	void removeFaceIndex(size_t index);
	void changeFaceIndex(size_t oldFaceIdx, size_t newFaceIdx);

	void dump(std::ostream& out) const;

	Vector3d _pt;
	std::vector<size_t> _faceIndices, _edgeIndices;
};

inline CVertex::CVertex() {
}

inline CVertex::CVertex(const Vector3d& pt)
	: _pt(pt) {
}

inline void CVertex::dump(std::ostream& out) const {
	out << std::setprecision(4) << std::fixed << "vert { pt:(" << _pt[0] << ", " << _pt[1] << ", " << _pt[2] << ")}\n";
}

}
