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

#include <tm_repo.h>
#include <tm_lineSegment.hpp>

using namespace TriMesh;

size_t CMeshRepo::numBytes() const
{
	size_t result = sizeof(CMeshRepo);

	for (const auto& v : _vertices) {
		result += v.numBytes();
	}
	for (const auto& e : _edges) {
		result += e.numBytes();
	}
	return result;
}

bool CMeshRepo::intersectsTri(const LineSegmentd& seg, size_t triIdx, double tol, RayHitd& hit) const
{
	const auto& tri = _tris[triIdx];
	const Vector3d* pts[] = {
		&_vertices[tri[0]]._pt,
		&_vertices[tri[1]]._pt,
		&_vertices[tri[2]]._pt,
	};

	if (seg.intersectTri(pts, hit, tol)) {
		hit.triIdx = triIdx;
		return true;
	}

	return false;
}