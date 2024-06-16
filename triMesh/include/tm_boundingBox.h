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
#include <tm_math.h>
#include <tm_rectangle.h>
#include <tm_vector3.h>

template <class SCALAR_TYPE>
class CBoundingBox3D {
public:
	using Scalar = SCALAR_TYPE;
	using POINT_TYPE = Vector3<SCALAR_TYPE>;

	CBoundingBox3D();
	CBoundingBox3D(const POINT_TYPE& pt0);
	CBoundingBox3D(const POINT_TYPE& pt0, const POINT_TYPE& pt1);
	void clear();
	bool empty() const;
	void merge(const POINT_TYPE& pt);
	void merge(const CBoundingBox3D& bboxIn);
	const POINT_TYPE& getMin() const;
	const POINT_TYPE& getMax() const;
	POINT_TYPE range() const;
	bool contains(const POINT_TYPE& pt, SCALAR_TYPE tol) const;
	bool contains(const CBoundingBox3D& other, SCALAR_TYPE tol) const;

	bool intersects(const Ray<SCALAR_TYPE>& ray, SCALAR_TYPE tol) const;
	bool intersects(const Ray<SCALAR_TYPE>& ray, std::vector<POINT_TYPE>& pts, SCALAR_TYPE tol) const;
	bool intersects(const LineSegment<SCALAR_TYPE>& seg, SCALAR_TYPE tol, int skipAxis) const;
	bool intersects(const LineSegment<SCALAR_TYPE>& seg, std::vector<POINT_TYPE>& pts, SCALAR_TYPE tol, int skipAxis) const;

	bool intersectsOrContains(const CBoundingBox3D& otherBox, SCALAR_TYPE tol) const;
	bool intersectsOrContains(const LineSegment<SCALAR_TYPE>& seg, SCALAR_TYPE tol, int skipAxis) const;
	bool intersectsOrContains(const POINT_TYPE& pt0, const POINT_TYPE& pt1, const POINT_TYPE& pt2, SCALAR_TYPE tol) const;

	void split(int axis, CBoundingBox3D& left, CBoundingBox3D& right, Scalar overlap = 0) const;
	void grow(Scalar dist);
	void growPercent(SCALAR_TYPE amount);
	void getEdges(LineSegment<SCALAR_TYPE> edgeSegs[12]) const;

	void write(std::ostream& out) const;
	void read(std::istream& in);
private:
	static const POINT_TYPE _axes[3];

	bool intersectsInner(const LineSegment<SCALAR_TYPE>& seg, std::vector<POINT_TYPE>& pts, SCALAR_TYPE tol, bool getAll, int skipAxis) const;

	POINT_TYPE _min, _max;
};

using CBoundingBox3Dd = CBoundingBox3D<double>;
using CBoundingBox3Df = CBoundingBox3D<float>;

template <typename SCALAR_TYPE>
inline void CBoundingBox3D<SCALAR_TYPE>::write(std::ostream& out) const
{
	writeVector3(out, _min);
	writeVector3(out, _max);
}

template <typename SCALAR_TYPE>
inline void CBoundingBox3D<SCALAR_TYPE>::read(std::istream& in)
{
	readVector3(in, _min);
	readVector3(in, _max);
}
