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

	NOTE****
	LineSegment_byref DOES NOT preserve point order. It sorts the points so the "smaller" comes first using the Vertex3 operator < method.
	This allows LineSegment_byref to be used in sets and maps.
*/

#include <tm_defines.h>

#include <cmath>
#include <vector>
#include <climits>
#include <cfloat>

#include <tm_vector3.h>

template<class T>
class Plane;

template<class T>
class Plane_byref;

template<class T>
struct Ray;

template<class T>
struct RayHit;

template<class T>
struct LineSegment_byref {
	using SCALAR_TYPE = T;
	using POINT_TYPE = Vector3<SCALAR_TYPE>;

	LineSegment_byref(const POINT_TYPE& p0, const POINT_TYPE& p1);
	SCALAR_TYPE calLength() const;
	POINT_TYPE calcDir() const;
	POINT_TYPE interpolate(SCALAR_TYPE t) const;
	SCALAR_TYPE parameterize(const POINT_TYPE& pt) const;
	bool contains(const POINT_TYPE& pt, SCALAR_TYPE& t, SCALAR_TYPE tol) const;
	Ray<SCALAR_TYPE> getRay() const;

	SCALAR_TYPE distanceToPoint(const POINT_TYPE& pt, SCALAR_TYPE& t) const;
	SCALAR_TYPE distanceToPoint(const POINT_TYPE& pt) const;

	bool intersectTri(const POINT_TYPE* pts[3], RayHit<SCALAR_TYPE>& hit, SCALAR_TYPE tol) const;
	bool intersectTri(const POINT_TYPE& pt0, const POINT_TYPE& pt1, const POINT_TYPE& pt2, RayHit<SCALAR_TYPE>& hit, SCALAR_TYPE tol) const;
	bool intersectPlane(const Plane<SCALAR_TYPE>& plane, RayHit<SCALAR_TYPE>& hit, SCALAR_TYPE tol) const;
	bool intersectPlane(const Plane_byref<SCALAR_TYPE>& plane, RayHit<SCALAR_TYPE>& hit, SCALAR_TYPE tol) const;
	bool intersectPlane(const POINT_TYPE* pts[3], RayHit<SCALAR_TYPE>& hit, SCALAR_TYPE tol) const;

	const POINT_TYPE &_pt0, &_pt1;
};

using LineSegment_byrefd = LineSegment_byref<double>;
using LineSegment_byreff = LineSegment_byref<float>;
