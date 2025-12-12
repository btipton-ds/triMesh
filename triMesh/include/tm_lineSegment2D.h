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
	LineSegment DOES NOT preserve point order. It sorts the points so the "smaller" comes first using the Vertex3 operator < method.
	This allows LineSegment to be used in sets and maps.
*/

#include <tm_defines.h>

#include <cmath>
#include <vector>
#include <climits>
#include <cfloat>

#include <tm_vector2.h>

template<class T>
class Plane;

template<class T>
struct Ray;

template<class T>
struct RayHit;

template<class T>
struct LineSegment2D {
	using SCALAR_TYPE = T;
	using POINT_TYPE = Vector2<SCALAR_TYPE>;

	LineSegment2D() = default;
	LineSegment2D(const POINT_TYPE& p0, const POINT_TYPE& p1);
	SCALAR_TYPE calLength() const;
	POINT_TYPE calcDir() const;
	POINT_TYPE interpolate(SCALAR_TYPE t) const;
	SCALAR_TYPE parameterize(const POINT_TYPE& pt) const;
	bool contains(const POINT_TYPE& pt, SCALAR_TYPE& t, SCALAR_TYPE tol) const;
	Ray<SCALAR_TYPE> getRay() const;

	SCALAR_TYPE distanceToPoint(const POINT_TYPE& pt, POINT_TYPE& closestPt, SCALAR_TYPE& t) const;
	SCALAR_TYPE distanceToPoint(const POINT_TYPE& pt, SCALAR_TYPE& t) const;
	SCALAR_TYPE distanceToPoint(const POINT_TYPE& pt) const;

	POINT_TYPE _pt0, _pt1;
};

using LineSegment2Dd = LineSegment2D<double>;
using LineSegment2Df = LineSegment2D<float>;
