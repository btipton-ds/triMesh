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

#include <tm_math.h>
#include <tm_ray.h>
#include <tm_lineSegment.h>

template<class VEC_TYPE>
LineSegment<VEC_TYPE>::LineSegment(const POINT_TYPE& p0, const POINT_TYPE& p1)
{
	_pts[0] = p0;
	_pts[1] = p1;
	if (_pts[1] < _pts[0])
		std::swap(_pts[0], _pts[1]);
}

template<class VEC_TYPE>
LineSegment<VEC_TYPE>::SCALAR_TYPE LineSegment<VEC_TYPE>::calLength() const {
	return (_pts[1] - _pts[0]).norm();
}

template<class VEC_TYPE>
typename LineSegment<VEC_TYPE>::POINT_TYPE LineSegment<VEC_TYPE>::calcDir() const {
	return safeNormalize<SCALAR_TYPE>(_pts[1] - _pts[0]);
}

template<class VEC_TYPE>
typename LineSegment<VEC_TYPE>::POINT_TYPE LineSegment<VEC_TYPE>::interpolate(SCALAR_TYPE t) const {
	return _pts[0] + t * (_pts[1] - _pts[0]);
}

template<class VEC_TYPE>
LineSegment<VEC_TYPE>::SCALAR_TYPE LineSegment<VEC_TYPE>::parameterize(const POINT_TYPE& pt) const {
	return (pt - _pts[0]).dot(calcDir());
}

template<class VEC_TYPE>
bool LineSegment<VEC_TYPE>::contains(const POINT_TYPE& pt, LineSegment<VEC_TYPE>::SCALAR_TYPE& t) const
{
	if (tolerantEquals(_pts[0], pt)) {
		t = 0;
		return true;
	}
	else if (tolerantEquals(_pts[1], pt)) {
		t = 1;
		return true;
	}
	else {
		POINT_TYPE vDir = _pts[1] - _pts[0];
		LineSegment<VEC_TYPE>::SCALAR_TYPE len = vDir.norm();
		if (len < SAME_DIST_TOL) {
			return false;
		}
		vDir /= len;
		POINT_TYPE vOrth = pt - _pts[0];

		SCALAR_TYPE dp = vOrth.dot(vDir);
		vOrth = vOrth - dp * vDir; // orthogonalize v1
		SCALAR_TYPE dist = vOrth.norm();
		if (dist > SAME_DIST_TOL)
			return false; // pt does not lie on the segment within tolerance.

		t = dp / len;
		return -SAME_DIST_TOL < dp && dp < (len + SAME_DIST_TOL); // return if the pt lies in [zero, len] within tolerance
	}
}

template<class VEC_TYPE>
Ray<typename LineSegment<VEC_TYPE>::SCALAR_TYPE> LineSegment<VEC_TYPE>::getRay() const {
	return Ray<SCALAR_TYPE>(_pts[0], calcDir());
}

template<class VEC_TYPE>
typename LineSegment<VEC_TYPE>::SCALAR_TYPE LineSegment<VEC_TYPE>::distanceToPoint(const POINT_TYPE& pt) const {
	SCALAR_TYPE t;
	return distanceToPoint(pt, t);
}

template<class VEC_TYPE>
typename LineSegment<VEC_TYPE>::SCALAR_TYPE LineSegment<VEC_TYPE>::distanceToPoint(const POINT_TYPE& pt, SCALAR_TYPE& t) const {
	POINT_TYPE dir(_pts[1] - _pts[0]);
	SCALAR_TYPE len = dir.norm();
	if (len < minNormalizeDivisor)
		return (SCALAR_TYPE)FLT_MAX;
	dir /= len;
	POINT_TYPE v0 = pt - _pts[0];
	SCALAR_TYPE dp = v0.dot(dir);
	t = dp / len;
	v0 = v0 - dir * dp;
	SCALAR_TYPE dist;
	if (t < 0) {
		t = (SCALAR_TYPE)-DBL_MAX;
		dist = (pt - _pts[0]).norm();
	}
	else if (t > 1) {
		t = (SCALAR_TYPE)DBL_MAX;
		dist = (pt - _pts[1]).norm();
	}
	else
		dist = v0.norm();

	return dist;
}

template<class VEC_TYPE>
bool LineSegment<VEC_TYPE>::intersectTri(const POINT_TYPE* pts[3], RayHit<typename SCALAR_TYPE>& hit) const
{
	POINT_TYPE unitDir = _pts[1] - _pts[0];
	SCALAR_TYPE l = unitDir.norm();
	unitDir /= l;
	Ray<SCALAR_TYPE> ray(_pts[0], unitDir);
	if (intersectRayTri(getRay(), pts, hit)) {
		POINT_TYPE v1 = hit.hitPt - _pts[0];
		SCALAR_TYPE d = unitDir.dot(v1);
		if (-SAME_DIST_TOL < d && d < l + SAME_DIST_TOL) {
			return true;
		}
		else {
			return false;
		}
	}
	return false;
}

template<class VEC_TYPE>
bool LineSegment<VEC_TYPE>::intersectTri(const POINT_TYPE& pt0, const POINT_TYPE& pt1, const POINT_TYPE& pt2, RayHit<typename SCALAR_TYPE>& hit) const
{
	const POINT_TYPE* pts[] = { &pt0, &pt1, &pt2 };
	return intersectTri(pts, hit);
}

template<class VEC_TYPE>
bool LineSegment<VEC_TYPE>::intersectPlane(const Plane<typename SCALAR_TYPE>& plane, RayHit<typename SCALAR_TYPE>& hit) const
{
	if (plane.intersectRay(getRay(), hit)) {
		POINT_TYPE v = hit.hitPt - _pts[0];
		SCALAR_TYPE t = v.dot(calcDir()) / calLength();
		return 0 <= t && t <= 1;
	}
	return false;
}

template<class VEC_TYPE>
bool LineSegment<VEC_TYPE>::intersectPlane(const POINT_TYPE* pts[3], RayHit<SCALAR_TYPE>& hit) const
{
	return intersectPlane(Plane<SCALAR_TYPE>(pts), hit);
}

template<class VEC_TYPE>
bool LineSegment<VEC_TYPE>::operator < (const LineSegment& rhs) const
{
	for (int i = 0; i < 2; i++) {
		if (_pts[i] < rhs._pts[i])
			return true;
		else if (rhs._pts[i] < _pts[i])
			return false;
	}
	return false;
}
