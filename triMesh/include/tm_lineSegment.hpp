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

template<class T>
LineSegment<T>::LineSegment(const POINT_TYPE& p0, const POINT_TYPE& p1)
{
	_pts[0] = p0;
	_pts[1] = p1;
}

template<class T>
LineSegment<T>::SCALAR_TYPE LineSegment<T>::calLength() const {
	return (_pts[1] - _pts[0]).norm();
}

template<class T>
typename LineSegment<T>::POINT_TYPE LineSegment<T>::calcDir() const {
	return safeNormalize<SCALAR_TYPE>(_pts[1] - _pts[0]);
}

template<class T>
typename LineSegment<T>::POINT_TYPE LineSegment<T>::interpolate(SCALAR_TYPE t) const {
	return _pts[0] + t * (_pts[1] - _pts[0]);
}

template<class T>
LineSegment<T>::SCALAR_TYPE LineSegment<T>::parameterize(const POINT_TYPE& pt) const {
	return (pt - _pts[0]).dot(calcDir());
}

template<class T>
bool LineSegment<T>::contains(const POINT_TYPE& pt, LineSegment<T>::SCALAR_TYPE& t, SCALAR_TYPE tol) const
{
	if (tolerantEquals(_pts[0], pt, tol)) {
		t = 0;
		return true;
	}
	else if (tolerantEquals(_pts[1], pt, tol)) {
		t = 1;
		return true;
	}
	else {
		POINT_TYPE vDir = _pts[1] - _pts[0];
		LineSegment<T>::SCALAR_TYPE len = vDir.norm();
		if (len < tol) {
			return false;
		}
		vDir /= len;
		POINT_TYPE vOrth = pt - _pts[0];

		SCALAR_TYPE dp = vOrth.dot(vDir);
		vOrth = vOrth - dp * vDir; // orthogonalize v1
		SCALAR_TYPE dist = vOrth.norm();
		if (dist > tol)
			return false; // pt does not lie on the segment within tolerance.

		t = dp / len;
		return -tol < dp && dp < (len + tol); // return if the pt lies in [zero, len] within tolerance
	}
}

template<class T>
Ray<typename LineSegment<T>::SCALAR_TYPE> LineSegment<T>::getRay() const {
	return Ray<SCALAR_TYPE>(_pts[0], calcDir());
}

template<class T>
typename LineSegment<T>::SCALAR_TYPE LineSegment<T>::distanceToPoint(const POINT_TYPE& pt) const {
	SCALAR_TYPE t;
	return distanceToPoint(pt, t);
}

template<class T>
typename LineSegment<T>::SCALAR_TYPE LineSegment<T>::distanceToPoint(const POINT_TYPE& pt, SCALAR_TYPE& t) const {
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

template<class T>
bool LineSegment<T>::intersectTri(const POINT_TYPE* pts[3], RayHit<T>& hit, T tol) const
{
	POINT_TYPE unitDir = _pts[1] - _pts[0];
	SCALAR_TYPE l = unitDir.norm();
	unitDir /= l;
	Ray<SCALAR_TYPE> ray(_pts[0], unitDir);
	if (intersectRayTri(getRay(), pts, hit)) {
		POINT_TYPE v1 = hit.hitPt - _pts[0];
		SCALAR_TYPE d = unitDir.dot(v1);
		if (-tol < d && d < l + tol) {
			return true;
		}
		else {
			return false;
		}
	}
	return false;
}

template<class T>
bool LineSegment<T>::intersectTri(const POINT_TYPE& pt0, const POINT_TYPE& pt1, const POINT_TYPE& pt2, RayHit<T>& hit, T tol) const
{
	const POINT_TYPE* pts[] = { &pt0, &pt1, &pt2 };
	return intersectTri(pts, hit, tol);
}

template<class T>
bool LineSegment<T>::intersectPlane(const Plane<T>& plane, RayHit<T>& hit, T tol) const
{
	if (plane.intersectRay(getRay(), hit, tol)) {
		POINT_TYPE v = hit.hitPt - _pts[0];
		SCALAR_TYPE t = v.dot(calcDir()) / calLength();
		return 0 <= t && t <= 1;
	}
	return false;
}

template<class T>
bool LineSegment<T>::intersectPlane(const POINT_TYPE* pts[3], RayHit<SCALAR_TYPE>& hit, SCALAR_TYPE tol) const
{
	return intersectPlane(Plane<SCALAR_TYPE>(pts), hit, tol);
}
