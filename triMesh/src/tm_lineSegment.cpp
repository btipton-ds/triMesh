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
T LineSegment<T>::calLength() const {
	return (_pts[1] - _pts[0]).norm();
}

template<class T>
typename LineSegment<T>::POINT_TYPE LineSegment<T>::calcDir() const {
	return safeNormalize<T>(_pts[1] - _pts[0]);
}

template<class T>
typename LineSegment<T>::POINT_TYPE LineSegment<T>::interpolate(T t) const {
	return _pts[0] + t * (_pts[1] - _pts[0]);
}

template<class T>
T LineSegment<T>::parameterize(const POINT_TYPE& pt) const {
	return (pt - _pts[0]).dot(calcDir());
}

template<class T>
Ray<T> LineSegment<T>::getRay() const {
	return Ray<T>(_pts[0], calcDir());
}

template<class T>
T LineSegment<T>::distanceToPoint(const POINT_TYPE& pt) const {
	T t;
	return distanceToPoint(pt, t);
}

template<class T>
T LineSegment<T>::distanceToPoint(const POINT_TYPE& pt, T& t) const {
	POINT_TYPE dir(_pts[1] - _pts[0]);
	T len = dir.norm();
	if (len < minNormalizeDivisor)
		return (T)FLT_MAX;
	dir /= len;
	POINT_TYPE v0 = pt - _pts[0];
	T dp = v0.dot(dir);
	t = dp / len;
	v0 = v0 - dir * dp;
	T dist;
	if (t < 0) {
		t = (T) -DBL_MAX;
		dist = (pt - _pts[0]).norm();
	}
	else if (t > 1) {
		t = (T) DBL_MAX;
		dist = (pt - _pts[1]).norm();
	}
	else
		dist = v0.norm();

	return dist;
}

template<class T>
bool LineSegment<T>::intersectTri(const POINT_TYPE* pts[3], RayHit<T>& hit) const
{
	POINT_TYPE unitDir = _pts[1] - _pts[0];
	T l = unitDir.norm();
	unitDir /= l;
	Ray<T> ray(_pts[0], unitDir);
	if (intersectRayTri(getRay(), pts, hit)) {
		POINT_TYPE v1 = hit.hitPt - _pts[0];
		T d = unitDir.dot(v1);
		if (-SAME_DIST_TOL < d && d < l + SAME_DIST_TOL) {
			return true;
		}
		else {
			return false;
		}
	}
	return false;
}

template<class T>
bool LineSegment<T>::intersectPlane(const Plane<T>& plane, RayHit<T>& hit) const
{
	if (plane.intersectRay(getRay(), hit)) {
		POINT_TYPE v = hit.hitPt - _pts[0];
		T t = v.dot(calcDir()) / calLength();
		return 0 <= t && t <= 1;
	}
	return false;
}

template<class T>
bool LineSegment<T>::intersectPlane(const POINT_TYPE* pts[3], RayHit<T>& hit) const
{
	return intersectPlane(Plane<T>(pts), hit);
}

template struct LineSegment<double>;
template struct LineSegment<float>;
