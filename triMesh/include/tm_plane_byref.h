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

#include <cmath>
#include <vector>
#include <climits>
#include <cfloat>

#include <tm_vector3.h>

template<class T>
struct Ray;
template<class T>
struct RayHit;
template<class T>
struct LineSegment;
template<class T>
struct LineSegment_byref;

template<class T>
class Plane_byref {
public:
	using POINT_TYPE = Vector3<T>;

	Plane_byref(const POINT_TYPE& origin, const POINT_TYPE& normal);

	bool intersectLine(const POINT_TYPE& pt0, const POINT_TYPE& pt1, RayHit<T>& hitPt, T tol) const;
	bool intersectLineSegment(const LineSegment<T>& seg, RayHit<T>& hitPt, T tol) const;
	bool intersectLineSegment(const LineSegment_byref<T>& seg, RayHit<T>& hitPt, T tol) const;

	bool intersectRay(const Ray<T>& ray, RayHit<T>& hit, T tol) const;
	bool intersectTri(const POINT_TYPE& pt0, const POINT_TYPE& pt1, const POINT_TYPE& pt2, LineSegment<T>& iSeg, T tol) const;
	bool intersectTri(const Vector3<T>* const* pts, LineSegment<T>& iSeg, T tol) const;
	bool isCoincident(const POINT_TYPE& other, T tol) const;
	bool isCoincident(const Plane_byref& other, T distTol, T cpTol) const;

	POINT_TYPE projectPoint(const POINT_TYPE& pt) const;
	T distanceToPoint(const POINT_TYPE& pt, bool absolute = true) const;

	const POINT_TYPE& getOrgin() const;
	const POINT_TYPE& getNormal() const;

private:
	const POINT_TYPE &_origin, &_normal;
};


template<class T>
inline T Plane_byref<T>::distanceToPoint(const POINT_TYPE& pt, bool absolute) const {
	T d = (pt - _origin).dot(_normal);
	if (absolute)
		return fabs(d);
	
	return d;
}

template<class T>
inline bool Plane_byref<T>::isCoincident(const POINT_TYPE& pt, T tol) const
{
	return distanceToPoint(pt, true) < tol;
}

template<class T>
inline const typename Plane_byref<T>::POINT_TYPE& Plane_byref<T>::getOrgin() const
{
	return _origin;
}

template<class T>
inline const typename Plane_byref<T>::POINT_TYPE& Plane_byref<T>::getNormal() const
{
	return _normal;
}

template<class T>
inline bool Plane_byref<T>::intersectRay(const Ray<T>& ray, RayHit<T>& hit, T tol) const
{
	const double MIN_COS_ANGLE = 1.0e-8;
	auto dp = ray._dir.dot(_normal);
	if (fabs(dp) < MIN_COS_ANGLE)
		return false;

	POINT_TYPE v = ray._origin - _origin;
#if 0
	v[0] = ray._origin[0] - _origin[0];
	v[1] = ray._origin[1] - _origin[1];
	v[2] = ray._origin[2] - _origin[2];
#endif

	auto h = v.dot(_normal);
	hit.dist = -h / dp;
	hit.hitPt = ray._origin + hit.dist * ray._dir;
#if 0
	hit.hitPt[0] = ray._origin[0] + hit.dist * ray._dir[0];
	hit.hitPt[1] = ray._origin[1] + hit.dist * ray._dir[1];
	hit.hitPt[2] = ray._origin[2] + hit.dist * ray._dir[2];
#endif

#if FULL_TESTS // Verification code
	POINT_TYPE vTest = hit.hitPt - _origin;
	T testDist = vTest.dot(_normal);
	if (fabs(testDist) > tol) {
		assert(!"Point not on plane");
	}

	if (ray.distToPt(hit.hitPt) > tol) {
		assert(!"Point not on ray");
	}
	
#endif
	return true;
}

template<class T>
inline bool Plane_byref<T>::intersectLine(const POINT_TYPE& pt0, const POINT_TYPE& pt1, RayHit<T>& hitPt, T tol) const
{
	POINT_TYPE v = pt1 - pt0;
	T lSqr = v.squaredNorm();
	if (lSqr < SAME_DIST_TOL_SQR)
		return false;

	Ray<T> ray(pt0, v / sqrt(lSqr));
	return intersectRay(ray, hitPt, tol);
}

using Plane_byrefd = Plane_byref<double>;
using Plane_byreff = Plane_byref<float>;
