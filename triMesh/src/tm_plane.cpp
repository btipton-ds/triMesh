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

#include <tm_ray.h>
#include <tm_plane.h>
#include <tm_math.h>

template<class T>
Plane<T>::Plane(const POINT_TYPE* pts[3])
	: Plane(*pts[0], triangleNormal(pts))
{}


template<class T>
Plane<T>::Plane(const POINT_TYPE& origin, const POINT_TYPE& normal)
	: _origin(origin)
	, _normal(normal)
{
	T tol = (T)1.0e-6;

	// Intersect the plan with principal axes to find a principal origin
	T minDist = (T)FLT_MAX;
	POINT_TYPE testOrigin;
	for (int i = 0; i < 3; i++) {
		POINT_TYPE dir(0, 0, 0);
		dir[i] = 1;
		POINT_TYPE pt;
		T dist;
		if (intersectLine(POINT_TYPE(0, 0, 0), dir, pt, dist) && fabs(dist) < minDist) {
			minDist = fabs(dist);
			testOrigin = pt;
		}
	}
#if FULL_TESTS
	T testDist = distanceToPoint(testOrigin);
	if (fabs(testDist) < tol) {
		_origin = testOrigin;
		testDist = distanceToPoint(origin);
		assert(fabs(testDist) < tol);
	}
	else
		assert(!"Principal origin out of tolerance");
#endif
}

template<class T>
bool Plane<T>::intersectLine(const POINT_TYPE& pt0, const POINT_TYPE& pt1, POINT_TYPE& pt, T& dist) const
{
	Ray<T> ray(pt0, (pt1 - pt0).normalized());
	RayHit<T> hitPt;
	if (intersectRay(ray, hitPt)) {
		pt = hitPt.hitPt;
		dist = hitPt.dist;
		return true;
	}

	return false;
}

template<class T>
bool Plane<T>::intersectRay(const Ray<T>& ray, RayHit<T>& hit) const
{
	auto dp = ray._dir.dot(_normal);
	if (fabs(dp) < minNormalizeDivisor)
		return false;

	POINT_TYPE v = _origin - ray._origin;
	auto h = v.dot(_normal);
	hit.dist = h / dp;
	hit.hitPt = ray._origin + hit.dist * ray._dir;

#if FULL_TESTS // Verification code
	POINT_TYPE vTest = hit.hitPt - origin;
	T testDist = vTest.dot(normal);
	if (fabs(testDist) > SAME_DIST_TOL) {
		assert(!"Point not on plane");
	}
#endif
	return true;
}

template<class T>
typename Plane<T>::POINT_TYPE Plane<T>::projectPoint(const POINT_TYPE& pt) const
{
	POINT_TYPE v = pt - _origin;
	v = v - _normal.dot(v) * _normal;
	POINT_TYPE result = _origin + v;
#if FULL_TESTS
	assert(distanceToPoint(result) < 1.0e-8);
#endif
	return result;
}

template class Plane<double>;
template class Plane<float>;
