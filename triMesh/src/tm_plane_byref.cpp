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
#include <tm_plane_byref.h>
#include <tm_lineSegment.h>
#include <tm_lineSegment_byref.h>

template<class T>
Plane_byref<T>::Plane_byref(const POINT_TYPE& origin, const POINT_TYPE& normal)
	: _origin(origin)
	, _normal(normal)
{
}

template<class T>
bool Plane_byref<T>::intersectLineSegment(const LineSegment<T>& seg, bool includeEndPoints, RayHit<T>& hitPt, T tol) const
{
	return seg.intersectPlane(*this, includeEndPoints, hitPt, tol);
}

template<class T>
bool Plane_byref<T>::intersectLineSegment(const LineSegment_byref<T>& seg, bool includeEndPoints, RayHit<T>& hitPt, T tol) const
{
	return seg.intersectPlane(*this, includeEndPoints, hitPt, tol);
}

template<class T>
bool Plane_byref<T>::intersectTri(const POINT_TYPE& pt0, const POINT_TYPE& pt1, const POINT_TYPE& pt2, LineSegment<T>& iSeg, T tol) const
{

	int numHits = 0;

	POINT_TYPE iPt0, iPt1;
	RayHit<T> hit;
	for (int i = 0; i < 3; i++) {
		int j = (i + 1) % 3;
		const auto& ptA = ptOf3(i, pt0, pt1, pt2);
		const auto& ptB = ptOf3(j, pt0, pt1, pt2);
		LineSegment<T> seg(ptA, ptB);

		if (intersectLineSegment(seg, true, hit, tol)) {
			if (numHits == 0)
				iPt0 = hit.hitPt;
			else if (numHits == 1) {
				iPt1 = hit.hitPt;
				iSeg = LineSegment<T>(iPt0, iPt1);
				return true;
			}
			numHits++;
		}
	}

	return false;
}

template<class T>
bool Plane_byref<T>::intersectTri(const Vector3<T>* const* pts, LineSegment<T>& iSeg, T tol) const
{
	return intersectTri(*pts[0], *pts[1], *pts[2], iSeg, tol);
}

template<class T>
bool Plane_byref<T>::isCoincident(const Plane_byref& other, T distTol, T cpTol) const
{
	T dist = distanceToPoint(other._origin);
	assert(dist >= 0);
	if (dist > distTol)
		return false;

	T cpSqr = _normal.cross(other._normal).squaredNorm();
	if (cpSqr > (cpTol * cpTol))
		return false;

	return true;
}

template<class T>
typename Plane_byref<T>::POINT_TYPE Plane_byref<T>::projectPoint(const POINT_TYPE& pt) const
{
	POINT_TYPE v = pt - _origin;
	v = v - _normal.dot(v) * _normal;
	POINT_TYPE result = _origin + v;
#if FULL_TESTS
	assert(distanceToPoint(result) < 1.0e-8);
#endif
	return result;
}

template class Plane_byref<double>;
template class Plane_byref<float>;
