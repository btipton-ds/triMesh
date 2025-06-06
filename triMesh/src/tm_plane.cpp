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
#include <tm_lineSegment.h>

template<class T>
Plane<T>::Plane(const POINT_TYPE* pts[3], bool initXRef)
	: Plane(*pts[0], *pts[1], *pts[2])
{
}

template<class T>
Plane<T>::Plane(const POINT_TYPE& pt0, const POINT_TYPE& pt1, const POINT_TYPE& pt2, bool initXRef)
{
	_origin = pt0;
	POINT_TYPE v0 = pt1 - pt0;
	POINT_TYPE v1 = pt2 - pt0;
	_normal = v1.cross(v0);
	_normal.normalize();
	if (initXRef) {
		_xRef = (pt1 - pt0).normalized();
		orthogonalize(_normal, _xRef);
	}
}

template<class T>
Plane<T>::Plane(const POINT_TYPE& origin, const POINT_TYPE& normal, bool alreadyNormalzed)
	: _origin(origin)
	, _normal(alreadyNormalzed ? normal : normal.normalized())
	, _xRef(0, 0, 0)
{
	orthogonalize(_normal, _xRef);
}

template<class T>
void Plane<T>::orthogonalize(const POINT_TYPE& v0, POINT_TYPE& v1)
{
	T tol = (T)1.0e-6;

	T dp;
	if (v1.squaredNorm() < tol * tol) {
		v1 = POINT_TYPE(1, 0, 0);
		dp = v0.dot(v1);
		if (fabs(dp) > 0.70701f) {
			v1 = POINT_TYPE(0, 1, 0);
			dp = v0.dot(v1);
			if (fabs(dp) > 0.70701f) {
				v1 = POINT_TYPE(0, 0, 1);
				dp = v0.dot(v1);
			}
		}
	} else {
		dp = v0.dot(v1);
	}
	v1 = v1 - dp * v0;
	v1.normalize();
}

template<class T>
void Plane<T>::makePrincipal()
{
	T tol = (T)1.0e-6;

	// Intersect the plane with principal axes to find a principal origin
	T minDist = (T)FLT_MAX;
	POINT_TYPE testOrigin(_origin);
	for (int i = 0; i < 3; i++) {
		POINT_TYPE dir(0, 0, 0);
		dir[i] = 1;
		RayHit<T> hit;
		Ray<T> ray(POINT_TYPE(0, 0, 0), dir);
		if (intersectRay(ray, hit, (T)SAME_DIST_TOL) && fabs(hit.dist) < minDist - SAME_DIST_TOL) {
			minDist = fabs(hit.dist);
			testOrigin = hit.hitPt;
		}
	}

	T testDist = distanceToPoint(testOrigin);
	if (fabs(testDist) < tol) {
		_origin = testOrigin;
		testDist = distanceToPoint(_origin);
		assert(fabs(testDist) < tol);
	}
	else
		assert(!"Principal origin out of tolerance");

}

template<class T>
void Plane<T>::setXRef(const POINT_TYPE& xRef)
{
	_xRef = xRef - xRef.dot(_normal) * _normal;
	_xRef.normalize();
}

template<class T>
bool Plane<T>::intersectLineSegment_rev0(const LineSegment<T>& seg, RayHit<T>& hitPt, T tol) const
{
	T d0 = (seg._pt0 - _origin).dot(_normal); // distanceToPoint(seg._pt0, false);
	T d1 = (seg._pt1 - _origin).dot(_normal); // distanceToPoint(seg._pt1, false);

	// This used to use ray intersect and only tested if pts[0] lies on the plane.
	// We keep that odd behavior for compatibility with other code
	if (fabs(d0) < tol) {
		hitPt.hitPt = seg._pt0;
		assert(distanceToPoint(hitPt.hitPt) < tol);
		hitPt.dist = d0;
		return true;
	}

	bool above0 = d0 >= 0;
	bool above1 = d1 >= 0;
	if (above0 != above1) {
		d0 = fabs(d0);
		d1 = fabs(d1);
		T l = d0 + d1;
		T t = d0 / l;
		hitPt.hitPt = seg.interpolate(t);
		assert(distanceToPoint(hitPt.hitPt) < tol);
		hitPt.dist = d0;
		return true;
	}
	return false;
}

template<class T>
bool Plane<T>::intersectLineSegment(const LineSegment<T>& seg, RayHit<T>& hitPt, T tol) const
{
	T d0 = (seg._pt0 - _origin).dot(_normal); // distanceToPoint(seg._pt0, false);
	T d1 = (seg._pt1 - _origin).dot(_normal); // distanceToPoint(seg._pt1, false);

	bool above0 = d0 >= 0;
	bool above1 = d1 >= 0;
	if (above0 != above1) {
		d0 = fabs(d0);
		d1 = fabs(d1);
		T l = d0 + d1;
		T t = d0 / l;
		hitPt.hitPt = seg.interpolate(t);
		assert(distanceToPoint(hitPt.hitPt) < tol);
		hitPt.dist = d0;
		return true;
	}
	return false;
}

template<class T>
bool Plane<T>::intersectTri(const POINT_TYPE& pt0, const POINT_TYPE& pt1, const POINT_TYPE& pt2, LineSegment<T>& iSeg, T tol) const
{

	int numHits = 0;

	POINT_TYPE iPt0, iPt1;
	RayHit<T> hit;

	{
		LineSegment<T> seg(pt0, pt1);

		if (intersectLineSegment(seg, hit, tol)) {
			iPt0 = hit.hitPt;
			numHits++;
		}
	}

	{
		LineSegment<T> seg(pt1, pt2);

		if (intersectLineSegment(seg, hit, tol)) {
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

	{
		LineSegment<T> seg(pt2, pt0);

		if (intersectLineSegment(seg, hit, tol)) {
			if (numHits == 0)
				iPt0 = hit.hitPt;
			else if (numHits == 1) {
				iPt1 = hit.hitPt;
				iSeg = LineSegment<T>(iPt0, iPt1);
				return true;
			}
		}
	}

	return false;
}

template<class T>
bool Plane<T>::intersectTri(const Vector3<T>* const* pts, LineSegment<T>& iSeg, T tol) const
{
	return intersectTri(*pts[0], *pts[1], *pts[2], iSeg, tol);
}

template<class T>
bool Plane<T>::intersectPlane(const Plane& otherPlane, Ray<T>& iSeg, T tol) const
{
	Vector3<T> dir = _normal.cross(otherPlane._normal);
	auto magSqr = dir.squaredNorm();
	if (magSqr < tol * tol)
		return false;

	dir /= sqrt(magSqr);

	// dir is perpendicular to both normals. vPerp cannot be zero.
	auto vPerp = dir.cross(otherPlane._normal).normalized();
	Ray<T> perpRay(otherPlane._origin, vPerp);
	RayHit<T> hp;
	if (intersectRay(perpRay, hp, tol)) {
		iSeg = Ray<T>(hp.hitPt, dir);
		return true;
	}

	return false;
}

template<class T>
bool Plane<T>::isCoincident(const Plane& other, T distTol, T cpTol) const
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
