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

template <class SCALAR_TYPE>
inline bool tolerantEquals(SCALAR_TYPE v0, SCALAR_TYPE v1, SCALAR_TYPE tol) {
	return fabs(v1 - v0) < tol;
}

template <class SCALAR_TYPE>
inline bool tolerantEquals(const Vector3< SCALAR_TYPE>& pt0, const Vector3< SCALAR_TYPE>& pt1, SCALAR_TYPE tol) {
	return (pt1 - pt0).squaredNorm() < tol * tol;
}

#define CHECK_NAN 1

template <class SCALAR_TYPE>
inline void checkNAN(SCALAR_TYPE val) {
#if CHECK_NAN
	if (std::isnan(val) || std::isinf(val))
		assert(!"nan");
#endif
}

template <class SCALAR_TYPE>
inline void checkNAN(const Vector3<SCALAR_TYPE>& pt) {
#if CHECK_NAN
	for (int i = 0; i < 3; i++)
		checkNAN(pt[i]);
#endif
}

template <class V>
inline bool Vector3Comp::operator()(const V& lhs, const V& rhs) const {
	for (int i = 0; i < 3; i++) {
		if (lhs[i] < rhs[i])
			return true;
		else if (lhs[i] > rhs[i])
			return false;
	}
	return false;
}

template <class V>
size_t Vector3Set<V>::count(const V& v) const {
	size_t result = 0;
	V delta(SAME_DIST_TOL, SAME_DIST_TOL, SAME_DIST_TOL);
	const auto lb = this->lower_bound(v - delta);
	const auto ub = this->upper_bound(v + delta);
	for (auto i = lb; i != ub; i++)
		result++;

	return result;
}

template<typename SCALAR_TYPE>
Vector3<SCALAR_TYPE> safeNormalize(const Vector3<SCALAR_TYPE>& v) {
	auto l = v.norm();
	if (l > minNormalizeDivisor)
		return v / l;
	std::string msg = std::string(__FILE__) + ":" + std::to_string(__LINE__) + std::string(" zero length vector");
	assert(!" zero length vector");
	throw std::runtime_error(msg);
}

template<class T>
double distanceFromPlane(const Vector3<T>& pt, const Plane<T>& plane) {
	return (pt - plane.getOrgin()).dot(plane.getNormal());
}

template<class T>
bool pointInTriangle(const Vector3<T>& pt0, const Vector3<T>& pt1, const Vector3<T>& pt2, const Vector3<T>& pt, T tol)
{
	const Vector3<T>* pts[] = { &pt0, &pt1, &pt2 };
	return pointInTriangle(pts, pt, tol);
}

template<class T>
bool pointInTriangle(const Vector3<T>* pts[3], const Vector3<T>& pt, T tol)
{
	Vector3<T> norm = triangleUnitNormal(pts);
	return pointInTriangle(pts, pt, norm, tol);
}

template<class T>
bool pointInTriangle(const Vector3<T>& pt0, const Vector3<T>& pt1, const Vector3<T>& pt2, const Vector3<T>& pt, const Vector3<T>& unitNorm, T tol)
{
	const Vector3<T>* pts[] = { &pt0, &pt1, &pt2 };
	return pointInTriangle(pts, pt, unitNorm, tol);
}

template<class T>
bool pointInTriangle(const Vector3<T>* pts[3], const Vector3<T>& pt, const Vector3<T>& unitNorm, T tol)
{
#ifdef _DEBUG
	{
		Vector3<T> v0 = (*pts[1]) - (*pts[0]);
		Vector3<T> v1 = (*pts[2]) - (*pts[0]);

		v0 = pt - (*pts[0]);
		T dp = v0.dot(unitNorm);
		assert(fabs(dp) < 1.0e-6);
	}
#endif

	for (size_t i = 0; i < 3; i++) {
		size_t j = (i + 1) % 3;
		Vector3<T> v0 = pt - (*pts[i]);
		Vector3<T> v1 = (*pts[j]) - (*pts[i]);
		v1.normalize();
		v0 = v0 - v1.dot(v0) * v1;
		Vector3<T> v2 = v1.cross(v0);
		T cp = v2.dot(unitNorm);
		if (cp < -tol)
			return false;
	}

	return true;
}

template<class T>
bool intersectRayTri(const Ray<T>& ray, const Vector3<T>& pt0, const Vector3<T>& pt1, const Vector3<T>& pt2, RayHit<T>& hit, T tol) {
	const Vector3<T>* pts[] = { &pt0, &pt1, &pt2 };
	return intersectRayTri(ray, pts, hit, tol);
}

template<class T>
bool intersectRayTri(const Ray<T>& ray, const Vector3<T>* pts[3], RayHit<T>& hit, T tol) {

	Vector3<T> v0 = *pts[1] - *pts[0];
	Vector3<T> v1 = *pts[2] - *pts[0];
	
	Vector3<T> norm = v0.cross(v1);
	T l = norm.norm();
	if (l < minNormalizeDivisor)
		return false;
	norm /= l;

	Plane<T> pl(*(pts[0]), norm);
	if (!pl.intersectRay(ray, hit, tol))
		return false;

	return pointInTriangle(pts, hit.hitPt, norm, tol);
}

template<class T>
Vector3<T> orthoganalizeVector(const Vector3<T>& v, const Vector3<T>& unitVector)
{
	return v - unitVector * unitVector.dot(v);
}

template<class T>
Vector3<T> orthoganalizePoint(const Vector3<T>& origin, const Vector3<T>& unitVector, const Vector3<T>& pt)
{
	Vector3<T> v = pt - origin;
	v = orthoganalize(v, unitVector);
	return origin + v;
}

template<class T>
Vector3<T> triangleUnitNormal(const Vector3<T>* pts[3]) {
	Vector3<T> v0 = *pts[1] - *pts[0];
	Vector3<T> v1 = *pts[2] - *pts[0];
	Vector3<T> n = safeNormalize(v0.cross(v1));
	return n;
}

template<class T>
Vector3<T> triangleUnitNormal(const Vector3<T> pts[3]) {
	Vector3<T> v0 = pts[1] - pts[0];
	Vector3<T> v1 = pts[2] - pts[0];
	Vector3<T> n = safeNormalize(v0.cross(v1));
	return n;
}

// LERP functions are usually used for points, but can be used for any kind of value that supports +, -  and *
template<class T>
inline Vector3<T> LERP(const Vector3<T>& p0, const Vector3<T>& p1, T t)
{
	return p0 + t * (p1 - p0);
}

// pts must be size 8 or greater. No bounds checking is done.
template<class T>
inline Vector3<T> TRI_LERP(const std::vector<Vector3<T>>& pts, T t, T u, T v)
{
	return TRI_LERP(pts.data(), t, u, v);
}

template<class T>
inline Vector3<T> TRI_LERP(const std::vector<Vector3<T>>& pts, const Vector3<T>& uvw)
{
	return TRI_LERP(pts, uvw[0], uvw[1], uvw[2]);
}

