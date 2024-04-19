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
#include <tm_plane.h>

#include <set>

using lint = long long;

template<class T>
struct LineSegment;

constexpr double NUMERIC_DIFF_TOL = 1.0e-5;
constexpr double SAME_DIST_TOL = 1.0e-8;
constexpr double OPTIMIZER_TOL = 1.0e-6;
constexpr double SAME_DIST_TOL_SQR = SAME_DIST_TOL * SAME_DIST_TOL;
constexpr double minNormalizeDivisor = 1.0e-12;

#ifndef stm1
#define stm1 0xffffffffffffffff
#endif

template <class SCALAR_TYPE>
inline bool tolerantEquals(SCALAR_TYPE v0, SCALAR_TYPE v1) {
	return fabs(v1 - v0) < SAME_DIST_TOL;
}

template <class SCALAR_TYPE>
inline bool tolerantEquals(const Vector3< SCALAR_TYPE>& pt0, const Vector3< SCALAR_TYPE>& pt1) {
	return (pt1 - pt0).squaredNorm() < SAME_DIST_TOL * SAME_DIST_TOL;
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

struct Vector3Comp {
	template <class V>
	inline bool operator()(const V& lhs, const V& rhs) const {
		for (int i = 0; i < 3; i++) {
			if (lhs[i] < rhs[i])
				return true;
			else if (lhs[i] > rhs[i])
				return false;
		}
		return false;
	}

};

template <class V>
class Vector3Set : public std::set<V, Vector3Comp> {
public:
	size_t count(const V& v) const {
		size_t result = 0;
		V delta(SAME_DIST_TOL, SAME_DIST_TOL, SAME_DIST_TOL);
		const auto lb = this->lower_bound(v - delta);
		const auto ub = this->upper_bound(v + delta);
		for (auto i = lb; i != ub; i++)
			result++;

		return result;
	}
};

using Vector3dSet = Vector3Set<Vector3d>;

template<typename SCALAR_TYPE>
Vector3<SCALAR_TYPE> safeNormalize(const Vector3<SCALAR_TYPE>& v) {
	auto l = v.norm();
	if (l > minNormalizeDivisor)
		return v / l;
	throw "zero length vector";
}

double distanceFromPlane(const Vector3d& pt, const Plane<double>& plane);
bool intersectRayTri(const Ray<double>& ray, Vector3d const * const pts[3], RayHit<double>& hit);
bool intersectRayTri(const Ray<float>& ray, Vector3f const* const pts[3], RayHit<float>& hit);
bool pointInTriangle(const Vector3d& pt0, const Vector3d& pt1, const Vector3d& pt2, const Vector3d& pt);
bool pointInTriangle(const Vector3d* pts[3], const Vector3d& pt);

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

Vector3d triangleNormal(Vector3d const* const pts[3]);
Vector3d triangleNormal(const Vector3d pts[3]);
Vector3f triangleNormal(Vector3f const* const pts[3]);
Vector3f triangleNormal(const Vector3f pts[3]);

Vector3d ngonCentroid(int numPoints, Vector3d const* const pts[]);
Vector3d ngonCentroid(int numPoints, const Vector3d pts[]);

inline Vector3d triangleCentroid(Vector3d const* const pts[3]) {
	return ngonCentroid(3, pts);
}
inline Vector3d triangleCentroid(const Vector3d pts[]) {
	return ngonCentroid(3, pts);
}

double volumeUnderTriangle(Vector3d const* const pts[3], const Vector3d& axis);

// LERP functions are usually used for points, but can be used for any kind of value that supports +, -  and *
template<class T>
inline T LERP(const T& p0, const T& p1, double t)
{
	return p0 + t * (p1 - p0);
}

template<class T>
inline T BI_LERP(const T& p0, const T& p1, const T& p2, const T& p3, double t, double u)
{
	T pt0 = LERP(p0, p1, t);
	T pt1 = LERP(p3, p2, t);

	return pt0 + u * (pt1 - pt0);
}

// pts must be size 8 or greater. No bounds checking is done.
template<class T>
inline T TRI_LERP(const T pts[8], double t, double u, double v)
{
	T pt0 = BI_LERP(pts[0], pts[1], pts[2], pts[3], t, u);
	T pt1 = BI_LERP(pts[4], pts[5], pts[6], pts[7], t, u);

	return pt0 + v * (pt1 - pt0);
}

// pts must be size 8 or greater. No bounds checking is done.
template<class T>
inline T TRI_LERP(const std::vector<T>& pts, double t, double u, double v)
{
	return TRI_LERP(pts.data(), t, u, v);
}

