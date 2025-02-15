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

template <class SCALAR_TYPE>
bool tolerantEquals(SCALAR_TYPE v0, SCALAR_TYPE v1, SCALAR_TYPE tol);

template <class SCALAR_TYPE>
bool tolerantEquals(const Vector3< SCALAR_TYPE>& pt0, const Vector3< SCALAR_TYPE>& pt1, SCALAR_TYPE tol);


template <class SCALAR_TYPE>
void checkNAN(SCALAR_TYPE val);

template <class SCALAR_TYPE>
void checkNAN(const Vector3<SCALAR_TYPE>& pt);

struct Vector3Comp {
	template <class V>
	bool operator()(const V& lhs, const V& rhs) const;
};

template <class V>
class Vector3Set : public std::set<V, Vector3Comp> {
public:
	size_t count(const V& v) const;
};

using Vector3dSet = Vector3Set<Vector3d>;

template<typename SCALAR_TYPE>
Vector3<SCALAR_TYPE> safeNormalize(const Vector3<SCALAR_TYPE>& v);
template<class T>
double distanceFromPlane(const Vector3<T>& pt, const Plane<T>& plane);

template<class T>
bool pointInTriangle(const Vector3<T>& pt0, const Vector3<T>& pt1, const Vector3<T>& pt2, const Vector3<T>& pt, T tol = (T)SAME_DIST_TOL);

template<class T>
bool pointInTriangle(const Vector3<T>* pts[3], const Vector3<T>& pt, T tol = (T)SAME_DIST_TOL);

template<class T>
bool intersectRayTri(const Ray<T>& ray, const Vector3<T>& pt0, const Vector3<T>& pt1, const Vector3<T>& pt2, RayHit<T>& hit = (T)SAME_DIST_TOL);

template<class T>
bool intersectRayTri(const Ray<T>& ray, const Vector3<T>* pts[3], RayHit<T>& hit);

template<class T>
bool intersectTriTri(const Vector3<T> triPts0[3], const Vector3<T> triPts1[3], T tol = (T)SAME_DIST_TOL);

template<class T>
bool intersectTriTri(const Vector3<T>* triPts0[3], const Vector3<T>* triPts1[3], T tol = (T)SAME_DIST_TOL);

template<class T>
Vector3<T> orthoganalizeVector(const Vector3<T>& v, const Vector3<T>& unitVector);

template<class T>
Vector3<T> orthoganalizePoint(const Vector3<T>& origin, const Vector3<T>& unitVector, const Vector3<T>& pt);

template<class T>
Vector3<T> triangleUnitNormal(const Vector3<T>* pts[3]);

template<class T>
Vector3<T> triangleUnitNormal(const Vector3<T> pts[3]);

Vector3d ngonCentroid(int numPoints, Vector3d const* const pts[]);
Vector3d ngonCentroid(int numPoints, const Vector3d pts[]);

Vector3d triangleCentroid(Vector3d const* const pts[3]);
Vector3d triangleCentroid(const Vector3d pts[]);

double volumeUnderTriangle(Vector3d const* const pts[3], const Vector3d& axis);

// LERP functions are usually used for points, but can be used for any kind of value that supports +, -  and *
template<class T>
Vector3<T> LERP(const Vector3<T>& p0, const Vector3<T>& p1, T t);

template<class T>
Vector3<T> BI_LERP(const Vector3<T>& p0, const Vector3<T>& p1, const Vector3<T>& p2, const Vector3<T>& p3, T t, T u);

// pts must be size 8 or greater. No bounds checking is done.
template<class T>
Vector3<T> TRI_LERP(const Vector3<T> pts[8], T t, T u, T v);

// pts must be size 8 or greater. No bounds checking is done.
template<class T>
Vector3<T> TRI_LERP(const std::vector<Vector3<T>>& pts, T t, T u, T v);

template<class T>
Vector3<T> TRI_LERP(const std::vector<Vector3<T>>& pts, const Vector3<T>& uvw);

template<class T>
bool TRI_LERP_INV(const Vector3<T>& pt, const std::vector<Vector3<T>>& pts, Vector3<T>& tuv, T tol = (T)1.0e-12);

#include <tm_math.hpp>
