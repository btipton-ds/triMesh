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

#include  <cmath>
#include <climits>
#include <cfloat>

#if !USE_EIGEN_VECTOR3
#include <tm_vector3.h>
#else
#include <Eigen/Dense>
#endif

#include <set>

using lint = long long;

constexpr double NUMERIC_DIFF_TOL = 1.0e-5;
constexpr double SAME_DIST_TOL = 1.0e-8;
constexpr double OPTIMIZER_TOL = 1.0e-6;
constexpr double SAME_DIST_TOL_SQR = SAME_DIST_TOL * SAME_DIST_TOL;
constexpr double minNormalizeDivisor = 1.0e-12;

#ifndef stm1
#define stm1 0xffffffffffffffff
#endif

#if USE_EIGEN_VECTOR3
using Eigen::Vector3d;
using Eigen::Vector3f;

// using size_t allows indexing into stl containers
using Vector3i = Eigen::Matrix<size_t, 1, 3>;

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
		throw "nan";
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

struct Ray {
	inline Ray (const Vector3d& origin, const Vector3d& dir)
		: _origin(origin)
		, _dir(dir)
	{}

	Vector3d _origin, _dir;
};

struct RayHit {
	size_t triIdx;
	double dist;
	Vector3d hitPt;

	inline RayHit() 
		: triIdx(stm1)
		, dist(0)
	{}

	inline bool operator < (const RayHit& rhs) const {
		return fabs(dist) < fabs(rhs.dist);
	}
};

struct Plane {
	inline Plane(const Vector3d& origin = Vector3d(), const Vector3d& normal = Vector3d())
		: _origin(origin)
		, _normal(normal)
	{}

	Plane(const Vector3d* pts[3]);
	inline double distanceToPoint(const Vector3d& pt) const {
		return fabs((pt - _origin).dot(_normal));
	}
	Vector3d _origin, _normal;
};

template<typename SCALAR_TYPE>
Vector3<SCALAR_TYPE> safeNormalize(const Vector3<SCALAR_TYPE>& v) {
	auto l = v.norm();
	if (l > minNormalizeDivisor)
		return v / l;
	throw "zero length vector";
}
struct LineSegment {
	inline LineSegment(const Vector3d& p0 = Vector3d(0, 0, 0), const Vector3d& p1 = Vector3d(0, 0, 0))
	{
		_pts[0] = p0;
		_pts[1] = p1;
	}
	inline double calLength() const {
		return (_pts[1] - _pts[0]).norm();
	}

	inline Vector3d calcDir() const {
		return safeNormalize(_pts[1] - _pts[0]);
	}

	inline Vector3d interpolate(double t) const {
		return _pts[0] + t * (_pts[1] - _pts[0]);
	}
	inline double parameterize(const Vector3d& pt) const {
		return (pt - _pts[0]).dot(calcDir());
	}

	inline Ray getRay() const {
		return Ray(_pts[0], calcDir());
	}

	double distanceToPoint(const Vector3d& pt, double& t) const;
	double distanceToPoint(const Vector3d& pt) const;

	Vector3d _pts[2];
};

double distanceFromPlane(const Vector3d& pt, const Plane& plane);
bool intersectRayPlane(const Ray& ray, const Plane& plane, RayHit& hit);
bool intersectRayTri(const Ray& ray, Vector3d const * const pts[3], RayHit& hit);
bool intersectLineSegTri(const LineSegment& seg, Vector3d const* const pts[3], RayHit& hit);
bool intersectLineSegPlane(const LineSegment& seg, const Plane& plane, RayHit& hit);
bool intersectLineSegPlane(const LineSegment& seg, const Vector3d* pts[3], RayHit& hit);

Vector3d triangleNormal(Vector3d const* const pts[3]);
Vector3d triangleNormal(const Vector3d pts[3]);

Vector3d ngonCentroid(int numPoints, Vector3d const* const pts[]);
Vector3d ngonCentroid(int numPoints, const Vector3d pts[]);

inline Vector3d triangleCentroid(Vector3d const* const pts[3]) {
	return ngonCentroid(3, pts);
}
inline Vector3d triangleCentroid(const Vector3d pts[]) {
	return ngonCentroid(3, pts);
}

double volumeUnderTriangle(Vector3d const* const pts[3], const Vector3d& axis);

inline Plane::Plane(const Vector3d* pts[3]) 
	: _origin(*pts[0])
	, _normal(triangleNormal(pts))
{}