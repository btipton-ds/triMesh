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

#include <tm_math.h>
#include <tm_plane.h>
#include <tm_ray.h>
#include <tm_lineSegment.hpp>

bool intersectRayPlane(const Ray<double>& ray, const Vector3d& origin, const Vector3d& normal, RayHitd& hit)
{
	auto dp = ray._dir.dot(normal);
	if (fabs(dp) < minNormalizeDivisor)
		return false;

	Vector3d v = origin - ray._origin;
	auto h = v.dot(normal);
	hit.dist = h / dp;
	hit.hitPt = ray._origin + hit.dist * ray._dir;

#if FULL_TESTS // Verification code
	Vector3d vTest = hit.hitPt - origin;
	double testDist = vTest.dot(normal);
	if (fabs(testDist) > SAME_DIST_TOL) {
		assert(!"Point not on plane");
	}
#endif
	return true;
}

Vector3d ngonCentroid(int numPoints, Vector3d const* const pts[]) {
	Vector3d result(0, 0, 0);
	for (int i = 0; i < numPoints; i++)
		result += *pts[i];
	result /= numPoints;
	return result;
}

Vector3d ngonCentroid(int numPoints, const Vector3d pts[]) {
	Vector3d result(0, 0, 0);
	for (int i = 0; i < numPoints; i++)
		result += pts[i];
	result /= numPoints;
	return result;
}

double volumeUnderTriangle(Vector3d const* const pts[3], const Vector3d& axis) {
	Vector3d centroid = triangleCentroid(pts);
	Vector3d v0 = *pts[1] - *pts[0];
	Vector3d v1 = *pts[2] - *pts[0];
	Vector3d cp = v0.cross(v1);
	double area = cp.dot(axis) * 0.5;
	double h = centroid.dot(axis);
	double vol = area * h;
	return vol;
}

template<class T>
Vector3<T> BI_LERP(const Vector3<T>& p0, const Vector3<T>& p1, const Vector3<T>& p2, const Vector3<T>& p3, T t, T u)
{
	auto pt0 = LERP(p0, p1, t);
	auto pt1 = LERP(p3, p2, t);

	return pt0 + u * (pt1 - pt0);
}

// pts must be size 8 or greater. No bounds checking is done.
template<class T>
Vector3<T> TRI_LERP(const Vector3<T> pts[8], T t, T u, T v)
{
	auto pt0 = BI_LERP(pts[0], pts[1], pts[2], pts[3], t, u);
	auto pt1 = BI_LERP(pts[4], pts[5], pts[6], pts[7], t, u);

	return pt0 + v * (pt1 - pt0);
}

template<class T>
bool TRI_LERP_INV(const Vector3<T>& pt, const std::vector<Vector3<T>>& pts, Vector3<T>& uvw)
{
	const double step = 1.0e-10;
	const double tol = 1.0e-10;
	const double tolSqr = tol * tol;
	if (pts.empty())
		return false;

	T a, b, c, l;
	Vector3<T> x, y, z, p0, p1, dir, v;

	x = pts[1] - pts[0];
	l = x.norm();
	x /= l * l;

	y = pts[3] - pts[0];
	l = y.norm();
	y /= l * l;

	z = pts[4] - pts[0];
	l = z.norm();
	z /= l * l;

	v = pt - pts[0];
	uvw = Vector3<T>(v.dot(x), v.dot(y), v.dot(z));
	bool done = false;
	int count = 0;
	while (!done && count++ < 100) {
		p0 = TRI_LERP(pts, (T)0, uvw[1], uvw[2]);
		p1 = TRI_LERP(pts, (T)1, uvw[1], uvw[2]);
		dir = p1 - p0;
		l = dir.norm();
		dir /= l;
		v = pt - p0;
		a = v.dot(dir) / l;

		p0 = TRI_LERP(pts, uvw[0], (T)0, uvw[2]);
		p1 = TRI_LERP(pts, uvw[0], (T)1, uvw[2]);
		dir = p1 - p0;
		l = dir.norm();
		dir /= l;
		v = pt - p0;
		b = v.dot(dir) / l;

		p0 = TRI_LERP(pts, uvw[0], uvw[1], (T)0);
		p1 = TRI_LERP(pts, uvw[0], uvw[1], (T)1);
		dir = p1 - p0;
		l = dir.norm();
		dir /= l;
		v = pt - p0;
		c = v.dot(dir) / l;

		uvw = Vector3<T>(a, b, c);
		Vector3<T> guess = TRI_LERP(pts, uvw);
		T distSqr = (guess - pt).squaredNorm();
		if (distSqr < tolSqr)
			return true;
	}
	return false;
}

template<class T>
bool collisionTriTri(const Vector3<T> triPts0[3], const Vector3<T> triPts1[3], T tol)
{
	RayHit<T> hit;
	Plane<T> triPlane0(triPts0[0], triPts0[1], triPts0[2]);

	// Check 1 against 0
	for (int i = 0; i < 3; i++) {
		int j = (i + 1) % 3;

		LineSegment<T> seg(triPts1[i], triPts1[j]);

		if (seg.intersectTri(triPts0[0], triPts0[1], triPts0[2], hit, tol))
			return true;		
	}

	Plane<T> triPlane1(triPts1[0], triPts1[1], triPts1[2]);
	// Check 0 against 1
	for (int i = 0; i < 3; i++) {
		int j = (i + 1) % 3;

		LineSegment<T> seg(triPts0[i], triPts0[j]);

		if (seg.intersectTri(triPts1[0], triPts1[1], triPts1[2], hit, tol))
			return true;
		
	}

	return false;
}

namespace {
	// After several attempts, textbook instantiation never linked correctly. This does.
	template<class T>
	bool instantiate()
	{
		try {
			Vector3<T> pt;
			Vector3<T>
				tri0[] = { pt, pt, pt },
				tri1[] = { pt, pt, pt };
			collisionTriTri<T>(tri0, tri1);

			std::vector<Vector3<T>> pts;
			Vector3<T> uvw;
			if (pts.empty())
				return false;
			auto a = TRI_LERP(pts, (T)0, (T)0, (T)0);
			a = TRI_LERP(pts.data(), (T)0, (T)0, (T)0);
			a = BI_LERP(pt, pt, pt, pt, (T)0, (T)0);
			return TRI_LERP_INV(pt, pts, uvw);
		} catch (...) {}
		return false;
	}
	static bool dummy0 = instantiate<double>();
	static bool dummy1 = instantiate<float>();

}
