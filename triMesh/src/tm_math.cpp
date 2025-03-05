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

template<class T>
Vector3<T> ngonCentroid(int numPoints, Vector3<T> const* const pts[]) {
	Vector3<T> result(0, 0, 0);
	for (int i = 0; i < numPoints; i++)
		result += *pts[i];
	result /= numPoints;
	return result;
}

template<class T>
Vector3<T> ngonCentroid(int numPoints, const Vector3<T> pts[]) {
	Vector3<T> const* pPts[] = {
		&pts[0],
		&pts[1],
		&pts[2],
	};
	return ngonCentroid(numPoints, pPts);
}

template<class T>
inline Vector3<T> triangleCentroid(Vector3<T> const* const pts[3]) {
	return ngonCentroid(3, pts);
}
template<class T>
inline Vector3<T> triangleCentroid(const Vector3<T> pts[]) {
	return ngonCentroid(3, pts);
}

template<class T>
T volumeUnderTriangle(Vector3<T> const* const pts[3], const Vector3<T>& axis) {
	Vector3<T> centroid = triangleCentroid(pts);
	Vector3<T> v0 = *pts[1] - *pts[0];
	Vector3<T> v1 = *pts[2] - *pts[0];
	Vector3<T> cp = v0.cross(v1);
	T area = cp.dot(axis) * (T)0.5;
	T h = centroid.dot(axis);
	T vol = area * h;
	return vol;
}

template<class T>
int triangleSplitWithPlane(Vector3<T> const triPts[3], const Plane<T>& plane, 
	Vector3<T> triPts0[3], Vector3<T> triPts1[3], Vector3<T> triPts2[3], T tol)
{
	int numSplits = 0;
	Vector3<T> splitPts[2];
	int splitLegIndices[2];
	int numIntersectVertIndices = 0;
	int intersectVertIndices[3];

	for (int i = 0; i < 3; i++) {
		int j = (i + 1) % 3;
		RayHit<T> hit;
		LineSegment<T> seg(triPts[i], triPts[j]);
		if (plane.intersectLineSegment(seg, hit, tol)) {
			if (hit.dist < tol) {
				// This vertex lies on the plane
				intersectVertIndices[numIntersectVertIndices++] = i;
			} else {
				for (int k = 0; k < numIntersectVertIndices; k++) {
					if (intersectVertIndices[k] == i || intersectVertIndices[k] == j)
						continue;
				}
				bool found = false;
				for (int k = 0; k < 3; k++) {
					if (tolerantEquals(hit.hitPt, triPts[k], tol)) {
						found = true;
						break;
					}
				}

				if (!found) {
					splitLegIndices[numSplits] = i;
					splitPts[numSplits] = hit.hitPt;
					numSplits++;
				}
			}
		}
	}

	// The found check will screen out a leg which lies on the plane.

	if (numSplits == 1 && numIntersectVertIndices == 1) {
		// The plane splits the triangle and a vertex lies on the plane

		int vertIdx = intersectVertIndices[0];
		triPts0[0] = triPts[vertIdx];
		triPts0[1] = triPts[(vertIdx + 1) % 3];
		triPts0[2] = splitPts[0];

		triPts1[0] = triPts[vertIdx];
		triPts1[1] = splitPts[0];
		triPts1[2] = triPts[(vertIdx + 2) % 3];
		return 2;
	} else if (numSplits == 2) {
		// The plane splits two legs
		int triIdx0;
		if (0 == splitLegIndices[0] && 2 == splitLegIndices[1]) {
			triIdx0 = 0;
		} else if (0 == splitLegIndices[0] && 1 == splitLegIndices[1]) {
			triIdx0 = 1;
		} else {
			triIdx0 = 2;
		}

		int triIdx1, triIdx2, ptIdx0, ptIdx1;

		triIdx1 = (triIdx0 + 1) % 3;
		triIdx2 = (triIdx1 + 1) % 3;

		switch (triIdx0) {
			case 0:
				ptIdx0 = 0;
				ptIdx1 = 1;
				break;
			case 1:
				ptIdx0 = 1;
				ptIdx1 = 0;
				break;
			case 2:
				ptIdx0 = 1;
				ptIdx1 = 0;
				break;
		}

		Vector3<T> quadPts[4];
		quadPts[0] = splitPts[ptIdx0];
		quadPts[1] = triPts[triIdx1];
		quadPts[2] = triPts[triIdx2];
		quadPts[3] = splitPts[ptIdx1];

		// Make the triangle
		triPts0[0] = triPts[triIdx0];
		triPts0[1] = splitPts[ptIdx0];
		triPts0[2] = splitPts[ptIdx1];

		// make the two triangles forming the quad
		triPts1[0] = quadPts[0];
		triPts1[1] = quadPts[1];
		triPts1[2] = quadPts[2];

		triPts2[0] = quadPts[0];
		triPts2[1] = quadPts[2];
		triPts2[2] = quadPts[3];

		return 3;
	}
	return 0;
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
bool TRI_LERP_INV(const Vector3<T>& pt, const std::vector<Vector3<T>>& pts, Vector3<T>& tuv, T tol)
{
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
	tuv = Vector3<T>(v.dot(x), v.dot(y), v.dot(z));
	bool done = false;
	int count = 0;
	while (!done && count++ < 100) {
		p0 = TRI_LERP(pts, (T)0, tuv[1], tuv[2]);
		p1 = TRI_LERP(pts, (T)1, tuv[1], tuv[2]);
		dir = p1 - p0;
		l = dir.norm();
		dir /= l;
		v = pt - p0;
		a = v.dot(dir) / l;

		p0 = TRI_LERP(pts, tuv[0], (T)0, tuv[2]);
		p1 = TRI_LERP(pts, tuv[0], (T)1, tuv[2]);
		dir = p1 - p0;
		l = dir.norm();
		dir /= l;
		v = pt - p0;
		b = v.dot(dir) / l;

		p0 = TRI_LERP(pts, tuv[0], tuv[1], (T)0);
		p1 = TRI_LERP(pts, tuv[0], tuv[1], (T)1);
		dir = p1 - p0;
		l = dir.norm();
		dir /= l;
		v = pt - p0;
		c = v.dot(dir) / l;

		tuv = Vector3<T>(a, b, c);
		Vector3<T> guess = TRI_LERP(pts, tuv);
		T distSqr = (guess - pt).squaredNorm();
		if (distSqr < tolSqr)
			return true;
	}
	return false;
}

template<class T>
bool intersectTriTri(const Vector3<T> triPts0[3], const Vector3<T> triPts1[3], T tol)
{
	const Vector3<T>* pPts0[] = { &triPts0[0], &triPts0[1], &triPts0[2] };
	const Vector3<T>* pPts1[] = { &triPts1[0], &triPts1[1], &triPts1[2] };
	return intersectTriTri(pPts0, pPts1, tol);
}

template<class T>
bool intersectTriTri(const Vector3<T>* triPts0[3], const Vector3<T>* triPts1[3], T tol)
{
	RayHit<T> hit;

	// Check 1 against 0
	const Vector3<T>* pts0[] = { triPts0[0], triPts0[1], triPts0[2] };
	Vector3<T> norm0 = triangleUnitNormal(pts0);
	Vector3<T> ctr0 = (*pts0[0] + *pts0[1] + *pts0[2]) / 3;
	Plane<T> triPlane0(ctr0, norm0);

	for (int i = 0; i < 3; i++) {
		int j = (i + 1) % 3;

		LineSegment<T> seg(*triPts1[i], *triPts1[j]);
		if (triPlane0.intersectLineSegment(seg, hit, tol)) {
			if (pointInTriangle<T>(triPts0, hit.hitPt, norm0, tol))
				return true;
		}
	}

	// Check 0 against 1
	const Vector3<T>* pts1[] = { triPts1[0], triPts1[1], triPts1[2] };
	Vector3<T> norm1 = triangleUnitNormal(pts1);
	Vector3<T> ctr1 = (*pts1[0] + *pts1[1] + *pts1[2]) / 3;
	Plane<T> triPlane1(ctr1, norm1);

	for (int i = 0; i < 3; i++) {
		int j = (i + 1) % 3;

		LineSegment<T> seg(*triPts0[i], *triPts0[j]);

		if (triPlane1.intersectLineSegment(seg, hit, tol)) {
			if (pointInTriangle<T>(triPts1, hit.hitPt, norm1, tol))
				return true;
		}
	}

	return false;
}

namespace {
	// After several attempts, textbook instantiation never linked correctly. This does.
	template<class T>
	bool instantiate0()
	{
		Vector3<T> ptsArr[] = {
			Vector3<T>(0,0,0),
			Vector3<T>(0,0,0),
			Vector3<T>(0,0,0),
		};
		Vector3<T> tri0[3], tri1[3], tri2[3];
		Plane<T> pl(Vector3<T>(0, 0, 0), Vector3<T>(1, 0, 0));
		return triangleSplitWithPlane<T>(ptsArr, pl, tri0, tri1, tri2) > 0;
	}

	template<class T>
	bool instantiate()
	{
		try {
			Vector3<T> ptsArr[] = {
				Vector3<T>(0,0,0),
				Vector3<T>(0,0,0),
				Vector3<T>(0,0,0),
			};
			triangleCentroid(ptsArr);

			Vector3<T> pt;
			Vector3<T>
				tri0[] = { pt, pt, pt },
				tri1[] = { pt, pt, pt };
			intersectTriTri<T>(tri0, tri1);

			instantiate0<T>();

			std::vector<Vector3<T>> pts;
			Vector3<T> tuv;
			if (pts.empty())
				return false;
			auto a = TRI_LERP(pts, (T)0, (T)0, (T)0);
			a = TRI_LERP(pts.data(), (T)0, (T)0, (T)0);
			a = BI_LERP(pt, pt, pt, pt, (T)0, (T)0);
			return TRI_LERP_INV(pt, pts, tuv);
		} catch (...) {}
		return false;
	}
	static bool dummy0 = instantiate<double>();
	static bool dummy1 = instantiate<float>();

}
