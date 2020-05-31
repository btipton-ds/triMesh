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

double LineSegment::distanceToPoint(const Vector3d& pt) const {
	double t;
	return distanceToPoint(pt, t);
}

double LineSegment::distanceToPoint(const Vector3d& pt, double& t) const {
	Vector3d dir(_pts[1] - _pts[0]);
	double len = dir.norm();
	if (len < minNormalizeDivisor)
		return DBL_MAX;
	dir /= len;
	Vector3d v0 = pt - _pts[0];
	double dp = v0.dot(dir);
	t = dp / len;
	v0 = v0 - dir * dp;
	double dist;
	if (t < 0) {
		t = -DBL_MAX;
		dist = (pt - _pts[0]).norm();
	} else if (t > 1) {
		t = DBL_MAX;
		dist = (pt - _pts[1]).norm();
	} else
		dist = v0.norm();

	return dist;
}

double distanceFromPlane(const Vector3d& pt, const Plane& plane) {
	return (pt - plane._origin).dot(plane._normal);
}

bool intersectRayPlane(const Ray& ray, const Plane& plane, RayHit& hit) {
	auto dp = ray._dir.dot(plane._normal);
	if (fabs(dp) < minNormalizeDivisor)
		return false;

	Vector3d v = plane._origin - ray._origin;
	auto h = v.dot(plane._normal);
	hit.dist = h / dp;
	hit.hitPt = ray._origin + hit.dist * ray._dir;

#if 1 // Verification code
	Vector3d vTest = hit.hitPt - plane._origin;
	double testDist = vTest.dot(plane._normal);
	if (fabs(testDist) > SAME_DIST_TOL) {
		throw "Point not on plane";
	}
#endif
	return true;
}

bool intersectRayTri(const Ray& ray, Vector3d const * const pts[3], RayHit& hit) {
	Vector3d v0 = *pts[1] - *pts[0];
	Vector3d v1 = *pts[2] - *pts[0];
	Vector3d norm = safeNormalize(v0.cross(v1));

	if (!intersectRayPlane(ray, Plane(*(pts[0]), norm), hit))
		return false;

	for (int i = 0; i < 3; i++) {
		int j = (i + 1) % 3;
		int k = (i + 2) % 3;
#if 0
		v0 = safeNormalize(*(pts[j]) - *(pts[i]));
		v1 = hit.hitPt - *(pts[i]);
		Vector3d v2 = *pts[k] - *pts[i];
		v2 = safeNormalize(v2 - v0.dot(v2) * v0);
		double d = v1.dot(v2);
		if (d < -SAME_DIST_TOL)
			return false;
#else
		v0 = *pts[j] - *pts[i];
		v1 = hit.hitPt - *pts[i];
		Vector3d cp = v0.cross(v1);
		double v = cp.dot(norm);
		if (v < 0) {
			v1 = v1 - v0 * v0.dot(v1);
			double err = v1.squaredNorm() / SAME_DIST_TOL_SQR - 1;
			if (err > NUMERIC_DIFF_TOL)
				return false;
		}
#endif
	}
	return true;
}

bool intersectLineSegTri(const LineSegment& seg, Vector3d const* const pts[3], RayHit& hit) {
	Vector3d unitDir = seg._pts[1] - seg._pts[0];
	double l = unitDir.norm();
	unitDir /= l;
	Ray ray(seg._pts[0], unitDir);
	if (intersectRayTri(seg.getRay(), pts, hit)) {
		Vector3d v1 = hit.hitPt - seg._pts[0];
		double d = unitDir.dot(v1);
		if (-SAME_DIST_TOL < d  && d < l + SAME_DIST_TOL) {
			return true;
		} else {
			return false;
		}
	}
	return false;
}

Vector3d triangleNormal(Vector3d const* const pts[3]) {
	Vector3d v0 = *pts[1] - *pts[0];
	Vector3d v1 = *pts[2] - *pts[0];
	Vector3d n = safeNormalize(v0.cross(v1));
	return n;
}

Vector3d triangleNormal(const Vector3d pts[3]) {
	Vector3d v0 = pts[1] - pts[0];
	Vector3d v1 = pts[2] - pts[0];
	Vector3d n = safeNormalize(v0.cross(v1));
	return n;
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

bool intersectLineSegPlane(const LineSegment& seg, const Plane& plane, RayHit& hit) {
	if (intersectRayPlane(seg.getRay(), plane, hit)) {
		Vector3d v = hit.hitPt - seg._pts[0];
		double t = v.dot(seg.calcDir()) / seg.calLength();
		return 0 <= t && t <= 1;
	}
	return false;
}

bool intersectLineSegPlane(const LineSegment& seg, const Vector3d* pts[3], RayHit& hit) {
	return intersectLineSegPlane(seg, Plane(pts), hit);
}
