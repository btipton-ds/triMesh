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
#include <tm_ray.h>

double distanceFromPlane(const Vector3d& pt, const Plane<double>& plane) {
	return (pt - plane.getOrgin()).dot(plane.getNormal());
}

bool intersectRayPlane(const Ray<double>& ray, const Vector3d& origin, const Vector3d& normal, RayHit<double>& hit)
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

bool intersectRayTri(const Ray<double>& ray, Vector3d const * const pts[3], RayHit<double>& hit) {
	Vector3d v0 = *pts[1] - *pts[0];
	Vector3d v1 = *pts[2] - *pts[0];
	Vector3d norm = safeNormalize(v0.cross(v1));

	Plane<double> pl(*(pts[0]), norm);
	if (!pl.intersectRay(ray, hit))
		return false;

	for (int i = 0; i < 3; i++) {
		int j = (i + 1) % 3;
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
	}
	return true;
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

Vector3f triangleNormal(Vector3f const* const pts[3]) {
	Vector3f v0 = *pts[1] - *pts[0];
	Vector3f v1 = *pts[2] - *pts[0];
	Vector3f n = safeNormalize(v0.cross(v1));
	return n;
}

Vector3f triangleNormal(const Vector3f pts[3]) {
	Vector3f v0 = pts[1] - pts[0];
	Vector3f v1 = pts[2] - pts[0];
	Vector3f n = safeNormalize(v0.cross(v1));
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

bool pointInTriangle(const Vector3d& pt0, const Vector3d& pt1, const Vector3d& pt2, const Vector3d& pt)
{
	const Vector3d* pts[] = { &pt0, &pt1, &pt2 };
	return pointInTriangle(pts, pt);
}

bool pointInTriangle(const Vector3d* pts[3], const Vector3d& pt)
{
	Vector3d v0 = (*pts[1]) - (*pts[0]);
	Vector3d v1 = (*pts[2]) - (*pts[0]);

	Vector3d norm = triangleNormal(pts);
	norm.normalize();

	v0 = pt - (*pts[0]);
	double dp = v0.dot(norm);
	if (fabs(dp) > 1.0e-6) {
		return false; // Pt not in plane
	}

	for (size_t i = 0; i < 3; i++) {
		size_t j = (i + 1) % 3;
		v0 = pt - (*pts[i]);
		v1 = (*pts[j]) - (*pts[i]);
		double cp = v1.cross(v0).dot(norm);
		if (cp < 0)
			return false;
	}

	return true;
}
