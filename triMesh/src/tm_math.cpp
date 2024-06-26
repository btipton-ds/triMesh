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

