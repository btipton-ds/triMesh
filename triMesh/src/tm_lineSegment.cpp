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

#include <tm_lineSegment.h>
#include <tm_math.h>

LineSegment::LineSegment(const Vector3d& p0, const Vector3d& p1)
{
	_pts[0] = p0;
	_pts[1] = p1;
}

double LineSegment::calLength() const {
	return (_pts[1] - _pts[0]).norm();
}

Vector3d LineSegment::calcDir() const {
	return safeNormalize<double>(_pts[1] - _pts[0]);
}

Vector3d LineSegment::interpolate(double t) const {
	return _pts[0] + t * (_pts[1] - _pts[0]);
}

double LineSegment::parameterize(const Vector3d& pt) const {
	return (pt - _pts[0]).dot(calcDir());
}

Ray LineSegment::getRay() const {
	return Ray(_pts[0], calcDir());
}

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
	}
	else if (t > 1) {
		t = DBL_MAX;
		dist = (pt - _pts[1]).norm();
	}
	else
		dist = v0.norm();

	return dist;
}

bool LineSegment::intersectTri(Vector3d const* const pts[3], RayHit& hit) const
{
	Vector3d unitDir = _pts[1] - _pts[0];
	double l = unitDir.norm();
	unitDir /= l;
	Ray ray(_pts[0], unitDir);
	if (intersectRayTri(getRay(), pts, hit)) {
		Vector3d v1 = hit.hitPt - _pts[0];
		double d = unitDir.dot(v1);
		if (-SAME_DIST_TOL < d && d < l + SAME_DIST_TOL) {
			return true;
		}
		else {
			return false;
		}
	}
	return false;
}

bool LineSegment::intersectPlane(const Plane& plane, RayHit& hit) const
{
	if (plane.intersectRay(getRay(), hit)) {
		Vector3d v = hit.hitPt - _pts[0];
		double t = v.dot(calcDir()) / calLength();
		return 0 <= t && t <= 1;
	}
	return false;
}

bool LineSegment::intersectPlane(const Vector3d* pts[3], RayHit& hit) const
{
	return intersectPlane(Plane(pts), hit);
}

