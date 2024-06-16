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

#include <tests.h>
#include <iostream>
#include <string>

#include <Eigen/Geometry>
#include <Eigen/src/Core/Matrix.h>
#include <tm_lineSegment.h>
#include <tm_boundingBox.h>
#include <tm_ray.h>

using namespace std;

namespace {
	using BB = CBoundingBox3Dd;
}

bool testContains() {
	BB bb(Vector3d(0, 0, 0), Vector3d(1, 1, 1));

	TEST_TRUE(bb.contains(bb.getMin(), SAME_DIST_TOL), "Contains it's own min corner?");
	TEST_TRUE(bb.contains(bb.getMax(), SAME_DIST_TOL), "Contains it's own max corner?");
	TEST_TRUE(bb.contains(0.5 * (bb.getMax() + bb.getMax()), SAME_DIST_TOL), "Contains it's own centroid?");
	TEST_FALSE(bb.contains(Vector3d(2,2,2), SAME_DIST_TOL), "Does not contain an outside point?");

	cout << "testContains passed \n";

	return true;
}

bool testIntersect() {
	Vector3d tolx(SAME_DIST_TOL, 0, 0);
	BB 
		a(Vector3d(0, 0, 0), Vector3d(1, 1, 1)),
		b(Vector3d(1, 1, 1), Vector3d(2, 2, 2)),
		c(Vector3d(1, 0, 0), Vector3d(2, 2, 2)),
		cTol(a.getMax() + tolx, Vector3d(2, 2, 2)),
		cNoTol(a.getMax() + 1.01* tolx, Vector3d(2, 2, 2)),
		d(Vector3d(0.25, 0.25, 0.25), Vector3d(0.75, 0.75, 0.75));

	TEST_TRUE(a.intersectsOrContains(a, SAME_DIST_TOL), "Box intersects or contains itself?");
	TEST_TRUE(a.intersectsOrContains(b, SAME_DIST_TOL), "Box intersects or contains at a corner?");
	TEST_TRUE(a.intersectsOrContains(c, SAME_DIST_TOL), "Box intersects or contains at a face?");
	TEST_TRUE(a.intersectsOrContains(cTol, SAME_DIST_TOL), "Box intersects or contains a face within tolerance?");
	TEST_FALSE(a.intersectsOrContains(cNoTol, SAME_DIST_TOL), "Box does not intersect a box out of tolerance?");
	TEST_TRUE(a.intersectsOrContains(d, SAME_DIST_TOL), "Box intersects or contains a box it contains?");
	TEST_TRUE(d.intersectsOrContains(a, SAME_DIST_TOL), "Box intersects or contains a box which contains it?");

	cout << "testIntersect passed \n";
	return true;
}

template<class T>
Ray<T> rotateAboutAxis(Ray<T>& axis, Ray<T>& val,int angleDeg)
{
	Ray<T> result;

	result._origin = rotatePointAboutAxis(axis, val._origin);
	result._dir = rotateVectorAboutAxis(axis, val._dir);

	return result;
}

bool testRayIntersect() {
	Vector3d tolx(SAME_DIST_TOL, 0, 0);
	BB a(Vector3d(0, 0, 0), Vector3d(1, 1, 1));
	Vector3d ctr = (a.getMin() + a.getMax()) * 0.5;
	Vector3d pt0 = ctr + Vector3d(1, 0, 0);
	Vector3d pt1 = ctr + Vector3d(1, 0.5, 0);
	Vector3d pt2 = ctr + Vector3d(1, 0.5 + SAME_DIST_TOL, 0);
	Vector3d pt3 = ctr + Vector3d(1, 0.5 + 1.01 * SAME_DIST_TOL, 0);

	TEST_TRUE(a.intersects(Ray<double>(ctr, Vector3d(1, 0, 0)), SAME_DIST_TOL), "Box centroid in (1, 0, 0) intersects box?");
	TEST_TRUE(a.intersects(Ray<double>(ctr, Vector3d(0, 1, 0)), SAME_DIST_TOL), "Box centroid in (0, 1, 0) intersects box?");
	TEST_TRUE(a.intersects(Ray<double>(ctr, Vector3d(0, 0, 1)), SAME_DIST_TOL), "Box centroid in (0, 0, 1) intersects box?");
	TEST_TRUE(a.intersects(Ray<double>(ctr, Vector3d(1, 1, 1).normalized()), SAME_DIST_TOL), "Box centroid in (1, 1, 1) intersects box?");

	TEST_TRUE(a.intersects(Ray<double>(ctr, Vector3d(-1, 0, 0)), SAME_DIST_TOL), "Box centroid in (-1, 0, 0) intersects box?");
	TEST_TRUE(a.intersects(Ray<double>(ctr, Vector3d(0, -1, 0)), SAME_DIST_TOL), "Box centroid in (0, -1, 0) intersects box?");
	TEST_TRUE(a.intersects(Ray<double>(ctr, Vector3d(0, 0, -1)), SAME_DIST_TOL), "Box centroid in (0, 0, -1) intersects box?");
	TEST_TRUE(a.intersects(Ray<double>(ctr, Vector3d(-1, -1, -1).normalized()), SAME_DIST_TOL), "Box centroid in (-1, -1, -1) intersects box?");

	TEST_TRUE(a.intersects(Ray<double>(pt0, Vector3d(1, 0, 0)), SAME_DIST_TOL), "Box centroid + (1,0,0) in (1, 0, 0) intersects box?");
	TEST_TRUE(a.intersects(Ray<double>(pt0, Vector3d(-1, 0, 0)), SAME_DIST_TOL), "Box centroid + (1,0,0) in (-1, 0, 0) intersects box?");
	TEST_FALSE(a.intersects(Ray<double>(pt0, Vector3d(0, 1, 0)), SAME_DIST_TOL), "Box centroid + (1,0,0) in (0, 1, 0) does not intersect box?");

	TEST_TRUE(a.intersects(Ray<double>(pt0, Vector3d(-1, 1, 0).normalized()), SAME_DIST_TOL), "Box centroid + (1,0,0) in (-1, 1, 0) intersect box?");
	TEST_FALSE(a.intersects(Ray<double>(pt0, Vector3d(-1, 1.1, 0).normalized()), SAME_DIST_TOL), "Box centroid + (1,0,0) in (-1, 1.1, 0) does not intersect box?");

	TEST_TRUE(a.intersects(Ray<double>(pt1, Vector3d(1, 0, 0)), SAME_DIST_TOL), "Box centroid + (1,0.5,0) in (1, 0, 0) intersects box?");
	TEST_TRUE(a.intersects(Ray<double>(pt1, Vector3d(-1, 0, 0)), SAME_DIST_TOL), "Box centroid + (1,0.5,0) in (-1, 0, 0) intersects box?");
	TEST_FALSE(a.intersects(Ray<double>(pt1, Vector3d(0, 1, 0)), SAME_DIST_TOL), "Box centroid + (1,0.5,0) in (0, 1, 0) does not intersect box?");

	TEST_TRUE(a.intersects(Ray<double>(pt2, Vector3d(1, 0, 0)), SAME_DIST_TOL), "Box centroid + (1,0.5 + tol,0) in (1, 0, 0) intersects box?");
	TEST_TRUE(a.intersects(Ray<double>(pt2, Vector3d(-1, 0, 0)), SAME_DIST_TOL), "Box centroid + (1,0.5 + tol,0) in (-1, 0, 0) intersects box?");
	TEST_FALSE(a.intersects(Ray<double>(pt2, Vector3d(0, 1, 0)), SAME_DIST_TOL), "Box centroid + (1,0.5 + tol,0) in (0, 1, 0) does not intersect box?");

	TEST_FALSE(a.intersects(Ray<double>(pt3, Vector3d(1, 0, 0)), SAME_DIST_TOL), "Box centroid + (1,0.5 + 1.01 * tol,0) in (1, 0, 0) does not intersects box?");
	TEST_FALSE(a.intersects(Ray<double>(pt3, Vector3d(-1, 0, 0)), SAME_DIST_TOL), "Box centroid + (1,0.5 + 1.01 * tol,0) in (-1, 0, 0) does not intersects box?");

	cout << "testRayIntersect passed \n";
	return true;
}

bool testRayIntersect1() {
	Vector3d tolx(SAME_DIST_TOL, 0, 0);
	BB a(Vector3d(0, 0, 0), Vector3d(1, 1, 1));
	Vector3d pt0(-1, 0, 0);

	Vector3d xAxis(1, 0, 0);

	Vector3d delta[] = {
		Vector3d(0, 0, 0),
		Vector3d(0, SAME_DIST_TOL, 0),
		Vector3d(0, -SAME_DIST_TOL, 0),
		Vector3d(0, 0, SAME_DIST_TOL),
		Vector3d(0, 0, -SAME_DIST_TOL),
	};
	for (int i = 0; i < 5; i++) {
		TEST_TRUE(a.intersects(Ray<double>(pt0 + delta[i] + Vector3d(0, 0.0, 0.0), xAxis), SAME_DIST_TOL), "Ray hits corner 0 with delta " + to_string(i) + "?");
		TEST_TRUE(a.intersects(Ray<double>(pt0 + delta[i] + Vector3d(0, 1.0, 0.0), xAxis), SAME_DIST_TOL), "Ray hits corner 1 with delta " + to_string(i) + "?");
		TEST_TRUE(a.intersects(Ray<double>(pt0 + delta[i] + Vector3d(0, 1.0, 1.0), xAxis), SAME_DIST_TOL), "Ray hits corner 2 with delta " + to_string(i) + "?");
		TEST_TRUE(a.intersects(Ray<double>(pt0 + delta[i] + Vector3d(0, 0.0, 1.0), xAxis), SAME_DIST_TOL), "Ray hits corner 3 with delta " + to_string(i) + "?");

		TEST_TRUE(a.intersects(Ray<double>(pt0 + delta[i] + Vector3d(0, 0.5, 0.0), xAxis), SAME_DIST_TOL), "Ray hits edge 0 with delta " + to_string(i) + "?");
		TEST_TRUE(a.intersects(Ray<double>(pt0 + delta[i] + Vector3d(0, 1.0, 0.5), xAxis), SAME_DIST_TOL), "Ray hits edge 1 with delta " + to_string(i) + "?");
		TEST_TRUE(a.intersects(Ray<double>(pt0 + delta[i] + Vector3d(0, 0.5, 1.0), xAxis), SAME_DIST_TOL), "Ray hits edge 2 with delta " + to_string(i) + "?");
		TEST_TRUE(a.intersects(Ray<double>(pt0 + delta[i] + Vector3d(0, 0.0, 0.5), xAxis), SAME_DIST_TOL), "Ray hits edge 3 with delta " + to_string(i) + "?");
	}

	cout << "testRayIntersect1 passed \n";
	return true;
}

void rotate(Vector3d& v)
{
	double temp = v[2];
	v[2] = v[1];
	v[1] = v[0];
	v[0] = temp;
}

void rotate(BB& b)
{
	Vector3d min = b.getMin();
	Vector3d max = b.getMax();

	rotate(min);
	rotate(max);
	b = BB(min, max);
}

bool testSheetIntersect() {
	BB a(Vector3d(0, 0, 0), Vector3d(1, 1, 1));
	BB b(Vector3d(0, 0, 0.5), Vector3d(1, 1, 0.5));
	BB c(Vector3d(0.25, 0.25, 0.5), Vector3d(0.75, 0.75, 0.5));
	BB d(Vector3d(-0.25, -0.25, 0.5), Vector3d(1.25, 1.25, 0.5));
	BB e(Vector3d(0.25, 0.25, 0.5), Vector3d(1.25, 1.25, 0.5));

	TEST_TRUE(a.intersectsOrContains(b, SAME_DIST_TOL), "Match sheet intersects?");
	TEST_TRUE(a.intersectsOrContains(c, SAME_DIST_TOL), "Contained sheet intersects?");
	TEST_TRUE(a.intersectsOrContains(d, SAME_DIST_TOL), "Oversized sheet intersects?");
	TEST_TRUE(a.intersectsOrContains(e, SAME_DIST_TOL), "Overlapping sheet intersects?");

	rotate(a);
	rotate(b);
	rotate(c);
	rotate(d);
	rotate(e);

	TEST_TRUE(a.intersectsOrContains(b, SAME_DIST_TOL), "Match sheet intersects 1?");
	TEST_TRUE(a.intersectsOrContains(c, SAME_DIST_TOL), "Contained sheet intersects 1?");
	TEST_TRUE(a.intersectsOrContains(d, SAME_DIST_TOL), "Oversized sheet intersects 1?");
	TEST_TRUE(a.intersectsOrContains(e, SAME_DIST_TOL), "Overlapping sheet intersects 1?");

	rotate(a);
	rotate(b);
	rotate(c);
	rotate(d);
	rotate(e);

	TEST_TRUE(a.intersectsOrContains(b, SAME_DIST_TOL), "Match sheet intersects 2?");
	TEST_TRUE(a.intersectsOrContains(c, SAME_DIST_TOL), "Contained sheet intersects 2?");
	TEST_TRUE(a.intersectsOrContains(d, SAME_DIST_TOL), "Oversized sheet intersects 2?");
	TEST_TRUE(a.intersectsOrContains(e, SAME_DIST_TOL), "Overlapping sheet intersects 2?");

	return true;
}

bool testTriIntersect() {
	BB a(Vector3d(0, 0, 0), Vector3d(1, 1, 1));

	Vector3d pts[][3] = {
		{
			Vector3d(0, 0, 0),
			Vector3d(1, 0, 0),
			Vector3d(1, 1, 0)
		},

		{
			Vector3d(0.1, 0.1, 0.1),
			Vector3d(1 - 0.1, 0.1, 0.1),
			Vector3d(1 - 0.1, 1 - 0.1, 0.1)
		},

		{
			Vector3d(1, 0.5, 0.5),
			Vector3d(2, 0.5, 0.5),
			Vector3d(2, 1, 0.5)
		},
		{
			Vector3d(1 + SAME_DIST_TOL, 0.5, 0.5),
			Vector3d(2, 0.5, 0.5),
			Vector3d(2, 1, 0.5)
		},
		{
			Vector3d(0.5, -0.5, 0.5),
			Vector3d(1.5, -0.5, 0.5),
			Vector3d(1.5,  0.5, 0.5)
		},
		{
			Vector3d(0.5, -0.51, 0.5),
			Vector3d(1.5, -0.51, 0.5),
			Vector3d(1.5, 0.5, 0.5)
		},
		{
			Vector3d(1 + 1.1 * SAME_DIST_TOL, 0.5, 0.5),
			Vector3d(2, 0.5, 0.5),
			Vector3d(2, 1, 0.5)
		},
	};

	for (int axis = 0; axis < 3; axis++) {
		int i = 0;

		TEST_TRUE(a.intersectsOrContains(pts[i][0], pts[i][1], pts[i][2], SAME_DIST_TOL), "Intersects triangle which lies on face?"); i++;
		TEST_TRUE(a.intersectsOrContains(pts[i][0], pts[i][1], pts[i][2], SAME_DIST_TOL), "Intersects triangle which lies inside box?"); i++;
		TEST_TRUE(a.intersectsOrContains(pts[i][0], pts[i][1], pts[i][2], SAME_DIST_TOL), "Intersects triangle which lies inside box?"); i++;
		TEST_TRUE(a.intersectsOrContains(pts[i][0], pts[i][1], pts[i][2], SAME_DIST_TOL), "Intersects triangle with no point inside the box?"); i++;
		TEST_TRUE(a.intersectsOrContains(pts[i][0], pts[i][1], pts[i][2], SAME_DIST_TOL), "Intersects triangle with edge intersecting box edge?"); i++;
		TEST_FALSE(a.intersectsOrContains(pts[i][0], pts[i][1], pts[i][2], SAME_DIST_TOL), "Intersects triangle with no point inside the box?"); i++;
		TEST_FALSE(a.intersectsOrContains(pts[i][0], pts[i][1], pts[i][2], SAME_DIST_TOL), "Fails to intersects triangle point lies out of tolerance on face?"); i++;

		for (i = 0; i < 7; i++) {
			rotate(pts[i][0]);
			rotate(pts[i][1]);
			rotate(pts[i][2]);
		}
	}
	
	return true;
}

bool testSegIntersect() {
	BB a(Vector3d(0, 0, 0), Vector3d(1, 1, 1));

	LineSegmentd segs[14] = {
		LineSegmentd(Vector3d(0, 0, 0), Vector3d(1, 0, 0)),
		LineSegmentd(Vector3d(0.1, 0.1, 0.1), Vector3d(1 - 0.1, 0.1, 0.1)),
		LineSegmentd(Vector3d(1 - 0.1, 0.1, 0.1), Vector3d(1 - 0.1, 1 - 0.1, 0.1)),
		LineSegmentd(Vector3d(1 - 0.1, 1 - 0.1, 0.1), Vector3d(0.1, 0.1, 0.1)),
		LineSegmentd(Vector3d(1, 0.5, 0.5), Vector3d(2, 0.5, 0.5)),
		LineSegmentd(Vector3d(1 + SAME_DIST_TOL, 0.5, 0.5), Vector3d(2, 0.5, 0.5)),
		LineSegmentd(Vector3d(1 + 2 * SAME_DIST_TOL, 0.5, 0.5), Vector3d(2, 0.5, 0.5)),
		LineSegmentd(Vector3d(-1, 0.5, 0.5), Vector3d(2, 0.5, 0.5)),
		LineSegmentd(Vector3d(0, 0.5, 0.5), Vector3d(2, 0.5, 0.5)),
		LineSegmentd(Vector3d(-1, 0.5, 0.5), Vector3d(1, 0.5, 0.5)),
		LineSegmentd(Vector3d(1, 0, 0.5), Vector3d(2, 1, 0.5)),
		LineSegmentd(Vector3d(1, -0.01, 0.5), Vector3d(2, 1, 0.5)),
		LineSegmentd(Vector3d(0, -0.5, 0.5), Vector3d(2, 0.5, 0.5)),
		LineSegmentd(Vector3d(0, -0.51, 0.5), Vector3d(2, 0.5, 0.5))
	};

	for (int axis = 0; axis < 3; axis++) {
		{
			bool result = (a.intersects(segs[0], SAME_DIST_TOL, -1)); if (!result) {
				cout << "Intersects segment which lies on face?" << " fail\n"; return false;
			}
		};
		TEST_TRUE(a.intersects(segs[4], SAME_DIST_TOL, -1), "Intersects segment point lies on face?");
		TEST_TRUE(a.intersects(segs[5], SAME_DIST_TOL, -1), "Intersects segment point lies on face plus tol?");
		TEST_FALSE(a.intersects(segs[6], SAME_DIST_TOL, -1), "Intersects segment point lies on face plus 2 tol?");

		TEST_TRUE(a.intersects(segs[7], SAME_DIST_TOL, -1), "over runs box on both sides?");
		TEST_TRUE(a.intersects(segs[8], SAME_DIST_TOL, -1), "over runs box on positive side?");
		TEST_TRUE(a.intersects(segs[9], SAME_DIST_TOL, -1), "over runs box on negative side?");

		TEST_TRUE(a.intersects(segs[10], SAME_DIST_TOL, -1), "seg hits edge of box?");
		TEST_FALSE(a.intersects(segs[11], SAME_DIST_TOL, -1), "seg hits edge of box?");

		TEST_TRUE(a.intersects(segs[12], SAME_DIST_TOL, -1), "seg hits edge of box?");
		TEST_FALSE(a.intersects(segs[13], SAME_DIST_TOL, -1), "seg hits edge of box?");

		TEST_TRUE(a.intersectsOrContains(segs[0], SAME_DIST_TOL, -1), "Intersects or contains segment which lies on face?");
		TEST_TRUE(a.intersectsOrContains(segs[1], SAME_DIST_TOL, -1), "Intersects or contains segment which lies inside box?");
		TEST_TRUE(a.intersectsOrContains(segs[2], SAME_DIST_TOL, -1), "Intersects or contains segment which lies inside box?");
		TEST_TRUE(a.intersectsOrContains(segs[3], SAME_DIST_TOL, -1), "Intersects or contains segment which lies inside box?");

		TEST_TRUE(a.intersectsOrContains(segs[4], SAME_DIST_TOL, -1), "Intersects or contains segment point lies on face?");
		TEST_TRUE(a.intersectsOrContains(segs[5], SAME_DIST_TOL, -1), "Intersects or contains segment point lies on face plus tol?");
		TEST_FALSE(a.intersectsOrContains(segs[6], SAME_DIST_TOL, -1), "Intersects sor contains egment point lies on face plus 2 tol?");

		TEST_TRUE(a.intersectsOrContains(segs[7], SAME_DIST_TOL, -1), "over runs box on both sides?");
		TEST_TRUE(a.intersectsOrContains(segs[8], SAME_DIST_TOL, -1), "over runs box on positive side?");
		TEST_TRUE(a.intersectsOrContains(segs[9], SAME_DIST_TOL, -1), "over runs box on negative side?");

		TEST_TRUE(a.intersectsOrContains(segs[10], SAME_DIST_TOL, -1), "seg hits edge of box?");
		TEST_FALSE(a.intersectsOrContains(segs[11], SAME_DIST_TOL, -1), "seg hits edge of box?");

		TEST_TRUE(a.intersectsOrContains(segs[12], SAME_DIST_TOL, -1), "seg hits edge of box?");
		TEST_FALSE(a.intersectsOrContains(segs[13], SAME_DIST_TOL, -1), "seg hits edge of box?");

		for (int i = 0; i < 14; i++) {
			rotate(segs[i]._pts[0]);
			rotate(segs[i]._pts[1]);
		}
	}

	return true;
}

bool testBoundingBox() {
	TEST_TRUE(testContains(), "testContains");
	TEST_TRUE(testIntersect(), "testIntersect");
	TEST_TRUE(testRayIntersect(), "testRayIntersect");
	TEST_TRUE(testRayIntersect1(), "testRayIntersect1");
	TEST_TRUE(testSheetIntersect(), "testSheetIntersect");
	TEST_TRUE(testTriIntersect(), "testTriIntersect");
	TEST_TRUE(testSegIntersect(), "testSegIntersect");

	cout << "testBoundingBox passed \n";

	return true;
}
