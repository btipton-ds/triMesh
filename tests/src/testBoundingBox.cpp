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

#include <tm_boundingBox.h>
#include <tm_ray.h>

using namespace std;

namespace {
	using BB = CBoundingBox3Dd;
}

int testContains() {
	BB bb(Vector3d(0, 0, 0), Vector3d(1, 1, 1));

	TEST_TRUE(bb.contains(bb.getMin()), "Contains it's own min corner?");
	TEST_TRUE(bb.contains(bb.getMax()), "Contains it's own max corner?");
	TEST_TRUE(bb.contains(0.5 * (bb.getMax() + bb.getMax())), "Contains it's own centroid?");
	TEST_FALSE(bb.contains(Vector3d(2,2,2)), "Does not contain an outside point?");

	cout << "testContains passed \n";

	return 0;
}

int testIntersect() {
	Vector3d tolx(SAME_DIST_TOL, 0, 0);
	BB 
		a(Vector3d(0, 0, 0), Vector3d(1, 1, 1)),
		b(Vector3d(1, 1, 1), Vector3d(2, 2, 2)),
		c(Vector3d(1, 1, 1), Vector3d(2, 2, 2)),
		cTol(a.getMax() + tolx, Vector3d(2, 2, 2)),
		cNoTol(a.getMax() + 1.01* tolx, Vector3d(2, 2, 2)),
		d(Vector3d(0.25, 0.25, 0.25), Vector3d(0.75, 0.75, 0.75));

	TEST_TRUE(a.intersects(a), "Box intersects itself?");
	TEST_TRUE(a.intersects(b), "Box intersects at a corner?");
	TEST_TRUE(a.intersects(c), "Box intersects at a face?");
	TEST_TRUE(a.intersects(cTol), "Box does not intersect at a face within tolerance?");
	TEST_FALSE(a.intersects(cNoTol), "Box does not intersect at a face out of tolerance?");
	TEST_TRUE(a.intersects(d), "Box intersects a box it contains?");
	TEST_TRUE(d.intersects(a), "Box intersects a which contains it?");

	cout << "testIntersect passed \n";
	return 0;
}

int testRayIntersect() {
	Vector3d tolx(SAME_DIST_TOL, 0, 0);
	BB a(Vector3d(0, 0, 0), Vector3d(1, 1, 1));
	Vector3d ctr = (a.getMin() + a.getMax()) * 0.5;
	Vector3d pt0 = ctr + Vector3d(1, 0, 0);
	Vector3d pt1 = ctr + Vector3d(1, 0.5, 0);
	Vector3d pt2 = ctr + Vector3d(1, 0.5 + SAME_DIST_TOL, 0);
	Vector3d pt3 = ctr + Vector3d(1, 0.5 + 1.01 * SAME_DIST_TOL, 0);

	TEST_TRUE(a.intersects(Ray(ctr, Vector3d(1, 0, 0))), "Box centroid in (1, 0, 0) intersects box?");
	TEST_TRUE(a.intersects(Ray(ctr, Vector3d(0, 1, 0))), "Box centroid in (0, 1, 0) intersects box?");
	TEST_TRUE(a.intersects(Ray(ctr, Vector3d(0, 0, 1))), "Box centroid in (0, 0, 1) intersects box?");
	TEST_TRUE(a.intersects(Ray(ctr, Vector3d(1, 1, 1).normalized())), "Box centroid in (1, 1, 1) intersects box?");

	TEST_TRUE(a.intersects(Ray(ctr, Vector3d(-1, 0, 0))), "Box centroid in (-1, 0, 0) intersects box?");
	TEST_TRUE(a.intersects(Ray(ctr, Vector3d(0, -1, 0))), "Box centroid in (0, -1, 0) intersects box?");
	TEST_TRUE(a.intersects(Ray(ctr, Vector3d(0, 0, -1))), "Box centroid in (0, 0, -1) intersects box?");
	TEST_TRUE(a.intersects(Ray(ctr, Vector3d(-1, -1, -1).normalized())), "Box centroid in (-1, -1, -1) intersects box?");

	TEST_TRUE(a.intersects(Ray(pt0, Vector3d(1, 0, 0))), "Box centroid + (1,0,0) in (1, 0, 0) intersects box?");
	TEST_TRUE(a.intersects(Ray(pt0, Vector3d(-1, 0, 0))), "Box centroid + (1,0,0) in (-1, 0, 0) intersects box?");
	TEST_FALSE(a.intersects(Ray(pt0, Vector3d(0, 1, 0))), "Box centroid + (1,0,0) in (0, 1, 0) does not intersect box?");

	TEST_TRUE(a.intersects(Ray(pt0, Vector3d(-1, 1, 0).normalized())), "Box centroid + (1,0,0) in (-1, 1, 0) intersect box?");
	TEST_FALSE(a.intersects(Ray(pt0, Vector3d(-1, 1.1, 0).normalized())), "Box centroid + (1,0,0) in (-1, 1.1, 0) does not intersect box?");

	TEST_TRUE(a.intersects(Ray(pt1, Vector3d(1, 0, 0))), "Box centroid + (1,0.5,0) in (1, 0, 0) intersects box?");
	TEST_TRUE(a.intersects(Ray(pt1, Vector3d(-1, 0, 0))), "Box centroid + (1,0.5,0) in (-1, 0, 0) intersects box?");
	TEST_FALSE(a.intersects(Ray(pt1, Vector3d(0, 1, 0))), "Box centroid + (1,0.5,0) in (0, 1, 0) does not intersect box?");

	TEST_TRUE(a.intersects(Ray(pt2, Vector3d(1, 0, 0))), "Box centroid + (1,0.5 + tol,0) in (1, 0, 0) intersects box?");
	TEST_TRUE(a.intersects(Ray(pt2, Vector3d(-1, 0, 0))), "Box centroid + (1,0.5 + tol,0) in (-1, 0, 0) intersects box?");
	TEST_FALSE(a.intersects(Ray(pt2, Vector3d(0, 1, 0))), "Box centroid + (1,0.5 + tol,0) in (0, 1, 0) does not intersect box?");

	TEST_FALSE(a.intersects(Ray(pt3, Vector3d(1, 0, 0))), "Box centroid + (1,0.5 + 1.01 * tol,0) in (1, 0, 0) does not intersects box?");
	TEST_FALSE(a.intersects(Ray(pt3, Vector3d(-1, 0, 0))), "Box centroid + (1,0.5 + 1.01 * tol,0) in (-1, 0, 0) does not intersects box?");

	cout << "testRayIntersect passed \n";
	return 0;
}

int testRayIntersect1() {
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
		TEST_TRUE(a.intersects(Ray(pt0 + delta[i] + Vector3d(0, 0.0, 0.0), xAxis)), "Ray hits corner 0 with delta " + to_string(i) + "?");
		TEST_TRUE(a.intersects(Ray(pt0 + delta[i] + Vector3d(0, 1.0, 0.0), xAxis)), "Ray hits corner 1 with delta " + to_string(i) + "?");
		TEST_TRUE(a.intersects(Ray(pt0 + delta[i] + Vector3d(0, 1.0, 1.0), xAxis)), "Ray hits corner 2 with delta " + to_string(i) + "?");
		TEST_TRUE(a.intersects(Ray(pt0 + delta[i] + Vector3d(0, 0.0, 1.0), xAxis)), "Ray hits corner 3 with delta " + to_string(i) + "?");

		TEST_TRUE(a.intersects(Ray(pt0 + delta[i] + Vector3d(0, 0.5, 0.0), xAxis)), "Ray hits edge 0 with delta " + to_string(i) + "?");
		TEST_TRUE(a.intersects(Ray(pt0 + delta[i] + Vector3d(0, 1.0, 0.5), xAxis)), "Ray hits edge 1 with delta " + to_string(i) + "?");
		TEST_TRUE(a.intersects(Ray(pt0 + delta[i] + Vector3d(0, 0.5, 1.0), xAxis)), "Ray hits edge 2 with delta " + to_string(i) + "?");
		TEST_TRUE(a.intersects(Ray(pt0 + delta[i] + Vector3d(0, 0.0, 0.5), xAxis)), "Ray hits edge 3 with delta " + to_string(i) + "?");
	}

	cout << "testRayIntersect1 passed \n";
	return 0;
}

int testSheetInterset() {
	BB a(Vector3d(0, 0, 0), Vector3d(1, 1, 1));
	BB b(Vector3d(0, 0, 0.5), Vector3d(1, 1, 0.5));
	BB c(Vector3d(0.25, 0.25, 0.5), Vector3d(0.75, 0.75, 0.5));
	BB d(Vector3d(-0.25, -0.25, 0.5), Vector3d(1.25, 1.25, 0.5));
	BB e(Vector3d(0.25, 0.25, 0.5), Vector3d(1.25, 1.25, 0.5));

	TEST_TRUE(a.intersects(b), "Match sheet intersects?");
	TEST_TRUE(a.intersects(c), "Contained sheet intersects?");
	TEST_TRUE(a.intersects(d), "Oversized sheet intersects?");
	TEST_TRUE(a.intersects(e), "Overlapping sheet intersects?");

	return 0;
}

int testBoundingBox() {
	if (testContains() != 0) return 1;
	if (testIntersect() != 0) return 1;
	if (testRayIntersect() != 0) return 1;
	if (testRayIntersect1() != 0) return 1;
	if (testSheetInterset() != 0) return 1;

	cout << "testBoundingBox passed \n";

	return 0;
}
