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
#include <tests.h>

#include <iostream>
#include <string>
#include <tm_math.h>
#include <tm_ray.h>
#include <tm_math_transforms.h>

using namespace std;
bool testDistToPlane() {
	Plane<double> plane(Vector3d(0, 0, 0), Vector3d(0, 0, 1));
	plane.makePrincipal();
	Vector3d pt0(0, 0, 1);
	Vector3d pt1(0, 0, -1);
	Vector3d pt2(1, 1, 1);

	TEST_TRUE(distanceFromPlane(pt0, plane) - 1 < SAME_DIST_TOL, "Point is 1 unit from plane?");
	TEST_TRUE(distanceFromPlane(pt1, plane) - -1 < SAME_DIST_TOL, "Point is -1 unit from plane?");
	TEST_TRUE(distanceFromPlane(pt2, plane) - 1 < SAME_DIST_TOL, "Point is 1 unit from plane?");

	return true;
}

bool testRayPlaneIntersect() {
	Plane<double> plane0(Vector3d(0, 0, 0), Vector3d(0, 0, 1));
	plane0.makePrincipal();
	Vector3d dir0(-1, -1, -1);
	Ray<double> ray0(plane0.getOrgin() - dir0, dir0);
	RayHit<double> hit;

	TEST_TRUE(plane0.intersectRay(ray0, hit, SAME_DIST_TOL), "Ray intersects plane?");
	TEST_TRUE(distanceFromPlane(hit.hitPt, plane0) < SAME_DIST_TOL, "Intersection point lies on plane?");

	Vector3d delta(.1, -.023, 1.35);
	ray0._origin += delta;
	TEST_TRUE(plane0.intersectRay(ray0, hit, SAME_DIST_TOL), "Ray + delta intersects plane?");
	TEST_TRUE(distanceFromPlane(hit.hitPt, plane0) < SAME_DIST_TOL, "Intersection point lies on plane?");

	Plane<double> plane1(Vector3d(0, 0, 0), Vector3d(1, 0, 0).cross(dir0).normalized());
	plane1.makePrincipal();
	TEST_FALSE(plane1.intersectRay(ray0, hit, SAME_DIST_TOL), "Ray does not intersect parrallel plane");
	TEST_TRUE(distanceFromPlane(hit.hitPt, plane0) < SAME_DIST_TOL, "Intersection point lies on plane?");

	cout << "testRayPlaneIntersect passed\n";

	return true;
}

bool testRayTriIntersectVerts() {
	Vector3d pt0(0, 0, 0);
	Vector3d pt1(1, 0, 0);
	Vector3d pt2(1, 1, 0);
	Vector3d* pts[] = {
		&pt0, &pt1, &pt2
	};

	Vector3d pt, dir(0, 1.0 / 3.0, 1), ctr(0, 0, 0);
	dir.normalize();
	RayHit<double> hit;
	for (int i = 0; i < 3; i++) {
		ctr += *pts[i];
	}
	ctr /= 3;

	for (int i = 0; i < 3; i++) {
		pt = *pts[i];
		Ray<double> ray(pt - dir, dir);
		TEST_TRUE(intersectRayTri(ray, pt0, pt1, pt2, hit), "Test on vert " + to_string(i));
	}

	for (int i = 0; i < 3; i++) {
		pt = *pts[i];
		Vector3d v = (ctr - pt).normalized();
		pt += SAME_DIST_TOL * v;
		Ray<double> ray(pt - dir, dir);
		TEST_TRUE(intersectRayTri(ray, pt0, pt1, pt2, hit), "Test inside vert " + to_string(i));
	}

	for (int i = 0; i < 3; i++) {
		pt = *pts[i];
		Vector3d v = (ctr - pt);
		Vector3d v2 = (pt - *pts[(i + 1) % 3]).normalized();
		v = (v - v2.dot(v) * v2).normalized();
		pt -= SAME_DIST_TOL * v;
		Ray<double> ray(pt - dir, dir);
		TEST_TRUE(intersectRayTri(ray, pt0, pt1, pt2, hit), "Test outside vert within tol " + to_string(i));
	}

	for (int i = 0; i < 3; i++) {
		pt = *pts[i];
		Vector3d v = (ctr - pt);
		Vector3d v2 = (pt - *pts[(i + 1) % 3]).normalized();
		v = (v - v2.dot(v) * v2).normalized();
		pt -= 2 * SAME_DIST_TOL * v;
		Ray<double> ray(pt - dir, dir);
		TEST_FALSE(intersectRayTri(ray, pt0, pt1, pt2, hit), "Test outside vert " + to_string(i));
	}

	cout << "testRayTriIntersectVerts passed\n";
	return true;
}

bool testRayTriIntersectEdges() {
	Vector3d pt0(0, 0, 0);
	Vector3d pt1(1, 0, 0);
	Vector3d pt2(1, 1, 0);
	Vector3d* pts[] = {
		&pt0, &pt1, &pt2
	};

	Vector3d pt, dir(0, 0, 1);
	RayHit<double> hit;

	for (int i = 0; i < 3; i++) {
		pt = (*pts[i] + *pts[(i + 1) % 3]) / 2;
		Ray<double> ray(pt - dir, dir);
		if (!intersectRayTri(ray, pt0, pt1, pt2, hit)) return 1;
	}

	cout << "testRayTriIntersectEdges passed\n";
	return true;
}

bool testRayTriIntersect() {
	Vector3d pt0(0, 0, 0);
	Vector3d pt1(1, 0, 0);
	Vector3d pt2(1, 1, 0);
	Vector3d* pts[] = {
		&pt0, &pt1, &pt2
	};

	int steps = 10;
	for (int i = 0; i < steps; i++) {
		for (int j = 0; j < steps; j++) {
			for (int k = 0; k < steps; k++) {
				Vector3d dir(.01 + i/(steps - 1.0), .01 + j / (steps - 1.0), .01 + k / (steps - 1.0));
				dir.normalize();
				Ray<double> ray(*pts[0] - dir, dir);
				RayHit<double> hit;

				if (!intersectRayTri(ray, pt0, pt1, pt2, hit)) return false;
				if ((hit.hitPt - pt0).norm() > SAME_DIST_TOL) return false;

				ray._origin = pt0 + Vector3d(-0.1, 0, 0) - dir;
				if (intersectRayTri(ray, pt0, pt1, pt2, hit)) return false;

				ray._origin = pt0 + Vector3d(0.1, 0, 0) - dir;
				if (!intersectRayTri(ray, pt0, pt1, pt2, hit)) return false;

				ray._origin = pt0 + Vector3d(0, -2 * SAME_DIST_TOL, 0) - dir;
				if (intersectRayTri(ray, pt0, pt1, pt2, hit)) return false;

				ray._origin = pt0 + Vector3d(0, -0.5 * SAME_DIST_TOL, 0) - dir;
				if (!intersectRayTri(ray, pt0, pt1, pt2, hit)) return false;

				ray._origin[1] = 0.5 * SAME_DIST_TOL;
				if (intersectRayTri(ray, pt0, pt1, pt2, hit)) return false;

			}
		}
	}

	cout << "testRayTriIntersect passed\n";
	return true;
}

template<class T>
bool testVectorRotation()
{
	Vector3<T> v2;
	Vector3<T> origin[] = { 
		Vector3<T>(0, 0, 0),
		Vector3<T>(1, 1, 1),
	};
	Vector3<T> axisX(1, 0, 0);
	Vector3<T> axisY(0, 1, 0);
	Vector3<T> axisZ(0, 0, 1);
	T angle = 90;

	v2 = rotateVectorAboutAxis<T>(Ray<T>(origin[0], axisX), Vector3<T>(0, 1, 0), angle);
	TEST_TRUE(equalTol<T>(v2, Vector3<T>(0, 0, 1)), "rotate x");
	v2 = rotateVectorAboutAxis<T>(Ray<T>(origin[0], axisX), Vector3<T>(0, 1, 0), -angle);
	TEST_TRUE(equalTol<T>(v2, Vector3<T>(0, 0, -1)), "rotate x");

	v2 = rotateVectorAboutAxis<T>(Ray<T>(origin[0], axisY), Vector3<T>(0, 0, 1), angle);
	TEST_TRUE(equalTol<T>(v2, Vector3<T>(1, 0, 0)), "rotate y");
	v2 = rotateVectorAboutAxis<T>(Ray<T>(origin[0], axisY), Vector3<T>(0, 0, 1), -angle);
	TEST_TRUE(equalTol<T>(v2, Vector3<T>(-1, 0, 0)), "rotate y");

	v2 = rotateVectorAboutAxis<T>(Ray<T>(origin[0], axisZ), Vector3<T>(1, 0, 0), angle);
	TEST_TRUE(equalTol<T>(v2, Vector3<T>(0, 1, 0)), "rotate z");
	v2 = rotateVectorAboutAxis<T>(Ray<T>(origin[0], axisZ), Vector3<T>(1, 0, 0), -angle);
	TEST_TRUE(equalTol<T>(v2, Vector3<T>(0, -1, 0)), "rotate z");


	v2 = rotateVectorAboutAxis<T>(Ray<T>(origin[1], axisX), Vector3<T>(0, 1, 0), angle);
	TEST_TRUE(equalTol<T>(v2, Vector3<T>(0, 0, 1)), "rotate x");
	v2 = rotateVectorAboutAxis<T>(Ray<T>(origin[1], axisX), Vector3<T>(0, 1, 0), -angle);
	TEST_TRUE(equalTol<T>(v2, Vector3<T>(0, 0, -1)), "rotate x");

	v2 = rotateVectorAboutAxis<T>(Ray<T>(origin[1], axisY), Vector3<T>(0, 0, 1), angle);
	TEST_TRUE(equalTol<T>(v2, Vector3<T>(1, 0, 0)), "rotate y");
	v2 = rotateVectorAboutAxis<T>(Ray<T>(origin[1], axisY), Vector3<T>(0, 0, 1), -angle);
	TEST_TRUE(equalTol<T>(v2, Vector3<T>(-1, 0, 0)), "rotate y");

	v2 = rotateVectorAboutAxis<T>(Ray<T>(origin[1], axisZ), Vector3<T>(1, 0, 0), angle);
	TEST_TRUE(equalTol<T>(v2, Vector3<T>(0, 1, 0)), "rotate z");
	v2 = rotateVectorAboutAxis<T>(Ray<T>(origin[1], axisZ), Vector3<T>(1, 0, 0), -angle);
	TEST_TRUE(equalTol<T>(v2, Vector3<T>(0, -1, 0)), "rotate z");


	cout << "testVectorRotation passed\n";
	return true;
}

template<class T>
bool testPointRotation()
{
	Vector3<T> v2;
	Vector3<T> origin[] = {
		Vector3<T>(0, 0, 0),
		Vector3<T>(1, 1, 1),
	};
	Vector3<T> axisX(1, 0, 0);
	Vector3<T> axisY(0, 1, 0);
	Vector3<T> axisZ(0, 0, 1);
	T angle = 90;

	v2 = rotatePointAboutAxis<T>(Ray<T>(origin[0], axisX), Vector3<T>(0, 1, 0), angle);
	TEST_TRUE(equalTol<T>(v2, Vector3<T>(0, 0, 1)), "rotate x");
	v2 = rotatePointAboutAxis<T>(Ray<T>(origin[0], axisX), Vector3<T>(0, 1, 0), -angle);
	TEST_TRUE(equalTol<T>(v2, Vector3<T>(0, 0, -1)), "rotate x");

	v2 = rotatePointAboutAxis<T>(Ray<T>(origin[0], axisY), Vector3<T>(0, 0, 1), angle);
	TEST_TRUE(equalTol<T>(v2, Vector3<T>(1, 0, 0)), "rotate y");
	v2 = rotatePointAboutAxis<T>(Ray<T>(origin[0], axisY), Vector3<T>(0, 0, 1), -angle);
	TEST_TRUE(equalTol<T>(v2, Vector3<T>(-1, 0, 0)), "rotate y");

	v2 = rotatePointAboutAxis<T>(Ray<T>(origin[0], axisZ), Vector3<T>(1, 0, 0), angle);
	TEST_TRUE(equalTol<T>(v2, Vector3<T>(0, 1, 0)), "rotate z");
	v2 = rotatePointAboutAxis<T>(Ray<T>(origin[0], axisZ), Vector3<T>(1, 0, 0), -angle);
	TEST_TRUE(equalTol<T>(v2, Vector3<T>(0, -1, 0)), "rotate z");


	v2 = rotatePointAboutAxis<T>(Ray<T>(origin[1], axisX), Vector3<T>(0, 1, 0), angle);
	TEST_TRUE(equalTol<T>(v2, Vector3<T>(0, 2, 1)), "rotate x");
	v2 = rotatePointAboutAxis<T>(Ray<T>(origin[1], axisX), Vector3<T>(0, 1, 0), -angle);
	TEST_TRUE(equalTol<T>(v2, Vector3<T>(0, 0, 1)), "rotate x");

	v2 = rotatePointAboutAxis<T>(Ray<T>(origin[1], axisY), Vector3<T>(0, 0, 1), angle);
	TEST_TRUE(equalTol<T>(v2, Vector3<T>(1, 0, 2)), "rotate y");
	v2 = rotatePointAboutAxis<T>(Ray<T>(origin[1], axisY), Vector3<T>(0, 0, 1), -angle);
	TEST_TRUE(equalTol<T>(v2, Vector3<T>(1, 0, 0)), "rotate y");

	v2 = rotatePointAboutAxis<T>(Ray<T>(origin[1], axisZ), Vector3<T>(1, 0, 0), angle);
	TEST_TRUE(equalTol<T>(v2, Vector3<T>(2, 1, 0)), "rotate z");
	v2 = rotatePointAboutAxis<T>(Ray<T>(origin[1], axisZ), Vector3<T>(1, 0, 0), -angle);
	TEST_TRUE(equalTol<T>(v2, Vector3<T>(0, 1, 0)), "rotate z");


	cout << "testPointRotation passed\n";
	return true;
}

bool testMath() {
	TEST_TRUE(testVectorRotation<double>(), "testVectorRotation<double>");
	TEST_TRUE(testPointRotation<double>(), "testPointRotation<double>");

	TEST_TRUE(testVectorRotation<float>(), "testVectorRotation<float>");
	TEST_TRUE(testPointRotation<float>(), "testPointRotation<float>");

	TEST_TRUE(testDistToPlane(), "testDistToPlane");
	TEST_TRUE(testRayPlaneIntersect(), "testRayPlaneIntersect");
	TEST_TRUE(testRayTriIntersectVerts(), "testRayTriIntersectVerts");
	TEST_TRUE(testRayTriIntersectEdges(), "testRayTriIntersectEdges");
	TEST_TRUE(testRayTriIntersect(), "testRayTriIntersect");

	cout << "testMath passed\n";
	return true;

}