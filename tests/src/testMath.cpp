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


template<class T>
bool testDistToPlane() {
	const T tol = sameDistTol<T>();
	Plane<T> plane(Vector3<T>(0, 0, 0), Vector3<T>(0, 0, 1));
	plane.makePrincipal();
	Vector3<T> pt0(0, 0, 1);
	Vector3<T> pt1(0, 0, -1);
	Vector3<T> pt2(1, 1, 1);

	TEST_TRUE(distanceFromPlane(pt0, plane) - 1 < tol, "Point is 1 unit from plane?");
	TEST_TRUE(distanceFromPlane(pt1, plane) - -1 < tol, "Point is -1 unit from plane?");
	TEST_TRUE(distanceFromPlane(pt2, plane) - 1 < tol, "Point is 1 unit from plane?");

	return true;
}

template<class T>
bool testRayPlaneIntersect() {
	const T tol = sameDistTol<T>();
	Plane<T> plane0(Vector3<T>(0, 0, 0), Vector3<T>(0, 0, 1));
	plane0.makePrincipal();
	Vector3<T> dir0(-1, -1, -1);
	Ray<T> ray0(plane0.getOrgin() - dir0, dir0);
	RayHit<T> hit;

	TEST_TRUE(plane0.intersectRay(ray0, hit, tol), "Ray intersects plane?");
	TEST_TRUE(distanceFromPlane(hit.hitPt, plane0) < tol, "Intersection point lies on plane?");

	Vector3<T> delta((T).1, (T)-.023, (T)1.35);
	ray0._origin += delta;
	TEST_TRUE(plane0.intersectRay(ray0, hit, tol), "Ray + delta intersects plane?");
	TEST_TRUE(distanceFromPlane(hit.hitPt, plane0) < tol, "Intersection point lies on plane?");

	Plane<T> plane1(Vector3<T>(0, 0, 0), Vector3<T>(1, 0, 0).cross(dir0).normalized());
	plane1.makePrincipal();
	TEST_FALSE(plane1.intersectRay(ray0, hit, tol), "Ray does not intersect parrallel plane");
	TEST_TRUE(distanceFromPlane(hit.hitPt, plane0) < tol, "Intersection point lies on plane?");

	cout << "testRayPlaneIntersect passed\n";

	return true;
}

template<class T>
bool testRayTriIntersectVerts() {
	const T tol = sameDistTol<T>();

	Vector3<T> pt0(0, 0, 0);
	Vector3<T> pt1(1, 0, 0);
	Vector3<T> pt2(1, 1, 0);
	Vector3<T>* pts[] = {
		&pt0, &pt1, &pt2
	};

	Vector3<T> pt, dir(0, (T)(1.0 / 3.0), 1), ctr(0, 0, 0);
	dir.normalize();
	RayHit<T> hit;
	for (int i = 0; i < 3; i++) {
		ctr += *pts[i];
	}
	ctr /= 3;

	for (int i = 0; i < 3; i++) {
		pt = *pts[i];
		Ray<T> ray(pt - dir, dir);
		TEST_TRUE(intersectRayTri(ray, pt0, pt1, pt2, hit, tol), "Test on vert " + to_string(i));
	}

	for (int i = 0; i < 3; i++) {
		pt = *pts[i];
		Vector3<T> v = (ctr - pt).normalized();
		pt += tol * v;
		Ray<T> ray(pt - dir, dir);
		TEST_TRUE(intersectRayTri(ray, pt0, pt1, pt2, hit, tol), "Test inside vert " + to_string(i));
	}

	for (int i = 0; i < 3; i++) {
		pt = *pts[i];
		Vector3<T> v = (ctr - pt);
		Vector3<T> v2 = (pt - *pts[(i + 1) % 3]).normalized();
		v = (v - v2.dot(v) * v2).normalized();
		pt -= (tol - tol / 10) * v; // The floating case needs a little more margin
		Ray<T> ray(pt - dir, dir);
		TEST_TRUE(intersectRayTri(ray, pt0, pt1, pt2, hit, tol), "Test outside vert within tol " + to_string(i));
	}

	for (int i = 0; i < 3; i++) {
		pt = *pts[i];
		Vector3<T> v = (ctr - pt);
		Vector3<T> v2 = (pt - *pts[(i + 1) % 3]).normalized();
		v = (v - v2.dot(v) * v2).normalized();
		pt -= 2 * tol * v;
		Ray<T> ray(pt - dir, dir);
		TEST_FALSE(intersectRayTri(ray, pt0, pt1, pt2, hit, tol), "Test outside vert " + to_string(i));
	}

	cout << "testRayTriIntersectVerts passed\n";
	return true;
}

template<class T>
bool testRayTriIntersectEdges() {
	const T tol = sameDistTol<T>();

	Vector3<T> pt0(0, 0, 0);
	Vector3<T> pt1(1, 0, 0);
	Vector3<T> pt2(1, 1, 0);
	Vector3<T>* pts[] = {
		&pt0, &pt1, &pt2
	};

	Vector3<T> pt, dir(0, 0, 1);
	RayHit<T> hit;

	for (int i = 0; i < 3; i++) {
		pt = (*pts[i] + *pts[(i + 1) % 3]) / 2;
		Ray<T> ray(pt - dir, dir);
		if (!intersectRayTri(ray, pt0, pt1, pt2, hit)) return 1;
	}

	cout << "testRayTriIntersectEdges passed\n";
	return true;
}

template<class T>
bool testRayTriIntersect() {
	const T tol = sameDistTol<T>();

	Vector3<T> pt0(0, 0, 0);
	Vector3<T> pt1(1, 0, 0);
	Vector3<T> pt2(1, 1, 0);
	Vector3<T>* pts[] = {
		&pt0, &pt1, &pt2
	};

	int steps = 10;
	for (int i = 0; i < steps; i++) {
		for (int j = 0; j < steps; j++) {
			for (int k = 0; k < steps; k++) {
				Vector3<T> dir((T)(.01 + i / (steps - 1.0)), (T)(.01 + j / (steps - 1.0)), (T)(.01 + k / (steps - 1.0)));
				dir.normalize();
				Ray<T> ray(*pts[0] - dir, dir);
				RayHit<T> hit;

				TEST_TRUE(intersectRayTri(ray, pt0, pt1, pt2, hit), "");
				TEST_TRUE((hit.hitPt - pt0).norm() < tol, "Hit point outside tolerance");

				ray._origin = pt0 + Vector3<T>((T)-0.1, 0, 0) - dir;
				TEST_FALSE(intersectRayTri(ray, pt0, pt1, pt2, hit), "Ray intersected triangle when it should not");

				ray._origin = pt0 + Vector3<T>((T)0.1, 0, 0) - dir;
				TEST_TRUE(intersectRayTri(ray, pt0, pt1, pt2, hit), "Ray missed triangle when it should hit");

				ray._origin = pt0 + Vector3<T>(0, -2 * tol, 0) - dir;
				TEST_FALSE(intersectRayTri(ray, pt0, pt1, pt2, hit), "Ray intersected triangle when it should not");

				ray._origin = pt0 + Vector3<T>(0, (T)(-0.5 * tol), 0) - dir;
				TEST_TRUE (intersectRayTri(ray, pt0, pt1, pt2, hit), "Ray missed triangle when it should hit");

				ray._origin[1] = (T)(0.5 * tol);
				TEST_FALSE(intersectRayTri(ray, pt0, pt1, pt2, hit), "Ray intersected triangle when it should not");
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
template<class T>
bool testTriangleSplitWithPlane() {
	const auto tol = sameDistTol<T>();

	Vector3<T> triPts[] = {
		Vector3<T>(0, 0, 0),
		Vector3<T>(4, 0, 0),
		Vector3<T>(2, 2, 0),
	};
	Vector3<T> splitTri0[3], splitTri1[3], splitTri2[3];

	int numNewTris;

	Vector3<T> norm(1, 0, 0);

	numNewTris = triangleSplitWithPlane(triPts, Plane<T>(Vector3<T>(-1, 0, 0), norm), splitTri0, splitTri1, splitTri2);
	TEST_EQUAL(numNewTris, 0, "Check num result triangles is 0");

	numNewTris = triangleSplitWithPlane(triPts, Plane<T>(Vector3<T>(5, 0, 0), norm), splitTri0, splitTri1, splitTri2);
	TEST_EQUAL(numNewTris, 0, "Check num result triangles is 0");

	numNewTris = triangleSplitWithPlane(triPts, Plane<T>(Vector3<T>(0, 0, 0), norm), splitTri0, splitTri1, splitTri2);
	TEST_EQUAL(numNewTris, 0, "Check num result triangles is 0");

	numNewTris = triangleSplitWithPlane(triPts, Plane<T>(Vector3<T>(4, 0, 0), norm), splitTri0, splitTri1, splitTri2);
	TEST_EQUAL(numNewTris, 0, "Check num result triangles is 0");

	numNewTris = triangleSplitWithPlane(triPts, Plane<T>(Vector3<T>(2, 0, 0), norm), splitTri0, splitTri1, splitTri2);
	TEST_EQUAL(numNewTris, 2, "Check num result triangles is 2");
	TEST_TRUE(tolerantEquals(splitTri0[0], Vector3<T>(2, 2, 0), tol), "Check vertex 0, 0 A");
	TEST_TRUE(tolerantEquals(splitTri0[1], Vector3<T>(0, 0, 0), tol), "Check vertex 0, 1 A");
	TEST_TRUE(tolerantEquals(splitTri0[2], Vector3<T>(2, 0, 0), tol), "Check vertex 0, 2 A");

	TEST_TRUE(tolerantEquals(splitTri1[0], Vector3<T>(2, 2, 0), tol), "Check vertex 1, 0 A");
	TEST_TRUE(tolerantEquals(splitTri1[1], Vector3<T>(2, 0, 0), tol), "Check vertex 1, 1 A");
	TEST_TRUE(tolerantEquals(splitTri1[2], Vector3<T>(4, 0, 0), tol), "Check vertex 1, 2 A");

	numNewTris = triangleSplitWithPlane(triPts, Plane<T>(Vector3<T>(1, 0, 0), norm), splitTri0, splitTri1, splitTri2);
	TEST_EQUAL(numNewTris, 3, "Check num result triangles is 3");
	TEST_TRUE(tolerantEquals(splitTri0[0], Vector3<T>(0, 0, 0), tol), "Check vertex 0, 0 B");
	TEST_TRUE(tolerantEquals(splitTri0[1], Vector3<T>(1, 0, 0), tol), "Check vertex 0, 1 B");
	TEST_TRUE(tolerantEquals(splitTri0[2], Vector3<T>(1, 1, 0), tol), "Check vertex 0, 2 B");

	TEST_TRUE(tolerantEquals(splitTri1[0], Vector3<T>(1, 0, 0), tol), "Check vertex 1, 0 B");
	TEST_TRUE(tolerantEquals(splitTri1[1], Vector3<T>(4, 0, 0), tol), "Check vertex 1, 1 B");
	TEST_TRUE(tolerantEquals(splitTri1[2], Vector3<T>(2, 2, 0), tol), "Check vertex 1, 2 B");

	TEST_TRUE(tolerantEquals(splitTri2[0], Vector3<T>(1, 0, 0), tol), "Check vertex 2, 0 B");
	TEST_TRUE(tolerantEquals(splitTri2[1], Vector3<T>(2, 2, 0), tol), "Check vertex 2, 1 B");
	TEST_TRUE(tolerantEquals(splitTri2[2], Vector3<T>(1, 1, 0), tol), "Check vertex 2, 2 B");

	cout << "testTriangleSplitWithPlane passed\n";
	return true;
}

template<class T>
bool testTempl() {
	TEST_TRUE(testVectorRotation<T>(), "testVectorRotation");
	TEST_TRUE(testPointRotation<T>(), "testPointRotation");

	TEST_TRUE(testDistToPlane<T>(), "testDistToPlane");
	TEST_TRUE(testRayPlaneIntersect<T>(), "testRayPlaneIntersect");
	TEST_TRUE(testRayTriIntersectEdges<T>(), "testRayTriIntersectEdges");
	TEST_TRUE(testRayTriIntersect<T>(), "testRayTriIntersect");
	TEST_TRUE(testRayTriIntersectVerts<T>(), "testRayTriIntersectVerts");
	TEST_TRUE(testTriangleSplitWithPlane<T>(), "testTriangleSplitWithPlane");

	return true;
}

bool testMath() {
	if (!testTempl<double>())
		return false;
	if (!testTempl<float>())
		return false;
	cout << "testMath passed\n";
	return true;

}