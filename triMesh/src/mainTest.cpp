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

#include <iostream>
#include <fstream>

#include <cmath>

#include <tm_fixedMath.h>
#include <tm_boundingBox.h>
#include <tm_ray.h>
#include <triMesh.h>

using namespace std;

#define TEST_TRUE(A, msg) \
if (!(A)) { \
	cout << msg << "\n"; \
	return false; \
}

#define TEST_FALSE(A, msg) \
if ((A)) { \
	cout << msg << "\n"; \
	return false; \
}

#define TEST_EQUAL(A, B, msg) \
if (!((A) == (B))) { \
	cout << msg << "\n"; \
	return false; \
}

#define TEST_NOT_EQUAL(A, B, msg) \
if (!((A) != (B))) { \
	cout << msg << "\n"; \
	return false; \
}

#define TEST_EQUAL_TOL(A, B, msg) \
if (!((fabs((A) - (B)) <= tol()))) { \
	cout << msg << " " << fabs((A) - (B)) << "\n"; \
	return false; \
}

#define TEST_NOT_EQUAL_TOL(A, B, msg) \
if (!((fabs((A) - (B)) > tol()))) { \
	cout << msg << " " << fabs((A) - (B)) << "\n"; \
	return false; \
}


namespace {
	using BB = CBoundingBox3Dd;
}

int testContains() {
	BB bb(Vector3d(0, 0, 0), Vector3d(1, 1, 1));

	TEST_TRUE(bb.contains(bb.getMin()), "Contains it's own min corner?");
	TEST_TRUE(bb.contains(bb.getMax()), "Contains it's own max corner?");
	TEST_TRUE(bb.contains(0.5 * (bb.getMax() + bb.getMax())), "Contains it's own centroid?");
	TEST_FALSE(bb.contains(Vector3d(2, 2, 2)), "Does not contain an outside point?");

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
		cNoTol(a.getMax() + 1.01 * tolx, Vector3d(2, 2, 2)),
		d(Vector3d(0.25, 0.25, 0.25), Vector3d(0.75, 0.75, 0.75));

	TEST_TRUE(a.intersects(a), "Box intersects itself?");
	TEST_TRUE(a.intersects(b), "Box intersects at a corner?");
	TEST_TRUE(a.intersects(c), "Box intersects at a face?");
	TEST_TRUE(a.intersects(cTol), "Box does not intersect at a face within tolerance?");
	TEST_FALSE(a.intersects(cNoTol), "Box does not intersect at a face out of tolerance?");
	TEST_TRUE(a.intersects(d), "Box intersects a box it contains?");
	TEST_TRUE(d.intersects(a), "Box intersects a box which contains it?");

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

	TEST_TRUE(a.intersects(Ray<double>(ctr, Vector3d(1, 0, 0))), "Box centroid in (1, 0, 0) intersects box?");
	TEST_TRUE(a.intersects(Ray<double>(ctr, Vector3d(0, 1, 0))), "Box centroid in (0, 1, 0) intersects box?");
	TEST_TRUE(a.intersects(Ray<double>(ctr, Vector3d(0, 0, 1))), "Box centroid in (0, 0, 1) intersects box?");
	TEST_TRUE(a.intersects(Ray<double>(ctr, Vector3d(1, 1, 1).normalized())), "Box centroid in (1, 1, 1) intersects box?");

	TEST_TRUE(a.intersects(Ray<double>(ctr, Vector3d(-1, 0, 0))), "Box centroid in (-1, 0, 0) intersects box?");
	TEST_TRUE(a.intersects(Ray<double>(ctr, Vector3d(0, -1, 0))), "Box centroid in (0, -1, 0) intersects box?");
	TEST_TRUE(a.intersects(Ray<double>(ctr, Vector3d(0, 0, -1))), "Box centroid in (0, 0, -1) intersects box?");
	TEST_TRUE(a.intersects(Ray<double>(ctr, Vector3d(-1, -1, -1).normalized())), "Box centroid in (-1, -1, -1) intersects box?");

	TEST_TRUE(a.intersects(Ray<double>(pt0, Vector3d(1, 0, 0))), "Box centroid + (1,0,0) in (1, 0, 0) intersects box?");
	TEST_TRUE(a.intersects(Ray<double>(pt0, Vector3d(-1, 0, 0))), "Box centroid + (1,0,0) in (-1, 0, 0) intersects box?");
	TEST_FALSE(a.intersects(Ray<double>(pt0, Vector3d(0, 1, 0))), "Box centroid + (1,0,0) in (0, 1, 0) does not intersect box?");

	TEST_TRUE(a.intersects(Ray<double>(pt0, Vector3d(-1, 1, 0).normalized())), "Box centroid + (1,0,0) in (-1, 1, 0) intersect box?");
	TEST_FALSE(a.intersects(Ray<double>(pt0, Vector3d(-1, 1.1, 0).normalized())), "Box centroid + (1,0,0) in (-1, 1.1, 0) does not intersect box?");

	TEST_TRUE(a.intersects(Ray<double>(pt1, Vector3d(1, 0, 0))), "Box centroid + (1,0.5,0) in (1, 0, 0) intersects box?");
	TEST_TRUE(a.intersects(Ray<double>(pt1, Vector3d(-1, 0, 0))), "Box centroid + (1,0.5,0) in (-1, 0, 0) intersects box?");
	TEST_FALSE(a.intersects(Ray<double>(pt1, Vector3d(0, 1, 0))), "Box centroid + (1,0.5,0) in (0, 1, 0) does not intersect box?");

	TEST_TRUE(a.intersects(Ray<double>(pt2, Vector3d(1, 0, 0))), "Box centroid + (1,0.5 + tol,0) in (1, 0, 0) intersects box?");
	TEST_TRUE(a.intersects(Ray<double>(pt2, Vector3d(-1, 0, 0))), "Box centroid + (1,0.5 + tol,0) in (-1, 0, 0) intersects box?");
	TEST_FALSE(a.intersects(Ray<double>(pt2, Vector3d(0, 1, 0))), "Box centroid + (1,0.5 + tol,0) in (0, 1, 0) does not intersect box?");

	TEST_FALSE(a.intersects(Ray<double>(pt3, Vector3d(1, 0, 0))), "Box centroid + (1,0.5 + 1.01 * tol,0) in (1, 0, 0) does not intersects box?");
	TEST_FALSE(a.intersects(Ray<double>(pt3, Vector3d(-1, 0, 0))), "Box centroid + (1,0.5 + 1.01 * tol,0) in (-1, 0, 0) does not intersects box?");

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
		TEST_TRUE(a.intersects(Ray<double>(pt0 + delta[i] + Vector3d(0, 0.0, 0.0), xAxis)), "Ray hits corner 0 with delta " + to_string(i) + "?");
		TEST_TRUE(a.intersects(Ray<double>(pt0 + delta[i] + Vector3d(0, 1.0, 0.0), xAxis)), "Ray hits corner 1 with delta " + to_string(i) + "?");
		TEST_TRUE(a.intersects(Ray<double>(pt0 + delta[i] + Vector3d(0, 1.0, 1.0), xAxis)), "Ray hits corner 2 with delta " + to_string(i) + "?");
		TEST_TRUE(a.intersects(Ray<double>(pt0 + delta[i] + Vector3d(0, 0.0, 1.0), xAxis)), "Ray hits corner 3 with delta " + to_string(i) + "?");

		TEST_TRUE(a.intersects(Ray<double>(pt0 + delta[i] + Vector3d(0, 0.5, 0.0), xAxis)), "Ray hits edge 0 with delta " + to_string(i) + "?");
		TEST_TRUE(a.intersects(Ray<double>(pt0 + delta[i] + Vector3d(0, 1.0, 0.5), xAxis)), "Ray hits edge 1 with delta " + to_string(i) + "?");
		TEST_TRUE(a.intersects(Ray<double>(pt0 + delta[i] + Vector3d(0, 0.5, 1.0), xAxis)), "Ray hits edge 2 with delta " + to_string(i) + "?");
		TEST_TRUE(a.intersects(Ray<double>(pt0 + delta[i] + Vector3d(0, 0.0, 0.5), xAxis)), "Ray hits edge 3 with delta " + to_string(i) + "?");
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

template<class T>
class Test_double_f
{
public:
	static double tol();
	TriMesh::CMeshPtr makeCylinder(const Vector3d& origin, double height, double radius);

	Test_double_f();
	bool testAll();
	bool testRemoveTri();
	bool testSqueezeEdge();
	bool testTriIntersectBox(const TriMesh::CMeshPtr& pMesh);
	bool testEdgeIntersectBox(const TriMesh::CMeshPtr& pMesh);
	bool testFindTri(const TriMesh::CMeshPtr& pMesh);
	bool testFindEdge(const TriMesh::CMeshPtr& pMesh);

private:
	bool testCreate();
	bool testAssign();
	bool testCompare();
	bool testMath();
};

template<>
double Test_double_f<int16_t>::tol()
{
	return 1.0e-4;
}

template<>
double Test_double_f<int32_t>::tol()
{
	return 1.0e-8;
}

template<>
double Test_double_f<int64_t>::tol()
{
	return 1.0e-10;
}

template<class T>
Test_double_f<T>::Test_double_f()
{
	if (sizeof(T) == 8)
		cout << "Step quanta[64]: " << double_f<T>::stepSize() << "\n";
	else if (sizeof(T) == 4)
		cout << "Step quanta[32]: " << double_f<T>::stepSize() << "\n";
	else if (sizeof(T) == 2)
		cout << "Step quanta[16]: " << double_f<T>::stepSize() << "\n";
	else
		cout << "Step quanta[???]: " << double_f<T>::stepSize() << "\n";
}

template<class T>
bool Test_double_f<T>::testAll()
{
	if (!testCreate()) return false;
	if (!testCompare()) return false;
	if (!testAssign()) return false;
	if (!testMath()) return false;

	return true;
}

template<class T>
bool Test_double_f<T>::testCreate()
{
	{
		double_f<T> val;
		TEST_TRUE(val == 0, "Test_double_f::testCreate 0 failed. not zero on create");
	}
	return true;
}

template<class T>
bool Test_double_f<T>::testAssign()
{
	double_f<T> dst;
	double dr;

	dst = double_f<T>(0.2);
	TEST_EQUAL_TOL(0.2, dst, "Test_double_f::testAssign fail: (1 = 2) == 2.");
	dst = 0.2;
	TEST_EQUAL_TOL(0.2, dst, "Test_double_f::testAssign fail: (1 = 2) == 2.");
	dst = double_f<T>(-0.2);
	TEST_EQUAL_TOL(-0.2, dst, "Test_double_f::testAssign fail: (1 = -2) == -2.");
	dst = -0.2;
	TEST_EQUAL_TOL(-0.2, dst, "Test_double_f::testAssign fail: (1 = -2) == -2.");

	dr = double_f<T>(0.3);
	TEST_EQUAL_TOL(dr, 0.3, "Test_double_f::testAssign fail: double x = double_f(3).");

	double_f<T> r = double_f<T>(0.5) - double_f<T>(0.2);
	dr = r;
	TEST_EQUAL_TOL(dr, 0.3, "Test_double_f::testAssign fail: double x = double_f<T>(5) - double_f<T>(2).");

	double_f<T> df;
	df = 0.23;
	TEST_EQUAL_TOL(df, 0.23, "Test_double_f::testAssign fail: df = 0.23.");

	return true;
}

template<class T>
bool Test_double_f<T>::testCompare()
{
	TEST_TRUE(double_f<T>(-0.5) == double_f<T>(-0.5), "Test_double_f::testCompare fail: -5 == 5.");
	TEST_TRUE(double_f<T>(0.7) == double_f<T>(0.7), "Test_double_f::testCompare fail: 7 == 7.");
	TEST_TRUE(double_f<T>(0.7333) == double_f<T>(0.7333), "Test_double_f::testCompare fail: 7.333 == 7.333.");

	TEST_FALSE(double_f<T>(-0.5) != double_f<T>(-0.5), "Test_double_f::testCompare fail: -5 != 5.");
	TEST_FALSE(double_f<T>(0.7) != double_f<T>(0.7), "Test_double_f::testCompare fail: 7 != 7.");
	TEST_FALSE(double_f<T>(0.7333) != double_f<T>(0.7333), "Test_double_f::testCompare fail: 7.333 != 7.333.");

	TEST_TRUE(double_f<T>(-0.5) < double_f<T>(0.5), "Test_double_f::testCompare fail: -5 < 5.");
	TEST_FALSE(double_f<T>(0.5) < double_f<T>(0.5), "Test_double_f::testCompare fail: 5 < 5.");

	TEST_TRUE(double_f<T>(1.0 / 3) > double_f<T>(-1.0 / 3), "Test_double_f::testCompare fail: 1.0 / 3 > -1.0 / 3");
	TEST_FALSE(double_f<T>(1.0 / 3) > double_f<T>(1.0 / 3), "Test_double_f::testCompare fail: 1.0 / 3 > -1.0 / 3.");

	TEST_TRUE(double_f<T>(-0.5) <= double_f<T>(0.5), "Test_double_f::testCompare fail: -5 <= 5.");
	TEST_TRUE(double_f<T>(0.5) <= double_f<T>(0.5), "Test_double_f::testCompare fail: 5 <= 5.");
	TEST_FALSE(double_f<T>(0.501) <= double_f<T>(0.5), "Test_double_f::testCompare fail: 5.01 <= 5.");

	TEST_TRUE(double_f<T>(0.5) >= double_f<T>(-0.5), "Test_double_f::testCompare fail: 5 >= -5.");
	TEST_TRUE(double_f<T>(0.5) >= double_f<T>(0.5), "Test_double_f::testCompare fail: 5 >= 5.");
	TEST_FALSE(double_f<T>(0.50) >= double_f<T>(0.501), "Test_double_f::testCompare fail: 5 >= 5.01.");

	return true;
}

template<class T>
bool Test_double_f<T>::testMath()
{
	double d = double_f<T>(0.5) - double_f<T>(0.2);
	TEST_EQUAL_TOL(0.3, double_f<T>(0.5) - double_f<T>(0.2), "Test_double_f::testMath 3 == 5 - 2 fail");
	TEST_EQUAL_TOL(double_f<T>(0.20 / 3.0), double_f<T>(0.2) / 3, "Test_double_f::testMath 2.0 / 3.0 == double_f<T>(2) / double_f<T>(3) fail");
	return true;
}

template<class T>
TriMesh::CMeshPtr Test_double_f<T>::makeCylinder(const Vector3d& origin, double height, double radius)
{
	Vector3d ll(origin - Vector3d(-radius, -radius, -height / 2)),
		ur(origin - Vector3d(radius, radius, height / 2));
	TriMesh::CMesh::BoundingBox bbox(ll, ur);
	bbox.grow(0.01);
	TriMesh::CMeshPtr result = make_shared<TriMesh::CMesh>(bbox);

	Vector3d upperOrigin = origin + Vector3d(0, 0, height / 2);
	Vector3d lowerOrigin = origin + Vector3d(0, 0, -height / 2);
	size_t steps = 2 * 360; // 1/2 degree
	for (size_t i = 0; i < steps; i++) {
		double t0 = i / (double) steps;
		double t1 = (i + 1) / (double) steps;
		double theta0 = 2 * M_PI * t0;
		double theta1 = 2 * M_PI * t1;
		Vector3d v0(cos(theta0), sin(theta0), 0);
		Vector3d v1(cos(theta1), sin(theta1), 0);
		Vector3d pt00 = upperOrigin + radius * v0;
		Vector3d pt01 = upperOrigin + radius * v1;

		result->addTriangle(upperOrigin, pt00, pt01);
		Vector3d pt10 = pt00 - Vector3d(0, 0, height);
		Vector3d pt11 = pt01 - Vector3d(0, 0, height);

		result->addTriangle(upperOrigin, pt11, pt10);

		result->addQuad(pt00, pt10, pt11, pt01);
	}

#if 0
	{
		ofstream out("D:/DarkSky/Projects/output/cyl.obj");
		result->dumpObj(out);
	}
#endif

	return result;
}

template<class T>
bool Test_double_f<T>::testRemoveTri()
{
	TriMesh::CMeshPtr pMesh = makeCylinder(Vector3d(0, 0, 0), 1, 2);

	TEST_TRUE(pMesh->testRemoveTri(1), "testRemoveTri(1) failed");
	TEST_TRUE(pMesh->verifyTopology(false), "testRemoveTri::verifyTopology 1 failed");
	TEST_TRUE(pMesh->testRemoveTri(0), "testRemoveTri(0) failed");
	TEST_TRUE(pMesh->verifyTopology(false), "testRemoveTri::verifyTopology 0 failed");
	TEST_TRUE(pMesh->testRemoveTri(pMesh->numTris() - 1), "testRemoveTri(pMesh->numTris() - 1) failed");
	TEST_TRUE(pMesh->verifyTopology(false), "testRemoveTri::verifyTopology n-1 failed");
	TEST_TRUE(pMesh->testRemoveTri(pMesh->numTris() / 2), "testRemoveTri(pMesh->numTris() / 2) failed");
	TEST_TRUE(pMesh->verifyTopology(false), "testRemoveTri::verifyTopology n/2 failed");

	cout << "testRemoveTri passed \n";
	return true;
}

template<class T>
bool Test_double_f<T>::testSqueezeEdge()
{
	TriMesh::CMeshPtr pMesh = makeCylinder(Vector3d(0, 0, 0), 1, 2);

	TEST_TRUE(pMesh->testSqueezeEdge(1), "testSqueezeEdge(1) failed");
	TEST_TRUE(pMesh->verifyTopology(false), "testSqueezeEdge::verifyTopology 1 failed");
	TEST_TRUE(pMesh->testSqueezeEdge(0), "testRemoveTri(0) failed");
	TEST_TRUE(pMesh->verifyTopology(false), "testSqueezeEdge::verifyTopology 0 failed");
	TEST_TRUE(pMesh->testSqueezeEdge(pMesh->numTris() - 1), "testRemoveTri(pMesh->numTris() - 1) failed");
	TEST_TRUE(pMesh->verifyTopology(false), "testSqueezeEdge::verifyTopology n-1 failed");
	TEST_TRUE(pMesh->testSqueezeEdge(pMesh->numTris() / 2), "testRemoveTri(pMesh->numTris() / 2) failed");
	TEST_TRUE(pMesh->verifyTopology(false), "testSqueezeEdge::verifyTopology n/2 failed");

	cout << "testSqueezeEdge passed \n";

	return true;
}


template<class T>
bool Test_double_f<T>::testTriIntersectBox(const TriMesh::CMeshPtr& pMesh)
{
	for (size_t i = 0; i < pMesh->numTris(); i++) {
		auto bbox = pMesh->getTriBBox(i);
		TEST_TRUE(pMesh->bboxIntersectsTri(bbox, i), "Tri intersect bbox failed");

		vector<TriMesh::CMesh::BoundingBox> boxes, tmp;
		TriMesh::CMesh::BoundingBox a, b;
		bbox.split(0, a, b);
		tmp.push_back(a);
		tmp.push_back(b);
		for (const auto& bb : tmp) {
			bb.split(1, a, b);
			boxes.push_back(a);
			boxes.push_back(b);
		}

		tmp = boxes;
		boxes.clear();
		for (const auto& bb : tmp) {
			bb.split(2, a, b);
			boxes.push_back(a);
			boxes.push_back(b);
		}

		int numIntersects = 0;
		for (const auto& bb : boxes) {
			if (pMesh->bboxIntersectsTri(bb, i))
				numIntersects++;
		}
		TEST_TRUE(numIntersects > 0, "Tri intersect sub bbox failed");
	}

	cout << "testTriIntersectBox passed \n";
	return true;
}

template<class T>
bool Test_double_f<T>::testEdgeIntersectBox(const TriMesh::CMeshPtr& pMesh)
{
	for (size_t i = 0; i < pMesh->numEdges(); i++) {
		TriMesh::CMesh::BoundingBox bbox = pMesh->getEdgeBBox(i);
		TEST_TRUE(pMesh->bboxIntersectsEdge(bbox, i), "Edge intersect bbox failed");

		vector<TriMesh::CMesh::BoundingBox> boxes, tmp;
		TriMesh::CMesh::BoundingBox a, b;
		bbox.split(0, a, b);
		tmp.push_back(a);
		tmp.push_back(b);
		for (const auto& bb : tmp) {
			bb.split(1, a, b);
			boxes.push_back(a);
			boxes.push_back(b);
		}

		tmp = boxes;
		boxes.clear();
		for (const auto& bb : tmp) {
			bb.split(2, a, b);
			boxes.push_back(a);
			boxes.push_back(b);
		}

		int numIntersects = 0;
		for (const auto& bb : boxes) {
			if (pMesh->bboxIntersectsEdge(bb, i))
				numIntersects++;
		}
		TEST_TRUE(numIntersects > 0, "Edge intersect sub bbox failed");
	}

	cout << "testEdgeIntersectBox passed \n";
	return true;
}

template<class T>
bool Test_double_f<T>::testFindTri(const TriMesh::CMeshPtr& pMesh)
{
	for (size_t i = 0; i < pMesh->numTris(); i += 20) {
		auto bbox = pMesh->getTriBBox(i);
		vector<size_t> indices;
		TEST_TRUE(pMesh->findTris(bbox, indices) > 0, "Failed to find triangle");
		auto iter = find(indices.begin(), indices.end(), i);
		TEST_TRUE(iter != indices.end(), "Triangle not in list");
	}

	for (size_t i = 0; i < pMesh->numTris(); i += 20) {
		auto bbox = pMesh->getTriBBox(i);
		vector<TriMesh::CMesh::SearchEntry> indices;
		TEST_TRUE(pMesh->findTris(bbox, indices) > 0, "Failed to find triangle");
		bool found = false;
		for (const auto entry : indices) {
			if (entry.getIndex() == i) {
				found = true;
				break;
			}
		}
		TEST_TRUE(found, "Triangle not in list");
	}

	cout << "testFindTri passed \n";
	return true;
}

template<class T>
bool Test_double_f<T>::testFindEdge(const TriMesh::CMeshPtr& pMesh)
{
	for (size_t i = 0; i < pMesh->numEdges(); i += 20) {
		auto bbox = pMesh->getEdgeBBox(i);
		vector<size_t> indices;
		TEST_TRUE(pMesh->findEdges(bbox, indices) > 0, "Failed to find edge");
		auto iter = find(indices.begin(), indices.end(), i);
		TEST_TRUE(iter != indices.end(), "Edge not in list");
	}

	for (size_t i = 0; i < pMesh->numEdges(); i += 20) {
		auto bbox = pMesh->getEdgeBBox(i);
		vector<TriMesh::CMesh::SearchEntry> indices;
		TEST_TRUE(pMesh->findEdges(bbox, indices) > 0, "Failed to find edge");
		bool found = false;
		for (const auto entry : indices) {
			if (entry.getIndex() == i) {
				found = true;
				break;
			}
		}
		TEST_TRUE(found, "Edge not in list");
	}

	cout << "testFindEdge passed \n";
	return true;
}

int main(int numArgs, char** args)
{
	{
		Test_double_f<int64_t> test64;
		TriMesh::CMeshPtr pMesh = test64.makeCylinder(Vector3d(0, 0, 0), 1, 2);

		if (!test64.testTriIntersectBox(pMesh)) return 1;
		if (!test64.testEdgeIntersectBox(pMesh)) return 1;
		if (!test64.testFindTri(pMesh)) return 1;
		if (!test64.testFindEdge(pMesh)) return 1;
	}
	if (testBoundingBox() != 0)
		cout << "Bounding box failed\n";

	Test_double_f<int64_t> test64;
	if (!test64.testAll()) return 1;

	if (!test64.testRemoveTri()) return 1;
	if (!test64.testSqueezeEdge()) return 1;


	cout << "Passed int64_t tests\n";

	Test_double_f<int32_t> test32;
	if (!test32.testAll()) return 1;
	cout << "Passed int32_t tests\n";

	Test_double_f<int16_t> test16;
	if (!test16.testAll()) return 1;
	cout << "Passed int16_t tests\n";

	cout << "Passed all tests\n";

	return 0;
}
