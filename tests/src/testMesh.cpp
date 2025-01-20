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
#include <fstream>

#include <triMesh.hpp>
#include <..\..\stlReader\include\readWriteStl.h>

using namespace std;

void makeSphere(int numSteps, double r, double phiRange, TriMesh::CMesh& mesh) {
	for (int j = 0; j < numSteps; j++) {
		double phi0 = EIGEN_PI * (-0.5 + j / ((double)numSteps) * phiRange);
		double phi1 = EIGEN_PI * (-0.5 + (j + 1) / ((double)numSteps) * phiRange);
		for (int i = 0; i < numSteps; i++) {
			double theta0 = 2.0 * EIGEN_PI * i / ((double)numSteps);
			double theta1 = 2.0 * EIGEN_PI * (i + 1) / ((double)numSteps);

			Vector3d pts[4];
			pts[0] = r * Vector3d(cos(theta0) * cos(phi0), sin(theta0) * cos(phi0), sin(phi0));
			pts[1] = r * Vector3d(cos(theta1) * cos(phi0), sin(theta1) * cos(phi0), sin(phi0));
			pts[2] = r * Vector3d(cos(theta1) * cos(phi1), sin(theta1) * cos(phi1), sin(phi1));
			pts[3] = r * Vector3d(cos(theta0) * cos(phi1), sin(theta0) * cos(phi1), sin(phi1));

			mesh.addQuad(pts[0], pts[1], pts[2], pts[3]);
		}
	}
}

bool test0() {
	TriMesh::CMesh mesh(Vector3d(-2, -2, -2), Vector3d(2, 2, 2));
	makeSphere(10, 2.0, 1, mesh);

	TEST_EQUAL(mesh.numVertices(), 92, "Correct number of vertices?");
	TEST_EQUAL(mesh.numEdges(), 270, "Correct number of edges?");
	TEST_EQUAL(mesh.numTris(), 180, "Correct number of triangles?");
	TEST_TRUE(mesh.isClosed(), "Mesh is closed?");

	cout << "Test Mesh 0 passed\n";

	return true;
}

bool test1() {
	TriMesh::CMesh mesh(Vector3d(-2, -2, -2), Vector3d(2, 2, 2));
	makeSphere(100, 2.0, 1, mesh);

	TEST_EQUAL(mesh.numVertices(), 9902, "Correct number of vertices?");
	TEST_EQUAL(mesh.numEdges(), 29700, "Correct number of edges?");
	TEST_EQUAL(mesh.numTris(), 19800, "Correct number of triangles?");
	TEST_TRUE(mesh.isClosed(), "Mesh is closed?");

	cout << "Test Mesh 1 passed\n";

	return true;
}

bool test2() {
	TriMesh::CMesh mesh(Vector3d(-2, -2, -2), Vector3d(2, 2, 2));
	makeSphere(10, 2.0, 0.5, mesh);

	TEST_EQUAL(mesh.numVertices(), 101, "Correct number of vertices?");
	TEST_EQUAL(mesh.numEdges(), 290, "Correct number of edges?");
	TEST_EQUAL(mesh.numTris(), 190, "Correct number of triangles?");
	TEST_FALSE(mesh.isClosed(), "Mesh is open?");

	cout << "Test Mesh 2 passed\n";

	return true;
}

bool testSplit1()
{
	int steps = 9;

	const std::string path("D:/DarkSky/Projects/output/");
	for (int i = 0; i < steps; i++) {
		Vector3d pts[] = {
			Vector3d(-1, 0, 0),
			Vector3d(1, 0, 0),
			Vector3d(0.8, 0.1, 0),
		};
		CBoundingBox3Dd bbox;
		for (int i = 0; i < 3; i++)
			bbox.merge(pts[i]);
		TriMesh::CMeshPtr pMesh= make_shared<TriMesh::CMesh>(bbox);

		pMesh->addTriangle(pts);
		
		double l = 1.99 / (i + 1);
		switch (i) {
		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
			break;
		case 5:
			l = 1.99 / (2 * 5);
			break;
		case 6:
			l = 1.99 / (4 * 5);
			break;
		case 7:
			l = 1.99 / (8 * 5);
			break;
		case 8:
			l = 1.99 / (16 * 5);
			break;
		case 9:
			l = 1.99 / (32 * 5);
			break;
		}

		if (i == 6) {
			int dbgBreak = 1;
		}

		pMesh->splitLongTris(l);
		{
			CReadWriteSTL writer(pMesh);

			std::stringstream ss;
			ss << "testSplit1_" << i << ".stl";
			std::string filename = ss.str();
			writer.write(pMesh, false, path, filename);
		}

		for (size_t i = 0; i < pMesh->numEdges(); i++) {
			auto edgeLen = pMesh->edgeLength(i);
			//TEST_TRUE(edgeLen < l, "Edge to long");
		}
		switch (i) {
		case 0:
			TEST_EQUAL(pMesh->numTris(), 2, "Expected 2 tris");
			break;
		case 1:
			TEST_EQUAL(pMesh->numTris(), 6, "Expected 6 tris");
			break;
		case 2:
			TEST_EQUAL(pMesh->numTris(), 9, "Expected 9 tris");
			break;
		case 3:
			TEST_EQUAL(pMesh->numTris(), 12, "Expected 12 tris");
			break;
		case 4:
			TEST_EQUAL(pMesh->numTris(), 14, "Expected 14 tris");
			break;
		case 5:
			TEST_EQUAL(pMesh->numTris(), 30, "Expected 30 tris");
			break;
		case 6:
			TEST_EQUAL(pMesh->numTris(), 64, "Expected 64 tris");
			break;
		case 7:
			TEST_EQUAL(pMesh->numTris(), 229, "Expected 229 tris");
			break;
		case 8:
			TEST_EQUAL(pMesh->numTris(), 844, "Expected 844 tris");
			break;
		case 9:
			TEST_EQUAL(pMesh->numTris(), 9, "Expected 9 tris");
			break;
		}
	}


	cout << "Test testSplit passed\n";

	return true;

}

bool testSplit2()
{
	int steps = 9;

	const std::string path("D:/DarkSky/Projects/output/");
	for (int i = 0; i < steps; i++) {
		Vector3d pts0[] = {
			Vector3d(-1, 0, 0),
			Vector3d(1, 0, 0),
			Vector3d(0.8, 0.1, 0),
		};

		Vector3d pts1[] = {
			Vector3d(-1, 0, 0),
			Vector3d(-0.8, -0.25, 0),
			Vector3d(1, 0, 0),
		};

		CBoundingBox3Dd bbox;
		for (int i = 0; i < 3; i++)
			bbox.merge(pts0[i]);

		for (int i = 0; i < 3; i++)
			bbox.merge(pts1[i]);

		TriMesh::CMeshPtr pMesh = make_shared<TriMesh::CMesh>(bbox);

		pMesh->addTriangle(pts0);
		pMesh->addTriangle(pts1);

		double l = 1.99 / (i + 1);
		switch (i) {
		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
			break;
		case 5:
			l = 1.99 / (2 * 5);
			break;
		case 6:
			l = 1.99 / (4 * 5);
			break;
		case 7:
			l = 1.99 / (8 * 5);
			break;
		case 8:
			l = 1.99 / (16 * 5);
			break;
		case 9:
			l = 1.99 / (32 * 5);
			break;
		}

		if (i == 6) {
			int dbgBreak = 1;
		}

		pMesh->splitLongTris(l);
		{
			CReadWriteSTL writer(pMesh);

			std::stringstream ss;
			ss << "testSplit2_" << i << ".stl";
			std::string filename = ss.str();
			writer.write(pMesh, false, path, filename);
		}

		for (size_t i = 0; i < pMesh->numEdges(); i++) {
			auto edgeLen = pMesh->edgeLength(i);
			//TEST_TRUE(edgeLen < l, "Edge to long");
		}
		switch (i) {
		case 0:
			TEST_EQUAL(pMesh->numTris(), 4, "Expected 4 tris");
			break;
		case 1:
			TEST_EQUAL(pMesh->numTris(), 12, "Expected 12 tris");
			break;
		case 2:
			TEST_EQUAL(pMesh->numTris(), 18, "Expected 18 tris");
			break;
		case 3:
			TEST_EQUAL(pMesh->numTris(), 24, "Expected 24 tris");
			break;
		case 4:
			TEST_EQUAL(pMesh->numTris(), 28, "Expected 28 tris");
			break;
		case 5:
			TEST_EQUAL(pMesh->numTris(), 72, "Expected 72 tris");
			break;
		case 6:
			TEST_EQUAL(pMesh->numTris(), 213, "Expected 213 tris");
			break;
		case 7:
			TEST_EQUAL(pMesh->numTris(), 780, "Expected 780 tris");
			break;
		case 8:
			TEST_EQUAL(pMesh->numTris(), 2942, "Expected 2942 tris");
			break;
		case 9:
			TEST_EQUAL(pMesh->numTris(), 9, "Expected 9 tris");
			break;
		}
	}


	cout << "Test testSplit passed\n";

	return true;

}

bool testSplit()
{
	if (!testSplit1())
		return false;

	if (!testSplit2())
		return false;

	return true;
}

bool testMesh() {
	TEST_TRUE(test0(), "Test0");
	TEST_TRUE(test1(), "Test1");
	TEST_TRUE(test2(), "Test2");
	TEST_TRUE(testSplit(), "TestSplit");

	cout << "Test mesh passed\n";

	return true;
}