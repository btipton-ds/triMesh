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
#include <tm_vector3.h>

#include <iostream>
#include <math.h>

#include <tests.h>

using namespace std;

template<typename NUM_TYPE>
class VectorTests {
	using v3 = Vector3<NUM_TYPE>;
public:

	bool testAll() {
		TEST_TRUE (testAssign(), "Failed testAssign");
		TEST_TRUE(testAdd(), "Failed testAdd");
		TEST_TRUE(testSub(), "Failed testSub");
		TEST_TRUE(testMult(), "Failed testMult");
		TEST_TRUE(testDiv(), "Failed testDiv");
		TEST_TRUE(testDot(), "Failed testDot");
		TEST_TRUE(testCross(), "Failed testCross");

		cout << "All vector3 tests passed\n";
		return true;
	}
	bool testAssign() {
		v3 a(1, 2, 3);

		TEST_EQUAL(a[0], 1, "Assign idx 0 ");
		TEST_EQUAL(a[1], 2, "Assign idx 1 ");
		TEST_EQUAL(a[2], 3, "Assign idx 2 ");

		{
			v3 b;
			b = a;
			TEST_EQUAL(b[0], 1, "Assign b= idx 0 ");
			TEST_EQUAL(b[1], 2, "Assign b= idx 1 ");
			TEST_EQUAL(b[2], 3, "Assign b= idx 2 ");
		}

		{
			v3 b(a);
			TEST_EQUAL(b[0], 1, "Assign b copy constructor idx 0 ");
			TEST_EQUAL(b[1], 2, "Assign b copy constructor idx 1 ");
			TEST_EQUAL(b[2], 3, "Assign b copy constructor idx 2 ");
		}
		return true;
	}

	bool testAdd() {
		{
			v3 a(3, 2, 1), b(1, 2, 3);
			v3 c = a + b;

			TEST_EQUAL(c, v3(4,4,4), "add 4");
		}
		{
			v3 a(2, 3, 4), b(-1, -2, -3);
			v3 c = a + b;

			TEST_EQUAL(c, v3(1, 1, 1), "add 1");
		}
		return true;
	}

	bool testSub() {
		{
			v3 a(1, 2, 3), b(1, 2, 3);
			v3 c = a - b;
			TEST_EQUAL(c, v3(0, 0, 0), "add 0");
		}
		return true;
	}

	bool testMult() {
		{
			v3 a(1, 2, 3);
			v3 c = a * 2;
			TEST_EQUAL(c, v3(2, 4, 6), "post mult");
		}
		{
			v3 a(1, 2, 3);
			NUM_TYPE k = 2;
			v3 c = k * a;
			TEST_EQUAL(c, v3(2, 4, 6), "pre mult");
		}
		return true;
	}

	bool testDiv() {
		{
			v3 a(2, 4, 6);
			v3 c = a / 2;
			TEST_EQUAL(c, v3(1, 2, 3), "div");
		}
		return true;
	}

	bool testDot() {
		{
			v3 a(1, 0, 0), b(2, 0, 0);
			NUM_TYPE c = a.dot(b);
			TEST_EQUAL(c, 2, "dot 1");
		}
		{
			v3 a(1, 0, 0), b(0, 1, 0);
			NUM_TYPE c = a.dot(b);
			TEST_EQUAL(c, 0, "dot 2");
		}
		{
			v3 a(1, 1, 0), b(1, 1, 1);
			NUM_TYPE c = a.dot(b);
			TEST_EQUAL(c, 2, "dot 3");
		}
		{
			v3 a(1, 1, 1), b(1, 1, 1);
			NUM_TYPE c = a.dot(b);
			TEST_EQUAL(c, 3, "dot 4");
		}
		return true;
	}

	bool testCross() {
		{
			v3 a(1, 0, 0), b(1, 0, 0);
			v3 c = a.cross(b);
			TEST_EQUAL(c, v3(0,0,0), "cross 0");
		}
		{
			v3 a(0, 1, 0), b(0, 0, 1);
			v3 c = a.cross(b);
			TEST_EQUAL(c, v3(1, 0, 0), "cross x");
		}
		{
			v3 a(0, 0, 1), b(1, 0, 0);
			v3 c = a.cross(b);
			TEST_EQUAL(c, v3(0, 1, 0), "cross y");
		}
		{
			v3 a(1, 0, 0), b(0, 1, 0);
			v3 c = a.cross(b);
			TEST_EQUAL(c, v3(0, 0, 1), "cross z");
		}
		return true;
	}

};


template<typename NUM_TYPE>
bool testAll() {
	VectorTests<NUM_TYPE> tests;

	return tests.testAll();
}

bool testNormsd() {
	using v3 = Vector3<double>;

	v3 a(1, 1, 1);
	double v = sqrt(3);
	double vInv = 1 / v;
	TEST_EQUAL(a.norm(), v, "test norm double");
	TEST_EQUAL(a.normalized(), v3(vInv, vInv, vInv), "test normalized double");
	TEST_EQUAL(a.squaredNorm(), 3, "test squaredNorm double");

	a.normalize();
	TEST_EQUAL(a, v3(vInv, vInv, vInv), "test normalize double");

	return true;
}

bool testVector3() {
	TEST_TRUE(testAll<float>(), "failed testAll<float>");
	TEST_TRUE(testAll<double>(), "failed testAll<double>");
	TEST_TRUE(testAll<size_t>(), "failed testAll<size_t>");

	TEST_TRUE(testNormsd(), "failed testNormsd");

	return true;
}