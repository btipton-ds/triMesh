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
#include <tm_vector2.h>

#include <iostream>
#include <math.h>

#include <tests.h>

using namespace std;

namespace {
	template<typename NUM_TYPE>
	class VectorTests {
		using v2 = Vector2<NUM_TYPE>;
	public:

		bool testAll() {
			TEST_TRUE(testAssign(), "Failed testAssign");
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
			v2 a(1, 2);

			TEST_EQUAL(a[0], 1, "Assign idx 0 ");
			TEST_EQUAL(a[1], 2, "Assign idx 1 ");

			{
				v2 b;
				b = a;
				TEST_EQUAL(b[0], 1, "Assign b= idx 0 ");
				TEST_EQUAL(b[1], 2, "Assign b= idx 1 ");
			}

			{
				v2 b(a);
				TEST_EQUAL(b[0], 1, "Assign b copy constructor idx 0 ");
				TEST_EQUAL(b[1], 2, "Assign b copy constructor idx 1 ");
			}
			return true;
		}

		bool testAdd() {
			{
				v2 a(3, 2), b(1, 2);
				v2 c = a + b;

				TEST_EQUAL(c, v2(4, 4), "add 4");
			}
			{
				v2 a(2, 3), b(-1, -2);
				v2 c = a + b;

				TEST_EQUAL(c, v2(1, 1), "add 1");
			}
			return true;
		}

		bool testSub() {
			{
				v2 a(1, 2), b(1, 2);
				v2 c = a - b;
				TEST_EQUAL(c, v2(0, 0), "add 0");
			}
			return true;
		}

		bool testMult() {
			{
				v2 a(1, 2);
				v2 c = a * 2;
				TEST_EQUAL(c, v2(2, 4), "post mult");
			}
			{
				v2 a(1, 2);
				NUM_TYPE k = 2;
				v2 c = k * a;
				TEST_EQUAL(c, v2(2, 4), "pre mult");
			}
			return true;
		}

		bool testDiv() {
			{
				v2 a(2, 4);
				v2 c = a / 2;
				TEST_EQUAL(c, v2(1, 2), "div");
			}
			return true;
		}

		bool testDot() {
			{
				v2 a(1, 0), b(2, 0);
				NUM_TYPE c = a.dot(b);
				TEST_EQUAL(c, 2, "dot 1");
			}
			{
				v2 a(1, 0), b(0, 1);
				NUM_TYPE c = a.dot(b);
				TEST_EQUAL(c, 0, "dot 2");
			}
			{
				v2 a(1, 1), b(1, 1);
				NUM_TYPE c = a.dot(b);
				TEST_EQUAL(c, 2, "dot 3");
			}
			{
				v2 a(1, 1), b(1, 1);
				NUM_TYPE c = a.dot(b);
				TEST_EQUAL(c, 2, "dot 4");
			}
			return true;
		}

		bool testCross() {
			{
				v2 a(1, 0), b(1, 0);
				auto c = a.cross(b);
				TEST_EQUAL(c, 0, "cross 0");
			}
			{
				v2 a(0, 1), b(0, 0);
				auto c = a.cross(b);
				TEST_EQUAL(c, 0, "cross 0");
			}
			{
				v2 a(1, 0), b(0, 1);
				auto c = a.cross(b);
				TEST_EQUAL(c, 1, "cross 1");
			}
			{
				v2 a(0, 1), b(1, 0);
				auto c = a.cross(b);
				TEST_EQUAL(c, -1, "cross -1");
			}
			return true;
		}

	};


	template<typename NUM_TYPE>
	bool testAll() {
		VectorTests<NUM_TYPE> tests;

		return tests.testAll();
	}

	bool testNorms2d() {
		using v2 = Vector2<double>;

		v2 a(1, 1);
		double v = sqrt(2);
		double vInv = 1 / v;
		TEST_EQUAL(a.norm(), v, "test norm double");
		TEST_EQUAL(a.normalized(), v2(vInv, vInv), "test normalized double");
		TEST_EQUAL(a.squaredNorm(), 2, "test squaredNorm double");

		a.normalize();
		TEST_EQUAL(a, v2(vInv, vInv), "test normalize double");

		return true;
	}
}

bool testVector2() {
	TEST_TRUE(testAll<float>(), "failed testAll<float>");
	TEST_TRUE(testAll<double>(), "failed testAll<double>");
	TEST_TRUE(testAll<size_t>(), "failed testAll<size_t>");

	TEST_TRUE(testNorms2d(), "failed testNormsd");

	return true;
}