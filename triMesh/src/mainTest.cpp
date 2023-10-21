#include <iostream>
#include <tm_fixedMath.h>

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

template<class T>
class Test_double_f
{
public:
	static double tol();
	Test_double_f();
	bool testAll();

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
		cout << "Stap quanta[64]: " << double_f<T>::stepSize() << "\n";
	else if (sizeof(T) == 4)
		cout << "Stap quanta[32]: " << double_f<T>::stepSize() << "\n";
	else if (sizeof(T) == 2)
		cout << "Stap quanta[16]: " << double_f<T>::stepSize() << "\n";
	else
		cout << "Stap quanta[???]: " << double_f<T>::stepSize() << "\n";
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

int main(int numArgs, char** args)
{
	Test_double_f<int64_t> test64;
	if (!test64.testAll()) return 1;
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
