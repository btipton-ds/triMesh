#pragma once

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

/*
	Vector3 is a light weight substitute for Eigen. TriMesh originally used Eigen, but the debugging overhead (both speed and examination in a debugger) made it unweildy.
	At one time a 1-1 substiution was possible. It is desired to retain this feature.
*/

#include <tm_defines.h>

#include <limits>
#include <iostream>

#if !USE_EIGEN_VECTOR3

template<typename SCALAR_TYPE>
class Vector3 {
public:
	using Scalar = SCALAR_TYPE;

	static SCALAR_TYPE defaultVal();

	Vector3();
	Vector3(SCALAR_TYPE x, SCALAR_TYPE y, SCALAR_TYPE z);
	Vector3(const Vector3& src) = default;

	Vector3 operator - () const;

	Vector3 operator - (const Vector3& rhs) const;
	Vector3 operator + (const Vector3& rhs) const;
	Vector3 operator * (SCALAR_TYPE rhs) const;
	Vector3 operator / (SCALAR_TYPE rhs) const;

	Vector3& operator -= (const Vector3& rhs);
	Vector3& operator += (const Vector3& rhs);
	Vector3& operator *= (SCALAR_TYPE rhs);
	Vector3& operator /= (SCALAR_TYPE rhs);

	bool operator < (const Vector3& rhs) const;
	bool operator == (const Vector3& rhs) const;
	bool operator != (const Vector3& rhs) const;

	Vector3& operator = (SCALAR_TYPE) = delete;

	Vector3 cross(const Vector3& rhs) const;
	SCALAR_TYPE dot(const Vector3& rhs) const;

	SCALAR_TYPE norm() const;
	SCALAR_TYPE squaredNorm() const;
	Vector3 normalized() const;
	void normalize();

	const SCALAR_TYPE& operator[](int idx) const;
	SCALAR_TYPE& operator[](int idx);

private:
	SCALAR_TYPE _val[3];
};

extern double EIGEN_PI;


template<typename SCALAR_TYPE>
Vector3<SCALAR_TYPE> operator * (SCALAR_TYPE lhs, const Vector3<SCALAR_TYPE>& rhs) {
	Vector3<SCALAR_TYPE> result(rhs);
	result *= lhs;
	return result;
}

template<typename SCALAR_TYPE>
std::ostream& operator << (std::ostream& out, const Vector3<SCALAR_TYPE>& v) {
	out << "[" << v[0] << ", " << v[1] << ", " << v[2] << "\n";
	return out;
}

using Vector3d = Vector3<double>;
using Vector3f = Vector3<float>;
using Vector3i = Vector3<size_t>;

#endif // USE_EIGEN_VECTOR3
