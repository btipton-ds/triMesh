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

#include <tm_vector3.h>

#include <math.h>
#include <cfloat>

#if !USE_EIGEN_VECTOR3
double EIGEN_PI = 4.0 * atan(1.0);
#endif

template<>
size_t defaultVal<size_t>() {
	return 0xffffffffffffffff;
}
template<>
float defaultVal<float>() {
	return FLT_MAX;
}

template<>
double defaultVal<double>() {
	return DBL_MAX;
}
#if !USE_EIGEN_VECTOR3

template<typename SCALAR_TYPE>
Vector3<SCALAR_TYPE>::Vector3() {
	_val[0] = _val[1] = _val[2] = defaultVal<SCALAR_TYPE>();
}

template<typename SCALAR_TYPE>
Vector3<SCALAR_TYPE>::Vector3(SCALAR_TYPE x, SCALAR_TYPE y, SCALAR_TYPE z)
{
	_val[0] = x;
	_val[1] = y;
	_val[2] = z;
}

template<typename SCALAR_TYPE>
Vector3<SCALAR_TYPE> Vector3<SCALAR_TYPE>::operator - () const {
	return Vector3 (-_val[0], -_val[1], -_val[2]);
}

template<>
Vector3<size_t> Vector3<size_t>::operator - () const {
	throw "Negation of unsigned is undefined";
}

template<typename SCALAR_TYPE>
Vector3<SCALAR_TYPE> Vector3<SCALAR_TYPE>::operator - (const Vector3& rhs) const {
	Vector3 result(*this);
	result -= rhs;
	return result;
}

template<typename SCALAR_TYPE>
Vector3<SCALAR_TYPE>& Vector3<SCALAR_TYPE>::operator -= (const Vector3& rhs) {
	for (int i = 0; i < 3; i++)
		_val[i] -= rhs._val[i];

	return *this;
}

template<typename SCALAR_TYPE>
Vector3<SCALAR_TYPE> Vector3<SCALAR_TYPE>::operator + (const Vector3& rhs) const {
	Vector3 result(*this);
	result += rhs;
	return result;
}

template<typename SCALAR_TYPE>
Vector3<SCALAR_TYPE>& Vector3<SCALAR_TYPE>::operator += (const Vector3& rhs) {
	for (int i = 0; i < 3; i++)
		_val[i] += rhs._val[i];
	return *this;
}

template<typename SCALAR_TYPE>
Vector3<SCALAR_TYPE> Vector3<SCALAR_TYPE>::operator * (SCALAR_TYPE rhs) const {
	Vector3 result(*this);
	result *= rhs;
	return result;
}

template<typename SCALAR_TYPE>
Vector3<SCALAR_TYPE>& Vector3<SCALAR_TYPE>::operator *= (SCALAR_TYPE rhs) {
	for (int i = 0; i < 3; i++)
		_val[i] *= rhs;
	return *this;
}

template<typename SCALAR_TYPE>
Vector3<SCALAR_TYPE> Vector3<SCALAR_TYPE>::operator / (SCALAR_TYPE rhs) const {
	Vector3 result(*this);
	result /= rhs;
	return result;
}

template<typename SCALAR_TYPE>
Vector3<SCALAR_TYPE>& Vector3<SCALAR_TYPE>::operator /= (SCALAR_TYPE rhs) {
	for (int i = 0; i < 3; i++)
		_val[i] /= rhs;
	return *this;
}

template<typename SCALAR_TYPE>
bool Vector3<SCALAR_TYPE>::operator < (const Vector3& rhs) const {
	for (int i = 0; i < 3; i++) {
		if (_val[i] < rhs._val[i])
			return true;
		else if (_val[i] > rhs._val[i])
			return false;
	}

	return false;
}

template<typename SCALAR_TYPE>
bool Vector3<SCALAR_TYPE>::operator == (const Vector3& rhs) const {
	for (int i = 0; i < 3; i++) {
		if (_val[i] != rhs._val[i])
			return false;
	}

	return true;
}

template<typename SCALAR_TYPE>
bool Vector3<SCALAR_TYPE>::operator != (const Vector3& rhs) const {
	return !operator== (rhs);
}

template<typename SCALAR_TYPE>
Vector3<SCALAR_TYPE> Vector3<SCALAR_TYPE>::cross(const Vector3& rhs) const {
	Vector3 result;
	result[0] = _val[1] * rhs._val[2] - _val[2] * rhs._val[1];
	result[1] = _val[2] * rhs._val[0] - _val[0] * rhs._val[2];
	result[2] = _val[0] * rhs._val[1] - _val[1] * rhs._val[0];
	return result;
}

template<typename SCALAR_TYPE>
SCALAR_TYPE Vector3<SCALAR_TYPE>::dot(const Vector3& rhs) const {
	return _val[0] * rhs._val[0] + _val[1] * rhs._val[1] + _val[2] * rhs._val[2];
}

template<typename SCALAR_TYPE>
SCALAR_TYPE abst(SCALAR_TYPE v) {
	return 0;
}

template<>
double Vector3<double>::norm() const {
	return sqrt(dot(*this));
}

template<>
float Vector3<float>::norm() const {
	return sqrtf(dot(*this));
}

template<>
size_t Vector3<size_t>::norm() const {
	throw "Norm of an int type Vector3d is undefined";
}

template<typename SCALAR_TYPE>
SCALAR_TYPE Vector3<SCALAR_TYPE>::squaredNorm() const {
	return dot(*this);
}

template<typename SCALAR_TYPE>
Vector3<SCALAR_TYPE> Vector3<SCALAR_TYPE>::normalized() const {
	Vector3 result(*this);
	result.normalize();
	return result;
}

template<typename SCALAR_TYPE>
void Vector3<SCALAR_TYPE>::normalize() {
	SCALAR_TYPE d = 1 / norm();
	operator *= (d);
}

template<typename SCALAR_TYPE>
const SCALAR_TYPE& Vector3<SCALAR_TYPE>::operator[](int idx) const {
	return _val[idx];
}

template<typename SCALAR_TYPE>
SCALAR_TYPE& Vector3<SCALAR_TYPE>::operator[](int idx) {
	return _val[idx];
}

template class Vector3<double>;
template class Vector3<float>;
template class Vector3<size_t>;

#endif
