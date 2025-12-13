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
	Vector2 is a light weight substitute for Eigen. TriMesh originally used Eigen, but the debugging overhead (both speed and examination in a debugger) made it unweildy.
	At one time a 1-1 substiution was possible. It is desired to retain this feature.
*/

#include <tm_defines.h>
#include <cmath>

#undef Success
#include <Eigen/Dense>

#include <limits>
#include <iostream>
#include <tm_defaultTolerances.h>

// NOTE CRITICAL - Eigen has an intermittent error SOMEWHERE that was causing nondeterministic errors in intersections!
// Vector2 had beend defined the same as Vector2, using Vector2 = Eigen::Matrix<T, 3, 1>, switching between our own and Eigen got rid
// of the nondeterminism. That cost almost 2 months and the error was never found.
//
// If we switch back to Eigen, this must be identified and fixed.

template <typename T>
class Vector2
{
public:
	using SCALAR_TYPE = T;

	Vector2() = default;
	Vector2(const Vector2& src) = default;
	Vector2(const Eigen::Matrix<T, 2, 1>& src);
	Vector2(T x, T y);
	Vector2(const T* data);
	template<class I>
	const T& operator[](I i) const;
	template<class I>
	T& operator[](I i);
	bool operator == (const Vector2& rhs);
	bool operator != (const Vector2& rhs);
	bool operator < (const Vector2& rhs) const;
	Vector2& operator -=(const Vector2& rhs);
	Vector2& operator +=(const Vector2& rhs);
	Vector2 operator -()  const;
	Vector2 operator -(const Vector2& rhs)  const;
	Vector2 operator +(const Vector2& rhs)  const;
	T cross(const Vector2& rhs) const;
	T dot(const Vector2& rhs) const;
	T squaredNorm() const;
	T norm() const;
	template<class U>
	Vector2& operator /=(U rhs);
	template<class U>
	Vector2& operator *=(U rhs);
	template<class U>
	Vector2 operator /(U rhs) const;
	template<class U>
	Vector2 operator *(U rhs) const;
	Vector2 normalized() const;
	void normalize();
	operator Eigen::Matrix<T, 2, 1>() const;
	const T* data() const;
	T* data();
	bool isNAN() const;
	private:
		T _data[2] = { 0, 0 };
};

template <typename T>
inline Vector2<T>::Vector2(const Eigen::Matrix<T, 2, 1>& src)
{
	_data[0] = src[0];
	_data[1] = src[1];
}

template <typename T>
inline Vector2<T>::Vector2(T x, T y)
{
	_data[0] = x;
	_data[1] = y;
}

template <typename T>
inline Vector2<T>::Vector2(const T* data)
{
	_data[0] = data[0];
	_data[1] = data[1];
}

template <typename T>
template<class I>
inline const T& Vector2<T>::operator[](I i) const
{
	return _data[i];
}

template <typename T>
template<class I>
inline T& Vector2<T>::operator[](I i)
{
	return _data[i];
}


template <typename T>
inline bool Vector2<T>::operator == (const Vector2& rhs) {
	return _data[0] == rhs._data[0] && _data[1] == rhs._data[1];
}

template <typename T>
inline bool Vector2<T>::operator != (const Vector2& rhs) {
	return _data[0] != rhs._data[0] || _data[1] != rhs._data[1];
}

template <typename T>
inline bool Vector2<T>::operator < (const Vector2& rhs) const {
	int i = 0;
	if (_data[i] < rhs._data[i])
		return true;
	else if (_data[i] > rhs._data[i])
		return false;

	i = 1;
	if (_data[i] < rhs._data[i])
		return true;
	else if (_data[i] > rhs._data[i])
		return false;


	return false;
}

template <typename T>
inline Vector2<T>& Vector2<T>::operator -=(const Vector2& rhs)
{
	auto pLhs = _data;
	auto pRhs = rhs._data;
	*pLhs++ -= *pRhs++;
	*pLhs++ -= *pRhs++;

	return *this;
}

template <typename T>
inline Vector2<T>& Vector2<T>::operator +=(const Vector2& rhs)
{
	auto pLhs = _data;
	auto pRhs = rhs._data;
	*pLhs++ += *pRhs++;
	*pLhs++ += *pRhs++;

	return *this;
}

template <typename T>
inline Vector2<T> Vector2<T>::operator -()  const
{
	Vector2 result(*this);
	auto d = result.data();
	d[0] = -d[0];
	d[1] = -d[1];
	return result;
}

template <typename T>
inline Vector2<T> Vector2<T>::operator -(const Vector2& rhs)  const
{
	Vector2 result(*this);
	result -= rhs;
	return result;
}

template <typename T>
inline Vector2<T> Vector2<T>::operator +(const Vector2& rhs)  const
{
	Vector2 result(*this);
	result += rhs;
	return result;
}

template <typename T>
inline T Vector2<T>::cross(const Vector2& rhs) const
{
	T result = _data[0] * rhs._data[1] - _data[1] * rhs._data[0];

	return result;
}

template <typename T>
inline T Vector2<T>::dot(const Vector2& rhs) const
{
	const auto d = data();
	const auto dr = rhs.data();
	return d[0] * dr[0] + d[1] * dr[1];
}

template <typename T>
inline T Vector2<T>::squaredNorm() const
{
	const auto d = data();
	return d[0] * d[0] + d[1] * d[1];
}

template <typename T>
inline T Vector2<T>::norm() const
{
	return ::sqrt(squaredNorm());
}

template <typename T>
template<class U>
inline Vector2<T>& Vector2<T>::operator /=(U rhs)
{
	T* d = data();
	d[0] /= (T)rhs;
	d[1] /= (T)rhs;

	return *this;
}

template <typename T>
template<class U>
inline Vector2<T>& Vector2<T>::operator *=(U rhs)
{
	T* d = data();
	d[0] *= (T)rhs;
	d[1] *= (T)rhs;

	return *this;
}

template <typename T>
template<class U>
inline Vector2<T> Vector2<T>::operator /(U rhs) const
{
	Vector2 result(*this);
	result /= (T)rhs;

	return result;
}

template <typename T>
template<class U>
inline Vector2<T> Vector2<T>::operator *(U rhs) const
{
	Vector2 result(*this);
	result *= (T)rhs;

	return result;
}

template <typename T>
inline Vector2<T> Vector2<T>::normalized() const
{
	Vector2 result(*this);
	T lInv = 1 / result.norm();
	auto d = result.data();
	d[0] *= lInv;
	d[1] *= lInv;

	return result;
}

template <typename T>
void Vector2<T>::normalize()
{
	auto lInv = 1 / norm();
	_data[0] *= lInv;
	_data[1] *= lInv;
}

template <typename T>
inline Vector2<T>::operator Eigen::Matrix<T, 2, 1>() const
{
	Eigen::Matrix<T, 2, 1> result(_data[0], _data[1]);

	return result;
}

template <typename T>
inline const T* Vector2<T>::data() const
{
	return _data;
}

template <typename T>
inline T* Vector2<T>::data()
{
	return _data;
}

template <typename T>
inline bool Vector2<T>::isNAN() const
{
	for (int i = 0; i < 2; i++) {
		if (std::isnan(_data[i]))
			return true;
	}
	return false;
}

template <typename T>
Vector2<T> operator *(T lhs, const Vector2<T>& rhs)
{
	Vector2<T> result(rhs);

	auto d = result.data();
	d[0] *= lhs;
	d[1] *= lhs;

	return result;
}

template<class T>
std::ostream& operator << (std::ostream& out, const Vector2<T>& val)
{
	out << val[0] << ", " << val[1];
	return out;
}

using Vector2d = Vector2<double>;
using Vector2f = Vector2<float>;

// using size_t allows indexing into stl containers
using Vector2i = Vector2<size_t>;

template<class T>
T defaultDistTol();

template<class T>
bool equalTol(const Vector2<T>& val0, const Vector2<T>& val1, T tol = defaultDistTol<T>());

