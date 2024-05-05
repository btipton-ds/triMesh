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

#undef Success
#include <Eigen/Dense>

#include <limits>
#include <iostream>

template<typename SCALAR_TYPE>
SCALAR_TYPE defaultVal();

// NOTE CRITICAL - Eigen has an intermittent error SOMEWHERE that was causing nondeterministic errors in intersections!
// Vector3 had beend defined the same as Vector2, using Vector3 = Eigen::Matrix<T, 3, 1>, switching between our own and Eigen got rid
// of the nondeterminism. That cost almost 2 months and the error was never found.
//
// If we switch back to Eigen, this must be identified and fixed.

template <typename T>
class Vector3
{
public:
	Vector3() = default;
	Vector3(const Vector3& src) = default;
	inline Vector3(const Eigen::Matrix<T, 3, 1>& src)
	{
	}
	inline Vector3(T x, T y, T z)
	{
		_data[0] = x;
		_data[1] = y;
		_data[2] = z;
	}

	inline Vector3(const T* data)
	{
		_data[0] = data[0];
		_data[1] = data[1];
		_data[2] = data[2];
	}

	template<class I>
	inline const T& operator[](I i) const
	{
		return _data[i];
	}

	template<class I>
	inline T& operator[](I i)
	{
		return _data[i];
	}

	
	inline bool operator == (const Vector3& rhs) {
		return _data[0] == rhs._data[0] && _data[1] == rhs._data[1] && _data[2] == rhs._data[2];
	}

	inline bool operator != (const Vector3& rhs) {
		return _data[0] != rhs._data[0] || _data[1] != rhs._data[1] || _data[2] != rhs._data[2];
	}

	inline bool operator < (const Vector3 & rhs) {
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

		i = 2;
		if (_data[i] < rhs._data[i])
			return true;
		else if (_data[i] > rhs._data[i])
			return false;
	}

	inline Vector3& operator -=(const Vector3& rhs)
	{
		auto d = data();
		const auto dRhs = rhs.data();

		d[0] -= dRhs[0];
		d[1] -= dRhs[1];
		d[2] -= dRhs[2];

		return *this;
	}

	inline Vector3& operator +=(const Vector3& rhs)
	{
		auto d = data();
		const auto dRhs = rhs.data();

		d[0] += dRhs[0];
		d[1] += dRhs[1];
		d[2] += dRhs[2];

		return *this;
	}

	inline Vector3 operator -()  const
	{
		Vector3 result(*this);
		auto d = result.data();
		d[0] = -d[0];
		d[1] = -d[1];
		d[2] = -d[2];
		return result;
	}

	inline Vector3 operator -(const Vector3& rhs)  const
	{
		Vector3 result(*this);
		result -= rhs;
		return result;
	}

	inline Vector3 operator +(const Vector3& rhs)  const
	{
		Vector3 result(*this);
		result += rhs;
		return result;
	}

	inline Vector3 cross(const Vector3& rhs) const
	{
		Vector3 result;
		result[0] = _data[1] * rhs._data[2] - _data[2] * rhs._data[1];
		result[1] = _data[2] * rhs._data[0] - _data[0] * rhs._data[2];
		result[2] = _data[0] * rhs._data[1] - _data[1] * rhs._data[0];

		return result;
	}

	inline T dot(const Vector3& rhs) const
	{
		const auto d = data();
		const auto dr = rhs.data();
		return d[0] * dr[0] + d[1] * dr[1] + d[2] * dr[2];
	}

	inline T squaredNorm() const
	{
		const auto d = data();
		return d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
	}

	inline T norm() const
	{
		return ::sqrt(squaredNorm());
	}

	template<class U>
	inline Vector3& operator /=(U rhs)
	{
		auto d = data();
		d[0] /= rhs;
		d[1] /= rhs;
		d[2] /= rhs;

		return *this;
	}

	template<class U>
	inline Vector3& operator *=(U rhs)
	{
		auto d = data();
		d[0] *= rhs;
		d[1] *= rhs;
		d[2] *= rhs;

		return *this;
	}

	template<class U>
	inline Vector3 operator /(U rhs) const
	{
		Vector3 result(*this);
		result /= rhs;

		return result;
	}

	template<class U>
	inline Vector3 operator *(U rhs) const
	{
		Vector3 result(*this);
		result *= rhs;

		return result;
	}

	inline Vector3 normalized() const
	{
		Vector3 result(*this);
		T l = result.norm();
		auto d = result.data();
		d[0] /= l;
		d[1] /= l;
		d[2] /= l;

		return result;
	}

	void normalize()
	{
		auto l = norm();
		_data[0] /= l;
		_data[1] /= l;
		_data[2] /= l;
	}

	inline operator Eigen::Matrix<T, 3, 1>() const
	{
		Eigen::Matrix<T, 3, 1> result(_data[0], _data[1], _data[2]);

		return result;
	}

	inline const T* data() const
	{
		return _data;
	}
	inline T* data()
	{
		return _data;
	}

	private:
		T _data[3] = { 0,0,0 };
};

template <typename T>
Vector3<T> operator *(T lhs, const Vector3<T>& rhs)
{
	Vector3<T> result(rhs);

	auto d = result.data();
	d[0] *= lhs;
	d[1] *= lhs;
	d[2] *= lhs;

	return result;
}

template<class T>
std::ostream& operator << (std::ostream& out, const Vector3<T>& val)
{
	out << val[0] << ", " << val[1] << ", " << val[2];
	return out;
}

// NOTE Vector3 using Eigen had a nondeterministic error. We may need to change Vector2 as well, but it uses very few features - probably safe.
template <typename T>
using Vector2 = Eigen::Matrix<T, 2, 1>;

using Vector3d = Vector3<double>;
using Vector3f = Vector3<float>;

// using size_t allows indexing into stl containers
using Vector3i = Vector3<size_t>;

template<class T>
bool equalTol(const Vector3<T>& val0, const Vector3<T>& val1);

template <typename T>
inline void writeVector3(std::ostream& out, const Vector3<T>& v)
{
	T tv[3] = { v[0], v[1], v[2] };
	out.write((char*)tv, 3 * sizeof(T));
}

template <typename T>
inline void readVector3(std::istream& in, Vector3<T>& v)
{
	T tv[3];
	in.read((char*)tv, 3 * sizeof(T));

	v[0] = tv[0];
	v[1] = tv[1];
	v[2] = tv[2];
}
