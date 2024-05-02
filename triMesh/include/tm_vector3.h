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

template <typename T>
using Vector3 = Eigen::Matrix<T, 3, 1>;

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
