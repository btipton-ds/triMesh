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

template<>
bool equalTol(const Vector3<float>& val0, const Vector3<float>& val1)
{
	Vector3<float> delta = val1 - val0;
	for (int i = 0; i < 3; i++) {
		if (fabs(delta[i]) > 1.0e-7f)
			return false;
	}
	return true;
}

template<>
bool equalTol(const Vector3<double>& val0, const Vector3<double>& val1)
{
	Vector3<double> delta = val1 - val0;
	for (int i = 0; i < 3; i++) {
		if (fabs(delta[i]) > 1.0e-16)
			return false;
	}
	return true;
}
