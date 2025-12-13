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

#include <tm_defines.h>
#include <tm_math.h>
#include <tm_ray.h>
#include <tm_plane_byref.h>
#include <tm_lineSegment2D_byref.h>
#include <tm_codeTemplates.h>

template<class T>
LineSegment2D_byref<T>::LineSegment2D_byref(const POINT_TYPE& p0, const POINT_TYPE& p1)
	: _pt0(p0)
	, _pt1(p1)
{
}

template<class T>
typename LineSegment2D_byref<T>::SCALAR_TYPE LineSegment2D_byref<T>::calLength() const {
	return (_pt1 - _pt0).norm();
}

template<class T>
typename LineSegment2D_byref<T>::POINT_TYPE LineSegment2D_byref<T>::calcDir() const {
	return safeNormalize(_pt1 - _pt0);
}

template<class T>
typename LineSegment2D_byref<T>::POINT_TYPE LineSegment2D_byref<T>::interpolate(SCALAR_TYPE t) const {
	return _pt0 + t * (_pt1 - _pt0);
}

template<class T>
typename LineSegment2D_byref<T>::SCALAR_TYPE LineSegment2D_byref<T>::parameterize(const POINT_TYPE& pt) const {
	return (pt - _pt0).dot(calcDir()) / calLength();
}

template<class T>
bool LineSegment2D_byref<T>::contains(const POINT_TYPE& pt, LineSegment2D_byref::SCALAR_TYPE& t, SCALAR_TYPE tol) const
{
	if (tolerantEquals(_pt0, pt, tol)) {
		t = 0;
		return true;
	}
	else if (tolerantEquals(_pt1, pt, tol)) {
		t = 1;
		return true;
	} else {
		const auto tolSqr = tol * tol;
		POINT_TYPE vDir = _pt1 - _pt0;
		auto lenSqr = vDir.squaredNorm();
		if (lenSqr < tolSqr) {
			return false;
		}
		auto len = sqrt(lenSqr);
		vDir /= len;
		POINT_TYPE vOrth = pt - _pt0;

		auto dp = vOrth.dot(vDir);
		vOrth = vOrth - dp * vDir; // orthogonalize v1
		auto distSqr = vOrth.squaredNorm();
		if (distSqr > tolSqr)
			return false; // pt does not lie on the segment within tolerance.
		t = dp / len;
		return -tol < dp && dp < (len + tol); // return if the pt lies in [zero, len] within tolerance
	}
}

template<class T>
bool LineSegment2D_byref<T>::intersects(const LineSegment2D<T>& other, POINT_TYPE& iPt, SCALAR_TYPE tol) const
{
LINE_SEG_2D_INTERSECT_TEMP
}

template<class T>
bool LineSegment2D_byref<T>::intersects(const LineSegment2D_byref<T>& other, POINT_TYPE& iPt, SCALAR_TYPE tol) const
{
LINE_SEG_2D_INTERSECT_TEMP
}

template<class T>
typename LineSegment2D_byref<T>::SCALAR_TYPE LineSegment2D_byref<T>::distanceToPoint(const POINT_TYPE& pt) const {
	SCALAR_TYPE t;
	return distanceToPoint(pt, t);
}

template<class T>
typename LineSegment2D_byref<T>::SCALAR_TYPE LineSegment2D_byref<T>::distanceToPoint(const POINT_TYPE& pt, SCALAR_TYPE& t) const {
	POINT_TYPE dir(_pt1 - _pt0);
	SCALAR_TYPE len = dir.norm();
	if (len < minNormalizeDivisor)
		return (SCALAR_TYPE)FLT_MAX;
	dir /= len;
	POINT_TYPE v0 = pt - _pt0;
	SCALAR_TYPE dp = v0.dot(dir);
	t = dp / len;
	v0 = v0 - dir * dp;
	SCALAR_TYPE dist;
	if (t < 0) {
		t = (SCALAR_TYPE)-DBL_MAX;
		dist = (pt - _pt0).norm();
	}
	else if (t > 1) {
		t = (SCALAR_TYPE)DBL_MAX;
		dist = (pt - _pt1).norm();
	}
	else
		dist = v0.norm();

	return dist;
}
