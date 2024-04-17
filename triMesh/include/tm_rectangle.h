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

#undef Success
#include <Eigen/Dense>

#include <tm_math.h>

template <class SCALAR_TYPE>
class CRectangle {
public:
	using POINT_TYPE = Eigen::Matrix<SCALAR_TYPE, 2, 1>;

	CRectangle(const POINT_TYPE& min = {}, const POINT_TYPE& max = {});

	bool contains(const POINT_TYPE& pt) const;
	bool intersects(const POINT_TYPE& pt0, const POINT_TYPE& pt1, const POINT_TYPE& pt2) const;
	bool intersects(const POINT_TYPE* pts[3]) const;

private:
	static bool intersectSegments(
		const POINT_TYPE& pt00, const POINT_TYPE& pt01, SCALAR_TYPE& t0, 
		const POINT_TYPE& pt10, const POINT_TYPE& pt11, SCALAR_TYPE& t1, 
		POINT_TYPE& iPt);

	POINT_TYPE _min, _max;
};

using CRectangled = CRectangle<double>;
using CRectanglef = CRectangle<float>;

template <class SCALAR_TYPE>
CRectangle<SCALAR_TYPE>::CRectangle(const POINT_TYPE& min, const POINT_TYPE& max)
	: _min(min)
	, _max(max)
{
#ifdef _DEBUG
	for (int i = 0; i < 2; i++)
		assert(_min[i] <= _max[i]);
#endif // _DEBUG

}

template <class SCALAR_TYPE>
bool CRectangle<SCALAR_TYPE>::contains(const POINT_TYPE& pt) const
{
	for (int axis = 0; axis < 2; axis++) {
		if (pt[axis] <= _min[axis] - SAME_DIST_TOL || pt[axis] >= _max[axis] + SAME_DIST_TOL) {
			return false;
		}
	}

	return true;
}

template <class SCALAR_TYPE>
bool CRectangle<SCALAR_TYPE>::intersects(const POINT_TYPE& pt0, const POINT_TYPE& pt1, const POINT_TYPE& pt2) const
{
	const POINT_TYPE* pts[] = {
		&pt0,
		&pt1,
		&pt2,
	};

	return intersects(pts);
}

template <class SCALAR_TYPE>
bool CRectangle<SCALAR_TYPE>::intersects(const POINT_TYPE* pts[3]) const
{
	POINT_TYPE corners[] = {
		POINT_TYPE(_min[0], _min[1]),
		POINT_TYPE(_max[0], _min[1]),
		POINT_TYPE(_max[0], _max[1]),
		POINT_TYPE(_min[0], _max[1]),
	};

	for (int ii = 0; ii < 4; ii++) {
		int jj = (ii + 1) % 4;
		const auto& pt00 = corners[ii];
		const auto& pt01 = corners[jj];

		for (int i = 0; i < 3; i++) {
			int j = (i + 1) % 3;
			const auto& pt10 = (*pts[i]);
			const auto& pt11 = (*pts[j]);

			SCALAR_TYPE t0, t1;
			POINT_TYPE iPt;
			if (intersectSegments(pt00, pt01, t0, pt10, pt11, t1, iPt)) {
				return true;
			}
		}
	}

	return false;
}

template <class SCALAR_TYPE>
bool CRectangle<SCALAR_TYPE>::intersectSegments(
	const POINT_TYPE& pt00, const POINT_TYPE& pt01, SCALAR_TYPE& t0,
	const POINT_TYPE& pt10, const POINT_TYPE& pt11, SCALAR_TYPE& t1,
	POINT_TYPE& iPt)
{
	const double paramTol = 1.0e-8;
	POINT_TYPE v0 = pt01 - pt00;

	POINT_TYPE xAxis(v0);
	auto len = xAxis.norm();
	if (len < 1.0e-12)
		return false;
	xAxis /= len;

	POINT_TYPE v1 = pt11 - pt10;
	v1 = v1 - xAxis.dot(v1) * xAxis;
	auto d0 = v1.norm();
	if (d0 < 1.0e-8)
		return false;

	POINT_TYPE v2 = pt11 - pt00;
	v2 = v2 - xAxis.dot(v2) * xAxis;
	auto d1 = v2.norm();

	t1 = d1 / d0;
	if (t1 < -paramTol || t1 > 1 + paramTol)
		return false;

	iPt = pt10 + t1 * (pt11 - pt10);

	t0 = (iPt - pt00).norm() / len;
	if (t0 < -paramTol || t0 > 1 + paramTol)
		return false;

	return true;
}
