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

#include <cmath>
#include <vector>
#include <climits>
#include <cfloat>

#include <tm_vector3.h>

template<class T>
inline const T& ptOf3(int idx, const T& pt0, const T& pt1, const T& pt2) {
	switch (idx) {
		default:
		case 0:
			return pt0;
		case 1:
			return pt1;
		case 2:
			return pt2;
	}
}

template<class T>
struct Ray;
template<class T>
struct RayHit;
template<class T>
struct LineSegment;

template<class T>
class Plane {
public:
	using POINT_TYPE = Vector3<T>;

	Plane() = default;
	Plane(const Plane& src) = default;
	Plane(const POINT_TYPE& origin, const POINT_TYPE& normal);
	Plane(const POINT_TYPE* pts[3]);

	bool intersectLine(const POINT_TYPE& pt0, const POINT_TYPE& pt1, POINT_TYPE& pt, T& t) const;
	bool intersectLineSegment(const LineSegment<T>& seg, POINT_TYPE& pt, T& t) const;
	bool intersectRay(const Ray<T>& ray, RayHit<T>& hit) const;
	bool intersectTri(const POINT_TYPE& pt0, const POINT_TYPE& pt1, const POINT_TYPE& pt2, LineSegment<T>& iSeg) const;

	POINT_TYPE projectPoint(const POINT_TYPE& pt) const;
	T distanceToPoint(const POINT_TYPE& pt) const;

	const POINT_TYPE& getOrgin() const;
	const POINT_TYPE& getNormal() const;

private:
	POINT_TYPE _origin, _normal;
};


template<class T>
inline T Plane<T>::distanceToPoint(const POINT_TYPE& pt) const {
	return fabs((pt - _origin).dot(_normal));
}

template<class T>
inline const typename Plane<T>::POINT_TYPE& Plane<T>::getOrgin() const
{
	return _origin;
}

template<class T>
inline typename const Plane<T>::POINT_TYPE& Plane<T>::getNormal() const
{
	return _normal;
}

