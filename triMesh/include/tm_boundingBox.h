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

template <class SCALAR_TYPE>
class CBoundingBox3D {
public:
	using Scalar = SCALAR_TYPE;
	using POINT_TYPE = Vector3< SCALAR_TYPE>;

	CBoundingBox3D();
	CBoundingBox3D(const POINT_TYPE& pt0);
	CBoundingBox3D(const POINT_TYPE& pt0, const POINT_TYPE& pt1);
	void clear();
	bool empty() const;
	void merge(const POINT_TYPE& pt);
	void merge(const CBoundingBox3D& bboxIn);
	inline const POINT_TYPE& getMin() const;
	inline const POINT_TYPE& getMax() const;
	inline POINT_TYPE range() const;
	bool contains(const POINT_TYPE& pt) const;
	bool contains(const CBoundingBox3D& other) const;
	// This is actually intersects or contains
	bool intersects(const CBoundingBox3D& otherBox) const;
	bool intersects(const Ray& ray) const;
	void split(int axis, CBoundingBox3D& left, CBoundingBox3D& right, Scalar overlap = 0) const;
	void grow(Scalar dist);
private:
	POINT_TYPE _min, _max;
};

using CBoundingBox3Dd = CBoundingBox3D<double>;
using CBoundingBox3Df = CBoundingBox3D<float>;

template <class SCALAR_TYPE>
CBoundingBox3D<SCALAR_TYPE>::CBoundingBox3D()
{
	clear();
}

template <class SCALAR_TYPE>
CBoundingBox3D<SCALAR_TYPE>::CBoundingBox3D(const POINT_TYPE& pt0)
{
	clear();
	merge(pt0);
}

template <class SCALAR_TYPE>
CBoundingBox3D<SCALAR_TYPE>::CBoundingBox3D(const POINT_TYPE& pt0, const POINT_TYPE& pt1)
{
	clear();
	merge(pt0);
	merge(pt1);
}

template <class SCALAR_TYPE>
void CBoundingBox3D<SCALAR_TYPE>::clear() {
	_min = POINT_TYPE(FLT_MAX, FLT_MAX, FLT_MAX);
	_max = POINT_TYPE(-FLT_MAX, -FLT_MAX, -FLT_MAX);
}

template <class SCALAR_TYPE>
bool CBoundingBox3D<SCALAR_TYPE>::empty() const {
	return (_min[0] > _max[0]);
}

template <class SCALAR_TYPE>
void CBoundingBox3D<SCALAR_TYPE>::merge(const POINT_TYPE& pt) {
	for (int i = 0; i < 3; i++) {
		if (pt[i] < _min[i])
			_min[i] = pt[i];
		if (pt[i] > _max[i])
			_max[i] = pt[i];
	}
}

template <class SCALAR_TYPE>
void CBoundingBox3D<SCALAR_TYPE>::merge(const CBoundingBox3D& bboxIn) {
	if (!bboxIn.empty()) {
		merge(bboxIn._min);
		merge(bboxIn._max);
	}
}

template <class SCALAR_TYPE>
inline const typename CBoundingBox3D<SCALAR_TYPE>::POINT_TYPE& CBoundingBox3D<SCALAR_TYPE>::getMin() const {
	return _min;
}

template <class SCALAR_TYPE>
inline const typename CBoundingBox3D<SCALAR_TYPE>::POINT_TYPE& CBoundingBox3D<SCALAR_TYPE>::getMax() const {
	return _max;
}

template <class SCALAR_TYPE>
inline typename CBoundingBox3D<SCALAR_TYPE>::POINT_TYPE CBoundingBox3D<SCALAR_TYPE>::range() const {
	return _max - _min;
}

template <class SCALAR_TYPE>
bool CBoundingBox3D<SCALAR_TYPE>::contains(const POINT_TYPE& pt) const {
	for (int i = 0; i < 3; i++) {
		auto delta = pt[i] - (_min[i] - SAME_DIST_TOL);
		if (delta < 0)
			return false;
		delta = pt[i] - (_max[i] + SAME_DIST_TOL);
		if (delta > 0)
			return false;
	}
	return true;
}

template <class SCALAR_TYPE>
bool CBoundingBox3D<SCALAR_TYPE>::contains(const CBoundingBox3D& other) const {
	return contains(other._min) && contains(other._max);
}

// This is actually intersects or contains
template <class SCALAR_TYPE>
bool CBoundingBox3D<SCALAR_TYPE>::intersects(const CBoundingBox3D& otherBox) const {
	for (int i = 0; i < 3; i++) {
		if (otherBox._min[i] > _max[i] + SAME_DIST_TOL)
			return false;
		if (otherBox._max[i] < _min[i] - SAME_DIST_TOL)
			return false;
	}
	return true;
}

template <class SCALAR_TYPE>
bool CBoundingBox3D<SCALAR_TYPE>::intersects(const Ray& ray) const {
	const Vector3d x(1, 0, 0), y(0, 1, 0), z(0, 0, 1);
	Plane planes[] = {
		Plane(_min, x), Plane(_max, x),
		Plane(_min, y), Plane(_max, y),
		Plane(_min, z), Plane(_max, z)
	};

	for (int i = 0; i < 6; i++) {
		RayHit hit;
		if (intersectRayPlane(ray, planes[i], hit) && contains(hit.hitPt))
			return true;

	}
	return false;
}

template <class SCALAR_TYPE>
void CBoundingBox3D<SCALAR_TYPE>::split(int axis, CBoundingBox3D& left, CBoundingBox3D& right, Scalar overlap) const {
	left = right = *this;
	auto mid = (_min[axis] + _max[axis]) * 0.5;

	const auto tol = overlap > 1e-6 ? (_max[axis] - _min[axis]) * overlap : 0;
	left._max[axis] = mid + tol;
	right._min[axis] = mid - tol;
}

template <class SCALAR_TYPE>
void CBoundingBox3D<SCALAR_TYPE>::grow(Scalar dist) {
	for (int i = 0; i < 3; i++) {
		_min[i] -= dist;
		_max[i] += dist;
	}
}
