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

#include <tm_ray.h>
#include <tm_lineSegment.h>
#include <tm_boundingBox.h>

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
		auto delta = pt[i] - _min[i];
		if (delta < -SAME_DIST_TOL)
			return false;
		delta = pt[i] - _max[i];
		if (delta > SAME_DIST_TOL)
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
bool CBoundingBox3D<SCALAR_TYPE>::intersects(const LineSegment<SCALAR_TYPE>& seg) const
{
	static const Vector3<SCALAR_TYPE> axes[] = {
		Vector3<SCALAR_TYPE>(1, 0, 0),
		Vector3<SCALAR_TYPE>(0, 1, 0),
		Vector3<SCALAR_TYPE>(0, 0, 1)
	};

	if (contains(seg._pts[0]) || contains(seg._pts[1]))
		return true;

	if (seg.calLength() < SAME_DIST_TOL)
		return false;

	auto ray = seg.getRay();
	for (size_t i = 0; i < 3; i++) {
		RayHit<SCALAR_TYPE> hit;
		Plane<SCALAR_TYPE> minPlane(_min, axes[i]);
		if (minPlane.intersectRay(ray, hit)) {
			if (contains(hit.hitPt))
				return true;
		}

		Plane<SCALAR_TYPE> maxPlane(_max, axes[i]);
		if (maxPlane.intersectRay(ray, hit)) {
			if (contains(hit.hitPt))
				return true;
		}
	}
	return false;
}

template <class SCALAR_TYPE>
bool CBoundingBox3D<SCALAR_TYPE>::intersects(const Ray<SCALAR_TYPE>& ray) const {
	const Vector3<SCALAR_TYPE> x(1, 0, 0), y(0, 1, 0), z(0, 0, 1);
	Plane<SCALAR_TYPE> planes[] = {
		Plane<SCALAR_TYPE>(_min, x), Plane<SCALAR_TYPE>(_max, x),
		Plane<SCALAR_TYPE>(_min, y), Plane<SCALAR_TYPE>(_max, y),
		Plane<SCALAR_TYPE>(_min, z), Plane<SCALAR_TYPE>(_max, z)
	};

	for (int i = 0; i < 6; i++) {
		RayHit<SCALAR_TYPE> hit;
		if (planes[i].intersectRay(ray, hit) && contains(hit.hitPt))
			return true;

	}
	return false;
}

template <class SCALAR_TYPE>
bool CBoundingBox3D<SCALAR_TYPE>::intersects(const POINT_TYPE& pt0, const POINT_TYPE& pt1, const POINT_TYPE& pt2) const
{
	static const Vector3<SCALAR_TYPE> axes[] = {
		Vector3<SCALAR_TYPE>(1, 0, 0),
		Vector3<SCALAR_TYPE>(0, 1, 0),
		Vector3<SCALAR_TYPE>(0, 0, 1)
	};

	if (contains(pt0) || contains(pt1) || contains(pt2))
		return true;

	LineSegment<SCALAR_TYPE> seg;
	for (int i = 0; i < 3; i++) {
		Plane<SCALAR_TYPE> minPlane(_min, axes[i]);
		if (minPlane.intersectTri(pt0, pt1, pt2, seg) && intersects(seg)) {
			return true;
		}

		Plane<SCALAR_TYPE> maxPlane(_max, axes[i]);
		if (maxPlane.intersectTri(pt0, pt1, pt2, seg) && intersects(seg)) {
			return true;
		}
	}

	return false;
}

template <class SCALAR_TYPE>
void CBoundingBox3D<SCALAR_TYPE>::getEdges(LineSegment<SCALAR_TYPE> edgeSegs[12]) const
{
	Vector3<SCALAR_TYPE> r = range();
	Vector3<SCALAR_TYPE> corners[] = {
		_min,
		_min + Vector3<SCALAR_TYPE>(r[0],    0, 0),
		_min + Vector3<SCALAR_TYPE>(r[0], r[1], 0),
		_min + Vector3<SCALAR_TYPE>(0,    r[1], 0),

		_min + Vector3<SCALAR_TYPE>(0,       0, r[2]),
		_min + Vector3<SCALAR_TYPE>(r[0],    0, r[2]),
		_min + Vector3<SCALAR_TYPE>(r[0], r[1], r[2]),
		_min + Vector3<SCALAR_TYPE>(0,    r[1], r[2]),
	};

	// x edgeSegs
	edgeSegs[0] = LineSegment<SCALAR_TYPE>(corners[0], corners[1]);
	edgeSegs[1] = LineSegment<SCALAR_TYPE>(corners[3], corners[2]);
	edgeSegs[2] = LineSegment<SCALAR_TYPE>(corners[4], corners[5]);
	edgeSegs[3] = LineSegment<SCALAR_TYPE>(corners[7], corners[6]);

	// y edgeSegs
	edgeSegs[4] = LineSegment<SCALAR_TYPE>(corners[0], corners[3]);
	edgeSegs[5] = LineSegment<SCALAR_TYPE>(corners[1], corners[2]);
	edgeSegs[6] = LineSegment<SCALAR_TYPE>(corners[4], corners[7]);
	edgeSegs[7] = LineSegment<SCALAR_TYPE>(corners[5], corners[6]);

	// z edgeSegs
	edgeSegs[8] = LineSegment<SCALAR_TYPE>(corners[0], corners[4]);
	edgeSegs[9] = LineSegment<SCALAR_TYPE>(corners[1], corners[5]);
	edgeSegs[10] = LineSegment<SCALAR_TYPE>(corners[2], corners[6]);
	edgeSegs[11] = LineSegment<SCALAR_TYPE>(corners[3], corners[7]);

}

template <class SCALAR_TYPE>
void CBoundingBox3D<SCALAR_TYPE>::split(int axis, CBoundingBox3D& left, CBoundingBox3D& right, Scalar overlap) const {
	left = right = *this;
	auto mid = (_min[axis] + _max[axis]) * (SCALAR_TYPE)0.5;

	const auto tol = overlap > 1e-6 ? (_max[axis] - _min[axis]) * overlap : (SCALAR_TYPE)0;
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

template <class SCALAR_TYPE>
void CBoundingBox3D<SCALAR_TYPE>::growPercent(SCALAR_TYPE amount)
{
	auto r = range() * amount;
	_min -= r;
	_max += r;
}

template class CBoundingBox3D<double>;
template class CBoundingBox3D<float>;
