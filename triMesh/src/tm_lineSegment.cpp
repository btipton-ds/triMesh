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

#include <tm_lineSegment.hpp>
#include <tm_lineSegment_byref.hpp>

template<class T>
bool LineSegment<T>::isCoincident(const LineSegment<T>::POINT_TYPE& pt, LineSegment<T>::SCALAR_TYPE tol) const
{
	T t;
	auto dist = distanceToPoint(pt, t);

	return fabs(dist) < tol;
}

template<class T>
bool LineSegment<T>::isCoincident(const LineSegment<T>& other, LineSegment<T>::SCALAR_TYPE tol) const
{
	return isCoincident(other._pt0, tol) && isCoincident(other._pt1, tol);
}

template<class T>
bool LineSegment<T>::isCoincident(const LineSegment_byref<T>& other, LineSegment<T>::SCALAR_TYPE tol) const
{
	return isCoincident(other._pt0, tol) && isCoincident(other._pt1, tol);
}

template<class T>
bool LineSegment<T>::isCoincident(const Ray<T>& other, LineSegment<T>::SCALAR_TYPE tol) const
{
	return isCoincident(other._origin, tol) && isCoincident(other._origin + other._dir, tol);
}

/*******************************************************************************************************/

template<class T>
bool LineSegment_byref<T>::isCoincident(const LineSegment_byref<T>::POINT_TYPE& pt, LineSegment_byref<T>::SCALAR_TYPE tol) const
{
	T t;
	auto dist = distanceToPoint(pt, t);

	return fabs(dist) < tol;
}

template<class T>
bool LineSegment_byref<T>::isCoincident(const LineSegment<T>& other, LineSegment_byref<T>::SCALAR_TYPE tol) const
{
	return isCoincident(other._pt0, tol) && isCoincident(other._pt1, tol);
}

template<class T>
bool LineSegment_byref<T>::isCoincident(const LineSegment_byref<T>& other, LineSegment_byref<T>::SCALAR_TYPE tol) const
{
	return isCoincident(other._pt0, tol) && isCoincident(other._pt1, tol);
}

template<class T>
bool LineSegment_byref<T>::isCoincident(const Ray<T>& other, LineSegment_byref<T>::SCALAR_TYPE tol) const
{
	return isCoincident(other._origin, tol) && isCoincident(other._origin + other._dir, tol);
}

template struct LineSegment<double>;
template struct LineSegment<float>;

template struct LineSegment_byref<double>;
template struct LineSegment_byref<float>;
