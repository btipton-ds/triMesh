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
#include <tm_vector3.h>
#include <tm_math.h>

template<class T>
using Matrix3 = Eigen::Matrix<T, 3, 3>;

template<class T>
using Matrix4 = Eigen::Matrix<T, 4, 4>;

template<class T>
using Vector4 = Eigen::Matrix<T, 4, 1>;

template<class A, class B>
A changeMatrixSize(const B& src)
{
	A result;
	result.setZero();
	if (src.cols() == src.rows()) {
		Eigen::Index l = min(src.rows(), result.rows());
		for (Eigen::Index i = 0; i < l; i++) {
			for (Eigen::Index j = 0; j < l; j++) {
				result(i, j) = src(i, j);
			}
		}
	}
	else {
		Eigen::Index l = min(src.rows(), result.rows());
		for (Eigen::Index i = 0; i < l; i++) {
			result(i, 0) = src(i, 0);
		}
	}
	return result;
}

template<class T>
Matrix4<T> createTranslation(const Vector3<T>& delta)
{
	Eigen::Transform<T, 3, Eigen::Affine> t = Eigen::Transform<T, 3, Eigen::Affine>::Identity();
	Vector3<T> x(delta);
	t.translate(x);
	Matrix4<T> result(t.matrix());
	return result;
}

template<class T>
Vector3<T> rotatePointAboutAxis(const Ray<T>& axis, const Vector3<T>& val, T angleDeg)
{
	static const T PI = (T)M_PI;

	Matrix3<T> rot3 = Eigen::AngleAxis<T>(angleDeg * PI / 180, axis._dir).toRotationMatrix();
	Matrix4<T> trans;
	Matrix4<T> rot4(changeMatrixSize<Matrix4<T>>(rot3));
	Matrix4<T> translate(createTranslation<T>(axis._origin));
	Matrix4<T> unTranslate(createTranslation<T>(-axis._origin));
	rot4(3, 3) = 1;

	trans.setIdentity();
	trans *= translate;
	trans *= rot4;
	trans *= unTranslate;

	Eigen::Matrix<T, 4, 1> val4(val[0], val[1], val[2], 1);
	Eigen::Matrix<T, 4, 1> r4 = trans * val4;
	Vector3<T> result(r4[0], r4[1], r4[2]);

	return result;
}

template<class T>
Vector3<T> rotateVectorAboutAxis(const Ray<T>& axis, const Vector3<T>& val, T angleDeg)
{
	static const T PI = (T)M_PI;

	Matrix3<T> rot3 = Eigen::AngleAxis<T>(angleDeg * PI / 180, axis._dir).toRotationMatrix();
	Vector3<T> result = rot3 * val;

	return result;
}

