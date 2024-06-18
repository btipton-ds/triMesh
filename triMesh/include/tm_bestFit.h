#pragma once
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
#include <vector>
#include <tm_plane.h>
#include <tm_vector3.h>
#include <tm_ray.h>

#undef Success
#include <Eigen/Dense>

template<class T>
bool bestFitPlane(const std::vector<Vector3<T>>& c, Plane<T>& plane, T& err)
{
	if (c.size() < 3)
		return false;
	// copy coordinates to  matrix in Eigen format
	size_t num_atoms = c.size();
	Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> coord(3, num_atoms);
	for (size_t i = 0; i < num_atoms; ++i) {
		const auto v = c[i];
		coord.col(i) = Eigen::Matrix<T, 3, 1>(v[0], v[1], v[2]);
	}

	// calculate centroid
	Eigen::Matrix<T, 3, 1> centroid(coord.row(0).mean(), coord.row(1).mean(), coord.row(2).mean());

	// subtract centroid
	coord.row(0).array() -= centroid(0); coord.row(1).array() -= centroid(1); coord.row(2).array() -= centroid(2);

	// we only need the left-singular matrix here
	//  http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
	auto svd = coord.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::Matrix<T, 3, 1> plane_normal = svd.matrixU().rightCols<1>();
	plane_normal.normalize();
	plane = Plane<T>(Vector3<T>(centroid[0], centroid[1], centroid[2]), Vector3<T>(plane_normal[0], plane_normal[1], plane_normal[2]));
	err = 0;
	for (size_t i = 0; i < c.size(); i++) {
		err += plane.distanceToPoint(c[i]);
	}
	err /= c.size();
	return true;
}

template<class T>
Ray<T> bestFitLine(const std::vector<Vector3<T>>& c, T& err)
{
	// copy coordinates to  matrix in Eigen format
	size_t num_atoms = c.size();
	Eigen::Matrix< T, Eigen::Dynamic, Eigen::Dynamic > centers(num_atoms, 3);
	for (size_t i = 0; i < num_atoms; ++i) {
		const auto& v = c[i];
		centers.row(i) = Eigen::Matrix<T, 3, 1>(v[0], v[1], v[2]);
	}

	Eigen::Matrix<T, 3, 1> origin = centers.colwise().mean();
	Eigen::MatrixXd centered = centers.rowwise() - origin.transpose();
	Eigen::MatrixXd cov = centered.adjoint() * centered;
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(cov);
	Eigen::Matrix<T, 3, 1> axis = eig.eigenvectors().col(2).normalized();

	Ray<T> result(Vector3d(origin[0], origin[1], origin[2]), Vector3d(axis[0], axis[1], axis[2]));
	err = 0;
	for (const auto& pt : c) {
		err += result.distToPt(pt);
	}
	err /= c.size();

	return result;
}

