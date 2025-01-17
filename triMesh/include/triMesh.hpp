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

#include <triMesh.h>

namespace TriMesh {

template<class T>
size_t CMesh::addRectPrism(const std::vector<Vector3<T>>& pts)
{
	if (pts.size() != 8)
		return -1;

	// add bottom and top
	addQuad(pts[0], pts[3], pts[2], pts[1]);
	addQuad(pts[4], pts[5], pts[6], pts[7]);

	// add left and right
	addQuad(pts[0], pts[4], pts[7], pts[3]);
	addQuad(pts[1], pts[2], pts[6], pts[5]);

	// add front and back
	addQuad(pts[0], pts[1], pts[5], pts[4]);
	addQuad(pts[2], pts[3], pts[7], pts[6]);

	return _tris.size();
}

template<typename LAMBDA>
const std::vector<float>& CMesh::getGlTriCurvatureColors(LAMBDA curvatureToColorFunc) const // size = GlPoints.size() / 3
{
	size_t requiredSize = 3 * 3 * _tris.size();
	if (_vertCurvature.empty()) {
		_glTriCurvatureColors.clear();
	}
	else if (_glTriCurvatureColors.size() == requiredSize)
		return _glTriCurvatureColors;
	else {
		_glTriCurvatureColors.clear();
		_glTriCurvatureColors.reserve(requiredSize);

		for (size_t triIdx = 0; triIdx < _tris.size(); triIdx++) {
			const auto& tri = _tris[triIdx];
			for (int i = 0; i < 3; i++) {
				auto curv = _vertCurvature[tri[i]];
				float rgb[3];
				if (curvatureToColorFunc(curv, rgb)) {
					_glTriCurvatureColors.push_back(rgb[0]);
					_glTriCurvatureColors.push_back(rgb[1]);
					_glTriCurvatureColors.push_back(rgb[2]);
				}
			}
		}
	}

	return _glTriCurvatureColors;
}

template<typename LAMBDA>
void CMesh::getGlEdges(LAMBDA curvatureToColorFunc, bool includeSmooth, std::vector<float>& points, std::vector<float>& colors,
	double sinSharpAngle, std::vector<unsigned int>& sharpIndices, std::vector<unsigned int>& smoothIndices) // size = GlPoints.size() / 3
{
	points.clear();
	colors.clear();
	sharpIndices.clear();
	smoothIndices.clear();

	points.reserve(2 * 3 * _edges.size());
	sharpIndices.reserve(2 * _edges.size());
	smoothIndices.reserve(2 * _edges.size());

	unsigned int indexCount = 0;
	for (size_t edgeIdx = 0; edgeIdx < _edges.size(); edgeIdx++) {
		float curv = (float)_edgeCurvature[edgeIdx];
		if (!includeSmooth && fabs(curv) < 0.1)
			continue;

		unsigned int idx0 = indexCount++;
		unsigned int idx1 = indexCount++;

		if (isEdgeSharp(edgeIdx, sinSharpAngle)) {
			sharpIndices.push_back(idx0);
			sharpIndices.push_back(idx1);
		}
		else {
			smoothIndices.push_back(idx0);
			smoothIndices.push_back(idx1);
		}

		float rgb[3];
		if (curvatureToColorFunc(curv, rgb)) {

			const auto& edge = _edges[edgeIdx];
			for (int i = 0; i < 2; i++) {
				const auto& pt = getVert(edge._vertIndex[i])._pt;
				for (int j = 0; j < 3; j++) {
					points.push_back((float)pt[j]);
					colors.push_back(rgb[j]);
				}
			}
		}
		else {
			for (int j = 0; j < 3; j++) {
				colors.push_back(0);
			}
		}
	}
	points.shrink_to_fit();
	colors.shrink_to_fit();
	sharpIndices.shrink_to_fit();
	smoothIndices.shrink_to_fit();
}

}
