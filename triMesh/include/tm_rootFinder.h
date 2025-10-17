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
#include <tm_util.h>

namespace RootFinder {


template <class T>
void normalizeStdVector(std::vector<T>& grad)
{
	T avgSqrs = 0;
	for (T v : grad) {
		avgSqrs += v * v;
	}
	avgSqrs = sqrt(avgSqrs);
	for (T& v : grad) {
		v /= avgSqrs;
	}
}

template <class T, typename ERR_FUNC, typename GRAD_FUNC>
bool findMin(const std::vector<T>& pt, std::vector<T>& params, const ERR_FUNC& errFunc, const GRAD_FUNC& gradFunc, T resultErr, T stepSize)
{
	T slope = DBL_MAX;
	T err = errFunc(params);
	std::vector<T> grad, tempParams;
	grad.resize(params.size(), 0);
	tempParams.resize(params.size(), 0);
	while (err > resultErr) {
		T err0, err1;
		gradFunc(params, grad);

		for (size_t i = 0; i < params.size(); i++) 
			tempParams[i] = params[i] - stepSize * grad[i];
		
		err0 = errFunc(tempParams);

		for (size_t i = 0; i < params.size(); i++) 
			tempParams[i] = params[i] + stepSize * grad[i];
		
		err1 = errFunc(tempParams);

		auto a = ((err0 - err) + (err1 - err)) / (2 * stepSize * stepSize);
		slope = (err1 - err0) / (2 * stepSize);
		T deltaX = FLT_MAX;
		if (fabs(a) > 1.0e-12) {
			// It took awhile to get this right.
			// As you approach the bottom the parabola, the slope goes to zero, so the slope estimated x approaches infinity,
			// but the parabolic estimate is now more accurate.
			// On the steep part of the parabola, a is almost zero and slope is large, so it produced the best estimate.
			// So...
			// Choose the one that's smallest.
			deltaX = -slope / (2 * a);
			T deltaX1 = -err / slope;
			if (fabs(deltaX1) < fabs(deltaX))
				deltaX = deltaX1;
		} else if (fabs(slope) > resultErr)
			deltaX = -err / slope;
		else {
			break;
		}

		if (fabs(deltaX) < FLT_MAX) {
			for (size_t i = 0; i < params.size(); i++) {
				params[i] += deltaX * grad[i];
			}
		}
		err = errFunc(params);
	}

	return true;
}

}