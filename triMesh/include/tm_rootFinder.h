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

#include <cmath>
#include <vector>
#include <tm_vector3.h>

namespace RootFinder {

template<class T, class ERR_FUNC, class GRAD_FUNC>
T findMin(const Vector3<T>& pt, std::vector<T>& params, const ERR_FUNC& errFunc, const GRAD_FUNC& gradFunc, T stepsize, T tol)
{
	const T DIV_ZERO_VAL = (T)1.0e-12;
	const T MIN_PARAM_DELTA = (T)1.0e-14;
	std::vector<T> gradient, tmpParams;
	gradient.resize(params.size());
	tmpParams.resize(params.size());

	auto err = errFunc(params, gradient, 0);

	int numConvergences = 0;
	while (fabs(err) > tol) {
		gradFunc(params, gradient);

		auto err0 = errFunc(params, gradient, -stepsize);
		auto err1 = errFunc(params, gradient, stepsize);

		auto a = ((err0 - err) + (err1 - err)) / (2 * stepsize * stepsize);
		auto b = (err1 - err0) / (2 * stepsize);

		T deltaX = 0;
		if (fabs(a) > DIV_ZERO_VAL) {
			deltaX = -b / (2 * a);
			if (fabs(b) > DIV_ZERO_VAL) {
				T deltaXLin = -err / b;

				// The best way to determine which delta reduces error the most is
				// to test both ways and choose one
				err0 = errFunc(params, gradient, deltaX);
				err1 = errFunc(params, gradient, deltaXLin);

				// since we have both, update the params and continue from here
				if (err0 < err1) {
					err = err0;
					for (size_t i = 0; i < params.size(); i++) {
						params[i] += gradient[i] * deltaX;
					}
				}
				else {
					err = err1;
					for (size_t i = 0; i < params.size(); i++) {
						params[i] += gradient[i] * deltaXLin;
					}
				}
				continue;
			}
		}
		else if (fabs(b) > DIV_ZERO_VAL) {
			deltaX = -err / b;
		}
		else {
			return err; // We've got the best answer we're going to get
		}

		if (fabs(deltaX) < MIN_PARAM_DELTA) {
			numConvergences++;
			if (numConvergences > 100)
				return err;
		}

		for (size_t i = 0; i < params.size(); i++) {
			params[i] += gradient[i] * deltaX;
		}
		err = errFunc(params, gradient, 0);
	}

	return err;
}

}
