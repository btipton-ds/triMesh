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
#include <tm_lineSegment.hpp>
#include <tm_lineSegment_byref.hpp>

template <class SCALAR_TYPE>
inline bool tolerantEquals(SCALAR_TYPE v0, SCALAR_TYPE v1, SCALAR_TYPE tol) {
	return fabs(v1 - v0) < tol;
}

template <class SCALAR_TYPE>
inline bool tolerantEquals(const Vector3< SCALAR_TYPE>& pt0, const Vector3< SCALAR_TYPE>& pt1, SCALAR_TYPE tol) {
	return (pt1 - pt0).squaredNorm() < tol * tol;
}

#define CHECK_NAN 1

template <class SCALAR_TYPE>
inline void checkNAN(SCALAR_TYPE val) {
#if CHECK_NAN
	if (std::isnan(val) || std::isinf(val))
		assert(!"nan");
#endif
}

template <class SCALAR_TYPE>
inline void checkNAN(const Vector3<SCALAR_TYPE>& pt) {
#if CHECK_NAN
	for (int i = 0; i < 3; i++)
		checkNAN(pt[i]);
#endif
}

template <class V>
inline bool Vector3Comp::operator()(const V& lhs, const V& rhs) const {
	for (int i = 0; i < 3; i++) {
		if (lhs[i] < rhs[i])
			return true;
		else if (lhs[i] > rhs[i])
			return false;
	}
	return false;
}

template <class V>
size_t Vector3Set<V>::count(const V& v) const {
	size_t result = 0;
	V delta(SAME_DIST_TOL, SAME_DIST_TOL, SAME_DIST_TOL);
	const auto lb = this->lower_bound(v - delta);
	const auto ub = this->upper_bound(v + delta);
	for (auto i = lb; i != ub; i++)
		result++;

	return result;
}

template<typename SCALAR_TYPE>
Vector3<SCALAR_TYPE> safeNormalize(const Vector3<SCALAR_TYPE>& v) {
	auto l = v.norm();
	if (l > minNormalizeDivisor)
		return v / l;
	std::string msg = std::string(__FILE__) + ":" + std::to_string(__LINE__) + std::string(" zero length vector");
	assert(!" zero length vector");
	throw std::runtime_error(msg);
}

template<class T>
double distanceFromPlane(const Vector3<T>& pt, const Plane<T>& plane) {
	return (pt - plane.getOrigin()).dot(plane.getNormal());
}

template<class T>
bool pointInTriangle(const Vector3<T>& pt0, const Vector3<T>& pt1, const Vector3<T>& pt2, const Vector3<T>& pt, T tol)
{
	const Vector3<T>* pts[] = { &pt0, &pt1, &pt2 };
	return pointInTriangle(pts, pt, tol);
}

template<class T>
bool pointInTriangle(const Vector3<T>* const* pts, const Vector3<T>& pt, T tol)
{
	Vector3<T> norm = triangleUnitNormal(pts);
	return pointInTriangle(pts, pt, norm, tol);
}

template<class T>
bool pointInTriangle(const Vector3<T>& pt0, const Vector3<T>& pt1, const Vector3<T>& pt2, const Vector3<T>& pt, const Vector3<T>& unitNorm, T tol)
{
	const Vector3<T>* pts[] = { &pt0, &pt1, &pt2 };
	return pointInTriangle(pts, pt, unitNorm, tol);
}

template<class T>
bool pointInTriangle(const Vector3<T>* const* pts, const Vector3<T>& pt, const Vector3<T>& unitNorm, T tol)
{
#ifdef _DEBUG
	{
		Vector3<T> v0 = (*pts[1]) - (*pts[0]);
		Vector3<T> v1 = (*pts[2]) - (*pts[0]);

		v0 = pt - (*pts[0]);
		T dp = v0.dot(unitNorm);
		assert(fabs(dp) < 1.0e-6);
	}
#endif

	size_t numPos = 0, numNeg = 0;
	for (size_t i = 0; i < 3; i++) {
		size_t j = (i + 1) % 3;

		Vector3<T> vLeg = (*pts[j]) - (*pts[i]);
		Vector3<T> vPerp = unitNorm.cross(vLeg);
		vPerp.normalize();

		Vector3<T> v0 = pt - (*pts[i]);
		T dist = v0.dot(vPerp);
		if (dist < tol)
			numNeg++;
		if (dist > -tol)
			numPos++;
	}

	return numNeg == 3 || numPos == 3;
}

template<class T>
bool intersectRayTri(const Ray<T>& ray, const Vector3<T>& pt0, const Vector3<T>& pt1, const Vector3<T>& pt2, RayHit<T>& hit, T tol) {
	const Vector3<T>* pts[] = { &pt0, &pt1, &pt2 };
	return intersectRayTri(ray, pts, hit, tol);
}

template<class T>
bool intersectRayTri(const Ray<T>& ray, const Vector3<T>* const* pts, RayHit<T>& hit, T tol) {

	Vector3<T> v0 = *pts[1] - *pts[0];
	Vector3<T> v1 = *pts[2] - *pts[0];
	
	Vector3<T> norm = v0.cross(v1);
	T l = norm.norm();
	if (l < minNormalizeDivisor)
		return false;
	norm /= l;

	Plane<T> pl(*(pts[0]), norm);
	if (!pl.intersectRay(ray, hit, tol))
		return false;

	return pointInTriangle(pts, hit.hitPt, norm, tol);
}

template<class T>
Vector3<T> orthoganalizeVector(const Vector3<T>& v, const Vector3<T>& unitVector)
{
	return v - unitVector * unitVector.dot(v);
}

template<class T>
Vector3<T> orthoganalizePoint(const Vector3<T>& origin, const Vector3<T>& unitVector, const Vector3<T>& pt)
{
	Vector3<T> v = pt - origin;
	v = orthoganalize(v, unitVector);
	return origin + v;
}

template<class T>
Vector3<T> triangleUnitNormal(const Vector3<T>* pts[3]) {
	Vector3<T> v0 = *pts[1] - *pts[0];
	Vector3<T> v1 = *pts[2] - *pts[0];
	Vector3<T> n = safeNormalize(v0.cross(v1));
	return n;
}

template<class T>
Vector3<T> triangleUnitNormal(const Vector3<T> pts[3]) {
	Vector3<T> v0 = pts[1] - pts[0];
	Vector3<T> v1 = pts[2] - pts[0];
	Vector3<T> n = safeNormalize(v0.cross(v1));
	return n;
}

// LERP functions are usually used for points, but can be used for any kind of value that supports +, -  and *
template<class T>
inline Vector3<T> LERP(const Vector3<T>& p0, const Vector3<T>& p1, T t)
{
	return p0 + t * (p1 - p0);
}

// pts must be size 8 or greater. No bounds checking is done.
template<class T>
inline Vector3<T> TRI_LERP(const std::vector<Vector3<T>>& pts, T t, T u, T v)
{
	return TRI_LERP(pts.data(), t, u, v);
}

template<class T>
inline Vector3<T> TRI_LERP(const std::vector<Vector3<T>>& pts, const Vector3<T>& uvw)
{
	return TRI_LERP(pts, uvw[0], uvw[1], uvw[2]);
}

template<class T>
Vector3<T> ngonCentroid(int numPoints, Vector3<T> const* const pts[]) {
	Vector3<T> result(0, 0, 0);
	for (int i = 0; i < numPoints; i++)
		result += *pts[i];
	result /= numPoints;
	return result;
}

template<class T>
Vector3<T> ngonCentroid(int numPoints, const Vector3<T> pts[]) {
	Vector3<T> const* pPts[] = {
		&pts[0],
		&pts[1],
		&pts[2],
	};
	return ngonCentroid(numPoints, pPts);
}

template<class T>
inline Vector3<T> triangleCentroid(Vector3<T> const* const pts[3]) {
	return ngonCentroid(3, pts);
}
template<class T>
inline Vector3<T> triangleCentroid(const Vector3<T> pts[]) {
	return ngonCentroid(3, pts);
}

template<class T>
T volumeUnderTriangle(Vector3<T> const* const pts[3], const Vector3<T>& axis) {
	Vector3<T> centroid = triangleCentroid(pts);
	Vector3<T> v0 = *pts[1] - *pts[0];
	Vector3<T> v1 = *pts[2] - *pts[0];
	Vector3<T> cp = v0.cross(v1);
	T area = cp.dot(axis) * (T)0.5;
	T h = centroid.dot(axis);
	T vol = area * h;
	return vol;
}

template<class T>
int triangleSplitWithPlane(Vector3<T> const triPts[3], const Plane<T>& plane,
	Vector3<T> triPts0[3], Vector3<T> triPts1[3], Vector3<T> triPts2[3], T tol)
{
	int numSplits = 0;
	Vector3<T> splitPts[2];
	int splitLegIndices[2];
	int numIntersectVertIndices = 0;
	int intersectVertIndices[3];

	for (int i = 0; i < 3; i++) {
		int j = (i + 1) % 3;
		RayHit<T> hit;
		LineSegment<T> seg(triPts[i], triPts[j]);
		if (plane.intersectLineSegment_rev0(seg, hit, tol)) {
			if (hit.dist < tol) {
				// This vertex lies on the plane
				intersectVertIndices[numIntersectVertIndices++] = i;
			}
			else {
				for (int k = 0; k < numIntersectVertIndices; k++) {
					if (intersectVertIndices[k] == i || intersectVertIndices[k] == j)
						continue;
				}
				bool found = false;
				for (int k = 0; k < 3; k++) {
					if (tolerantEquals(hit.hitPt, triPts[k], tol)) {
						found = true;
						break;
					}
				}

				if (!found) {
					splitLegIndices[numSplits] = i;
					splitPts[numSplits] = hit.hitPt;
					numSplits++;
				}
			}
		}
	}

	// The found check will screen out a leg which lies on the plane.

	if (numSplits == 1 && numIntersectVertIndices == 1) {
		// The plane splits the triangle and a vertex lies on the plane

		int vertIdx = intersectVertIndices[0];
		triPts0[0] = triPts[vertIdx];
		triPts0[1] = triPts[(vertIdx + 1) % 3];
		triPts0[2] = splitPts[0];

		triPts1[0] = triPts[vertIdx];
		triPts1[1] = splitPts[0];
		triPts1[2] = triPts[(vertIdx + 2) % 3];
		return 2;
	}
	else if (numSplits == 2) {
		// The plane splits two legs
		int triIdx0;
		if (0 == splitLegIndices[0] && 2 == splitLegIndices[1]) {
			triIdx0 = 0;
		}
		else if (0 == splitLegIndices[0] && 1 == splitLegIndices[1]) {
			triIdx0 = 1;
		}
		else {
			triIdx0 = 2;
		}

		int triIdx1, triIdx2, ptIdx0, ptIdx1;

		triIdx1 = (triIdx0 + 1) % 3;
		triIdx2 = (triIdx1 + 1) % 3;

		switch (triIdx0) {
		case 0:
			ptIdx0 = 0;
			ptIdx1 = 1;
			break;
		case 1:
			ptIdx0 = 1;
			ptIdx1 = 0;
			break;
		case 2:
			ptIdx0 = 1;
			ptIdx1 = 0;
			break;
		}

		Vector3<T> quadPts[4];
		quadPts[0] = splitPts[ptIdx0];
		quadPts[1] = triPts[triIdx1];
		quadPts[2] = triPts[triIdx2];
		quadPts[3] = splitPts[ptIdx1];

		// Make the triangle
		triPts0[0] = triPts[triIdx0];
		triPts0[1] = splitPts[ptIdx0];
		triPts0[2] = splitPts[ptIdx1];

		// make the two triangles forming the quad
		triPts1[0] = quadPts[0];
		triPts1[1] = quadPts[1];
		triPts1[2] = quadPts[2];

		triPts2[0] = quadPts[0];
		triPts2[1] = quadPts[2];
		triPts2[2] = quadPts[3];

		return 3;
	}
	return 0;
}

template<class T>
Vector3<T> BI_LERP(const Vector3<T>& p0, const Vector3<T>& p1, const Vector3<T>& p2, const Vector3<T>& p3, T t, T u)
{
	auto pt0 = LERP(p0, p1, t);
	auto pt1 = LERP(p3, p2, t);

	return pt0 + u * (pt1 - pt0);
}

// pts must be size 8 or greater. No bounds checking is done.
template<class T>
Vector3<T> TRI_LERP(const Vector3<T> pts[8], T t, T u, T v)
{
	auto pt0 = BI_LERP(pts[0], pts[1], pts[2], pts[3], t, u);
	auto pt1 = BI_LERP(pts[4], pts[5], pts[6], pts[7], t, u);

	return pt0 + v * (pt1 - pt0);
}

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
				err0 = errFunc(params, gradient, deltaX);
				err1 = errFunc(params, gradient, deltaXLin);
				if (err1 < err0)
					deltaX = deltaXLin;
			}
		} else if (fabs(b) > DIV_ZERO_VAL) {
			deltaX = -err / b;
		} else {
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
template<class T>
bool TRI_LERP_INV(const Vector3<T>& pt, const std::vector<Vector3<T>>& pts, Vector3<T>& tuv, T tol)
{
	const double tolSqr = tol * tol;
	if (pts.empty())
		return false;

	/*
	Current method is relaxation. Not the best but it works.
	*/
	T a, b, c, l;
	Vector3<T> x, y, z, p0, p1, dir, v;

	x = pts[1] - pts[0];
	l = x.norm();
	x /= l * l;

	y = pts[3] - pts[0];
	l = y.norm();
	y /= l * l;

	z = pts[4] - pts[0];
	l = z.norm();
	z /= l * l;

	v = pt - pts[0];
	tuv = Vector3<T>(v.dot(x), v.dot(y), v.dot(z));
#if 1
	auto errFunc = [&pt, &pts](const std::vector<T>& params, const std::vector<T>& gradient, T stepsize)->T {

		const auto& testPt = TRI_LERP(pts, 
			params[0] + gradient[0] * stepsize, 
			params[1] + gradient[1] * stepsize, 
			params[2] + gradient[2] * stepsize);
		Vector3<T> v = testPt - pt;
		return v.norm();
		};

	auto gradFunc = [&pt, &pts, &errFunc](const std::vector<T>& params, std::vector<T>& gradient) {
		const auto t = params[0];
		const auto u = params[1];
		const auto v = params[2];
#if 0
		std::vector<T> tmpParams(params);
		T err = errFunc(tmpParams);
		const T dp = (T)1.0e-6;
		for (int i = 0; i < gradient.size(); i++) {
			T curParam = tmpParams[i];

			tmpParams[i] = curParam - dp;
			T err0 = errFunc(tmpParams);

			tmpParams[i] = curParam + dp;
			T err1 = errFunc(tmpParams);

			gradient[i] = (err1 - err0) / (2 * dp);
			tmpParams[i] = curParam;
		}
#else
		Vector3<T> v01(pts[1] - pts[0]);
		Vector3<T> v03(pts[3] - pts[0]);
		Vector3<T> v04(pts[4] - pts[0]);
		Vector3<T> v45(pts[5] - pts[4]);
		Vector3<T> vDen = (-((((pts[6] - pts[7]) * t - (v45) * t + pts[7] - pts[4]) * u - ((pts[2] - pts[3]) * t - v01 * t + v03) * u + (v45) * t - v01 * t + v04) * v) - ((pts[2] - pts[3]) * t - v01 * t + v03) * u - v01 * t + pt - pts[0]);
		T den = 2 * vDen.norm();

		Vector3<T> termA = (-(((-pts[7] + pts[6] - pts[5] + pts[4]) * u - (-pts[3] + pts[2] - pts[1] + pts[0]) * u + v45 - pts[1] + pts[0]) * v)
			- (-pts[3] + pts[2] - pts[1] + pts[0]) * u - pts[1] + pts[0]);
		Vector3<T> termB = (-((((pts[6] - pts[7]) * t - (v45) * t + pts[7] - pts[4]) * u -
			((pts[2] - pts[3]) * t - v01 * t + v03) * u + (v45) * t - v01 * t + v04) * v) - 
			((pts[2] - pts[3]) * t - v01 * t + v03) * u - v01 * t + pt - pts[0]);

		gradient[0] = 2 * (
			termA[0] * termB[0] + 
			termA[1] * termB[1] +
			termA[2] * termB[2] 
			) / den;

		termA = (-(((pts[6] - pts[7]) * t - (v45) * t - (pts[2] - pts[3]) * t + v01 * t + pts[7] - pts[4] - pts[3] + pts[0]) * v) -
			(pts[2] - pts[3]) * t + v01 * t - pts[3] + pts[0]);
		termB = (-((((pts[6] - pts[7]) * t - (v45) * t + pts[7] - pts[4]) * u -
			((pts[2] - pts[3]) * t - v01 * t + v03) * u + (v45) * t - v01 * t + v04) * v) -
			((pts[2] - pts[3]) * t - v01 * t + v03) * u - v01 * t + pt - pts[0]);

		gradient[1] = 2 * (
			termA[0] * termB[0] +
			termA[1] * termB[1] +
			termA[2] * termB[2]
			) / den;

		termA = (-(((pts[6] - pts[7]) * t - (v45) * t + pts[7] - pts[4]) * u) + ((pts[2] - pts[3]) * t - v01 * t + v03) * u - 
			(v45) * t + v01 * t - pts[4] + pts[0]);
		termB = (-((((pts[6] - pts[7]) * t - (v45) * t + pts[7] - pts[4]) * u - ((pts[2] - pts[3]) * t - v01 * t + v03) * u + 
			(v45) * t - v01 * t + v04) * v) - ((pts[2] - pts[3]) * t - v01 * t + v03) * u - v01 * t + pt - pts[0]);

		gradient[2] = 2 * (
			termA[0] * termB[0] +
			termA[1] * termB[1] +
			termA[2] * termB[2]
			) / den;
#endif

		T sum = 0;
		for (const auto& v : gradient)
			sum += v * v;
		sum = sqrt(sum);
		for (auto& v : gradient)
			v /= sum;
	};

	std::vector<T> params = { tuv[0], tuv[1], tuv[2] };
	T err = RootFinder::findMin(pt, params, errFunc, gradFunc, (T)1.0e-8, tol);
	if (err < tol) {
		for (int i = 0; i < 3; i++) {
			T& val = params[i];
			if (val < 0) {
				if (val > -tol)
					val = 0;
				else
					return false;
			} else if (val > 1) {
				if (val < 1 + tol)
					val = 1;
				else
					return false;
			}

			tuv[i] = val;
		}
		return true;
	}

	return false;
#else
	bool done = false;
	int count = 0;
	while (!done && count++ < 100) {
		p0 = TRI_LERP(pts, (T)0, tuv[1], tuv[2]);
		p1 = TRI_LERP(pts, (T)1, tuv[1], tuv[2]);
		dir = p1 - p0;
		l = dir.norm();
		dir /= l;
		v = pt - p0;
		a = v.dot(dir) / l;

		p0 = TRI_LERP(pts, tuv[0], (T)0, tuv[2]);
		p1 = TRI_LERP(pts, tuv[0], (T)1, tuv[2]);
		dir = p1 - p0;
		l = dir.norm();
		dir /= l;
		v = pt - p0;
		b = v.dot(dir) / l;

		p0 = TRI_LERP(pts, tuv[0], tuv[1], (T)0);
		p1 = TRI_LERP(pts, tuv[0], tuv[1], (T)1);
		dir = p1 - p0;
		l = dir.norm();
		dir /= l;
		v = pt - p0;
		c = v.dot(dir) / l;

		tuv = Vector3<T>(a, b, c);
		Vector3<T> guess = TRI_LERP(pts, tuv);
		T distSqr = (guess - pt).squaredNorm();
		if (distSqr < tolSqr)
			return true;
	}
	return false;
#endif
}

template<class T>
bool intersectTriTri(const Vector3<T> triPts0[3], const Vector3<T> triPts1[3], T tol)
{
	const Vector3<T>* pPts0[] = { &triPts0[0], &triPts0[1], &triPts0[2] };
	const Vector3<T>* pPts1[] = { &triPts1[0], &triPts1[1], &triPts1[2] };
	return intersectTriTri(pPts0, pPts1, tol);
}

template<class T>
bool intersectTriTri(class Vector3<T> const* const* triPts0, class Vector3<T> const* const* triPts1, T tol)
{
#if 1
	RayHit<T> hit;

	Plane<T> triPlane0(triPts0, false);

	LineSegment<T> iSeg0;
	if (!triPlane0.intersectTri(triPts1, iSeg0, tol))
		return false;

	Plane<T> triPlane1(triPts1, false);

	LineSegment<T> iSeg1;
	if (!triPlane1.intersectTri(triPts0, iSeg1, tol))
		return false;

#if 0
	Ray<T> ray0(iSeg0._pt0, iSeg0.calcDir());
	if (ray0.distToPt(iSeg1._pt0) > tol || ray0.distToPt(iSeg1._pt1) > tol)
		return false;

	Ray<T> ray1(iSeg1._pt0, iSeg1.calcDir());
	if (ray0.distToPt(iSeg1._pt0) > tol || ray0.distToPt(iSeg1._pt1) > tol)
		return false;
#endif

	auto looseTol = 10 * tol;
	T t;
	if (iSeg0.contains(iSeg1._pt0, t, looseTol) || iSeg0.contains(iSeg1._pt1, t, looseTol))
		return true;

	if (iSeg1.contains(iSeg0._pt0, t, looseTol) || iSeg1.contains(iSeg0._pt1, t, looseTol))
		return true;

	return false;
#else
	RayHit<T> hit;

	// Check 1 against 0
	const Vector3<T>* pts0[] = { triPts0[0], triPts0[1], triPts0[2] };
	Vector3<T> norm0 = triangleUnitNormal(pts0);
	Plane_byref<T> triPlane0(*pts0[0], norm0);

	for (int i = 0; i < 3; i++) {
		int j = (i + 1) % 3;

		LineSegment_byref<T> seg(*triPts1[i], *triPts1[j]);
		if (triPlane0.intersectLineSegment_rev0(seg, hit, tol)) {
			if (pointInTriangle<T>(triPts0, hit.hitPt, norm0, tol))
				return true;
		}
	}

	// Check 0 against 1
	const Vector3<T>* pts1[] = { triPts1[0], triPts1[1], triPts1[2] };
	Vector3<T> norm1 = triangleUnitNormal(pts1);
	Plane_byref<T> triPlane1(*pts1[0], norm1);

	for (int i = 0; i < 3; i++) {
		int j = (i + 1) % 3;

		LineSegment_byref<T> seg(*triPts0[i], *triPts0[j]);

		if (triPlane1.intersectLineSegment_rev0(seg, hit, tol)) {
			if (pointInTriangle<T>(triPts1, hit.hitPt, norm1, tol))
				return true;
		}
	}

	return false;
#endif
}
