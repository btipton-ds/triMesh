#pragma once

#define LINE_SEG_2D_INTERSECT_TEMP \
	const auto tolSqr = tol * tol;\
const auto paramTol = tol;\
\
Vector2<T> segOrigin = _pt0; \
Vector2<T> xAxis = (_pt1 - _pt0); \
auto ourLenSqr = xAxis.squaredNorm(); \
if (ourLenSqr < tolSqr) \
	return false; /* Degenerate segment */ \
\
auto ourLen = sqrt(ourLenSqr);\
xAxis /= ourLen;\
\
Vector2<T> yAxis(-xAxis[1], xAxis[0]);\
\
Vector2<T> localRayStart(xAxis.dot(other._pt0 - segOrigin), yAxis.dot(other._pt0 - segOrigin));\
Vector2<T> localRayTip(xAxis.dot(other._pt1 - segOrigin), yAxis.dot(other._pt1 - segOrigin));\
Vector2<T> localRayDir = (localRayTip - localRayStart).normalized();\
\
T x;\
auto x0 = localRayStart[0];\
auto y0 = localRayStart[1];\
if (fabs(localRayDir[0]) < paramTol) {\
	/* Vertical ray*/ \
	x = x0;\
}\
else {\
	/* y0 = x0 * slope + b*/ \
	auto slope = localRayDir[1] / localRayDir[0];\
	if (fabs(slope) > paramTol) {\
		auto b = y0 - x0 * slope;\
		x = -b / slope;\
	}\
	else {\
		return false; /* This is an infinite number intersections or no intersection. Let another seg handle this case.*/ \
	}\
}\
\
iPt = segOrigin + x * xAxis;\
if (distanceToPoint(iPt) > tol || other.distanceToPoint(iPt) > tol)\
		return false; \
return true;
