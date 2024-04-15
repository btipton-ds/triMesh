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
#include <stdint.h>
#include <assert.h>

#include  <cmath>
#include <climits>

#define operatorDecla(RETURN, OP, PARAM_TYPE, CONST) \
	RETURN operator OP (PARAM_TYPE rhs) CONST;

#define operatorDecl(RETURN, OP, CONST) \
	operatorDecla(RETURN, OP, const double_f&, CONST) \
	operatorDecla(RETURN, OP, double,          CONST) \
	operatorDecla(RETURN, OP, int,             CONST) \
	operatorDecla(RETURN, OP, size_t,          CONST)

template<typename BASE_TYPE>
class double_f {
public:
	double_f();
	double_f(double val);
	double_f(const double_f& src);

	static BASE_TYPE maxVal();
	static double stepSize();
	static double toDouble(const double_f& src);
	static double toDouble(BASE_TYPE src);
	static double_f&& fromDouble(double src);

	operator double() const;
	double_f& operator = (double rhs);
	double_f& operator = (const double_f& rhs);

	// Each operator needs an input type matching whatever types that will be used, to avoid ambiguous casts.
	// There are at least four input types for each operator, so - macros.

	operatorDecl(bool, ==, const );
	operatorDecl(bool, != , const);
	operatorDecl(bool, < , const);
	operatorDecl(bool, <= , const);
	operatorDecl(bool, > , const);
	operatorDecl(bool, >= , const);

#if 0
	operatorDecl(double_f&, +=, );
	operatorDecl(double_f&, -=, );
	operatorDecl(double_f&, *=, );
	operatorDecl(double_f&, /=, );

	operatorDecl(double_f, +, const);
	operatorDecl(double_f, -, const);
	operatorDecl(double_f, *, const);
	operatorDecl(double_f, /, const);
#endif
private:
	static BASE_TYPE toBaseType(double val);

	BASE_TYPE _val;
};

template<>
inline int64_t double_f<int64_t>::maxVal()
{
	return INT64_MAX >> 1;
}

template<>
inline int32_t double_f<int32_t>::maxVal()
{
	return INT32_MAX >> 1;
}

template<>
inline int16_t double_f<int16_t>::maxVal()
{
	return INT16_MAX >> 1;
}

template<typename BASE_TYPE>
inline double double_f<BASE_TYPE>::stepSize()
{
	return 4.0 / maxVal();
}

template<typename BASE_TYPE>
BASE_TYPE double_f<BASE_TYPE>::toBaseType(double src)
{
#if FULL_TESTS
	if (src > 4) {
		assert(!"Out of bounds");
		src = 4;
	} else if (src < -4) {
		assert(!"Out of bounds");
		src = -4;
	}
#endif

	double dVal = maxVal() * 0.25 * src + 0.5;
	BASE_TYPE result = (BASE_TYPE)(dVal);
	return result;
}

template<typename BASE_TYPE>
inline double double_f<BASE_TYPE>::toDouble(BASE_TYPE src)
{
	double result = 4.0 * src / maxVal();
	return result;
}

template<typename BASE_TYPE>
inline double double_f<BASE_TYPE>::toDouble(const double_f& src)
{
	return toDouble(src._val);
}

template<typename BASE_TYPE>
inline double_f<BASE_TYPE>&& double_f<BASE_TYPE>::fromDouble(double src)
{
	double_f result;
	result._val = toBaseType(src);
	return result;
}

template<typename BASE_TYPE>
inline double_f<BASE_TYPE>::double_f()
	: _val(0)
{
}

template<typename BASE_TYPE>
inline double_f<BASE_TYPE>::double_f(double src)
{
	_val = toBaseType(src);
}

template<typename BASE_TYPE>
inline double_f<BASE_TYPE>::double_f(const double_f& src)
	: _val(src._val)
{
}

template<typename BASE_TYPE>
inline double_f<BASE_TYPE>::operator double() const
{
	return toDouble(*this);
}

template<typename BASE_TYPE>
inline double_f<BASE_TYPE>& double_f<BASE_TYPE>::operator = (double rhs)
{
	_val = toBaseType(rhs);
	return *this;
}

template<typename BASE_TYPE>
inline double_f<BASE_TYPE>& double_f<BASE_TYPE>::operator = (const double_f& rhs)
{
	_val = rhs._val;
	return *this;
}

#define operatorCompareImplRaw(OP) \
template<typename BASE_TYPE> \
inline bool double_f<BASE_TYPE>::operator OP (const double_f& rhs) const \
{ \
	return _val OP rhs._val; \
}	

#define operatorCompareImplCast(OP, PARAM_TYPE) \
template<typename BASE_TYPE> \
inline bool double_f<BASE_TYPE>::operator OP (PARAM_TYPE rhs) const \
{ \
	return toDouble(_val) OP (double)rhs; \
}	

#define operatorCompare0Impl(OP) \
	operatorCompareImplRaw(OP) \
	operatorCompareImplCast(OP , double) \
	operatorCompareImplCast(OP , int) \
	operatorCompareImplCast(OP , size_t)
#if 1
// Use this to recreate the implementations if they need to be changed. Expand them for debugging
operatorCompare0Impl(== )
operatorCompare0Impl(!= )
operatorCompare0Impl(< )
operatorCompare0Impl(<= )
operatorCompare0Impl(> )
operatorCompare0Impl(>= )
#endif


/************************************************************* end of comparison op expansion ****************************/

#define operatorEqualImplRaw(OP) \
template<typename BASE_TYPE> \
inline double_f<BASE_TYPE>& double_f<BASE_TYPE>::operator OP= (const double_f& rhs) \
{ \
	_val OP= rhs._val; \
	return *this; \
}	

#define operatorEqualImplCast(OP, PARAM_TYPE) \
template<typename BASE_TYPE> \
inline double_f<BASE_TYPE>& double_f<BASE_TYPE>::operator OP= (PARAM_TYPE rhs) \
{ \
	_val = toBaseType(toDouble(_val) OP rhs); \
	return *this; \
}	

#define operatorEqualImpl(OP) \
	operatorEqualImplRaw(OP) \
	operatorEqualImplCast(OP , double) \
	operatorEqualImplCast(OP , int) \
	operatorEqualImplCast(OP , size_t)

#define operatorEqualImplAllCast(OP) \
	operatorEqualImplCast(OP, const double_f&) \
	operatorEqualImplCast(OP , double) \
	operatorEqualImplCast(OP , int) \
	operatorEqualImplCast(OP , size_t)

/*********************** expansion of ?= operators ************************/
#if 0
operatorEqualImpl(+)
operatorEqualImpl(-)
operatorEqualImplAllCast(*)
operatorEqualImplAllCast(/ )
#endif

/************************************************************* end of comparison op ?= expansion ****************************/

#define operatorImplRefa(OP, PARAM_TYPE) \
template<typename BASE_TYPE> \
inline double_f<BASE_TYPE> double_f<BASE_TYPE>::operator OP (PARAM_TYPE rhs) const \
{ \
	double_f self(*this); \
	self OP= rhs; \
	return self; \
}	

#define operatorImplRef(OP) \
	operatorImplRefa(OP, const double_f&) \
	operatorImplRefa(OP , double) \
	operatorImplRefa(OP , int) \
	operatorImplRefa(OP , size_t)
	
/*********************** expansion of ? operators ************************/
#if 0
operatorImplRef(+)
operatorImplRef(-)
operatorImplRef(*)
operatorImplRef(/ )
#endif


/************************************************************* end of comparison op ? expansion ****************************/

using double_64 = double_f<int64_t>;
using double_32 = double_f<int32_t>;
using double_16 = double_f<int16_t>;
