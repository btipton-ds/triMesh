#pragma once
/*
This file is part of the DistFieldHexMesh application/library.

	The DistFieldHexMesh application/library is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	The DistFieldHexMesh application/library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	This link provides the exact terms of the GPL license <https://www.gnu.org/licenses/>.

	The author's interpretation of GPL 3 is that if you receive money for the use or distribution of the DistFieldHexMesh application/library or a derivative product, GPL 3 no longer applies.

	Under those circumstances, the author expects and may legally pursue a reasoble share of the income. To avoid the complexity of agreements and negotiation, the author makes
	no specific demands in this regard. Compensation of roughly 1% of net or $5 per user license seems appropriate, but is not legally binding.

	In lay terms, if you make a profit by using the DistFieldHexMesh application/library (violating the spirit of Open Source Software), I expect a reasonable share for my efforts.

	Robert R Tipton - Author

	Dark Sky Innovative Solutions http://darkskyinnovation.com/
*/

#include <tm_defines.h>
#include <vector>
#include <tm_iterator.h>


#define ITER_TEMPL_DECL template <class SOURCE, class T, int IterType>
#define ITER_DECL tm_iterator<SOURCE, T, IterType>

namespace TriMesh {
	using namespace ::std;

ITER_TEMPL_DECL
ITER_DECL::tm_iterator(const SOURCE* pSource, std::conditional_t<IS_ITER_CONST, size_t const*, size_t*> pEntry)
: _pSource(const_cast<SOURCE*>(pSource))
, _pEntry(const_cast<size_t*>(pEntry))
{
}

	
ITER_TEMPL_DECL
bool ITER_DECL::operator == (const tm_iterator& rhs) const
{
	//	assert(_pSource == _pSource);
	return _pEntry == rhs._pEntry;
}

	
ITER_TEMPL_DECL
bool ITER_DECL::operator != (const tm_iterator& rhs) const
{
	//	assert(_pSource == _pSource);
	return _pEntry != rhs._pEntry;
}

	
ITER_TEMPL_DECL
bool ITER_DECL::operator < (const tm_iterator& rhs) const
{
	//	assert(_pSource == _pSource);
	return _pEntry < rhs._pEntry;
}

	
ITER_TEMPL_DECL
bool ITER_DECL::operator > (const tm_iterator& rhs) const
{
	//	assert(_pSource == _pSource);
	return _pEntry > rhs._pEntry;
}

	
ITER_TEMPL_DECL
ITER_DECL& ITER_DECL::operator ++ ()		// prefix
{
	if (!_pEntry)
		return *this;

	if (IterType == FORW_CONST || IterType == FORW)
		_pEntry++;
	else
		_pEntry--;
	return *this;
}

	
ITER_TEMPL_DECL
ITER_DECL ITER_DECL::operator ++ (int)
{
	tm_iterator tmp(*this);
	++*this; // call prefix ++
	return tmp;
}
	
ITER_TEMPL_DECL
ITER_DECL& ITER_DECL::operator --()
{
	if (!_pEntry)
		return *this;

	if (IterType == FORW_CONST || IterType == FORW)
		_pEntry--;
	else
		_pEntry++;
	return *this;
}
	
ITER_TEMPL_DECL
ITER_DECL ITER_DECL::operator --(int)
{
	tm_iterator tmp(*this);
	--*this; // call prefix --
	return tmp;
}
	
ITER_TEMPL_DECL
ITER_DECL ITER_DECL::operator + (size_t rhs) const
{
	tm_iterator result(*this);
	if (IterType == FORW_CONST || IterType == FORW)
		result._pEntry += rhs;
	else
		result._pEntry -= rhs;
	return result;
}
	
ITER_TEMPL_DECL
ITER_DECL ITER_DECL::operator - (size_t rhs) const
{
	tm_iterator result(*this);
	if (IterType == FORW_CONST || IterType == FORW)
		result._pEntry -= rhs;
	else
		result._pEntry += rhs;
	return result;
}
	
ITER_TEMPL_DECL
size_t ITER_DECL::operator - (const tm_iterator& rhs) const
{
	if (IterType == FORW_CONST || IterType == FORW) {
		return (size_t)(_pEntry - rhs._pEntry);
	}
	else {
		return (size_t)(rhs._pEntry - _pEntry);
	}
}
	
ITER_TEMPL_DECL
inline typename ITER_DECL::reference ITER_DECL::operator *() const
{
	return *get();
}
	
ITER_TEMPL_DECL
inline typename ITER_DECL::pointer ITER_DECL::operator->() const
{
	return get();
}
	
ITER_TEMPL_DECL
inline typename ITER_DECL::pointer ITER_DECL::get() const
{
	size_t idx = -1;
	if (_pEntry)
		idx = (size_t)(_pEntry - _pSource->_indices.data());
	else if (_pSource) {
		// null indicates end of the array
		idx = -1;
	}
	if (0 <= idx && idx < _pSource->size())
		return &(*_pSource)[idx];

	return nullptr;
}

}

#undef ITER_TEMPL_DECL
#undef ITER_DECL
