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
#include <type_traits>
#include <memory>
#include <map>
#include <vector>
#include <iostream>

#define IS_ITER_CONST (IterType < FORW)

namespace TriMesh {
	enum ITER_DIR {
		FORW_CONST,
		REV_CONST,
		FORW,
		REV,
	};

	template <class SOURCE, class T, int IterType>
	class tm_iterator
	{
	public:
		using iterator_category = std::random_access_iterator_tag;
		using difference_type = std::ptrdiff_t;

		using value_type = std::remove_cv_t<T>;
		using pointer = std::conditional_t<IS_ITER_CONST, T const*, T*>;
		using reference = std::conditional_t<IS_ITER_CONST, T const&, T&>;

		tm_iterator() = default;
		tm_iterator(const SOURCE* pSource, std::conditional_t<IS_ITER_CONST, size_t const*, size_t*> pEntry);
		tm_iterator(const tm_iterator& src) = default;

		bool operator == (const tm_iterator& rhs) const;
		bool operator != (const tm_iterator& rhs) const;
		bool operator < (const tm_iterator& rhs) const;
		bool operator > (const tm_iterator& rhs) const;

		tm_iterator& operator ++ ();		// prefix
		tm_iterator& operator --();		// prefix
		tm_iterator operator ++ (int);	// postfix
		tm_iterator operator --(int);		// postfix

		tm_iterator operator + (size_t val) const;

		tm_iterator operator - (size_t val) const;
		size_t operator - (const tm_iterator& rhs) const;

		reference operator *() const;
		pointer operator->() const;
		pointer get() const;

	private:
		size_t* _pEntry; // Resizing/reserving invalidates the iterator. Don't keep them across these calls
		SOURCE* _pSource;
	};

}