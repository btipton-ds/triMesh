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
#include <memory>
#include <vector>
#include <tm_vector3.h>
#include <tm_iterator.h>

#define IS_ITER_CONST (IterType < FORW)

namespace TriMesh {
	class CMesh;
	using CMeshPtr = std::shared_ptr<CMesh>;

	template<class T>
	class ProxyVector
	{
	public:
		using iterator = tm_iterator<ProxyVector<T>, T, FORW>;
		using const_iterator = tm_iterator<ProxyVector<T>, T, FORW_CONST>;
		using reverse_iterator = tm_iterator<ProxyVector<T>, T, REV>;
		using const_reverse_iterator = tm_iterator<ProxyVector<T>, T, REV_CONST>;

		friend class iterator;
		friend class const_iterator;
		friend class reverse_iterator;
		friend class const_reverse_iterator;

		ProxyVector(CMesh* pMesh);

		T& operator[](size_t idx);
		const T& operator[](size_t idx) const;
		size_t size() const;
		void push_back(const T& idx);
		void pop_back();

		const_iterator begin() const noexcept;
		iterator begin() noexcept;
		const_iterator end() const noexcept;
		iterator end() noexcept;

		std::vector<size_t>& getIndices();
		const std::vector<size_t>& getIndices() const;

	protected:
		CMesh* _pMesh;
		std::vector<size_t> _indices;
	};

}
