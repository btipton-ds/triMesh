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
#include <tm_proxyVector.h>
#include <triMesh.h>

namespace TriMesh {
	template<class T>
	ProxyVector<T>::ProxyVector(CMesh* pMesh)
		: _pMesh(pMesh)
	{}

	template<class T>
	ProxyVector<T>::~ProxyVector()
	{
		clear();
	}

	template<class T>
	size_t ProxyVector<T>::numBytes() const
	{
		size_t result = 0;

		result += _indices.capacity() * sizeof(T);

		return result;
	}

	template<class T>
	inline std::vector<size_t>& ProxyVector<T>::getIndices()
	{
		return _indices;
	}

	template<class T>
	inline const std::vector<size_t>& ProxyVector<T>::getIndices() const
	{
		return _indices;
	}

	template<class T>
	T& ProxyVector<T>::operator[](size_t idx)
	{
		auto& vals = _pMesh->getRepo()->get<T>();
		return vals[_indices[idx]];
	}

	template<class T>
	const T& ProxyVector<T>::operator[](size_t idx) const
	{
		auto& vals = _pMesh->getRepo()->get<T>();
		return vals[_indices[idx]];
	}

	template<class T>
	size_t ProxyVector<T>::size() const
	{
		return _indices.size();
	}

	template<class T>
	void ProxyVector<T>::clear()
	{
		while (size() > 0)
			pop_back();
	}

	template<class T>
	void ProxyVector<T>::push_back(const T& val)
	{
		const auto& pRepo = _pMesh->getRepo();
		auto& vals = pRepo->get<T>();
		size_t idx = pRepo->getAvailEntry<T>();
		if (idx < vals.size()) {
			vals[idx] = val;
		} else {
			idx = vals.size();
			vals.push_back(val);
		}
		_indices.push_back(idx);
	}

	template<class T>
	void ProxyVector<T>::pop_back()
	{
		const auto& pRepo = _pMesh->getRepo();
		if (!_indices.empty())
			pRepo->release<T>(_indices.size() - 1);
		_indices.pop_back();
	}

	template<class T>
	ProxyVector<T>::const_iterator ProxyVector<T>::begin() const noexcept
	{
		const size_t* p = nullptr;
		if (!_indices.empty())
			p = &_indices.front();
		return const_iterator(this, p);
	}

	template<class T>
	ProxyVector<T>::iterator ProxyVector<T>::begin() noexcept
	{
		size_t* p = nullptr;
		if (!_indices.empty())
			p = &_indices.front();
		return iterator(this, p);
	}

	template<class T>
	ProxyVector<T>::const_iterator ProxyVector<T>::end() const noexcept
	{
		const size_t* p = nullptr;
		if (!_indices.empty())
		{
			p = &_indices.back();
			p++;
		}
		return const_iterator(this, p);
	}

	template<class T>
	ProxyVector<T>::iterator ProxyVector<T>::end() noexcept
	{
		size_t* p = nullptr;
		if (!_indices.empty())
		{
			p = &_indices.back();
			p++;
		}
		return iterator(this, p);
	}
}
