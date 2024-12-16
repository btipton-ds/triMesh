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

#include <tm_proxyEdges.h>
#include <tm_iterator.hpp>
#include <tm_repo.h>

using namespace std;
using namespace TriMesh;


ProxyEdges::ProxyEdges(const CMeshRepoPtr& pRepo)
	: _pRepo(pRepo)
{}

CEdge& ProxyEdges::operator[](size_t idx)
{
	auto& edges = _pRepo->getEdges();
	return edges[_edgeIndices[idx]];
}

const CEdge& ProxyEdges::operator[](size_t idx) const
{
	auto& edges = _pRepo->getEdges();
	return edges[_edgeIndices[idx]];
}

size_t ProxyEdges::size() const
{
	return _edgeIndices.size();
}

void ProxyEdges::push_back(const CEdge& val)
{
	auto& edges = _pRepo->getEdges();
	size_t idx = edges.size();
	edges.push_back(val);
	_edgeIndices.push_back(idx);
}

void ProxyEdges::pop_back()
{
	_edgeIndices.pop_back();
}

void ProxyEdges::read(std::istream& in)
{
}

void ProxyEdges::write(std::ostream& out) const
{
}

ProxyEdges::const_iterator ProxyEdges::begin() const noexcept
{
	const size_t* p = nullptr;
	if (!_edgeIndices.empty())
		p = &_edgeIndices.front();
	return const_iterator(this, p);
}

ProxyEdges::iterator ProxyEdges::begin() noexcept
{
	size_t* p = nullptr;
	if (!_edgeIndices.empty())
		p = &_edgeIndices.front();
	return iterator(this, p);
}

ProxyEdges::const_iterator ProxyEdges::end() const noexcept
{
	const size_t* p = nullptr;
	if (!_edgeIndices.empty())
	{
		p = &_edgeIndices.back();
		p++;
	}
	return const_iterator(this, p);
}

ProxyEdges::iterator ProxyEdges::end() noexcept
{
	size_t* p = nullptr;
	if (!_edgeIndices.empty())
	{
		p = &_edgeIndices.back();
		p++;
	}
	return iterator(this, p);
}

template class ProxyEdges::iterator;
template class ProxyEdges::const_iterator;
template class ProxyEdges::reverse_iterator;
template class ProxyEdges::const_reverse_iterator;
