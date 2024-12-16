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

#include <tm_proxyVertices.h>
#include <tm_iterator.hpp>
#include <tm_repo.h>

using namespace std;
using namespace TriMesh;


ProxyVertices::ProxyVertices(const CMeshRepoPtr& pRepo)
	: _pRepo(pRepo)
{}

CVertex& ProxyVertices::operator[](size_t idx)
{
	auto& verts = _pRepo->getVertices();
	return verts[_vertIndices[idx]];
}

const CVertex& ProxyVertices::operator[](size_t idx) const
{
	auto& verts = _pRepo->getVertices();
	return verts[_vertIndices[idx]];
}

size_t ProxyVertices::size() const
{
	return _vertIndices.size();
}

void ProxyVertices::push_back(const CVertex& val)
{
	auto& verts = _pRepo->getVertices();
	size_t idx = verts.size();
	verts.push_back(val);
	_vertIndices.push_back(idx);
}

void ProxyVertices::pop_back()
{
	_vertIndices.pop_back();
}

void ProxyVertices::read(std::istream& in)
{
}

void ProxyVertices::write(std::ostream& out) const
{
}

ProxyVertices::const_iterator ProxyVertices::begin() const noexcept
{
	return const_iterator(this, _vertIndices.data());
}

ProxyVertices::iterator ProxyVertices::begin() noexcept
{
	return iterator(this, _vertIndices.data());
}

ProxyVertices::const_iterator ProxyVertices::end() const noexcept
{
	return const_iterator(this, _vertIndices.data() + _vertIndices.size() + 1);
}

ProxyVertices::iterator ProxyVertices::end() noexcept
{
	return iterator(this, _vertIndices.data() + _vertIndices.size() + 1);
}

template class ProxyVertices::iterator;
template class ProxyVertices::const_iterator;
template class ProxyVertices::reverse_iterator;
template class ProxyVertices::const_reverse_iterator;
