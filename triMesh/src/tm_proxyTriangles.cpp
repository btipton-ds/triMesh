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

#include <tm_proxyTriangles.h>
#include <tm_iterator.hpp>
#include <tm_repo.h>
#include <tm_ioUtil.h>

using namespace std;
using namespace TriMesh;


ProxyTriangles::ProxyTriangles(const CMeshRepoPtr& pRepo)
	: _pRepo(pRepo)
{}

Vector3i& ProxyTriangles::operator[](size_t idx)
{
	auto& tris = _pRepo->getTris();
	return tris[_indices[idx]];
}

const Vector3i& ProxyTriangles::operator[](size_t idx) const
{
	auto& tris = _pRepo->getTris();
	return tris[_indices[idx]];
}

size_t ProxyTriangles::size() const
{
	return _indices.size();
}

void ProxyTriangles::push_back(const Vector3i& val)
{
	auto& tris = _pRepo->getTris();
	size_t idx = tris.size();
	tris.push_back(val);
	_indices.push_back(idx);
}

void ProxyTriangles::pop_back()
{
	_indices.pop_back();
}

void ProxyTriangles::read(std::istream& in)
{
	uint8_t version = 0;
	in.read((char*)&version, sizeof(version));

	IoUtil::read(in, _indices);
}

void ProxyTriangles::write(std::ostream& out) const
{
	uint8_t version = 0;
	out.write((char*)&version, sizeof(version));

	IoUtil::write(out, _indices);
}

ProxyTriangles::const_iterator ProxyTriangles::begin() const noexcept
{
	const size_t* p = nullptr;
	if (!_indices.empty())
		p = &_indices.front();
	return const_iterator(this, p);
}

ProxyTriangles::iterator ProxyTriangles::begin() noexcept
{
	size_t* p = nullptr;
	if (!_indices.empty())
		p = &_indices.front();
	return iterator(this, p);
}

ProxyTriangles::const_iterator ProxyTriangles::end() const noexcept
{
	const size_t* p = nullptr;
	if (!_indices.empty())
	{
		p = &_indices.back();
		p++;
	}
	return const_iterator(this, p);
}

ProxyTriangles::iterator ProxyTriangles::end() noexcept
{
	size_t* p = nullptr;
	if (!_indices.empty())
	{
		p = &_indices.back();
		p++;
	}
	return iterator(this, p);
}

template class ProxyTriangles::iterator;
template class ProxyTriangles::const_iterator;
template class ProxyTriangles::reverse_iterator;
template class ProxyTriangles::const_reverse_iterator;
