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

#include <string>
#include <vector>
#include <istream>
#include <tm_math.h>
#include <triMesh.h>

class CReadWriteSTL {
public:
	CReadWriteSTL();
	CReadWriteSTL(const TriMesh::CMeshPtr& meshPtr);

	bool read(const std::string& path, const std::string& filename);
	bool read(const std::wstring& path, const std::wstring& filename);
	bool write(const TriMesh::CMeshPtr& pMesh, bool binary, const std::string& path, const std::string& filename);
	bool write(const TriMesh::CMeshPtr& pMesh, bool binary, const std::wstring& path, const std::wstring& filename);

	const std::shared_ptr<TriMesh::CMesh>& getMeshPtr() const {
		return _meshPtr;
	}

#ifndef WIN32
	inline static std::string fromWString(const std::wstring& str)
	{
		return std::string(str.begin(), str.end());
	}
#endif
private:

	bool readText(std::istream& in, std::vector<Vector3f>& points);
	bool readBinary(std::istream& in, std::vector<Vector3f>& points);

	bool writeText(std::ostream& out, std::vector<Vector3f>& points);
	bool writeBinary(std::ostream& out, std::vector<Vector3f>& points);

	TriMesh::CMeshPtr _meshPtr;
};
