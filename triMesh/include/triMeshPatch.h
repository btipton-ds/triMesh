#pragma once

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
#include <map>
#include <vector>
#include <iostream>

#include <tm_math.h>
#include <tm_spatialSearch.h>
#include <tm_edge.h>
#include <tm_ray.h>
#include <tm_vertex.h>

namespace TriMesh {
class TriMesh;
using CMeshConstPtr = std::shared_ptr<const CMesh>;

class Patch {
public:
	bool empty() const;
	void addFace(const std::vector<size_t>& pFace);
	const std::vector<std::vector<size_t>>& getSharpEdgeChains() const;
	void clearSharpEdgeChains();
	const std::vector<std::vector<size_t>>& getFaces() const;
	void finishCreation(const CMesh* pMesh, double sinSharpEdgeAngle);

private:
	std::vector<std::vector<size_t>> _sharpEdgeChains;
	std::vector<std::vector<size_t>> _faces;
};

using PatchPtr = std::shared_ptr<Patch>;

}
