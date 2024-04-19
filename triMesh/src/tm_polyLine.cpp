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

#include <tm_polyLine.h>

#include <string>
#include <tm_math.h>
#include <tm_lineSegment.h>
#include <triMesh.h>

namespace TriMesh {

	using namespace std;

	CPolyLine::CPolyLine() {
	}

	void CPolyLine::save(ostream& out) const {
		out << "Polyline version 1\n";

		out << "CL: " << (_isClosed ? "true" : "false") << "\n";
		out << "l " << _vertIdx.size() << "\n";
		for (size_t idx : _vertIdx)
			out << idx << " ";
		out << "\n";
	}

	bool CPolyLine::read(istream& in) {
		string str0, str1;
		int version;

		in >> str0 >> str1 >> version;
		if (str0 != "Polyline" || str1 != "version")
			return false;

		in >> str0 >> str1;
		if (str0 != "CL:")
			return false;
		_isClosed = str1 == "true";

		size_t numVerts;
		in >> str0 >> numVerts;
		if (str0 != "l")
			return false;

		_vertIdx.resize(numVerts);
		for (auto& v : _vertIdx)
			in >> v;

		return true;
	}

	bool CPolyLine::addEdgeToLine(const CEdge& edge) {
		if (_isClosed)
			return false;
		bool added = false;
		if (_vertIdx.empty()) {
			_vertIdx.push_back(edge._vertIndex[0]);
			_vertIdx.push_back(edge._vertIndex[1]);
			added = true;
		} else if (edge._vertIndex[0] == _vertIdx.front()) {
			_vertIdx.insert(_vertIdx.begin(), edge._vertIndex[1]);
			added = true;
		} else if (edge._vertIndex[1] == _vertIdx.front()) {
			_vertIdx.insert(_vertIdx.begin(), edge._vertIndex[0]);
			added = true;
		} else if (edge._vertIndex[0] == _vertIdx.back()) {
			_vertIdx.insert(_vertIdx.end(), edge._vertIndex[1]);
			added = true;
		}
		else if (edge._vertIndex[1] == _vertIdx.back()) {
			_vertIdx.insert(_vertIdx.end(), edge._vertIndex[0]);
			added = true;
		}

		if (added && _vertIdx.front() == _vertIdx.back()) {
			_isClosed = true;
			_vertIdx.pop_back();
		}
		return added;
	}

	LineSegment<double> CPolyLine::getSegment(const CMesh& mesh, size_t idx0) const {
		size_t idx1 = _isClosed ? ((idx0 + 1) % _vertIdx.size()) : idx0 + 1;

		return LineSegment<double>(mesh.getVert(_vertIdx[idx0])._pt, mesh.getVert(_vertIdx[idx1])._pt);
	}

	bool CPolyLine::findClosestPointOnPolyline(const CMesh& mesh, const Vector3d& testPt, size_t& plIdx, double& dist, double& param) const {
		size_t numSegs = _vertIdx.size();
		if (!_isClosed)
			numSegs--;

		double minDist = DBL_MAX;
		for (size_t i = 0; i < numSegs; i++) {
			auto seg = getSegment(mesh, i);
			double t, d;
			d = seg.distanceToPoint(testPt, t);
			if (d < minDist) {
				minDist = d;
				plIdx = i;
				param = t;
				if (minDist < SAME_DIST_TOL)
					break;
			}
		}

		dist = minDist;
		return dist < SAME_DIST_TOL;
	}

}
