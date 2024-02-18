#include <tm_edge.h>
#include <triMesh.h>

using namespace TriMesh;

LineSegment CEdge::getSeg(const CMesh* pMesh) const
{
	Vector3d pt0 = pMesh->getVert(_vertIndex[0])._pt;
	Vector3d pt1 = pMesh->getVert(_vertIndex[1])._pt;
	return LineSegment(pt0, pt1);
}

CEdge::CEdge(size_t vertIdx0, size_t vertIdx1)
	: _numFaces(0)
{
	_faceIndex[0] = _faceIndex[1] = stm1;
	if (vertIdx0 <= vertIdx1) {
		_vertIndex[0] = vertIdx0;
		_vertIndex[1] = vertIdx1;
	}
	else {
		_vertIndex[0] = vertIdx1;
		_vertIndex[1] = vertIdx0;
	}
}

bool CEdge::operator < (const CEdge& rhs) const {
	for (int i = 0; i < 2; i++) {
		if (_vertIndex[i] < rhs._vertIndex[i])
			return true;
		else if (_vertIndex[i] > rhs._vertIndex[i])
			return false;
	}
	return false;
}

bool CEdge::operator == (const CEdge& rhs) const {
	return _vertIndex[0] == rhs._vertIndex[0] && _vertIndex[1] == rhs._vertIndex[1];
}

void CEdge::addFace(size_t faceIdx) {
	for (int i = 0; i < _numFaces; i++) {
		if (_faceIndex[i] == faceIdx)
			return;
	}
	if (_numFaces < 2) {
		_faceIndex[_numFaces++] = faceIdx;
	} else {
//		assert(!"An edge cannot have more than two triangles attached to it");
	}
}

inline void CEdge::dump(std::ostream& out) const {
	out << "edge { verts:(" << _vertIndex[0] << ", " << _vertIndex[1] << "), faces(";
	for (int i = 0; i < _numFaces; i++) {
		out << _faceIndex[i] << ",";
	}
	out << ") }\n";
}
