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

#include <iostream>
#include <fstream>
#include <sstream>

#include <readStl.h>
#include <stdint.h>

using namespace std;

namespace {
	template <class T>
	inline Vector3d toV3d(const T& v) {
		return Vector3d(v[0], v[1], v[2]);
	}
}

CReadSTL::CReadSTL() {
	_meshPtr = make_shared<TriMesh::CMesh>();
}

CReadSTL::CReadSTL(const TriMesh::CMeshPtr& meshPtr) {
	_meshPtr = meshPtr;
}

bool CReadSTL::read(const std::string& path, const std::string& filename) 
{
	std::vector<Vector3f> points;
	bool readSuccessful = false;

	{
		ifstream in(path + filename);
		if (!in.good()) {
			return false;
		}

		string key;
		getline(in, key);
		if (key == "solid") {
			readSuccessful = readText(in, points);
			if (!readSuccessful)
				return false;
		}
	}

	if (!readSuccessful) {
		ifstream in(path + filename, ios_base::binary);
		if (!in.good()) {
			return false;
		}
		if (!readBinary(in, points))
			return false;
	}

	CBoundingBox3Dd modelBBox;
	for (const auto& pt : points) {
		modelBBox.merge(toV3d(pt));
	}
	cout << "min BBox: " << modelBBox.getMin()[0] << ", " << modelBBox.getMin()[1] << ", " << modelBBox.getMin()[2] << "\n";
	cout << "max BBox: " << modelBBox.getMax()[0] << ", " << modelBBox.getMax()[1] << ", " << modelBBox.getMax()[2] << "\n";

	modelBBox.grow(1.0e-3);
	_meshPtr->reset(modelBBox);
	for (size_t i = 0; i < points.size() - 2;  i += 3) {
		_meshPtr->addTriangle(toV3d(points[i]), toV3d(points[i + 1]), toV3d(points[i + 2]));
	}

	points.clear();

//	_meshPtr->verifyFindAllTris();

	cout << "numVertices: " << _meshPtr->numVertices() << "\n";
	cout << "numEdges: " << _meshPtr->numEdges() << "\n";
	cout << "numTris: " << _meshPtr->numTris() << "\n";
	cout << "isClosed: " << (_meshPtr->isClosed() ? "true" : "false") << "\n";

	return true;
}

bool CReadSTL::read(const std::wstring& path, const std::wstring& filename) {
	std::vector<Vector3f> points;
	bool readSuccessful = false;

	{
		ifstream in(path + filename);
		if (!in.good()) {
			return false;
		}

		string key;
		getline(in, key);
		if (key == "solid") {
			readSuccessful = readText(in, points);
			if (!readSuccessful)
				return false;
		}
	}

	if (!readSuccessful) {
		ifstream in(path + filename, ios_base::binary);
		if (!in.good()) {
			return false;
		}
		if (!readBinary(in, points))
			return false;
	}

	CBoundingBox3Dd modelBBox;
	for (const auto& pt : points) {
		modelBBox.merge(toV3d(pt));
	}
	cout << "min BBox: " << modelBBox.getMin()[0] << ", " << modelBBox.getMin()[1] << ", " << modelBBox.getMin()[2] << "\n";
	cout << "max BBox: " << modelBBox.getMax()[0] << ", " << modelBBox.getMax()[1] << ", " << modelBBox.getMax()[2] << "\n";

	modelBBox.grow(1.0e-3);
	_meshPtr->reset(modelBBox);
	for (size_t i = 0; i < points.size() - 2; i += 3) {
		_meshPtr->addTriangle(toV3d(points[i]), toV3d(points[i + 1]), toV3d(points[i + 2]));
	}

	points.clear();

	//	_meshPtr->verifyFindAllTris();

	cout << "numVertices: " << _meshPtr->numVertices() << "\n";
	cout << "numEdges: " << _meshPtr->numEdges() << "\n";
	cout << "numTris: " << _meshPtr->numTris() << "\n";
	cout << "isClosed: " << (_meshPtr->isClosed() ? "true" : "false") << "\n";

	return true;
}

bool CReadSTL::readText(std::istream& in, std::vector<Vector3f>& points)
{
	while (!in.eof()) {
		string str;
		getline(in, str);
		if (str.find("outer loop") != string::npos) {
			for (int i = 0; i < 3; i++) {
				string str2;
				getline(in, str2);
				stringstream ss(str2);
				string vertStr;
				Vector3f pt;
				ss >> vertStr >> pt[0] >> pt[1] >> pt[2];
				points.push_back(pt);
			}
		}
	}

	return true;
}

bool CReadSTL::readBinary(std::istream& in, std::vector<Vector3f>& points)
{
	if (!in.good())
		return false;

	uint8_t header[100];
	in.read((char*)header, 80 * sizeof(uint8_t));

	if (!in.good())
		return false;

	uint32_t numTris;
	in.read((char*)&numTris, sizeof(uint32_t));

	if (!in.good())
		return false;

	points.reserve(3 * numTris);
	for (unsigned int i = 0; i < numTris; i++) {
		Vector3f norm, pt0, pt1, pt2;

		size_t vecSize = sizeof(Vector3f);
		in.read((char*) &norm, vecSize);
		in.read((char*) &pt0, vecSize);
		in.read((char*) &pt1, vecSize);
		in.read((char*) &pt2, vecSize);

//		points.push_back(norm);
		points.push_back(pt0);
		points.push_back(pt1);
		points.push_back(pt2);

		uint16_t attributes;
		in.read((char*)&attributes, sizeof(uint16_t));

		if (!in.good())
			return false;
	}

	return true;
}
