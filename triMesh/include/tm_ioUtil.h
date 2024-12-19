#pragma once

/*
This file is part of the DistFieldHexMesh application/library.

	The DistFieldHexMesh application/library is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	The DistFieldHexMesh application/library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	This link provides the exact terms of the GPL license <https://www.gnu.org/licenses/>.

	The author's interpretation of GPL 3 is that if you receive money for the use or distribution of the DistFieldHexMesh application/library or a derivative product, GPL 3 no longer applies.

	Under those circumstances, the author expects and may legally pursue a reasoble share of the income. To avoid the complexity of agreements and negotiation, the author makes
	no specific demands in this regard. Compensation of roughly 1% of net or $5 per user license seems appropriate, but is not legally binding.

	In lay terms, if you make a profit by using the DistFieldHexMesh application/library (violating the spirit of Open Source Software), I expect a reasonable share for my efforts.

	Robert R Tipton - Author

	Dark Sky Innovative Solutions http://darkskyinnovation.com/
*/

#include <iostream>
#include <vector>
#include <set>
#include <map>
#include <tm_vector3.h>

namespace TriMesh
{
	template<class T>
	class ProxyVector;
}

namespace IoUtil
{
	template<class T>
	void write(std::ostream& out, const std::set<T>& vals)
	{
		size_t num = vals.size();
		out.write((char*)&num, sizeof(num));

		for (const auto& val : vals) {
			out.write((char*)&val, sizeof(T));
		}
	}

	template<class T>
	void read(std::istream& in, std::set<T>& vals)
	{
		size_t num;
		in.read((char*)&num, sizeof(num));

		for (size_t i = 0; i < num; i++) {
			T val;
			in.read((char*)&val, sizeof(T));
			vals.insert(val);
		}
	}

	template<class T>
	void writeObj(std::ostream& out, const std::set<T>& vals)
	{
		size_t num = vals.size();
		out.write((char*)&num, sizeof(num));
		for (const auto& val : vals) {
			val.write(out);
		}
	}

	template<class T>
	void readObj(std::istream& in, std::set<T>& vals)
	{
		size_t num;
		in.read((char*)&num, sizeof(num));

		for (size_t i = 0; i < num; i++) {
			T id;
			id.read(in);
			vals.insert(id);
		}
	}

	/************************************************************************************/
	template<class T>
	void write(std::ostream& out, const std::vector<T>& vals)
	{
		size_t num = vals.size();
		out.write((char*)&num, sizeof(num));
		if (num > 0) {
			out.write((char*)vals.data(), num * sizeof(T));
		}
	}

	template<class T>
	void read(std::istream& in, std::vector<T>& vals)
	{
		size_t num;
		in.read((char*)&num, sizeof(num));
		if (num > 0) {
			vals.resize(num);
			in.read((char*)vals.data(), num * sizeof(T));
		}
	}

	template<class T>
	void writeObj(std::ostream& out, const std::vector<T>& vals)
	{
		size_t num = vals.size();
		out.write((char*)&num, sizeof(num));
		for (size_t i = 0; i < vals.size(); i++) {
			vals[i].write(out);
		}
	}

	template<class T>
	void readObj(std::istream& in, std::vector<T>& vals)
	{
		size_t num;
		in.read((char*)&num, sizeof(num));
		if (num > 0) {
			vals.resize(num);
			for (size_t i = 0; i < num; i++) {
				vals[i].read(in);
			}
		}
	}

	template<class T>
	void write(std::ostream& out, const ::TriMesh::ProxyVector<T>& vals)
	{
		std::vector<T> tmp;
		tmp.insert(tmp.end(), vals.begin(), vals.end());
		size_t num = tmp.size();
		out.write((char*)&num, sizeof(num));
		out.write((char*)tmp.data(), num * sizeof(T));
	}

	template<class T>
	void read(std::istream& in, ::TriMesh::ProxyVector<T>& vals)
	{
		size_t num;
		in.read((char*)&num, sizeof(num));
		if (num > 0) {
			std::vector<T> tmp;
			tmp.resize(num);
			in.read((char*)tmp.data(), num * sizeof(T));

			for (const auto& val : tmp)
				vals.push_back(val);
		}
	}

	template<class T>
	void writeObj(std::ostream& out, const ::TriMesh::ProxyVector<T>& vals, size_t meshId)
	{
		size_t num = vals.size();
		out.write((char*)&num, sizeof(num));
		for (size_t i = 0; i < vals.size(); i++) {
			vals[i].write(out, meshId);
		}
	}

	template<class T>
	void readObj(std::istream& in, ::TriMesh::ProxyVector<T>& vals, size_t meshId)
	{
		size_t num;
		in.read((char*)&num, sizeof(num));
		if (num > 0) {
			for (size_t i = 0; i < num; i++) {
				vals.push_back(T());
				T& val = vals[vals.size() - 1];
				val.read(in, meshId);
			}
		}
	}

	template<class T>
	void writeVector3(std::ostream& out, const std::vector<Vector3<T>>& vals)
	{
		size_t num = vals.size();
		out.write((char*)&num, sizeof(num));
		for (const auto& val : vals) {
			::writeVector3(out, val);
		}
	}

	template<class T>
	void readVector3(std::istream& in, std::vector<Vector3<T>>& vals)
	{
		size_t num;
		in.read((char*)&num, sizeof(num));

		vals.resize(num);
		for (auto& val : vals) {
			::readVector3(in, val);
		}
	}

	/************************************************************************************/
	template<class T, class U>
	void write(std::ostream& out, const std::map<T, U>& val)
	{
		size_t num = val.size();
		out.write((char*)&num, sizeof(num));
		for (const auto& pair : val) {
			pair.first.write(out);
			pair.second.write(out);

		}
	}

	template<class T, class U>
	void read(std::istream& in, std::map<T, U>& val)
	{
		size_t num = val.size();
		in.read((char*)&num, sizeof(num));
		for (size_t i = 0; i < num; i++) {
			T t;
			U u;
			t.read(in);
			u.read(in);
			val.insert(std::make_pair(t, u));
		}
	}



}
