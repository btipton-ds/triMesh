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

#include <tm_defines.h>
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
	/*
	This creates precise write/read functions for each type, then deletes conversions which might change size
	bool is handled separately and routed through a uint8_t in case bool's size differs
	between compilers. 

	It's not supposed to, but clang++ 19 on windows and ubuntu Linux is getting a size mismatch.
	*/

	inline bool& writeChecksEnabled()
	{
		static bool enabled = false;
		return enabled;
	}

	inline bool& readChecksEnabled()
	{
		static bool enabled = false;
		return enabled;
	}

	template<class T>
	inline void writeWithChecks(std::ostream& out, const T& val)
	{
		if (writeChecksEnabled()) {
			size_t sv = sizeof(T);
			out.write((const char*)&sv, sizeof(sv));
		}
		out.write((const char*)&val, sizeof(val));
	}

	template<class T>
	inline void readWithChecks(std::istream& in, T& val)
	{
		if (readChecksEnabled()) {
			size_t sv;
			in.read((char*)&sv, sizeof(sv));
			if (sv != sizeof(val)) {
				std::cout << "readWithChecks size mismatch " << __FILE__ << ":" << __LINE__ << "\n";
			}
		}
		in.read((char*)&val, sizeof(val));
	}

	template<class T>
	inline void writeWithChecks(std::ostream& out, const T* pVal, size_t count)
	{
		size_t st = count * sizeof(T);
		if (writeChecksEnabled()) {
			out.write((const char*)&st, sizeof(st));
		}
		out.write((const char*)pVal, st);
	}

	template<class T>
	inline void readWithChecks(std::istream& in, T* pVal, size_t count)
	{
		if (readChecksEnabled()) {
			size_t st;
			in.read((char*)&st, sizeof(st));
			if (st / count != sizeof(T)) {
				std::cout << "readWithChecks size mismatch " << __FILE__ << ":" << __LINE__ << "\n";
			}
		}
		in.read((char*)pVal, count * sizeof(T));
	}


#define WRITE_READ(VAL_TYPE) \
	inline void write(std::ostream& out, const VAL_TYPE& val)\
	{\
		writeWithChecks(out, val);\
	}\
\
	inline void read(std::istream& in, VAL_TYPE& val)\
	{\
		readWithChecks(in, val);\
	}

	WRITE_READ(uint8_t)
	WRITE_READ(uint16_t)
	WRITE_READ(uint32_t)
	WRITE_READ(uint64_t)

	WRITE_READ(int8_t)
	WRITE_READ(int16_t)
	WRITE_READ(int32_t)
	WRITE_READ(int64_t)

	WRITE_READ(float)
	WRITE_READ(double)

	inline void write(std::ostream& out, const bool val)
	{
		uint8_t iVal = val ? 1 : 0;
		writeWithChecks(out, iVal);
	}

	inline void read(std::istream& in, bool& val)
	{
		uint8_t iVal;
		readWithChecks(in, iVal);
		val = iVal == 1 ? true : false;
	}

	inline void write(std::ostream& out, const std::string& val)
	{
		size_t num = val.size();
		write(out, num);
		writeWithChecks(out, val.data(), num);
	}

	inline void read(std::istream& in, std::string& val)
	{
		size_t num;
		read(in, num);
		val.resize(num);
		readWithChecks(in, val.data(), num);
	}


	inline void write(std::ostream& out, const std::wstring& wval)
	{
		const size_t bufSize = 1024;
		setlocale(LC_ALL, "en_US.utf8");

		size_t retVal, newSize;
		char buf[bufSize];
		auto rtn_val = wcstombs_s(&retVal, buf, bufSize, wval.data(), wval.size());
		if (rtn_val == 0) {
			std::string val(buf);
			write(out, val);
		} else {
			write(out, std::string("Error"));
			std::cout << "Could not write utf8 string\n";
		}
	}

	inline void read(std::istream& in, std::wstring& wval)
	{
		const size_t bufSize = 1024;
		std::string val;
		read(in, val);

		setlocale(LC_ALL, "en_US.utf8");

		size_t retVal, newSize;
		wchar_t buf[bufSize];
		auto pData = val.data();
		auto dataSize = val.size() + 1;
		auto rtn_val = mbstowcs_s(&retVal, buf, bufSize, pData, dataSize);
		if (rtn_val == 0)
			wval = std::wstring(buf);
		else {
			wval = L"Error";
			std::cout << "Could not read utf8 string\n";
		}
	}

	template<class T>
	inline void write(std::ostream& out, T const* const pData, size_t num)
	{
		size_t size = sizeof(T) * num;
		out.write((const char*)pData, size);
	}

	template<class T>
	inline void read(std::istream& in, T* pData, size_t num)
	{
		size_t size = sizeof(T) * num;
		in.read((char*)pData, size);
	}

	template<class T>
	void write(std::ostream& out, const T) = delete;

	template<class T>
	void read(std::istream& in, const T) = delete;

	template<class T>
	void write(std::ostream& out, const std::set<T>& vals)
	{
		size_t num = vals.size();
		write(out, num);

		for (const auto& val : vals) {
			write(out, val);
		}
	}

	template<class T>
	void read(std::istream& in, std::set<T>& vals)
	{
		size_t num;
		read(in, num);

		for (size_t i = 0; i < num; i++) {
			T val;
			read(in, val);
			vals.insert(val);
		}
	}

	template<class T>
	void writeObj(std::ostream& out, const std::set<T>& vals)
	{
		size_t num = vals.size();
		write(out, num);
		for (const auto& val : vals) {
			val.write(out);
		}
	}

	template<class T>
	void readObj(std::istream& in, std::set<T>& vals)
	{
		size_t num;
		read(in, num);

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
		write(out, num);
		if (num > 0) {
			write(out, vals.data(), num);
		}
	}

	template<class T>
	void read(std::istream& in, std::vector<T>& vals)
	{
		size_t num;
		read(in, num);

		if (num > 0) {
			vals.resize(num);
			read(in, vals.data(), num);
		}
	}

	template<class T>
	void writeObj(std::ostream& out, const std::vector<T>& vals)
	{
		size_t num = vals.size();
		write(out, num);

		for (size_t i = 0; i < vals.size(); i++) {
			vals[i].write(out);
		}
	}

	template<class T>
	void readObj(std::istream& in, std::vector<T>& vals)
	{
		size_t num;
		read(in, num);
		if (num > 0) {
			vals.resize(num);
			for (size_t i = 0; i < num; i++) {
				vals[i].read(in);
			}
		}
	}

	template<class T>
	void writeObj(std::ostream& out, const std::vector<T>& vals, size_t meshId)
	{
		size_t num = vals.size();
		write(out, num);
		for (size_t i = 0; i < vals.size(); i++) {
			vals[i].write(out, meshId);
		}
	}

	template<class T>
	void readObj(std::istream& in, std::vector<T>& vals, size_t meshId)
	{
		size_t num;
		read(in, num);
		if (num > 0) {
			for (size_t i = 0; i < num; i++) {
				vals.push_back(T());
				T& val = vals[vals.size() - 1];
				val.read(in, meshId);
			}
		}
	}

	template <typename T>
	inline void write(std::ostream& out, const Vector3<T>& v)
	{
		write(out, v.data(), 3);
	}

	template <typename T>
	inline void read(std::istream& in, Vector3<T>& v)
	{
		read(in, v.data(), 3);
	}

	template<class T>
	void write(std::ostream& out, const std::vector<Vector3<T>>& vals)
	{
		size_t num = vals.size();
		write(out, num);
		for (const auto& val : vals) {
			write(out, val);
		}
	}

	template<class T>
	void read(std::istream& in, std::vector<Vector3<T>>& vals)
	{
		size_t num;
		read(in, num);

		vals.resize(num);
		for (auto& val : vals) {
			read(in, val);
		}
	}

	/************************************************************************************/
	template<class T, class U>
	void write(std::ostream& out, const std::map<T, U>& val)
	{
		size_t num = val.size();
		write(out, num);
		for (const auto& pair : val) {
			pair.first.write(out);
			pair.second.write(out);

		}
	}

	template<class T, class U>
	void read(std::istream& in, std::map<T, U>& val)
	{
		size_t num = val.size();
		read(in, num);

		for (size_t i = 0; i < num; i++) {
			T t;
			U u;
			t.read(in);
			u.read(in);
			val.insert(std::make_pair(t, u));
		}
	}

}
