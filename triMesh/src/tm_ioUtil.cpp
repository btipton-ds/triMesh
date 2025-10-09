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

#include <tm_ioUtil.h>

namespace IoUtil
{

bool& writeChecksEnabled()
{
	static bool enabled = false;
	return enabled;
}

bool& readChecksEnabled()
{
	static bool enabled = false;
	return enabled;
}

void write(std::ostream& out, const bool val)
{
	uint8_t iVal = val ? 1 : 0;
	writeWithChecks(out, iVal);
}

void read(std::istream& in, bool& val)
{
	uint8_t iVal;
	readWithChecks(in, iVal);
	val = iVal == 1 ? true : false;
}

void write(std::ostream& out, const std::string& val)
{
	size_t num = val.size();
	write(out, num);
	writeWithChecks(out, val.data(), num);
}

void read(std::istream& in, std::string& val)
{
	size_t num;
	read(in, num);
	val.resize(num);
	readWithChecks(in, val.data(), num);
}


void write(std::ostream& out, const std::wstring& wval)
{
	const size_t bufSize = 1024;
	char buf[bufSize];

	setlocale(LC_ALL, "en_US.utf8");

#ifdef _WIN32
	size_t sizeRead;
	auto rtn_val = wcstombs_s(&sizeRead, buf, bufSize, wval.data(), wval.size());
	if (rtn_val == 0) {
		std::string val(buf);
		write(out, val);
	}
	else {
		write(out, std::string("Error"));
		std::cout << "Could not write utf8 string\n";
	}
#else
	auto sizeRead = wcstombs(buf, wval.data(), bufSize);
	std::string val(buf);
	write(out, val);
#endif
}

void read(std::istream& in, std::wstring& wval)
{
	const size_t bufSize = 1024;
	wchar_t buf[bufSize];

	std::string val;
	read(in, val);

	setlocale(LC_ALL, "en_US.utf8");

#ifdef _WIN32
	size_t retVal, newSize;
	auto pData = val.data();
	auto dataSize = val.size() + 1;
	auto rtn_val = mbstowcs_s(&retVal, buf, bufSize, pData, dataSize);
	if (rtn_val == 0)
		wval = std::wstring(buf);
	else {
		wval = L"Error";
		std::cout << "Could not read utf8 string\n";
	}
#else
	auto pData = val.data();
	auto dataSize = val.size() + 1;
	size_t newSize = mbstowcs(buf, pData, bufSize);
	wval = std::wstring(buf);
#endif
}

}