#pragma once
#include "stdafx.hpp"

struct alignas(32) CompactTri
{
	uint64_t data[4];
};


#ifdef ARCH_X86
template <typename RET>
inline RET _deserialize(std::ifstream& stream)
{
	RET value;
	stream.read((char*)&value, sizeof(RET));
	return value;
}

template <typename RET>
inline void _deserialize_vector(std::vector<RET>& vector, std::ifstream& stream)
{
	size_t size = _deserialize<size_t>(stream);
	vector.resize(size);
	stream.read((char*)vector.data(), vector.size() * sizeof(RET));
}
#endif