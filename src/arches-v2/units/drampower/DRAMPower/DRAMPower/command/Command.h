#ifndef DRAMPOWER_COMMAND_COMMAND_H
#define DRAMPOWER_COMMAND_COMMAND_H

#include <units/drampower/DRAMPower/DRAMPower/Types.h>
#include <units/drampower/DRAMPower/DRAMPower/command/CmdType.h>

#include <optional>

namespace DRAMPower {

struct TargetCoordinate {
    std::size_t bank = 0;
    std::size_t bankGroup = 0;
    std::size_t rank = 0;
	std::size_t row = 0;
	std::size_t column = 0;

	TargetCoordinate() = default;
	TargetCoordinate(std::size_t bank_id, std::size_t bank_group_id, std::size_t rank_id)
		: bank(bank_id), bankGroup(bank_group_id), rank(rank_id)
	{};

	TargetCoordinate(std::size_t bank_id, std::size_t bank_group_id, std::size_t rank_id, std::size_t row_id)
		: bank(bank_id), bankGroup(bank_group_id), rank(rank_id), row(row_id)
	{};

	TargetCoordinate(std::size_t bank_id, std::size_t bank_group_id, std::size_t rank_id, std::size_t row_id, std::size_t column_id)
		: bank(bank_id), bankGroup(bank_group_id), rank(rank_id), row(row_id), column(column_id)
	{};
};

class Command {
public:
    Command() = default;
	Command(timestamp_t timestamp, CmdType type, TargetCoordinate targetCoord = {}, const uint8_t * data = nullptr, std::size_t sz_bits = 0)
		: type(type)
		, targetCoordinate(targetCoord)
        , timestamp(timestamp)
		, data(data)
		, sz_bits(sz_bits)
	{};

public:
    TargetCoordinate targetCoordinate;

    CmdType type;

    timestamp_t timestamp = 0;
    const uint8_t * data = 0x00; // ToDo: buffer{ptr, sz} / TLM Standard
	std::size_t sz_bits;
    //uint64_t burstLength;
public:
};

}

#endif /* DRAMPOWER_COMMAND_COMMAND_H */
