#pragma once

#include "stdafx.hpp"

#include "riscv.hpp"


namespace Arches { namespace ISA {

//MIPS64r6
class ErrNoSuchInstr final : public std::runtime_error {
	public:
		explicit ErrNoSuchInstr(std::string const& msg) : std::runtime_error(msg) {}
		explicit ErrNoSuchInstr(RISCV::Instruction const& instr) : ErrNoSuchInstr("No such instruction: " + std::to_string(instr.data)) {}
		virtual ~ErrNoSuchInstr() = default;
};
class ErrNotImplInstr final : public std::runtime_error {
	public:
		explicit ErrNotImplInstr(std::string const& msg) : std::runtime_error(msg) {}
		explicit ErrNotImplInstr(RISCV::Instruction const& instr) : ErrNotImplInstr("Instruction " + std::to_string(instr.data) + " not implemented!") {}
		virtual ~ErrNotImplInstr() = default;
};

}}
