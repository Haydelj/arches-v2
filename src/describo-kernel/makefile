RISCV_CC = riscv64-unknown-elf-g++
RISCV_OBJDUMP = riscv64-unknown-elf-objdump
X86_CC = g++

RISCV_FLAGS = -nostartfiles -emain -mno-relax -mabi=lp64f -Ofast -g
RISCV_OBJDUMP_FLAGS = -d -x -S
X86_FLAGS = -Ofast -lpthread


make: ray-caster path-tracer


ray-caster: ray-caster-riscv ray-caster-riscv-disasm ray-caster-x86

ray-caster-riscv: ray-caster.cpp
	$(RISCV_CC) $(RISCV_FLAGS) ./ray-caster.cpp -o ./bin/riscv/ray-caster

ray-caster-riscv-disasm: ray-caster-riscv
	$(RISCV_OBJDUMP) $(RISCV_OBJDUMP_FLAGS) ./bin/riscv/ray-caster > "./bin/riscv/ray-caster.dump"

ray-caster-x86: ray-caster.cpp
	$(X86_CC) $(X86_FLAGS) ./ray-caster.cpp -o ./bin/x86/ray-caster


path-tracer: path-tracer-riscv path-tracer-riscv-disasm path-tracer-x86

path-tracer-riscv: path-tracer.cpp
	$(RISCV_CC) $(RISCV_FLAGS) ./path-tracer.cpp -o ./bin/riscv/path-tracer

path-tracer-riscv-disasm: path-tracer-riscv
	$(RISCV_OBJDUMP) $(RISCV_OBJDUMP_FLAGS) ./bin/riscv/path-tracer > "./bin/riscv/path-tracer.dump"

path-tracer-x86: path-tracer.cpp
	$(X86_CC) $(X86_FLAGS) ./path-tracer.cpp -o ./bin/x86/path-tracer


clean:
	rm -f bin/riscv/*
	rm -f bin/x86/*