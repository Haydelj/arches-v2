make: riscv riscv-dissasm

riscv: ./*.cpp ./*.hpp
	riscv64-unknown-elf-g++ -mno-relax -nostartfiles -emain -fconserve-stack -Wstack-usage=512 -mabi=lp64f -Ofast -I ../../include ./main.cpp -o ./riscv/kernel

riscv-dissasm: riscv
	riscv64-unknown-elf-objdump -d -x ./riscv/kernel > "./riscv/kernel.dump"

clean:
	rm -f riscv/*