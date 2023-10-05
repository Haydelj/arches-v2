make: riscv riscv-dissasm

riscv: src/*
	riscv64-unknown-elf-g++ -mno-relax -nostartfiles -emain -Wstack-usage=512 -mabi=lp64f -Ofast ./src/main.cpp -o ./riscv/kernel

riscv-dissasm: riscv
	riscv64-unknown-elf-objdump -d -x ./riscv/kernel > "./riscv/kernel.dump"

clean:
	rm -f riscv/*