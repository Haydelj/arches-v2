make: riscv riscv-dissasm

riscv: ./*
	riscv64-unknown-elf-g++ -ffixed-ft8 -ffixed-ft9 -ffixed-ft10 -ffixed-ft11 -mno-relax -nostartfiles -emain -Wstack-usage=512 -mabi=lp64f -Ofast -I ../../include ./main.cpp -o ./riscv/kernel

riscv-dissasm: riscv
	riscv64-unknown-elf-objdump -d -x ./riscv/kernel > "./riscv/kernel.dump"

clean:
	rm -f riscv/*