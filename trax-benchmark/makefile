make: riscv riscv-dissasm

#-mno-relax 
riscv: src/path-tracer.cpp
	riscv64-unknown-elf-g++ -nostartfiles -emain -Wstack-usage=512 -mabi=lp64f -Ofast ./src/main.cpp -o ./riscv/path-tracer

riscv-dissasm: riscv
	riscv64-unknown-elf-objdump -d -x ./riscv/path-tracer > "./riscv/path-tracer.dump"

clean:
	rm -f riscv/*