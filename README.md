gpio.v
   ``` module gpio (
    input  wire        clk,
    input  wire        reset,
    input  wire        wr,
    input  wire [31:0] wr_data,
    output wire [31:0] rd_data,
    output wire [31:0] gpio_out
);

    reg [31:0] gpio_register;

    always @(posedge clk) begin
        if (reset)
            gpio_register <= 32'b0;
        else if (wr)
            gpio_register <= wr_data;
    end

    assign rd_data    = gpio_register;
    assign gpio_out = gpio_register;

endmodule



HERE IS THE gpio_test.c


#include "io.h"

#define GPIO_BASE 0x20000000

volatile unsigned int *gpio = (unsigned int *)GPIO_BASE;

int main() {
    unsigned int val;

    *gpio = 0xB2B2B2B2;
    val = *gpio;

    print_hex(val);
    putchar('\n');   // <-- ALWAYS exists

    while (1);
}

gpio IP_inst (
    .clk   (clk),
    .reset (resetn),
    .wr    (gpio_we),
    .wr_data (mem_wdata),
    .rd_data (gpio_rdata),
    .gpio_out (gpio_out)
);



 HERE IS THE IP INSTANTIATION

wire [31:0] gpio_rdata;
wire [31:0] gpio_out;
wire        gpio_we;

localparam GPIO_BASE = 32'h2000_0000;
wire gpio_sel = (mem_addr == GPIO_BASE);
assign gpio_we = mem_we & gpio_sel;






HERE IS THE Makefile



ARCH = rv32i
ABI = ilp32

PROGRAM = gpio_test
RAM_SIZE = 6144

RVASFLAGS = -march=$(ARCH) -mabi=$(ABI)
RVCFLAGS  = -fno-pic -march=$(ARCH) -mabi=$(ABI) \
            -fno-stack-protector -w -Wl,--no-relax

RVINCS = -I ./LIBFEMTOGL -I ./LIBFEMTORV32 -I ./LIBFEMTOC

LIBOBJECTS = putchar.o wait.o print.o memcpy.o errno.o perf.o

RVAS      = riscv64-unknown-elf-as
RVLD      = riscv64-unknown-elf-ld
RVOBJCOPY = riscv64-unknown-elf-objcopy
RVOBJDUMP = riscv64-unknown-elf-objdump
RVGCC     = riscv64-unknown-elf-gcc

all: $(PROGRAM).hex

.c.o:
	$(RVGCC) $(RVCFLAGS) $(RVINCS) -c $<

.S.o:
	$(RVAS) $(RVASFLAGS) $< -o $@

$(PROGRAM).bram.elf: $(PROGRAM).o start.o $(LIBOBJECTS)
	$(RVLD) -T bram.ld -m elf32lriscv -nostdlib $^ ./libgcc.a -o $@

$(PROGRAM).hex: $(PROGRAM).bram.elf
	./firmware_words $< -ram $(RAM_SIZE) -max_addr $(RAM_SIZE) -out $@
	cp $@ ../RTL/firmware.hex
	mkdir -p ../RTL/obj_dir
	cp $@ ../RTL/obj_dir/firmware.hex
	echo $@ > ../RTL/firmware.txt

clean:
	rm -f *.o *.elf *.hex *.bram.elf
