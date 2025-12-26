gpio.v
```module gpio (
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
