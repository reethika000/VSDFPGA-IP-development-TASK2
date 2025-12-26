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
