module testbench ();

reg clk, mode, reset;
reg [7:0] data_in;
reg [31:0] address;

wire Wait, hit1, hit2;
wire [7:0] data_out;

cache_controller test ( clk, mode, reset, data_in, address, Wait, hit1, hit2, data_out );

initial clk = 1'b0;
always #5 clk = ~clk;

initial
#50 $stop;

initial begin
//address = {{24{1'b0}},1'b1,{7{1'b0}}};
address = 32'd0;
reset = 1;
mode = 0;
end

initial #6 reset = 0;

endmodule