module main_mem(
input [31:0] rd_address, wr_address, in_data,
output [31:0] out_data);

parameter address_size = 32;
parameter no_of_lines = 1024;
parameter no_of_bits_per_line = 32;
parameter byte_size = 8;
integer i;

reg [address_size-1:0] mem [0:no_of_lines-1];


assign out_data = mem[rd_address];

always @(*) mem[wr_address] = in_data;

initial begin
for ( i=0; i<no_of_lines; i=i+1 ) 
mem[i] = i;
end

endmodule