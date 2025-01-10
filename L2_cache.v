module L2_cache(
input [1:0] offset,
input [4:0] inp_index,
input [8-1:0] inp_data,
input [25-1:0] new_tag,
input [31:0] L1_L2_data,
input L1_L2_valid, L1_L2_dirty,
input [7:0] new_LRU,
input [1:0] L1_L2_way,
input [1:0] L2_L1_way, 
output [31:0] L2_L1_data,
output reg [4*8-1:0] out_data,
output [25-1:0] tag,
output [7:0] out_LRU,
output [3:0] out_valid, 
output [3:0] out_dirty );

parameter no_of_lines = 128;
parameter no_of_bits_per_line = 32;
parameter byte_size = 8;
parameter address_size = 32;
parameter tag_size = 25;
parameter no_of_ways = 4;
parameter index_size = 5;
parameter no_of_LRU_bits = 2;
parameter no_of_sets = 32;
integer i, j;

reg [no_of_bits_per_line-1:0] L2_mem [0:no_of_ways-1][0:no_of_sets-1];
reg [tag_size-1:0] L2tag [0:no_of_ways-1][0:no_of_sets-1];
reg valid [0:no_of_ways-1][0:no_of_sets-1];
reg dirty [0:no_of_ways-1][0:no_of_sets-1];
reg [no_of_LRU_bits-1:0] LRU [0:no_of_ways-1][0:no_of_sets-1];

assign tag = {L2tag[3][inp_index], L2tag[2][inp_index],L2tag[1][inp_index], L2tag[0][inp_index]};
assign out_valid = {valid[3][inp_index], valid[2][inp_index], valid[1][inp_index], valid[0][inp_index]};
assign L2_L1_data = L2_mem[L2_L1_way][inp_index];

always @ (*) begin
case ( offset )
2'd0 : out_data = { L2_mem[3][inp_index][7:0], L2_mem[2][inp_index][7:0], L2_mem[1][inp_index][7:0], L2_mem[0][inp_index][7:0] };
2'd1 : out_data = { L2_mem[3][inp_index][15:8], L2_mem[2][inp_index][15:8], L2_mem[1][inp_index][15:8], L2_mem[0][inp_index][15:8] };
2'd2 : out_data = { L2_mem[3][inp_index][23:16], L2_mem[2][inp_index][23:16], L2_mem[1][inp_index][23:16], L2_mem[0][inp_index][23:16] };
2'd3 : out_data = { L2_mem[3][inp_index][31:24], L2_mem[2][inp_index][31:24], L2_mem[1][inp_index][31:24], L2_mem[0][inp_index][31:24] };
endcase
end

always @ (*) begin
LRU[2'd0][inp_index] = new_LRU[1:0];
LRU[2'd1][inp_index] = new_LRU[3:2];
LRU[2'd2][inp_index] = new_LRU[5:4];
LRU[2'd3][inp_index] = new_LRU[7:6];
end
	
initial begin
for ( i=0; i<no_of_sets; i=i+1 ) begin
   for ( j=0; j<no_of_ways; j=j+1 ) begin
      L2tag[j][i] = {tag_size{1'b0}};
      valid[j][i] = 1'b1;
      LRU[j][i] = {no_of_LRU_bits{1'b0}};
   end
end
end

initial begin
#2 L2tag[0][0] = 25'd1;
L2_mem[0][0] = {no_of_bits_per_line{1'b1}};
end

endmodule