module L1_cache(
input [1:0] offset,
input [3:0] inp_index,
input [8-1:0] inp_data,
input [26-1:0] new_tag,
input [1:0] new_LRU,
input inp_way, inp_valid, inp_dirty,
input [31:0] L2_data,
output reg [15:0] out_data,
output [63:0] victim_data,
output [52-1:0] tag,
output [1:0] out_LRU,
output [1:0] out_valid, 
output [1:0] victim_dirty );

parameter no_of_lines = 32;
parameter no_of_bits_per_line = 32;
parameter byte_size = 8;
parameter address_size = 32;
parameter tag_size = 26;
parameter no_of_ways = 2;
parameter index_size = 4;
parameter no_of_LRU_bits = 1;
parameter no_of_sets = 16;
integer i, j;

reg [no_of_bits_per_line-1:0] L1_mem [0:no_of_ways-1][0:no_of_sets-1];
reg [tag_size-1:0] L1tag [0:no_of_ways-1][0:no_of_sets-1];
reg valid [0:no_of_ways-1][0:no_of_sets-1];
reg dirty [0:no_of_ways-1][0:no_of_sets-1];
reg LRU [0:no_of_ways-1][0:no_of_sets-1];

assign tag = {L1tag[1][inp_index], L1tag[0][inp_index]};
assign out_valid = {valid[1][inp_index], valid[0][inp_index]};
assign out_LRU = {LRU[1][inp_index], LRU[0][inp_index]};

assign victim_data = { L1_mem[1][inp_index], L1_mem[0][inp_index] };
assign victim_dirty = { dirty[1][inp_index], dirty[0][inp_index] };

always @ (*) begin
case ( offset )
2'd0 : out_data = { L1_mem[1][inp_index][7:0], L1_mem[0][inp_index][7:0] };
2'd1 : out_data = { L1_mem[1][inp_index][15:8], L1_mem[0][inp_index][15:8] };
2'd2 : out_data = { L1_mem[1][inp_index][23:16], L1_mem[0][inp_index][23:16] };
2'd3 : out_data = { L1_mem[1][inp_index][31:24], L1_mem[0][inp_index][31:24] };
endcase
end

always @ (*) begin
LRU[1][inp_index] = new_LRU[1];
LRU[0][inp_index] = new_LRU[0];
end

always @ (*) begin  //L2 data being upgraded to L1
L1_mem[inp_way][inp_index] = L2_data;
L1tag[inp_way][inp_index] = new_tag;
valid[inp_way][inp_index] = inp_valid;
dirty[inp_way][inp_index] = inp_dirty;
end

initial begin
for ( i=0; i<no_of_sets; i=i+1 ) begin
   for ( j=0; j<no_of_ways; j=j+1 ) begin
      L1tag[j][i] = {tag_size{1'b0}};
      valid[j][i] = 1'b1;
      LRU[j][i] = {no_of_LRU_bits{1'b0}};
   end
end
end
initial begin
L1tag[0][0] = 0;
L1_mem[0][0] = {no_of_bits_per_line{1'b1}};
end

endmodule