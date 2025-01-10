module cache_controller (
input clk, mode, reset,
input [8-1:0] data_in,
input [32-1:0] address,
output reg Wait, hit1, hit2,
output reg [8-1:0] data_out );

parameter no_of_bits_per_line = 32;
parameter byte_size = 8;
parameter address_size = 32;
parameter blk_offset = 2;
parameter L2_latency = 2;
parameter Main_mem_latency = 10;
parameter L1_tag_size = 26;
parameter L1_index_size = 4;
parameter L2_tag_size = 25;
parameter L2_index_size = 5;
parameter L1_cache_no_of_ways = 2;
parameter L2_cache_no_of_ways = 4;
parameter read = 0;
parameter write = 1;
integer i, j;

reg [address_size-1:0] stored_address;
reg [byte_size-1:0] stored_data_in;
reg stored_mode;
reg data_upgrade;
reg [L1_tag_size-1:0] L1_victim_tag;
reg L1_victim_dirty, L1_upgrade_pt1;
reg [31:0] L1_victim_data;

wire [address_size-blk_offset-1:0] mem_address;
wire [L1_tag_size-1:0] L1_tag;
wire [L2_tag_size-1:0] L2_tag;

wire [L1_index_size-1:0] L1_index;
wire [2*byte_size-1:0] L1_inp_data, L1_out_data;
reg [L1_tag_size-1:0] L1_new_tag;
wire [L1_cache_no_of_ways*L1_tag_size-1:0] L1_existing_tag;
wire [1:0] L1_valid, L1_LRU, L1_dirty;
wire [L1_cache_no_of_ways*no_of_bits_per_line-1:0] L1_data;
reg [1:0] L1_new_LRU;
wire L1_new_way;
reg [31:0] L2_L1_data_in;
reg L1_inp_valid, L1_inp_way;
L1_cache L1_instance ( address[1:0], L1_index, L1_inp_data, L1_new_tag, L1_new_way, 
L1_new_LRU, L1_inp_way, L1_inp_valid, L2_L1_data_in, L1_out_data, L1_data, L1_existing_tag, L1_LRU, L1_valid, L1_dirty );

wire [L2_index_size-1:0] L2_index;
wire [byte_size-1:0] L2_inp_data;
wire [4*8-1:0] L2_out_data;
reg [L2_tag_size-1:0] L2_new_tag;
wire [L2_cache_no_of_ways*L2_tag_size-1:0] L2_existing_tag;
wire [1:0] L2_line_locator, L2_line_locator2;
wire [2*L2_cache_no_of_ways-1:0] L2_LRU;
wire [L2_cache_no_of_ways-1:0] L2_valid, L2_dirty;
wire [L2_cache_no_of_ways*no_of_bits_per_line-1:0] L2_data;
reg [2*L2_cache_no_of_ways:0] L2_new_LRU;
wire [31:0] L2_L1_data;
reg [1:0] L2_outp_way;
reg [31:0] L1_L2_data;
reg L1_L2_valid, L1_L2_dirty;
reg [1:0] L1_L2_way;
L2_cache L2_instance ( address[1:0], L2_index, L2_inp_data, L2_new_tag, L1_L2_data, L1_L2_valid, L1_L2_dirty,
 L2_new_LRU, L1_L2_way, L2_L1_data, L2_out_data, L2_data, L2_existing_tag, L2_LRU, L2_valid, L2_dirty );
 
reg [31:0] rd_address, wr_address, in_data;
wire [31:0] out_data;
main_mem main_mem_instance( rd_address, wr_address, in_data, out_data);

//Assigning indexes and tags to caches and main memory
assign mem_address = (stored_address>>2) % 11'd1024;
assign L1_index = mem_address % 5'd16;
assign L1_tag = mem_address >> 3'd4;
assign L2_index = mem_address % 6'd32;
assign L2_tag = mem_address >> 3'd5;
assign L2_line_locator = i;
assign L2_line_locator2 = j;

always @ (posedge clk) begin

if ( reset ) begin
stored_address <= 0;
stored_data_in <= 0;
stored_mode <= 0;
Wait <= 0;
hit1 <= 0;
hit2 <= 0;
data_out <= 0;
L1_new_LRU <= 0;
L2_new_LRU <= 0;
data_upgrade <= 0;
end
else if ( Wait == 1'b0 ) begin
   stored_address <= address;
	stored_data_in <= data_in;
   stored_mode <= mode;
	end
	if ( mode == read ) begin
	  if ( Wait == 1'b0 ) begin
        if ( L1_tag == L1_existing_tag[L1_tag_size-1:0] & L1_valid[0]) begin
		     hit1 <= 1'b1;
   		  data_out <= L1_out_data[7:0];
			  L1_new_LRU <= {1'b0, 1'b1};
		  end
		  else if ( L1_tag == L1_existing_tag[2*L1_tag_size-1:0] & L1_valid[1]) begin
		     hit1 <= 1'b1;
   		  data_out <= L1_out_data[15:0];
			  L1_new_LRU <= {1'b1, 1'b0};
		  end
		  else begin                             //Not found in L1 !!!!!!!!!!!
		     hit1 <= 1'b0;
			  Wait <= 1'b1;
		  end
	  end
	  else if ( hit1 == 1'b0 ) begin
	    if ( ~data_upgrade )
	     case ( L2_tag ) 
		     L2_existing_tag[L2_tag_size-1:0] : if (L2_valid[0]) begin
			                                        hit2 <= 1'b1;
															    data_out <= L2_out_data[7:0];
															    L2_new_LRU[1:0] <= 2'd3;
																 data_upgrade <= 1'b1;
																 L2_outp_way <= 2'd0;
																 if ( L2_LRU[3:2] > L2_LRU[1:0] ) 
																    L2_new_LRU[3:2] <= L2_LRU[3:2] - 2'd1;
																 if ( L2_LRU[5:4] > L2_LRU[1:0] ) 
																    L2_new_LRU[5:4] <= L2_LRU[5:4] - 2'd1;
																 if ( L2_LRU[7:6] > L2_LRU[1:0] ) 
																    L2_new_LRU[7:6] <= L2_LRU[7:6] - 2'd1;
																 end
															 
			  L2_existing_tag[(2*L2_tag_size-1)-:L2_tag_size] : if (L2_valid[1]) begin
			                                        hit2 <= 1'b1;
															    data_out <= L2_out_data[15:8];
															    L2_new_LRU[3:2] <= 2'd3;
																 data_upgrade <= 1'b1;
																 L2_outp_way <= 2'd1;
																 if ( L2_LRU[1:0] > L2_LRU[3:2] ) 
																    L2_new_LRU[1:0] <= L2_LRU[1:0] - 2'd1;
																 if ( L2_LRU[5:4] > L2_LRU[3:2] ) 
																    L2_new_LRU[5:4] <= L2_LRU[5:4] - 2'd1;
																 if ( L2_LRU[7:6] > L2_LRU[3:2] ) 
																    L2_new_LRU[7:6] <= L2_LRU[7:6] - 2'd1;
																 end
															 
			  L2_existing_tag[(3*L2_tag_size-1)-:L2_tag_size] : if (L2_valid[2]) begin
			                                        hit2 <= 1'b1;
															    data_out <= L2_out_data[23:16];
															    L2_new_LRU[5:4] <= 2'd3;
																 data_upgrade <= 1'b1;
																 L2_outp_way <= 2'd2;
																 if ( L2_LRU[1:0] > L2_LRU[5:4] ) 
																    L2_new_LRU[1:0] <= L2_LRU[1:0] - 2'd1;
																 if ( L2_LRU[3:2] > L2_LRU[5:4] ) 
																    L2_new_LRU[3:2] <= L2_LRU[3:2] - 2'd1;
																 if ( L2_LRU[7:6] > L2_LRU[5:4] ) 
																    L2_new_LRU[7:6] <= L2_LRU[7:6] - 2'd1;
																 end
															 
			  L2_existing_tag[(4*L2_tag_size-1)-:L2_tag_size] : if (L2_valid[3]) begin
			                                        hit2 <= 1'b1;
															    data_out <= L2_out_data[31:24];
															    L2_new_LRU[7:6] <= 2'd3;
																 data_upgrade <= 1'b1;
																 L2_outp_way <= 2'd0;
																 if ( L2_LRU[3:2] > L2_LRU[7:6] ) 
																    L2_new_LRU[3:2] <= L2_LRU[3:2] - 2'd1;
																 if ( L2_LRU[5:4] > L2_LRU[7:6] ) 
																    L2_new_LRU[5:4] <= L2_LRU[5:4] - 2'd1;
																 if ( L2_LRU[1:0] > L2_LRU[7:6] ) 
																    L2_new_LRU[1:0] <= L2_LRU[1:0] - 2'd1;
																 end
															 											 
			  default : begin                      // Not found in L2 !!!!!!!!!!!!!
			            hit2 <= 1'b0;
							end
		  endcase
		 end
		  else if ( hit2 & data_upgrade & ~L1_upgrade_pt1) begin   // Upgrade data to L1
		     if ( ~L1_LRU[0] & L1_valid[0] ) begin
			     L1_victim_data = L1_data[31:0];
				  L1_victim_tag = L1_tag[0];
				  L1_victim_dirty = L1_dirty[0];
				  L1_upgrade_pt1 = 1'b1;
				  for ( i=0;i<L2_cache_no_of_ways; i=i+1 ) begin
				     if ( L2_LRU[i] == 2'd0 & L2_valid[i] & L2_dirty) begin
					     wr_address = stored_address;
					     in_data = L2_data;
				     end
				  end
				  for ( i=0; i<L2_cache_no_of_ways; i=i+1 ) begin
				     if ( L2_LRU[i] == 2'd0 ) begin
					     L2_new_tag = L1_victim_tag;
						  L1_L2_data = L1_victim_data;
						  L1_L2_valid = 1'b1;
						  L1_L2_dirty = L1_victim_dirty;
						  end
						  end
			  end
			  else if ( ~L1_LRU[0] & ~L1_valid[0] ) begin
			     L1_inp_way <= 1'b0;
				  L2_L1_data_in <= L2_L1_data;
				  L1_new_tag <= L2_tag;
				  L1_inp_valid <= 1'b1;
				  L1_upgrade_pt1 <= 1'b1;
			  end
			  else if ( ~L1_LRU[1] & L1_valid[1] ) begin
			     L1_victim_data <= L1_data[63:32];
				  L1_victim_tag <= L1_tag[1];
				  L1_victim_dirty <= L1_dirty[1];
				  L1_upgrade_pt1 <= 1'b1;
				  for ( i=0;i<L2_cache_no_of_ways; i=i+1 ) begin
				     if ( L2_LRU[i] == 2'd0 & L2_valid[i] & L2_dirty) begin
					     wr_address = stored_address;
					     in_data = L2_data;
				     end
				  end
				  for ( i=0; i<L2_cache_no_of_ways; i=i+1 ) begin
				     if ( L2_LRU[i] == 2'd0 ) begin
					     L2_new_tag = L1_victim_tag;
						  L1_L2_data = L1_victim_data;
						  L1_L2_valid = 1'b1;
						  L1_L2_dirty = L1_victim_dirty;
						  end
						  end
			  end
			  else if ( ~L1_LRU[1] & ~L1_valid[1] ) begin
			     L1_inp_way <= 1'b1;
				  L2_L1_data_in <= L2_L1_data;
				  L1_new_tag <= L2_tag;
				  L1_inp_valid <= 1'b1;
				  L1_upgrade_pt1 <= 1'b1;
			  end
	  end
 end
end
endmodule
		   
		   
