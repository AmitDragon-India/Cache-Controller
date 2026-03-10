module l2_cache #(
  parameter ADDR_W            = 32,
  
  parameter int L1_LINE_BYTES = 32,
  parameter int L1_LINE_W     = L1_LINE_BYTES*8,

  parameter int L2_LINE_BYTES = 128,
  parameter int L2_LINE_W     = L2_LINE_BYTES*8,

  // 32KB default: 32KB/32B = 1024 lines; 4-way => 256 sets
  parameter WAYS        = 4,
  parameter SETS        = 16
)(
  input  logic              clk,
  input  logic              rst_n,

  // ============================
  // Upper interface (from L1)
  // ============================
  input  logic                 up_req_valid,
  output logic                 up_req_ready,
  input  logic                 up_req_rw,        // 0=readline, 1=writeline (writeback)
  input  logic [ADDR_W-1:0]    up_req_addr,      // byte address (line aligned)
  input  logic [L1_LINE_W-1:0] up_req_wline,

  output logic                 up_resp_valid,    // only for reads
  output logic [L1_LINE_W-1:0] up_resp_rline,

  // ============================
  // Lower interface (to MEM)
  // ============================
  output logic              mem_req_valid,
  input  logic              mem_req_ready,
  output logic              mem_req_rw,        // 0=readline, 1=writeline
  output logic [ADDR_W-1:0] mem_req_addr,
  output logic [L2_LINE_W-1:0] mem_req_wline,

  input  logic              mem_resp_valid,
  input  logic [L2_LINE_W-1:0] mem_resp_rline
);

  // ----------------------------
  // Derived parameters
  // ----------------------------
  localparam int L1_OFF_BITS = $clog2(L1_LINE_BYTES);   // 5
  localparam int L2_OFF_BITS = $clog2(L2_LINE_BYTES);   // 7
  localparam int INDEX_BITS  = $clog2(SETS);
  localparam int TAG_BITS    = ADDR_W - L2_OFF_BITS - INDEX_BITS;
  
  localparam int SUBLINES = L2_LINE_BYTES / L1_LINE_BYTES; // 4
  localparam int SUB_BITS = $clog2(SUBLINES);              // 2
  logic [SUB_BITS-1:0] subline_q;


  // ----------------------------
  // L2 storage arrays
  // ----------------------------
  logic [TAG_BITS-1:0] tag_arr   [0:SETS-1][0:WAYS-1];
  logic               valid_arr [0:SETS-1][0:WAYS-1];
  logic               dirty_arr [0:SETS-1][0:WAYS-1];
  logic [L2_LINE_W-1:0] data_arr [0:SETS-1][0:WAYS-1];

  // 4-way Tree-PLRU bits per set: 3 bits
  // b0: root, b1: left subtree, b2: right subtree
  // Convention used here: each bit points to which side is LRU:
  // b0=0 => left subtree is LRU,  b0=1 => right subtree is LRU
  // b1=0 => way0 is LRU,         b1=1 => way1 is LRU
  // b2=0 => way2 is LRU,         b2=1 => way3 is LRU
  logic [2:0] plru [0:SETS-1];

  // ----------------------------
  // Request registers (blocking)
  // ----------------------------
  logic              req_valid_q;
  logic              req_rw_q;
  logic [ADDR_W-1:0] req_addr_q;
  logic [L1_LINE_W-1:0] req_wline_q;

  logic [INDEX_BITS-1:0] idx_q;
  logic [TAG_BITS-1:0]   tag_q;

  // ----------------------------
  // Hit detect (combinational)
  // ----------------------------
  logic hit_any_c;
  logic [WAYS-1:0] hit_way_vec_c;
  logic [1:0] hit_way_c; // 0..3

  assign hit_way_vec_c[0] = valid_arr[idx_q][0] && (tag_arr[idx_q][0] == tag_q);
  assign hit_way_vec_c[1] = valid_arr[idx_q][1] && (tag_arr[idx_q][1] == tag_q);
  assign hit_way_vec_c[2] = valid_arr[idx_q][2] && (tag_arr[idx_q][2] == tag_q);
  assign hit_way_vec_c[3] = valid_arr[idx_q][3] && (tag_arr[idx_q][3] == tag_q);

  assign hit_any_c = |hit_way_vec_c;

  // Priority encode hit way (deterministic)
  always_comb begin
    if (hit_way_vec_c[0])      hit_way_c = 2'd0;
    else if (hit_way_vec_c[1]) hit_way_c = 2'd1;
    else if (hit_way_vec_c[2]) hit_way_c = 2'd2;
    else if (hit_way_vec_c[3]) hit_way_c = 2'd3;
    else                       hit_way_c = 2'd0;
  end

  // ----------------------------
  // Victim selection (invalid preferred, else PLRU)
  // ----------------------------
  logic [1:0] victim_way_c;

  function automatic [1:0] plru_choose(input logic [2:0] bits);
    begin
      if (bits[2'd2] === 1'b0) begin end // dummy (avoid lint warnings)
      // choose subtree
      if (bits[2] === 1'b0) begin end

      if (bits[2:2] == 1'b0) begin end // dummy

      // b0 decides subtree
      if (bits[2'd2] == 1'b1) begin end // dummy

      if (bits[0] == 1'b0) begin
        // left subtree LRU => choose between way0/way1 using b1
        plru_choose = (bits[1] == 1'b0) ? 2'd0 : 2'd1;
      end else begin
        // right subtree LRU => choose between way2/way3 using b2
        plru_choose = (bits[2] == 1'b0) ? 2'd2 : 2'd3;
      end
    end
  endfunction

  always_comb begin
    // prefer invalid ways first
    if (!valid_arr[idx_q][0])      victim_way_c = 2'd0;
    else if (!valid_arr[idx_q][1]) victim_way_c = 2'd1;
    else if (!valid_arr[idx_q][2]) victim_way_c = 2'd2;
    else if (!valid_arr[idx_q][3]) victim_way_c = 2'd3;
    else                           victim_way_c = plru_choose(plru[idx_q]);
  end

  // victim info
  logic victim_dirty_c;
  logic [TAG_BITS-1:0] victim_tag_c;
  logic [L2_LINE_W-1:0]   victim_line_c;

  assign victim_dirty_c = dirty_arr[idx_q][victim_way_c];
  assign victim_tag_c   = tag_arr[idx_q][victim_way_c];
  assign victim_line_c  = data_arr[idx_q][victim_way_c];

  // Line-aligned addresses
  logic [ADDR_W-1:0] req_line_addr;
  logic [ADDR_W-1:0] victim_line_addr;

  assign req_line_addr    = {tag_q, idx_q, {L2_OFF_BITS{1'b0}}};
  assign victim_line_addr = {victim_tag_c, idx_q, {L2_OFF_BITS{1'b0}}};

  // ----------------------------
  // PLRU update (on access/fill)
  // ----------------------------
  task automatic plru_touch(input logic [INDEX_BITS-1:0] set_i, input logic [1:0] way_i);
    begin
      // Using our convention: bits point to LRU side.
      // Touching a way makes the *other* side become LRU.
      case (way_i)
        2'd0: begin plru[set_i][0] <= 1'b1; plru[set_i][1] <= 1'b1; end // accessed way0 => right subtree LRU, way1 LRU
        2'd1: begin plru[set_i][0] <= 1'b1; plru[set_i][1] <= 1'b0; end // accessed way1 => right subtree LRU, way0 LRU
        2'd2: begin plru[set_i][0] <= 1'b0; plru[set_i][2] <= 1'b1; end // accessed way2 => left subtree LRU, way3 LRU
        2'd3: begin plru[set_i][0] <= 1'b0; plru[set_i][2] <= 1'b0; end // accessed way3 => left subtree LRU, way2 LRU
        default: begin end
      endcase
    end
  endtask

  // ----------------------------
  // FSM
  // ----------------------------
  typedef enum logic [2:0] {
    S_IDLE,
    S_LOOKUP,
    S_HIT_RESP,
    S_MISS_SELECT,
    S_WB_REQ,
    S_REFILL_REQ,
    S_REFILL_WAIT,
    S_MISS_RESP
  } state_t;

  state_t state;

  // Which way chosen for fill
  logic [1:0] fill_way_q;

  // MEM request regs
  logic              mem_req_valid_r;
  logic              mem_req_rw_r;
  logic [ADDR_W-1:0] mem_req_addr_r;
  logic [L2_LINE_W-1:0] mem_req_wline_r;

  assign mem_req_valid = mem_req_valid_r;
  assign mem_req_rw    = mem_req_rw_r;
  assign mem_req_addr  = mem_req_addr_r;
  assign mem_req_wline = mem_req_wline_r;
  
  logic [L2_LINE_W-1:0] tmp_line;


  // Ready when idle (blocking)
  assign up_req_ready = (state == S_IDLE);

  // ----------------------------
  // Sequential
  // ----------------------------
  
  integer s,w;
  integer base;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state          <= S_IDLE;

      req_valid_q    <= 1'b0;
      req_rw_q       <= 1'b0;
      req_addr_q     <= '0;
      req_wline_q    <= '0;

      idx_q          <= '0;
      tag_q          <= '0;

      up_resp_valid  <= 1'b0;
      up_resp_rline  <= '0;

      mem_req_valid_r <= 1'b0;
      mem_req_rw_r    <= 1'b0;
      mem_req_addr_r  <= '0;
      mem_req_wline_r <= '0;

      fill_way_q     <= 2'd0;

      // (Optional init for sim: valid/dirty/plru to 0)
       
       for (s=0; s<SETS; s=s+1) begin
         plru[s] = 3'b000;
         for (w=0; w<WAYS; w=w+1) begin
           valid_arr[s][w] = 1'b0;
           dirty_arr[s][w] = 1'b0;
           tag_arr[s][w]   = '0;
           //data_arr[s][w]  = '0;
         end
       end

    end else begin
      up_resp_valid <= 1'b0;

      // clear mem_req_valid after handshake
      if (mem_req_valid_r && mem_req_ready) begin
        mem_req_valid_r <= 1'b0;
      end

      unique case (state)

        // ----------------------------
        // Accept L1 line request
        // ----------------------------
        S_IDLE: begin
          if (up_req_valid) begin
            req_valid_q <= 1'b1;
            req_rw_q    <= up_req_rw;
            req_addr_q  <= up_req_addr;
            req_wline_q <= up_req_wline;

            idx_q <= up_req_addr[L2_OFF_BITS + INDEX_BITS - 1 : L2_OFF_BITS];
				tag_q <= up_req_addr[ADDR_W-1 : L2_OFF_BITS + INDEX_BITS];
				
				subline_q <= up_req_addr[L2_OFF_BITS-1 : L1_OFF_BITS]; // addr[6:5] for 128B/32B

            state <= S_LOOKUP;
          end
        end

        // ----------------------------
        // Lookup decision
        // ----------------------------
        S_LOOKUP: begin
          if (hit_any_c) state <= S_HIT_RESP;
          else           state <= S_MISS_SELECT;
        end

        // ----------------------------
        // Hit handling
        // ----------------------------
        S_HIT_RESP: begin
		  base = subline_q * L1_LINE_W;
          if (!req_rw_q) begin
            // Read hit: return full line to L1
            up_resp_rline <= data_arr[idx_q][hit_way_c][base +: L1_LINE_W];
            up_resp_valid <= 1'b1;
          end else begin
            // Write hit (L1 writeback line): overwrite full line & mark dirty
            tmp_line = data_arr[idx_q][hit_way_c];
				tmp_line[base +: L1_LINE_W] = req_wline_q;
				data_arr[idx_q][hit_way_c] <= tmp_line;
				dirty_arr[idx_q][hit_way_c] <= 1'b1;

            // no response for writeback
          end

          // Update PLRU as MRU touched
          plru_touch(idx_q, hit_way_c);

          req_valid_q <= 1'b0;
          state       <= S_IDLE;
        end

        // ----------------------------
        // Miss select: choose fill way, possibly write back victim
        // ----------------------------
        S_MISS_SELECT: begin
          fill_way_q <= victim_way_c;

          if (victim_dirty_c) begin
            if (!mem_req_valid_r) begin
              mem_req_valid_r <= 1'b1;
              mem_req_rw_r    <= 1'b1; // writeback
              mem_req_addr_r  <= victim_line_addr;
              mem_req_wline_r <= victim_line_c;
              state           <= S_WB_REQ;
            end
          end else begin
            state <= S_REFILL_REQ;
          end
        end

        // ----------------------------
        // Writeback request to MEM
        // ----------------------------
        S_WB_REQ: begin
          if (mem_req_valid_r && mem_req_ready) begin
            state <= S_REFILL_REQ;
          end
        end

        // ----------------------------
        // Refill request
        // ----------------------------
        S_REFILL_REQ: begin
			  if (!mem_req_valid_r) begin
				 mem_req_valid_r <= 1'b1;
				 mem_req_rw_r    <= 1'b0; // read 128B
				 mem_req_addr_r  <= req_line_addr;
				 mem_req_wline_r <= '0;
				 state           <= S_REFILL_WAIT;
			  end
			end


        // ----------------------------
        // Wait for MEM response
        // ----------------------------
        S_REFILL_WAIT: begin
			  if (mem_resp_valid) begin
				 data_arr[idx_q][fill_way_q]  <= mem_resp_rline;
				 tag_arr[idx_q][fill_way_q]   <= tag_q;
				 valid_arr[idx_q][fill_way_q] <= 1'b1;

				 // Now handle read vs write request
				 if (req_rw_q) begin
					base     = subline_q * L1_LINE_W;
					tmp_line = mem_resp_rline;
					tmp_line[base +: L1_LINE_W] = req_wline_q;
					data_arr[idx_q][fill_way_q] <= tmp_line;
					dirty_arr[idx_q][fill_way_q] <= 1'b1;
				 end else begin
					dirty_arr[idx_q][fill_way_q] <= 1'b0;
				 end

				 plru_touch(idx_q, fill_way_q);
				 state <= S_MISS_RESP;
			  end
			end

        // ----------------------------
        // Return refilled line to L1
        // ----------------------------
        S_MISS_RESP: begin
			  base = subline_q * L1_LINE_W;

			  up_resp_rline <= data_arr[idx_q][fill_way_q][base +: L1_LINE_W];
			  up_resp_valid <= (!req_rw_q);   // only reads respond

			  req_valid_q <= 1'b0;
			  state       <= S_IDLE;
			end


        default: state <= S_IDLE;

      endcase
    end
  end

endmodule
