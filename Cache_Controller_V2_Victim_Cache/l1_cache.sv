module l1_cache #(
  parameter ADDR_W           = 32,
  parameter DATA_W           = 32,
  parameter L1_LINE_BYTES    = 32,
  parameter WAYS             = 2,
  parameter SETS             = 8,
  parameter L1_LINE_W        = L1_LINE_BYTES*8,
  
  // VC parameters
  parameter VC_ENTRIES       = 8
  
  //localparam int OFFSET_BITS = $clog2(LINE_BYTES),                // 5
  //localparam int INDEX_BITS  = $clog2(SETS),                       // 6
  //localparam int TAG_BITS    = ADDR_W - OFFSET_BITS - INDEX_BITS,  // 21
  //localparam int LINE_W      = LINE_BYTES * 8                      // 256
)(
  input  logic                 clk,
  input  logic                 rst_n,

  // ----------------------------
  // CPU request
  // ----------------------------
  input  logic                 cpu_req_valid,
  input  logic                 cpu_req_rw,        // 0=read, 1=write
  input  logic [ADDR_W-1:0]    cpu_addr,
  input  logic [DATA_W-1:0]    cpu_wdata,
  input  logic [DATA_W/8-1:0]  cpu_wstrb,         // 4-bit for 32-bit
  output logic                 cpu_req_ready,

  // ----------------------------
  // CPU response
  // ----------------------------
  output logic                 cpu_resp_valid,
  output logic [DATA_W-1:0]    cpu_rdata,

  // ----------------------------
  // L1 <-> L2 (line transactions)
  // ----------------------------
  output logic                 l2_req_valid,
  input  logic                 l2_req_ready,
  output logic                 l2_req_rw,          // 0=readline, 1=writeline
  output logic [ADDR_W-1:0]    l2_req_addr,        // byte addr (line aligned)
  output logic [L1_LINE_W-1:0] l2_req_wline,

  input  logic                 l2_resp_valid,
  input  logic [L1_LINE_W-1:0]    l2_resp_rline
);

// Derived parameters (keep inside module body)
  localparam int OFFSET_BITS = $clog2(L1_LINE_BYTES);
  localparam int INDEX_BITS  = $clog2(SETS);
  localparam int TAG_BITS    = ADDR_W - OFFSET_BITS - INDEX_BITS;
  
  localparam int VC_PTR_W    = $clog2(VC_ENTRIES);


  // ----------------------------
  // Arrays
  // ----------------------------
  logic [TAG_BITS-1:0] tag_arr   [SETS][WAYS];
  logic               valid_arr [SETS][WAYS];
  logic               dirty_arr [SETS][WAYS];
  logic [L1_LINE_W-1:0] data_arr [SETS][WAYS];
  logic               lru_bit   [SETS]; // 2-way LRU (1 bit)

  // LRU convention:
  // lru_bit=0 => victim way0
  // lru_bit=1 => victim way1
  
  
  // ----------------------------
// Victim Cache (VC) - FIFO
// Fully associative VC_ENTRIES entries
// ----------------------------
  logic                 vc_valid   [VC_ENTRIES];
  logic                 vc_dirty   [VC_ENTRIES];
  logic [TAG_BITS-1:0]  vc_tag     [VC_ENTRIES];
  logic [INDEX_BITS-1:0] vc_idx    [VC_ENTRIES];
  logic [L1_LINE_W-1:0] vc_data    [VC_ENTRIES];

  logic [VC_PTR_W-1:0]  vc_tail_ptr; // points to next insertion (FIFO tail)
  // helper for VC lookups
  logic                 vc_hit_c;
  int                   vc_hit_e; // index of hit or -1
  
  
  // ----------------------------
	// VC free-slot chooser (warm-up optimization)
	// ----------------------------
  logic               vc_has_free;
  logic [VC_PTR_W-1:0] vc_free_idx;
  logic [VC_PTR_W-1:0] vc_ins_idx;   // where we will insert saved_victim
  logic vc_was_full_q;
  

  // ----------------------------
  // Helper: apply byte strobe to a selected 32-bit word inside a line
  // ----------------------------
  function automatic [L1_LINE_W-1:0] apply_wstrb_to_line(
      input [L1_LINE_W-1:0]  line_in,
      input [$clog2(L1_LINE_BYTES/4)-1:0] wsel,  // 0..7 for 32B line
      input [DATA_W-1:0]   wdata,
      input [DATA_W/8-1:0] wstrb
  );
    logic [L1_LINE_W-1:0] line_out;
    int base;
    line_out = line_in;
    base = wsel * DATA_W;

    if (wstrb[0]) line_out[base + 0*8 +: 8] = wdata[0*8 +: 8];
    if (wstrb[1]) line_out[base + 1*8 +: 8] = wdata[1*8 +: 8];
    if (wstrb[2]) line_out[base + 2*8 +: 8] = wdata[2*8 +: 8];
    if (wstrb[3]) line_out[base + 3*8 +: 8] = wdata[3*8 +: 8];

    return line_out;
  endfunction

  // ----------------------------
  // Request registers (blocking cache: one outstanding CPU req)
  // ----------------------------
  logic                req_valid_q;
  logic                req_rw_q;
  logic [ADDR_W-1:0]    req_addr_q;
  logic [DATA_W-1:0]    req_wdata_q;
  logic [DATA_W/8-1:0]  req_wstrb_q;

  // Decoded fields (registered)
  logic [OFFSET_BITS-1:0] off_q;
  logic [INDEX_BITS-1:0]  idx_q;
  logic [TAG_BITS-1:0]    tag_q;
  logic [$clog2(L1_LINE_BYTES/4)-1:0] word_sel_q;

  // ----------------------------
  // Hit detect (combinational using registered idx/tag)
  // ----------------------------
  logic hit0_c, hit1_c, hit_any_c;
  assign hit0_c    = valid_arr[idx_q][0] && (tag_arr[idx_q][0] == tag_q);
  assign hit1_c    = valid_arr[idx_q][1] && (tag_arr[idx_q][1] == tag_q);
  assign hit_any_c = hit0_c | hit1_c;

  // Selected way for hit
  logic hit_way_c; // 0 or 1
  assign hit_way_c = hit1_c ? 1'b1 : 1'b0; // if both true (shouldn't), prefer way1

  // ----------------------------
  // Victim selection (combinational)
  // Prefer invalid way, else LRU victim
  // ----------------------------
  logic victim_way_c;
  always_comb begin
    if (!valid_arr[idx_q][0])       victim_way_c = 1'b0;
    else if (!valid_arr[idx_q][1])  victim_way_c = 1'b1;
    else                            victim_way_c = lru_bit[idx_q];
  end

  logic victim_dirty_c;
  assign victim_dirty_c = dirty_arr[idx_q][victim_way_c];

  // Victim contents (for writeback address/data)
  logic [TAG_BITS-1:0]  victim_tag_c;
  logic [L1_LINE_W-1:0]    victim_line_c;
  assign victim_tag_c  = tag_arr[idx_q][victim_way_c];
  assign victim_line_c = data_arr[idx_q][victim_way_c];

  // Build line-aligned addresses
  logic [ADDR_W-1:0] req_line_addr;
  logic [ADDR_W-1:0] victim_line_addr;

  assign req_line_addr    = {tag_q, idx_q, {OFFSET_BITS{1'b0}}};
  assign victim_line_addr = {victim_tag_c, idx_q, {OFFSET_BITS{1'b0}}};

  // ----------------------------
  // FSM
  // ----------------------------
  typedef enum logic [3:0] {
    S_IDLE,
    S_LOOKUP,
    S_HIT_RESP,
	 S_VC_LOOKUP,    // new: check VC on miss
    S_VC_SWAP,      // new: swap VC line with L1 victim
    S_MISS_SELECT,
	 S_VC_EVICT_WB,  // new: writeback a dirty VC entry before overwriting it
    S_WB_REQ,
    S_REFILL_REQ,
    S_REFILL_WAIT,
    S_MISS_RESP
  } state_t;

  state_t state;

  // L2 request regs
  logic l2_req_valid_r;
  logic l2_req_rw_r;
  logic [ADDR_W-1:0] l2_req_addr_r;
  logic [L1_LINE_W-1:0] l2_req_wline_r;

  assign l2_req_valid = l2_req_valid_r;
  assign l2_req_rw    = l2_req_rw_r;
  assign l2_req_addr  = l2_req_addr_r;
  assign l2_req_wline = l2_req_wline_r;
  
  
  // ----------------------------
// VC helper regs
// ----------------------------
  //logic [VC_PTR_W-1:0] vc_evict_idx; // used when we need to writeback old VC entry
  state_t next_state_after_vc_wb;    // where to go after we finish writing back overwritten VC entry
  
  

  // Which way are we operating on during miss fill?
  logic fill_way_q;

  // CPU ready/resp
  assign cpu_req_ready = (state == S_IDLE);
  
  // ----------------------------
// VC lookup combinational
// ----------------------------
  always_comb begin
    vc_hit_c = 1'b0;
    vc_hit_e = -1;
    for (int e=0; e<VC_ENTRIES; e++) begin
      if (vc_valid[e] && (vc_idx[e] == idx_q) && (vc_tag[e] == tag_q)) begin
        vc_hit_c = 1'b1;
        vc_hit_e = e;
      end
    end
  end
  
  always_comb begin
    vc_has_free = 1'b0;
    vc_free_idx = '0;
    for (int e = 0; e < VC_ENTRIES; e++) begin
      if (!vc_valid[e] && !vc_has_free) begin
        vc_has_free = 1'b1;
        vc_free_idx = e[VC_PTR_W-1:0];
      end
    end
  end

  always_comb begin
    // If there is an invalid slot, use it. Else evict FIFO tail.
    vc_ins_idx = vc_has_free ? vc_free_idx : vc_tail_ptr;
  end

  
  
  // saved victim (when doing VC-related writeback/insert)
  logic [L1_LINE_W-1:0] saved_victim_line;
  logic [TAG_BITS-1:0]  saved_victim_tag;
  logic                  saved_victim_valid;
  logic                  saved_victim_dirty;
  logic [INDEX_BITS-1:0] saved_victim_idx;
  
  logic [VC_PTR_W-1:0] vc_ins_idx_q;
  
  
  // ----------------------------
  // Sequential logic
  // ----------------------------
  
  integer s, w;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state          <= S_IDLE;

      req_valid_q    <= 1'b0;
      req_rw_q       <= 1'b0;
      req_addr_q     <= '0;
      req_wdata_q    <= '0;
      req_wstrb_q    <= '0;

      off_q          <= '0;
      idx_q          <= '0;
      tag_q          <= '0;
      word_sel_q     <= '0;

      cpu_resp_valid <= 1'b0;
      cpu_rdata      <= '0;

      l2_req_valid_r <= 1'b0;
      l2_req_rw_r    <= 1'b0;
      l2_req_addr_r  <= '0;
      l2_req_wline_r <= '0;

      fill_way_q     <= 1'b0;
		
		vc_ins_idx_q <= '0;
		
		vc_tail_ptr    <= '0;
      //vc_evict_idx   <= '0;
      next_state_after_vc_wb <= S_IDLE;
		
		

      // Optional: initialize arrays for sim
      for (int s=0; s<SETS; s++) begin
       lru_bit[s] <= 1'b0;
			 for (int w=0; w<WAYS; w++) begin
				valid_arr[s][w] <= 1'b0;
				dirty_arr[s][w] <= 1'b0;
				tag_arr[s][w]   <= '0;
				//data_arr[s][w]  <= '0;
			 end
       end
		 
		 for (int e=0; e<VC_ENTRIES; e++) begin
        vc_valid[e] <= 1'b0;
        vc_dirty[e] <= 1'b0;
        vc_tag[e]   <= '0;
        vc_idx[e]   <= '0;
        // vc_data[e] left uninitialized
      end

		 

    end else begin
      // defaults
      cpu_resp_valid <= 1'b0;

      // default: keep l2_req_valid unless explicitly cleared after handshake
      // We'll clear l2_req_valid when accepted.
      if (l2_req_valid_r && l2_req_ready) begin
        l2_req_valid_r <= 1'b0;
      end

      unique case (state)

        // ----------------------------
        // IDLE: accept CPU request
        // ----------------------------
        S_IDLE: begin
          if (cpu_req_valid) begin
            req_valid_q <= 1'b1;
            req_rw_q    <= cpu_req_rw;
            req_addr_q  <= cpu_addr;
            req_wdata_q <= cpu_wdata;
            req_wstrb_q <= cpu_wstrb;

            off_q      <= cpu_addr[OFFSET_BITS-1:0];
            idx_q      <= cpu_addr[OFFSET_BITS + INDEX_BITS - 1 : OFFSET_BITS];
            tag_q      <= cpu_addr[ADDR_W-1 : OFFSET_BITS + INDEX_BITS];
            word_sel_q <= cpu_addr[OFFSET_BITS-1:2];

            state <= S_LOOKUP;
          end
        end

        // ----------------------------
        // LOOKUP: evaluate hit signals combinationally next cycle
        // ----------------------------
        S_LOOKUP: begin
          // We do decisions in next state for clean 2-cycle behavior
          if (hit_any_c) state <= S_HIT_RESP;
          else           state <= S_VC_LOOKUP; // check victim cache first on miss
        end
		  
		  
		  // ----------------------------
        // VC_LOOKUP: if VC hit -> swap, else go to normal miss handling
        // ----------------------------
        S_VC_LOOKUP: begin
          if (vc_hit_c) begin
            state <= S_VC_SWAP;
          end else begin
            state <= S_MISS_SELECT;
          end
        end
		  
		  // ----------------------------
        // VC_SWAP: swap VC[vc_hit_e] with L1 victim way
        // After swap, treat as if we just filled L1 and go to MISS_RESP to finish request
        // ----------------------------
        S_VC_SWAP: begin
          // Capture L1 victim contents
          logic [L1_LINE_W-1:0] l1_victim_line;
          logic [TAG_BITS-1:0]  l1_victim_tag;
          logic                  l1_victim_valid;
          logic                  l1_victim_dirty;
          int e;
          e = vc_hit_e;

          l1_victim_line  = data_arr[idx_q][victim_way_c];
          l1_victim_tag   = tag_arr[idx_q][victim_way_c];
          l1_victim_valid = valid_arr[idx_q][victim_way_c];
          l1_victim_dirty = dirty_arr[idx_q][victim_way_c];

          // 1) bring VC line into L1 victim way (installed as the requested line)
          data_arr[idx_q][victim_way_c]  <= vc_data[e];
          tag_arr[idx_q][victim_way_c]   <= tag_q; // requested tag (we matched vc_tag/e)
          valid_arr[idx_q][victim_way_c] <= 1'b1;
          dirty_arr[idx_q][victim_way_c] <= vc_dirty[e];

          // 2) put evicted L1 victim into the VC entry (swap)
			 if (l1_victim_valid) begin
			   vc_data[e]  <= l1_victim_line;
			   vc_tag[e]   <= l1_victim_tag;
			   vc_idx[e]   <= idx_q;
			   vc_valid[e] <= 1'b1;
			   vc_dirty[e] <= l1_victim_dirty;
			 end else begin
			   // L1 victim was invalid => VC entry becomes empty
			   vc_valid[e] <= 1'b0;
			   vc_dirty[e] <= 1'b0;
			   // (optional) leave tag/idx/data unchanged or clear them for readability
			   vc_tag[e] <= '0; vc_idx[e] <= '0; vc_data[e] <= '0;
			 end
          // Update LRU: touched way becomes MRU (other is victim)
          lru_bit[idx_q] <= (victim_way_c == 1'b0) ? 1'b1 : 1'b0;

          // Now serve the original request using MISS_RESP path (which uses the newly installed line)
			 // Tell MISS_RESP which way contains the requested line now
			 fill_way_q <= victim_way_c;
          state <= S_MISS_RESP;
        end
		  
		  

        // ----------------------------
        // HIT_RESP: serve read or update write, update LRU
        // ----------------------------
        S_HIT_RESP: begin
          logic [L1_LINE_W-1:0] line_sel;
          logic [DATA_W-1:0] word_sel;

          line_sel = data_arr[idx_q][hit_way_c];
          word_sel = line_sel[word_sel_q*DATA_W +: DATA_W];

          if (!req_rw_q) begin
            cpu_rdata      <= word_sel;
            cpu_resp_valid <= 1'b1;
          end else begin
            // write hit
            data_arr[idx_q][hit_way_c]  <= apply_wstrb_to_line(line_sel, word_sel_q, req_wdata_q, req_wstrb_q);
            dirty_arr[idx_q][hit_way_c] <= 1'b1;
            cpu_resp_valid              <= 1'b1;
          end

          // LRU update: make other way victim
          // access way0 => victim way1 => lru_bit=1
          // access way1 => victim way0 => lru_bit=0
          lru_bit[idx_q] <= (hit_way_c == 1'b0) ? 1'b1 : 1'b0;

          req_valid_q <= 1'b0;
          state       <= S_IDLE;
        end

// ----------------------------
        // MISS_SELECT: choose fill way; insert victim into VC (FIFO) if valid.
        // If the VC FIFO slot we will overwrite is dirty, we must write it back to L2 first.
        // ----------------------------
        S_MISS_SELECT: begin
          
         // Use immediate wires for decisions (do NOT branch on saved_* in same cycle)
		    logic this_victim_valid;
		    logic this_victim_dirty;
			 logic [INDEX_BITS-1:0] this_victim_idx;
		    logic [L1_LINE_W-1:0] this_victim_line;
		    logic [TAG_BITS-1:0]  this_victim_tag;
			 
			 fill_way_q <= victim_way_c;


		    this_victim_valid = valid_arr[idx_q][victim_way_c];
		    this_victim_dirty = dirty_arr[idx_q][victim_way_c];
		    this_victim_line  = victim_line_c;
		    this_victim_tag   = victim_tag_c;
			 this_victim_idx = idx_q;

		    // Save for later states (VC_EVICT_WB uses these)
		    saved_victim_line  <= this_victim_line;
		    saved_victim_tag   <= this_victim_tag;
		    saved_victim_valid <= this_victim_valid;
		    saved_victim_dirty <= this_victim_dirty;
		    saved_victim_idx   <= this_victim_idx;

          // If victim is valid -> insert into VC (we will not write it to L2 immediately)
          if (this_victim_valid) begin
				 logic [VC_PTR_W-1:0] ins;
				 ins = vc_ins_idx;             // free slot if available, else FIFO tail

				 vc_ins_idx_q  <= ins;         // latch insertion slot for WB case
				 vc_was_full_q <= !vc_has_free; // latch fullness decision *now*

            // If the tail entry is occupied and dirty, we must write it back before overwriting
            if (vc_valid[ins] && vc_dirty[ins]) begin
              // schedule writeback of the old VC entry
              //vc_evict_idx <= tail;
              // prepare l2_req to write old vc entry
              if (!l2_req_valid_r) begin
                l2_req_valid_r <= 1'b1;
                l2_req_rw_r    <= 1'b1; // write line
                l2_req_addr_r  <= {vc_tag[ins], vc_idx[ins], {OFFSET_BITS{1'b0}}};
                l2_req_wline_r <= vc_data[ins];
                // remember to continue after this writeback: we'll insert the saved victim then proceed to refill
                next_state_after_vc_wb <= S_REFILL_REQ;
                state <= S_VC_EVICT_WB;
              end
            end else begin
              // Insert saved L1 victim directly into VC tail (no VC-writeback needed)
              vc_data[ins]  <= this_victim_line;
				  vc_tag[ins]   <= this_victim_tag;
				  vc_idx[ins]   <= this_victim_idx;
				  vc_valid[ins] <= 1'b1;
				  vc_dirty[ins] <= this_victim_dirty;
              // advance tail pointer (FIFO)
              if (!vc_has_free) begin
					   vc_tail_ptr <= vc_tail_ptr + 1; // FIFO only when we actually evicted/overwrote
				  end

              state <= S_REFILL_REQ;
          end 
			 end else begin
            // victim was invalid; no VC insertion required - go straight to refill
            state <= S_REFILL_REQ;
          end
        end

        // ----------------------------
        // S_VC_EVICT_WB: wait for L2 to accept the writeback of the old VC entry, then overwrite VC slot and continue
        // ----------------------------
        S_VC_EVICT_WB: begin
          // Wait for handshake - when handshake observed, l2_req_valid_r was cleared earlier in top-level default
          if (l2_req_valid_r && l2_req_ready) begin
            // After handshake completes (we cleared l2_req_valid_r in default), we can now overwrite the VC tail slot with saved_victim
            logic [VC_PTR_W-1:0] ins;
				ins = vc_ins_idx_q;
            vc_data[ins]  <= saved_victim_line;
            vc_tag[ins]   <= saved_victim_tag;
            vc_idx[ins]   <= saved_victim_idx;
            vc_valid[ins] <= saved_victim_valid;
            vc_dirty[ins] <= saved_victim_dirty;

            // advance tail only if VC was full when we chose ins
		    if (vc_was_full_q) begin
				vc_tail_ptr <= vc_tail_ptr + 1;
			 end

            // continue to next state (we set earlier)
            state <= next_state_after_vc_wb;
          end
        end

        // ----------------------------
        // WB_REQ: wait until L2 accepts writeback
        // ----------------------------
        S_WB_REQ: begin
          if (l2_req_valid_r && l2_req_ready) begin
            // after handshake, go refill
            state <= S_REFILL_REQ;
          end
        end

        // ----------------------------
        // REFILL_REQ: send read request for requested line
        // ----------------------------
        S_REFILL_REQ: begin
          if (!l2_req_valid_r) begin
            l2_req_valid_r <= 1'b1;
            l2_req_rw_r    <= 1'b0; // read line
            l2_req_addr_r  <= req_line_addr;
            l2_req_wline_r <= '0;
            state          <= S_REFILL_WAIT;
          end
        end

        // ----------------------------
        // REFILL_WAIT: wait for l2_resp_valid
        // ----------------------------
        S_REFILL_WAIT: begin
          if (l2_resp_valid) begin
            // Install line
            data_arr[idx_q][fill_way_q]  <= l2_resp_rline;
            tag_arr[idx_q][fill_way_q]   <= tag_q;
            valid_arr[idx_q][fill_way_q] <= 1'b1;
            dirty_arr[idx_q][fill_way_q] <= 1'b0;

            // Update LRU: accessed/fill_way is MRU => other becomes victim
            lru_bit[idx_q] <= (fill_way_q == 1'b0) ? 1'b1 : 1'b0;

            state <= S_MISS_RESP;
          end
        end

        // ----------------------------
        // MISS_RESP: finish original CPU request using the newly installed line
        // ----------------------------
        S_MISS_RESP: begin
          logic [L1_LINE_W-1:0] line_sel;
          line_sel = data_arr[idx_q][fill_way_q];

          if (!req_rw_q) begin
            cpu_rdata      <= line_sel[word_sel_q*DATA_W +: DATA_W];
            cpu_resp_valid <= 1'b1;
          end else begin
            // write-allocate: update bytes after fill
            data_arr[idx_q][fill_way_q]  <= apply_wstrb_to_line(line_sel, word_sel_q, req_wdata_q, req_wstrb_q);
            dirty_arr[idx_q][fill_way_q] <= 1'b1;
            cpu_resp_valid               <= 1'b1;
          end

          req_valid_q <= 1'b0;
          state       <= S_IDLE;
        end

        default: state <= S_IDLE;

      endcase
    end
  end

endmodule
