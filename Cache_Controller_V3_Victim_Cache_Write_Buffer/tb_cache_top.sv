module tb_cache_top;

  // ----------------------------
  // Parameters
  // ----------------------------
  localparam int ADDR_W      = 32;
  localparam int DATA_W      = 32;
  
  localparam int L1_LINE_BYTES = 32;
  localparam int L1_LINE_W     = L1_LINE_BYTES*8;

  localparam int L2_LINE_BYTES = 128;
  localparam int L2_LINE_W     = L2_LINE_BYTES*8;


  // L1: 2-way, 64 sets
  localparam int L1_WAYS = 2;
  localparam int L1_SETS = 8;

  // L2: 4-way, 256 sets (32KB)
  localparam int L2_WAYS = 4;
  localparam int L2_SETS = 16;
  
  localparam int VC_ENTRIES  = 8;
  
  localparam int WB_ENTRIES  = 4;   // or 8

  // Main memory
  localparam int MEM_LINES   = 2048;
  localparam int MEM_LATENCY = 5;

  // ----------------------------
  // DUT signals
  // ----------------------------
  logic clk, rst_n;

  logic                 cpu_req_valid;
  logic                 cpu_req_ready;
  logic                 cpu_req_rw;
  logic [ADDR_W-1:0]    cpu_addr;
  logic [DATA_W-1:0]    cpu_wdata;
  logic [DATA_W/8-1:0]  cpu_wstrb;

  logic                 cpu_resp_valid;
  logic [DATA_W-1:0]    cpu_rdata;

  // TB vars (module-scope for Quartus-safe style)
  logic [ADDR_W-1:0] A, B, C, D;
  logic [ADDR_W-1:0] E, F, G, H, I;
  logic [ADDR_W-1:0] L, M, N, O, P, Q;
  logic [ADDR_W-1:0] R, S, T, U, V;
  logic [ADDR_W-1:0] J, K;
  logic [DATA_W-1:0] rdata;
  logic [L2_LINE_W-1:0] line_tmp;

  integer i;
  
  localparam int L1_OFF_BITS = $clog2(L1_LINE_BYTES); // 5
  localparam int L1_IDX_BITS = $clog2(L1_SETS);       // 3
  localparam int L1_TAG_BITS = ADDR_W - L1_OFF_BITS - L1_IDX_BITS;
  
  localparam int L2_OFF_BITS = $clog2(L2_LINE_BYTES); // 7
  localparam int L2_IDX_BITS = $clog2(L2_SETS);       // 4
  localparam int L2_TAG_BITS = ADDR_W - L2_OFF_BITS - L2_IDX_BITS;




  // ----------------------------
  // Clock
  // ----------------------------
  initial clk = 1'b0;
  always #5 clk = ~clk;

  // ----------------------------
  // Instantiate cache_top
  // ----------------------------
  cache_top #(
    .ADDR_W(ADDR_W),
    .DATA_W(DATA_W),
    .L1_LINE_BYTES(L1_LINE_BYTES),
    .L1_LINE_W(L1_LINE_W),
    .L2_LINE_BYTES(L2_LINE_BYTES),
    .L2_LINE_W(L2_LINE_W),


    .L1_WAYS(L1_WAYS),
    .L1_SETS(L1_SETS),

    .L2_WAYS(L2_WAYS),
    .L2_SETS(L2_SETS),
	 
	 //New
	 .VC_ENTRIES(VC_ENTRIES),
	 .WB_ENTRIES(WB_ENTRIES),

    .MEM_LINES(MEM_LINES),
    .MEM_LATENCY(MEM_LATENCY)
  ) dut (
    .clk(clk),
    .rst_n(rst_n),

    .cpu_req_valid(cpu_req_valid),
    .cpu_req_ready(cpu_req_ready),
    .cpu_req_rw(cpu_req_rw),
    .cpu_addr(cpu_addr),
    .cpu_wdata(cpu_wdata),
    .cpu_wstrb(cpu_wstrb),

    .cpu_resp_valid(cpu_resp_valid),
    .cpu_rdata(cpu_rdata)
  );

  // ----------------------------
  // Helper functions
  // ----------------------------
  function automatic [L1_LINE_W-1:0] make_l1_line_pattern(input logic [31:0] base);
  logic [L1_LINE_W-1:0] line;
  begin
    line = '0;
    for (i=0; i<8; i=i+1) begin
      line[i*32 +: 32] = base + i;
    end
    make_l1_line_pattern = line;
  end
endfunction


function automatic [ADDR_W-1:0] mem_line_align(input logic [ADDR_W-1:0] a);
  begin
    // 128B align => clear 7 LSBs
    mem_line_align = {a[ADDR_W-1:7], 7'b0};
  end
endfunction

function automatic [L2_LINE_W-1:0] put_subline32_into_line128(
  input logic [L2_LINE_W-1:0] line128_in,
  input logic [1:0]           sub,          // 0..3 (addr[6:5])
  input logic [L1_LINE_W-1:0] subline32
);
  logic [L2_LINE_W-1:0] line128_out;
  int base;
  begin
    line128_out = line128_in;
    base = sub * L1_LINE_W;                 // 0,256,512,768
    line128_out[base +: L1_LINE_W] = subline32;
    put_subline32_into_line128 = line128_out;
  end
endfunction

task automatic preload_subline(
  input logic [ADDR_W-1:0] addr,
  input logic [31:0]        base
);
  logic [L2_LINE_W-1:0] memline;
  logic [1:0] sub;
  begin
    sub = addr[6:5]; // which 32B chunk inside the 128B line

    // read existing 128B line (so we don't destroy other sublines)
    dut.u_mem.peek_line(mem_line_align(addr), memline);

    // modify only the required 32B region
    memline = put_subline32_into_line128(memline, sub, make_l1_line_pattern(base));

    // write back full 128B line
    dut.u_mem.preload_line(mem_line_align(addr), memline);
  end
endtask


  
  
  function automatic [ADDR_W-1:0] make_l1_conflict_addr(
  input logic [ADDR_W-1:0] base,
  input int k
);
  localparam int L1_OFFSET_BITS = 5;                 // LINE_BYTES=32
  localparam int L1_INDEX_BITS  = $clog2(L1_SETS);   // uses TB param
  localparam int L1_TAG_LSB     = L1_OFFSET_BITS + L1_INDEX_BITS;
  begin
    make_l1_conflict_addr = { base[ADDR_W-1:L1_TAG_LSB] + k, base[L1_TAG_LSB-1:0] };
  end
endfunction

function automatic [ADDR_W-1:0] make_l2_conflict_addr(
  input logic [ADDR_W-1:0] base,
  input int k
);
  localparam int L2_OFF_BITS   = 7;                // 128B
  localparam int L2_INDEX_BITS = $clog2(L2_SETS);  // 4 for 16 sets
  localparam int L2_TAG_LSB    = L2_OFF_BITS + L2_INDEX_BITS; // 11
  begin
    // keep [10:0] same (offset+index), bump tag by k
    make_l2_conflict_addr = { base[ADDR_W-1:L2_TAG_LSB] + k, base[L2_TAG_LSB-1:0] };
  end
endfunction


function automatic [L2_IDX_BITS-1:0] l2_idx(input logic [ADDR_W-1:0] a);
  return a[L2_OFF_BITS + L2_IDX_BITS - 1 : L2_OFF_BITS];
endfunction

function automatic [L2_TAG_BITS-1:0] l2_tag(input logic [ADDR_W-1:0] a);
  return a[ADDR_W-1 : L2_OFF_BITS + L2_IDX_BITS];
endfunction

task automatic peek_l1_set(input logic [ADDR_W-1:0] addr);
  localparam int L1_OFF_BITS = $clog2(L1_LINE_BYTES);
  localparam int L1_IDX_BITS = $clog2(L1_SETS);
  localparam int L1_TAG_BITS = ADDR_W - L1_OFF_BITS - L1_IDX_BITS;

  logic [L1_IDX_BITS-1:0] idx;
  logic [L1_TAG_BITS-1:0] tag;
  int w;
  begin
    idx = addr[L1_OFF_BITS + L1_IDX_BITS - 1 : L1_OFF_BITS];
    tag = addr[ADDR_W-1 : L1_OFF_BITS + L1_IDX_BITS];

    $display("---- L1 peek addr=%h ----", addr);
    $display("  L1 SET = %0d  (idx bits=%b)  TAG=%h", idx, idx, tag);
    for (w=0; w<L1_WAYS; w++) begin
      $display(" way%0d: V=%0d D=%0d TAG=%h HIT=%0d",
        w,
        dut.u_l1.valid_arr[idx][w],
        dut.u_l1.dirty_arr[idx][w],
        dut.u_l1.tag_arr[idx][w],
        (dut.u_l1.valid_arr[idx][w] && (dut.u_l1.tag_arr[idx][w] == tag))
      );
    end
  end
endtask




task automatic peek_l2_set(input logic [ADDR_W-1:0] addr);
  logic [L2_IDX_BITS-1:0] idx;
  logic [L2_TAG_BITS-1:0] tag;
  int w;
  begin
    idx = l2_idx(addr);
    tag = l2_tag(addr);

    $display("---- L2 peek addr=%h ----", addr);
    $display("  L2 SET = %0d  (idx bits=%b)  TAG=%h", idx, idx, tag);

    for (w = 0; w < L2_WAYS; w = w+1) begin
      $display(" way%0d: V=%0d D=%0d TAG=%h HIT=%0d",
        w,
        dut.u_l2.valid_arr[idx][w],
        dut.u_l2.dirty_arr[idx][w],
        dut.u_l2.tag_arr[idx][w],
        (dut.u_l2.valid_arr[idx][w] && (dut.u_l2.tag_arr[idx][w] == tag))
      );
    end
  end
endtask

task automatic peek_l2_line128(input logic [ADDR_W-1:0] addr);
  logic [L2_IDX_BITS-1:0] idx;
  logic [L2_TAG_BITS-1:0] tag;
  int w, hitw;
  bit found;
  begin
    idx = l2_idx(addr);
    tag = l2_tag(addr);
    hitw = 0;
	 found = 0;

    for (w=0; w<L2_WAYS; w++) begin
		if (dut.u_l2.valid_arr[idx][w] && (dut.u_l2.tag_arr[idx][w] == tag)) begin
			hitw  = w;
			found = 1;
		end
	 end

    if (!found) begin
      $display("---- L2 line NOT present for addr=%h (idx=%0d tag=%h) ----", addr, idx, tag);
    end else begin
      $display("---- L2 line present: addr=%h idx=%0d tag=%h hitway=%0d ----", addr, idx, tag, hitw);
      print_memline128("[TB] L2 line128", dut.u_l2.data_arr[idx][hitw]);
    end
  end
endtask


task automatic peek_vc();
  int e;
  begin
    $display("---- VC contents ----");
    for (e=0; e<VC_ENTRIES; e++) begin
      $display(" VC[%0d]: V=%0d D=%0d idx=%0d tag=%h",
        e,
        dut.u_l1.vc_valid[e],
        dut.u_l1.vc_dirty[e],
        dut.u_l1.vc_idx[e],
        dut.u_l1.vc_tag[e]
      );
    end
  end
endtask


task automatic print_addr_fields(input logic [ADDR_W-1:0] a, input string name);
  begin
    $display("[%s] %h | L1(idx=%0d tag=%h) | L2(idx=%0d tag=%h)",
      name, a,
      a[L1_OFF_BITS + L1_IDX_BITS - 1 : L1_OFF_BITS],
      a[ADDR_W-1 : L1_OFF_BITS + L1_IDX_BITS],
      a[L2_OFF_BITS + L2_IDX_BITS - 1 : L2_OFF_BITS],
      a[ADDR_W-1 : L2_OFF_BITS + L2_IDX_BITS]
    );
  end
endtask


task automatic idle_cycles(input int n);
  begin
    repeat(n) @(posedge clk);
  end
endtask


task automatic peek_wb();
  int e;
  int w;
  begin
    $display("---- WB contents @t=%0t (cycle=%0d) ----", $time, cycle);
    $display(" head=%0d tail=%0d count=%0d full=%0d empty=%0d",
      dut.u_l1.wb_head_ptr,
      dut.u_l1.wb_tail_ptr,
      dut.u_l1.wb_count,
      dut.u_l1.wb_full,
      dut.u_l1.wb_empty
    );

    for (e=0; e<WB_ENTRIES; e++) begin
      $display(" WB[%0d]: V=%0d D=%0d idx=%0d tag=%h",
        e,
        dut.u_l1.wb_valid[e],
        dut.u_l1.wb_dirty[e],
        dut.u_l1.wb_idx[e],
        dut.u_l1.wb_tag[e]
      );

      if (dut.u_l1.wb_valid[e]) begin
        for (w=0; w<8; w++) begin
          $display("    word[%0d] = %h", w, dut.u_l1.wb_data[e][w*32 +: 32]);
        end
      end
    end
  end
endtask
  

//  function automatic [ADDR_W-1:0] line_align(input logic [ADDR_W-1:0] a);
 //   begin
  //    line_align = {a[ADDR_W-1:5], 5'b0};
 //   end
 // endfunction

  // ----------------------------
  // CPU tasks
  // ----------------------------
  task automatic cpu_read(input logic [ADDR_W-1:0] addr, output logic [DATA_W-1:0] r);
    begin
      @(posedge clk);
      while (!cpu_req_ready) @(posedge clk);

      cpu_req_valid <= 1'b1;
      cpu_req_rw    <= 1'b0;
      cpu_addr      <= addr;
      cpu_wdata     <= '0;
      cpu_wstrb     <= '0;

      @(posedge clk);
      cpu_req_valid <= 1'b0;

      while (!cpu_resp_valid) @(posedge clk);
      r = cpu_rdata;
    end
  endtask

  task automatic cpu_write(input logic [ADDR_W-1:0] addr,
                           input logic [DATA_W-1:0] wdata,
                           input logic [DATA_W/8-1:0] wstrb);
    begin
      @(posedge clk);
      while (!cpu_req_ready) @(posedge clk);

      cpu_req_valid <= 1'b1;
      cpu_req_rw    <= 1'b1;
      cpu_addr      <= addr;
      cpu_wdata     <= wdata;
      cpu_wstrb     <= wstrb;

      @(posedge clk);
      cpu_req_valid <= 1'b0;

      while (!cpu_resp_valid) @(posedge clk);
    end
  endtask

  task automatic print_memline128(input string name, input logic [L2_LINE_W-1:0] line);
  begin
    $display("%s:", name);
    for (i=0; i<32; i=i+1) begin
      $display("  word[%0d] = %h", i, line[i*32 +: 32]);
    end
  end
endtask

always @(posedge clk) begin
  if (dut.u_l1.l2_req_valid && dut.u_l1.l2_req_ready) begin
    $display("[HS] rw=%0d addr=%h from_wb=%0d state=%0d time=%0t",
      dut.u_l1.l2_req_rw,
      dut.u_l1.l2_req_addr,
      dut.u_l1.l2_req_from_wb_r,
      dut.u_l1.state,
      $time
    );
  end
end


int unsigned cycle;

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) cycle <= 0;
  else        cycle <= cycle + 1;
end


task automatic mark(input string msg);
  $display("[MARK] t=%0t cycle=%0d : %s", $time, cycle, msg);
endtask

// Add this to your tb_cache_top
always @(posedge clk) begin
  if (dut.u_l1.state == dut.u_l1.S_WB_HIT_COMMIT) begin
    $display("\n[DATA MOVE: WB -> L1]");
    $display("  Time        : %0t", $time);
    $display("  Address     : %h", {dut.u_l1.tag_q, dut.u_l1.idx_q, 5'b0});
    $display("  WB Slot     : %0d", dut.u_l1.pend_wb_e);
    $display("  Target Way  : %0d", dut.u_l1.victim_way_c);
    $display("  Data (HEX)  : %h", dut.u_l1.pend_wb_line);
    
    // Check if it was a write-hit (data might be modified)
    if (dut.u_l1.req_rw_q) begin
      $display("  Note        : Data was modified by CPU Write before L1 install.");
    end
  end
end

// ----------------------------
// Monitor Idle Counter
// ----------------------------
always @(posedge clk) begin
    // We access the counter via the hierarchy: dut.u_l1 (assuming u_l1 is the L1 instance name)
    // Only print when it's non-zero to avoid flooding the log during active CPU hits
    if (dut.u_l1.idle_ctr > 0) begin
        $display("[IDLE MONITOR] t=%0t | Cycle=%0d | idle_ctr=%0d | WB_Count=%0d", 
                 $time, cycle, dut.u_l1.idle_ctr, dut.u_l1.wb_count);
    end
    
    // Catch the moment the WB actually starts draining because of the timeout
    if (dut.u_l1.wb_drain_en && (dut.u_l1.idle_ctr == dut.u_l1.MAX_IDLE)) begin
        $display("[IDLE MONITOR] t=%0t | *** TIMEOUT REACHED: Starting Background WB Drain ***", $time);
    end
end


  // ----------------------------
  // Test sequence
  // ----------------------------
  initial begin
    // init signals
    cpu_req_valid = 0;
    cpu_req_rw    = 0;
    cpu_addr      = 0;
    cpu_wdata     = 0;
    cpu_wstrb     = 0;

    // reset
    rst_n = 0;
    repeat(5) @(posedge clk);
    rst_n = 1;
    repeat(2) @(posedge clk);

    // ----------------------------
    // Preload main memory
    // We can directly call u_mem preload task through hierarchy:
    // dut.u_mem.preload_line(...)
    // ----------------------------
    A = 32'h0000_0044;

// Deterministic L1 conflicts: same OFFSET+INDEX bits, different TAGs
B = make_l1_conflict_addr(A, 1);
C = make_l1_conflict_addr(A, 2);
D = make_l1_conflict_addr(A, 3);

//For checking VC hit for A, data transfer from L1 to VC
J = make_l1_conflict_addr(A, 4);
K = make_l1_conflict_addr(A, 5);

E = make_l2_conflict_addr(A, 1);
F = make_l2_conflict_addr(A, 2);
G = make_l2_conflict_addr(A, 3);
H = make_l2_conflict_addr(A, 4);
I = make_l2_conflict_addr(A, 5);

//For Checking eviction from VC to L2
L = make_l1_conflict_addr(A, 6);   
M = make_l1_conflict_addr(A, 7);   
N = make_l1_conflict_addr(A, 12);  
O = make_l1_conflict_addr(A, 9);  
P = make_l1_conflict_addr(A, 10);  
Q = make_l1_conflict_addr(A, 11);  

//For Checking eviction from L2 to Mem
R = make_l2_conflict_addr(A, 6);
S = make_l2_conflict_addr(A, 7);
T = make_l2_conflict_addr(A, 8);
U = make_l2_conflict_addr(A, 9);
V = make_l2_conflict_addr(A, 10);

// Optional debug: show L1 index bits [10:5] (should be identical)
print_addr_fields(A, "A");
print_addr_fields(B, "B");
print_addr_fields(C, "C");
print_addr_fields(D, "D");
print_addr_fields(E, "E");
print_addr_fields(F, "F");
print_addr_fields(G, "G");
print_addr_fields(H, "H");
print_addr_fields(I, "I");

print_addr_fields(J, "J");
print_addr_fields(K, "K");
print_addr_fields(L, "L");
print_addr_fields(M, "M");
print_addr_fields(N, "N");
print_addr_fields(O, "O");
print_addr_fields(P, "P");
print_addr_fields(Q, "Q");


print_addr_fields(R, "R");
print_addr_fields(S, "S");
print_addr_fields(T, "T");
print_addr_fields(U, "U");
print_addr_fields(V, "V");


// Preload main memory lines for each address
preload_subline(A, 32'h1111_0000);
preload_subline(B, 32'h2222_0000);
preload_subline(C, 32'h3333_0000);
preload_subline(D, 32'h4444_0000);
preload_subline(J, 32'hDDDD_0000);
preload_subline(K, 32'h9999_0000);

preload_subline(E, 32'hEEEE_0000);
preload_subline(F, 32'hFFFF_0000);
preload_subline(G, 32'hAAAA_0000);
preload_subline(H, 32'hBBBB_0000);
preload_subline(I, 32'hCCCC_0000);

preload_subline(L, 32'h6666_0000);
preload_subline(M, 32'h7777_0000);
preload_subline(N, 32'hCCCC_1000);
preload_subline(O, 32'hDDDD_1000);
preload_subline(P, 32'hEEEE_1000);
preload_subline(Q, 32'hFFFF_1000);

preload_subline(R, 32'hAAAA_2000);
preload_subline(S, 32'hBBBB_2000);
preload_subline(T, 32'hCCCC_2000);
preload_subline(U, 32'hDDDD_2000);
preload_subline(V, 32'hEEEE_2000);




    // ----------------------------
    // 1) Read A (cold miss -> fetch from MEM via L2)
    // ----------------------------
    cpu_read(A, rdata);
    $display("[TB] Read A = %h", rdata);

    // ----------------------------
    // 2) Write a byte into A (hit in L1)
    // ----------------------------
    cpu_write(A, 32'h0000_00AA, 4'b0001);

    // Read back A to confirm byte write
    cpu_read(A, rdata);
    $display("[TB] Read A after byte write = %h", rdata);

    // ----------------------------
    // 3) Cause pressure (access B, C, D)
    // This aims to evict dirty A from L1 into Victim Cache
    // ----------------------------
    cpu_read(B, rdata);
    $display("[TB] Read B = %h", rdata);
	 
	 cpu_write(B, 32'h0000_00BB, 4'b0001);

    cpu_read(B, rdata);
    $display("[TB] Read B after byte write = %h", rdata);


    cpu_read(C, rdata);
    $display("[TB] Read C = %h", rdata);
	 
	 cpu_write(C, 32'h0000_00CC, 4'b0001);


    cpu_read(C, rdata);
    $display("[TB] Read C after byte write = %h", rdata);


    cpu_read(D, rdata);
    $display("[TB] Read D = %h", rdata);
	 
	 cpu_write(D, 32'h0000_00DD, 4'b0001);

    // Read back A to confirm byte write
    cpu_read(D, rdata);
    $display("[TB] Read D after byte write = %h", rdata);
	 
	 
	 cpu_read(E, rdata);
    $display("[TB] Read E = %h", rdata);
	 
	 cpu_write(E, 32'h0000_00EE, 4'b0001);

    cpu_read(E, rdata);
    $display("[TB] Read E after byte write = %h", rdata);
	 
	 cpu_read(F, rdata);
    $display("[TB] Read F = %h", rdata);
	 
	 cpu_write(F, 32'h0000_00FF, 4'b0001);

    cpu_read(F, rdata);
    $display("[TB] Read F after byte write = %h", rdata);
	 
	 cpu_read(G, rdata);
    $display("[TB] Read G = %h", rdata);
	 
	 cpu_read(H, rdata);
    $display("[TB] Read H = %h", rdata);
	 
	 cpu_read(I, rdata);
    $display("[TB] Read I = %h", rdata);
	 
	 
	 cpu_read(J, rdata);
    $display("[TB] Read J = %h", rdata);
	 
	 mark("Before A got evicted to WB");
	 peek_l1_set(A);
	 peek_vc();
	 peek_wb();
    peek_l2_set(A);
	 peek_l2_line128(A);
	 
	 
	 cpu_read(K, rdata);
    $display("[TB] Read K = %h", rdata);
	 
	 peek_wb();
	 
	 mark("Write Timing of A");
	 cpu_write(A, 32'h0000_00AC, 4'b0001);
	 
	 mark("Check the content of WB again after Writing a new value in A");
	 peek_l1_set(A);
	 peek_vc();
	 peek_wb();
    peek_l2_set(A);
	 peek_l2_line128(A);
	 
	 mark("Read Timing of A");
	 cpu_read(A, rdata);
    $display("[TB] Read A after byte write = %h", rdata);
	 
	 cpu_read(L, rdata);
	 $display("[TB] Read L = %h", rdata);
	 
	 idle_cycles(10);
	
	 cpu_read(M, rdata);
	 $display("[TB] Read M = %h", rdata);
	 
	 peek_vc();
	 peek_wb();
	 
	 cpu_read(N, rdata);
	 $display("[TB] Read N = %h", rdata);
	 
	 mark("Check the content of WB, D is also evicted");
	 
	 peek_wb();
 
	
	 cpu_read(O, rdata);
	 $display("[TB] Read O = %h", rdata);
	 
	 peek_wb();

	
	 cpu_read(P, rdata);
	 $display("[TB] Read P = %h", rdata);
	
	
	 mark("Check the WB F got evicted");
	 peek_wb();
    
    cpu_read(Q, rdata);
	 $display("[TB] Read Q = %h", rdata);
	 
	 cpu_read(R, rdata);
	 $display("[TB] Read R = %h", rdata);

	 cpu_read(S, rdata);
	 $display("[TB] Read S = %h", rdata);
	
	 cpu_read(T, rdata);
	 $display("[TB] Read T = %h", rdata);
	
	 cpu_read(U, rdata);
	 $display("[TB] Read U = %h", rdata);
	
	 cpu_read(V, rdata);
	 $display("[TB] Read V = %h", rdata);
	 
	 peek_vc();
    peek_l1_set(A);
	 peek_l1_set(B);
	 peek_l1_set(C);
	 peek_l1_set(D);
	 peek_l1_set(E);
	 peek_l1_set(F);
    peek_l2_set(A);
	 peek_l2_line128(A);
	 peek_l2_set(B);
	 peek_l2_line128(B);
	 peek_l2_set(C);
	 peek_l2_line128(C);
	 peek_l2_set(D);
	 peek_l2_line128(D);
	 peek_l2_set(E);
	 peek_l2_line128(E);
	 peek_l2_set(F);
	 peek_l2_line128(F);
    
	 // ----------------------------
    // Peek memory line for A to see if writeback happened to MEM
    // Note: writeback path is L1-> Victim Cache -> L2 first; L2->MEM occurs only if L2 evicts dirty.
    // So MEM may still show old line if L2 kept it.
    // ----------------------------
    dut.u_mem.peek_line(mem_line_align(A), line_tmp);
	 print_memline128("[TB] MEM line(A) peek", line_tmp);
	 
	 dut.u_mem.peek_line(mem_line_align(B), line_tmp);
	 print_memline128("[TB] MEM line(B) peek", line_tmp);
	 
	 dut.u_mem.peek_line(mem_line_align(C), line_tmp);
    print_memline128("[TB] MEM line(C) peek", line_tmp);
	 
	 dut.u_mem.peek_line(mem_line_align(D), line_tmp);
    print_memline128("[TB] MEM line(D) peek", line_tmp);
	 
	 dut.u_mem.peek_line(mem_line_align(E), line_tmp);
    print_memline128("[TB] MEM line(E) peek", line_tmp);
	 
	 dut.u_mem.peek_line(mem_line_align(F), line_tmp);
    print_memline128("[TB] MEM line(F) peek", line_tmp);

    $display("[TB] DONE");
    #50;
    $finish;
  end

endmodule
