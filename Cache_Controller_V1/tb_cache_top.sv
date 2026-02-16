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
  logic [DATA_W-1:0] rdata;
  logic [L2_LINE_W-1:0] line_tmp;

  integer i;
  
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


task automatic peek_l2_set(input logic [ADDR_W-1:0] addr);
  logic [L2_IDX_BITS-1:0] idx;
  logic [L2_TAG_BITS-1:0] tag;
  int w;
  begin
    idx = l2_idx(addr);
    tag = l2_tag(addr);

    $display("---- L2 peek for addr=%h (idx=%0d tag=%h) ----", addr, idx, tag);

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

E = make_l2_conflict_addr(A, 1);
F = make_l2_conflict_addr(A, 2);
G = make_l2_conflict_addr(A, 3);
H = make_l2_conflict_addr(A, 4);
I = make_l2_conflict_addr(A, 5);

// Optional debug: show L1 index bits [10:5] (should be identical)
$display("[TB] A=%h idx=%0d", A, A[7:5]);
$display("[TB] B=%h idx=%0d", B, B[7:5]);
$display("[TB] C=%h idx=%0d", C, C[7:5]);
$display("[TB] D=%h idx=%0d", D, D[7:5]);

$display("[TB] E=%h idx=%0d", E, E[10:7]);
$display("[TB] F=%h idx=%0d", F, F[10:7]);
$display("[TB] G=%h idx=%0d", G, G[10:7]);
$display("[TB] H=%h idx=%0d", H, H[10:7]);
$display("[TB] I=%h idx=%0d", I, I[10:7]);

// Preload main memory lines for each address
preload_subline(A, 32'h1111_0000);
preload_subline(B, 32'h2222_0000);
preload_subline(C, 32'h3333_0000);
preload_subline(D, 32'h4444_0000);

preload_subline(E, 32'hEEEE_0000);
preload_subline(F, 32'hFFFF_0000);
preload_subline(G, 32'hAAAA_0000);
preload_subline(H, 32'hBBBB_0000);
preload_subline(I, 32'hCCCC_0000);



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
    // This aims to evict dirty A from L1 (and eventually from L2 if enough pressure)
    // ----------------------------
    cpu_read(B, rdata);
    $display("[TB] Read B = %h", rdata);

    cpu_read(C, rdata);
    $display("[TB] Read C = %h", rdata);

    cpu_read(D, rdata);
    $display("[TB] Read D = %h", rdata);
	 
	 // Access them to force L2 set to overflow (4-way => 5th causes eviction)
	cpu_read(E, rdata);
	$display("[TB] Read E = %h", rdata);
	
	cpu_read(F, rdata);
	$display("[TB] Read F = %h", rdata);
	
	cpu_read(G, rdata);
	$display("[TB] Read G = %h", rdata);
	
	cpu_read(H, rdata);
	$display("[TB] Read H = %h", rdata);
	
	cpu_read(I, rdata);
	$display("[TB] Read I = %h", rdata);
	
	 
	 // NOW L1 should have written back dirty A into L2
    peek_l2_set(A);
    peek_l2_line128(A);

    // ----------------------------
    // Peek memory line for A to see if writeback happened to MEM
    // Note: writeback path is L1->L2 first; L2->MEM occurs only if L2 evicts dirty.
    // So MEM may still show old line if L2 kept it.
    // ----------------------------
    dut.u_mem.peek_line(mem_line_align(A), line_tmp);
    print_memline128("[TB] MEM line(A) peek", line_tmp);

    $display("[TB] DONE");
    #50;
    $finish;
  end

endmodule
