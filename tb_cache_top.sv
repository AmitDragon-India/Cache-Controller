module tb_cache_top;

  // ----------------------------
  // Parameters
  // ----------------------------
  localparam int ADDR_W      = 32;
  localparam int DATA_W      = 32;
  localparam int LINE_BYTES  = 32;
  localparam int LINE_W      = LINE_BYTES*8;

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
  logic [DATA_W-1:0] rdata;
  logic [LINE_W-1:0] line_tmp;

  integer i;

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
    .LINE_BYTES(LINE_BYTES),
    .LINE_W(LINE_W),

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
  function automatic [LINE_W-1:0] make_line_pattern(input logic [31:0] base);
    logic [LINE_W-1:0] line;
    begin
      line = '0;
      for (i=0; i<8; i=i+1) begin
        line[i*32 +: 32] = base + i;
      end
      make_line_pattern = line;
    end
  endfunction
  
  
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

  

  function automatic [ADDR_W-1:0] line_align(input logic [ADDR_W-1:0] a);
    begin
      line_align = {a[ADDR_W-1:5], 5'b0};
    end
  endfunction

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

  task automatic print_line(input string name, input logic [LINE_W-1:0] line);
    begin
      $display("%s:", name);
      for (i=0; i<8; i=i+1) begin
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

// Optional debug: show L1 index bits [10:5] (should be identical)
$display("[TB] A=%h idx=%0d", A, A[10:5]);
$display("[TB] B=%h idx=%0d", B, B[10:5]);
$display("[TB] C=%h idx=%0d", C, C[10:5]);
$display("[TB] D=%h idx=%0d", D, D[10:5]);

// Preload main memory lines for each address
dut.u_mem.preload_line(line_align(A), make_line_pattern(32'h1111_0000));
dut.u_mem.preload_line(line_align(B), make_line_pattern(32'h2222_0000));
dut.u_mem.preload_line(line_align(C), make_line_pattern(32'h3333_0000));
dut.u_mem.preload_line(line_align(D), make_line_pattern(32'h4444_0000));

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

    // ----------------------------
    // Peek memory line for A to see if writeback happened to MEM
    // Note: writeback path is L1->L2 first; L2->MEM occurs only if L2 evicts dirty.
    // So MEM may still show old line if L2 kept it.
    // ----------------------------
    dut.u_mem.peek_line(line_align(A), line_tmp);
    print_line("[TB] MEM line(A) peek", line_tmp);

    $display("[TB] DONE");
    #50;
    $finish;
  end

endmodule
