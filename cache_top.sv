module cache_top #(
  parameter ADDR_W      = 32,
  parameter DATA_W      = 32,
  parameter LINE_BYTES  = 32,
  parameter LINE_W      = LINE_BYTES*8,

  // L1 params
  parameter L1_WAYS     = 2,
  parameter L1_SETS     = 8, //64

  // L2 params (default 32KB: 4-way, 256 sets)
  parameter L2_WAYS     = 4,
  parameter L2_SETS     = 16, //256

  // Main memory params
  parameter MEM_LINES   = 4096,
  parameter MEM_LATENCY = 5
)(
  input  logic                 clk,
  input  logic                 rst_n,

  // ============================
  // CPU word interface
  // ============================
  input  logic                 cpu_req_valid,
  output logic                 cpu_req_ready,
  input  logic                 cpu_req_rw,        // 0=read, 1=write
  input  logic [ADDR_W-1:0]    cpu_addr,
  input  logic [DATA_W-1:0]    cpu_wdata,
  input  logic [DATA_W/8-1:0]  cpu_wstrb,

  output logic                 cpu_resp_valid,
  output logic [DATA_W-1:0]    cpu_rdata
);

  // =========================================================
  // Wires between L1 and L2 (line interface)
  // =========================================================
  logic                 l1_l2_req_valid;
  logic                 l1_l2_req_ready;
  logic                 l1_l2_req_rw;       // 0=readline, 1=writeline
  logic [ADDR_W-1:0]    l1_l2_req_addr;
  logic [LINE_W-1:0]    l1_l2_req_wline;

  logic                 l2_l1_resp_valid;
  logic [LINE_W-1:0]    l2_l1_resp_rline;

  // =========================================================
  // Wires between L2 and Main Memory (line interface)
  // =========================================================
  logic                 l2_mem_req_valid;
  logic                 l2_mem_req_ready;
  logic                 l2_mem_req_rw;      // 0=readline, 1=writeline
  logic [ADDR_W-1:0]    l2_mem_req_addr;
  logic [LINE_W-1:0]    l2_mem_req_wline;

  logic                 mem_resp_valid;
  logic [LINE_W-1:0]    mem_resp_rline;

  // ============================
  // Instantiate L1
  // ============================
  l1_cache #(
    .ADDR_W(ADDR_W),
    .DATA_W(DATA_W),
    .LINE_BYTES(LINE_BYTES),
    .WAYS(L1_WAYS),
    .SETS(L1_SETS),
    .LINE_W(LINE_W)
  ) u_l1 (
    .clk(clk),
    .rst_n(rst_n),

    // CPU
    .cpu_req_valid(cpu_req_valid),
    .cpu_req_ready(cpu_req_ready),
    .cpu_req_rw(cpu_req_rw),
    .cpu_addr(cpu_addr),
    .cpu_wdata(cpu_wdata),
    .cpu_wstrb(cpu_wstrb),

    .cpu_resp_valid(cpu_resp_valid),
    .cpu_rdata(cpu_rdata),

    // To L2
    .l2_req_valid(l1_l2_req_valid),
    .l2_req_ready(l1_l2_req_ready),
    .l2_req_rw(l1_l2_req_rw),
    .l2_req_addr(l1_l2_req_addr),
    .l2_req_wline(l1_l2_req_wline),

    // From L2
    .l2_resp_valid(l2_l1_resp_valid),
    .l2_resp_rline(l2_l1_resp_rline)
  );

  // ============================
  // Instantiate L2
  // ============================
  l2_cache #(
    .ADDR_W(ADDR_W),
    .LINE_BYTES(LINE_BYTES),
    .LINE_W(LINE_W),
    .WAYS(L2_WAYS),
    .SETS(L2_SETS)
  ) u_l2 (
    .clk(clk),
    .rst_n(rst_n),

    // Upper (from L1)
    .up_req_valid(l1_l2_req_valid),
    .up_req_ready(l1_l2_req_ready),
    .up_req_rw(l1_l2_req_rw),
    .up_req_addr(l1_l2_req_addr),
    .up_req_wline(l1_l2_req_wline),

    .up_resp_valid(l2_l1_resp_valid),
    .up_resp_rline(l2_l1_resp_rline),

    // Lower (to MEM)
    .mem_req_valid(l2_mem_req_valid),
    .mem_req_ready(l2_mem_req_ready),
    .mem_req_rw(l2_mem_req_rw),
    .mem_req_addr(l2_mem_req_addr),
    .mem_req_wline(l2_mem_req_wline),

    .mem_resp_valid(mem_resp_valid),
    .mem_resp_rline(mem_resp_rline)
  );

  // ============================
  // Instantiate Main Memory
  // ============================
  main_mem #(
    .ADDR_W(ADDR_W),
    .LINE_BYTES(LINE_BYTES),
    .LINE_W(LINE_W),
    .RD_LATENCY(MEM_LATENCY),
    .MEM_LINES(MEM_LINES)
  ) u_mem (
    .clk(clk),
    .rst_n(rst_n),

    .mem_req_valid(l2_mem_req_valid),
    .mem_req_ready(l2_mem_req_ready),
    .mem_req_rw(l2_mem_req_rw),
    .mem_req_addr(l2_mem_req_addr),
    .mem_req_wline(l2_mem_req_wline),

    .mem_resp_valid(mem_resp_valid),
    .mem_resp_rline(mem_resp_rline)
  );

endmodule
