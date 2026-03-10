module main_mem #(
  parameter ADDR_W     = 32,
  parameter LINE_BYTES = 128,
  parameter LINE_W     = LINE_BYTES*8,
  parameter RD_LATENCY = 5,          // cycles from accepted read to resp_valid
  parameter MEM_LINES  = 2048 //4096        // number of cache lines (4096*32B = 128KB)
)(
  input  logic              clk,
  input  logic              rst_n,

  // Line request
  input  logic              mem_req_valid,
  output logic              mem_req_ready,
  input  logic              mem_req_rw,        // 0=readline, 1=writeline
  input  logic [ADDR_W-1:0] mem_req_addr,      // byte addr (line aligned)
  input  logic [LINE_W-1:0] mem_req_wline,

  // Line response (only for reads)
  output logic              mem_resp_valid,
  output logic [LINE_W-1:0] mem_resp_rline
);

  localparam int OFFSET_BITS = $clog2(LINE_BYTES);
  localparam int INDEX_BITS  = $clog2(MEM_LINES);

  // Simple indexed memory (tool-friendly)
  logic [LINE_W-1:0] mem [0:MEM_LINES-1];

  // Always ready (blocking caches above assume this)
  assign mem_req_ready = 1'b1;

  // Read pipeline regs
  logic pending;
  integer rd_count;
  logic [INDEX_BITS-1:0] rd_index;

  // Compute line index from address (low bits above offset)
  wire [INDEX_BITS-1:0] line_index;
  assign line_index = mem_req_addr[OFFSET_BITS + INDEX_BITS - 1 : OFFSET_BITS];

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      mem_resp_valid <= 1'b0;
      mem_resp_rline <= '0;

      pending        <= 1'b0;
      rd_count       <= 0;
      rd_index       <= '0;
    end else begin
      mem_resp_valid <= 1'b0; // default pulse low

      // Accept request
      if (mem_req_valid && mem_req_ready) begin
        if (mem_req_rw) begin
          // WRITE LINE
          mem[line_index] <= mem_req_wline;
        end else begin
          // READ LINE
          pending  <= 1'b1;
          rd_count <= RD_LATENCY;
          rd_index <= line_index;
        end
      end

      // Serve pending read after latency
      if (pending) begin
        if (rd_count == 0) begin
          mem_resp_valid <= 1'b1;
          mem_resp_rline <= mem[rd_index];
          pending        <= 1'b0;
        end else begin
          rd_count <= rd_count - 1;
        end
      end
    end
  end

  // -------------------------------------------------
  // Optional TB helpers (safe style)
  // -------------------------------------------------
  task preload_line(
    input logic [ADDR_W-1:0] addr,
    input logic [LINE_W-1:0] line
  );
    logic [INDEX_BITS-1:0] idx;
    begin
      idx = addr[OFFSET_BITS + INDEX_BITS - 1 : OFFSET_BITS];
      mem[idx] = line;
    end
  endtask

  task peek_line(
    input  logic [ADDR_W-1:0] addr,
    output logic [LINE_W-1:0] line
  );
    logic [INDEX_BITS-1:0] idx;
    begin
      idx  = addr[OFFSET_BITS + INDEX_BITS - 1 : OFFSET_BITS];
      line = mem[idx];
    end
  endtask

endmodule
