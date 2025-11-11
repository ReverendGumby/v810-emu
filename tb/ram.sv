module ram
  #(parameter AW,
    parameter DW)
  (
   input            CLK,
   input            nCE,
   input            nWE,
   input            nOE,
   input [DW/8-1:0] nBE,
   input [AW-1:0]   A,
   input [DW-1:0]   DI,
   output [DW-1:0]  DO
 );

localparam SIZE = 1 << AW;

bit [DW-1:0]    mem [0:SIZE-1];
bit [DW-1:0]    dor;

task clear;
    for (int i = 0; i < SIZE; i++)
        mem[i] = '0;
endtask

task load_hex(string fn);
    clear();
    $readmemh(fn, mem);
endtask

task load_hex16(string fn);
bit [15:0] tmp [SIZE * 2];
    assert(DW / 16 == 2);
    //$display("Loading %s", fn);
    for (int i = 0; i < SIZE * 2; i++)
        tmp[i] = '0;
    $readmemh(fn, tmp);
    for (int i = 0; i < SIZE * 2; i++)
        mem[i / 2][i[0]*16+:16] = tmp[i];
endtask

task load_bin(string fn);
integer fin, code;
    begin
        clear();
        fin = $fopen(fn, "r");
        assert(fin != 0) else $fatal(1, "unable to open file");
        code = $fread(mem, fin, 0, SIZE);
        assert(code == SIZE) else $fatal(1, "file too short");
    end
endtask

always @(negedge CLK) begin
    dor <= mem[A];
end

assign DO = ~(nCE | nOE) ? dor : {DW{1'bz}};

always @(posedge CLK) begin
    for (int i = 0; i < DW/8; i++)
        if (~(nCE | nWE | nBE[i]))
            mem[A][i*8+:8] <= DI[i*8+:8];
end

endmodule
