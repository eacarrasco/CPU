`timescale 1ns / 1ps

module bb(Pcurr,Pprev,Gcurr,Gprev,Pout,Gout);
	input Pcurr,Pprev,Gcurr,Gprev;
	output Pout, Gout;
	
	assign Pout = Pcurr & Pprev;
	assign Gout = Gcurr | (Pcurr & Gprev);
endmodule

module prefixAdder(A,B,cin,sum,cout);
	input [15:0] A,B;
	input cin;
	output [15:0] sum;
	output cout;
	
	wire [14:0] P,G;
	wire [7:0] P0,G0,P1,G1,P2,G2,P3,G3;
	
	assign P[0] = A[0] | B[0];
	assign G[0] = A[0] & B[0];
	assign P[1] = A[1] | B[1];
	assign G[1] = A[1] & B[1];
	assign P[2] = A[2] | B[2];
	assign G[2] = A[2] & B[2];
	assign P[3] = A[3] | B[3];
	assign G[3] = A[3] & B[3];
	assign P[4] = A[4] | B[4];
	assign G[4] = A[4] & B[4];
	assign P[5] = A[5] | B[5];
	assign G[5] = A[5] & B[5];
	assign P[6] = A[6] | B[6];
	assign G[6] = A[6] & B[6];
	assign P[7] = A[7] | B[7];
	assign G[7] = A[7] & B[7];
	assign P[8] = A[8] | B[8];
	assign G[8] = A[8] & B[8];
	assign P[9] = A[9] | B[9];
	assign G[9] = A[9] & B[9];
	assign P[10] = A[10] | B[10];
	assign G[10] = A[10] & B[10];
	assign P[11] = A[11] | B[11];
	assign G[11] = A[11] & B[11];
	assign P[12] = A[12] | B[12];
	assign G[12] = A[12] & B[12];
	assign P[13] = A[13] | B[13];
	assign G[13] = A[13] & B[13];
	assign P[14] = A[14] | B[14];
	assign G[14] = A[14] & B[14];
	
	bb bb01(P[0],1'b0,G[0],cin,P0[0],G0[0]);
	bb bb02(P[2],P[1],G[2],G[1],P0[1],G0[1]);
	bb bb03(P[4],P[3],G[4],G[3],P0[2],G0[2]);
	bb bb04(P[6],P[5],G[6],G[5],P0[3],G0[3]);
	bb bb05(P[8],P[7],G[8],G[7],P0[4],G0[4]);
	bb bb06(P[10],P[9],G[10],G[9],P0[5],G0[5]);
	bb bb07(P[12],P[11],G[12],G[11],P0[6],G0[6]);
	bb bb08(P[14],P[13],G[14],G[13],P0[7],G0[7]);
	
	bb bb11(P[1],P0[0],G[1],G0[0],P1[0],G1[0]);
	bb bb12(P0[1],P0[0],G0[1],G0[0],P1[1],G1[1]);
	bb bb13(P[5],P0[2],G[5],G0[2],P1[2],G1[2]);
	bb bb14(P0[3],P0[2],G0[3],G0[2],P1[3],G1[3]);
	bb bb15(P[9],P0[4],G[9],G0[4],P1[4],G1[4]);
	bb bb16(P0[5],P0[4],G0[5],G0[4],P1[5],G1[5]);
	bb bb17(P[13],P0[6],G[13],G0[6],P1[6],G1[6]);
	bb bb18(P0[7],P0[6],G0[7],G0[6],P1[7],G1[7]);
	
	bb bb21(P[3],P1[1],G[3],G1[1],P2[0],G2[0]);
	bb bb22(P0[2],P1[1],G0[2],G1[1],P2[1],G2[1]);
	bb bb23(P1[2],P1[1],G1[2],G1[1],P2[2],G2[2]);
	bb bb24(P1[3],P1[1],G1[3],G1[1],P2[3],G2[3]);
	bb bb25(P[11],P1[5],G[11],G1[5],P2[4],G2[4]);
	bb bb26(P0[6],P1[5],G0[6],G1[5],P2[5],G2[5]);
	bb bb27(P1[6],P1[5],G1[6],G1[5],P2[6],G2[6]);
	bb bb28(P1[7],P1[5],G1[7],G1[5],P2[7],G2[7]);
	
	bb bb31(P[7],P2[3],G[7],G2[3],P3[0],G3[0]);
	bb bb32(P0[4],P2[3],G0[4],G2[3],P3[1],G3[1]);
	bb bb33(P1[4],P2[3],G1[4],G2[3],P3[2],G3[2]);
	bb bb34(P1[5],P2[3],G1[5],G2[3],P3[3],G3[3]);
	bb bb35(P2[4],P2[3],G2[4],G2[3],P3[4],G3[4]);
	bb bb36(P2[5],P2[3],G2[5],G2[3],P3[5],G3[5]);
	bb bb37(P2[6],P2[3],G2[6],G2[3],P3[6],G3[6]);
	bb bb38(P2[7],P2[3],G2[7],G2[3],P3[7],G3[7]);
	
	assign sum[0] = cin ^ (A[0] ^ B[0]);
	assign sum[1] = G0[0] ^ (A[1] ^ B[1]);
	assign sum[2] = G1[0] ^ (A[2] ^ B[2]);
	assign sum[3] = G1[1] ^ (A[3] ^ B[3]);
	assign sum[4] = G2[0] ^ (A[4] ^ B[4]);
	assign sum[5] = G2[1] ^ (A[5] ^ B[5]);
	assign sum[6] = G2[2] ^ (A[6] ^ B[6]);
	assign sum[7] = G2[3] ^ (A[7] ^ B[7]);
	assign sum[8] = G3[0] ^ (A[8] ^ B[8]);
	assign sum[9] = G3[1] ^ (A[9] ^ B[9]);
	assign sum[10] = G3[2] ^ (A[10] ^ B[10]);
	assign sum[11] = G3[3] ^ (A[11] ^ B[11]);
	assign sum[12] = G3[4] ^ (A[12] ^ B[12]);
	assign sum[13] = G3[5] ^ (A[13] ^ B[13]);
	assign sum[14] = G3[6] ^ (A[14] ^ B[14]);
	assign sum[15] = G3[7] ^ (A[15] ^ B[15]);
	assign cout = (A[15] & B[15]) | (G3[7] & (A[15] | B[15]));
endmodule

module shifter(
    input [31:0] in,
    input [4:0] shamt,
    input [1:0] control,
    output [31:0] out
    );
    
    // 01 = SLL, 10 = SRL, 11 = SRA
    wire ext;
    
    assign ext = control[0] & in[31];
    
    assign out = ({32{~control[1]}} & 
                 ((in                    & {32{~shamt[4] & ~shamt[3] & ~shamt[2] & ~shamt[1] & ~shamt[0]}}) |
                 ({in[30:0], 1'b0}       & {32{~shamt[4] & ~shamt[3] & ~shamt[2] & ~shamt[1] &  shamt[0]}}) |
                 ({in[29:0], {2{1'b0}}}  & {32{~shamt[4] & ~shamt[3] & ~shamt[2] &  shamt[1] & ~shamt[0]}}) |
                 ({in[28:0], {3{1'b0}}}  & {32{~shamt[4] & ~shamt[3] & ~shamt[2] &  shamt[1] &  shamt[0]}}) |
                 ({in[27:0], {4{1'b0}}}  & {32{~shamt[4] & ~shamt[3] &  shamt[2] & ~shamt[1] & ~shamt[0]}}) |
                 ({in[26:0], {5{1'b0}}}  & {32{~shamt[4] & ~shamt[3] &  shamt[2] & ~shamt[1] &  shamt[0]}}) |
                 ({in[25:0], {6{1'b0}}}  & {32{~shamt[4] & ~shamt[3] &  shamt[2] &  shamt[1] & ~shamt[0]}}) |
                 ({in[24:0], {7{1'b0}}}  & {32{~shamt[4] & ~shamt[3] &  shamt[2] &  shamt[1] &  shamt[0]}}) |
                 ({in[23:0], {8{1'b0}}}  & {32{~shamt[4] &  shamt[3] & ~shamt[2] & ~shamt[1] & ~shamt[0]}}) |
                 ({in[22:0], {9{1'b0}}}  & {32{~shamt[4] &  shamt[3] & ~shamt[2] & ~shamt[1] &  shamt[0]}}) |
                 ({in[21:0], {10{1'b0}}} & {32{~shamt[4] &  shamt[3] & ~shamt[2] &  shamt[1] & ~shamt[0]}}) |
                 ({in[20:0], {11{1'b0}}} & {32{~shamt[4] &  shamt[3] & ~shamt[2] &  shamt[1] &  shamt[0]}}) |
                 ({in[19:0], {12{1'b0}}} & {32{~shamt[4] &  shamt[3] &  shamt[2] & ~shamt[1] & ~shamt[0]}}) |
                 ({in[18:0], {13{1'b0}}} & {32{~shamt[4] &  shamt[3] &  shamt[2] & ~shamt[1] &  shamt[0]}}) |
                 ({in[17:0], {14{1'b0}}} & {32{~shamt[4] &  shamt[3] &  shamt[2] &  shamt[1] & ~shamt[0]}}) |
                 ({in[16:0], {15{1'b0}}} & {32{~shamt[4] &  shamt[3] &  shamt[2] &  shamt[1] &  shamt[0]}}) |
                 ({in[15:0], {16{1'b0}}} & {32{ shamt[4] & ~shamt[3] & ~shamt[2] & ~shamt[1] & ~shamt[0]}}) |
                 ({in[14:0], {17{1'b0}}} & {32{ shamt[4] & ~shamt[3] & ~shamt[2] & ~shamt[1] &  shamt[0]}}) |
                 ({in[13:0], {18{1'b0}}} & {32{ shamt[4] & ~shamt[3] & ~shamt[2] &  shamt[1] & ~shamt[0]}}) |
                 ({in[12:0], {19{1'b0}}} & {32{ shamt[4] & ~shamt[3] & ~shamt[2] &  shamt[1] &  shamt[0]}}) |
                 ({in[11:0], {20{1'b0}}} & {32{ shamt[4] & ~shamt[3] &  shamt[2] & ~shamt[1] & ~shamt[0]}}) |
                 ({in[10:0], {21{1'b0}}} & {32{ shamt[4] & ~shamt[3] &  shamt[2] & ~shamt[1] &  shamt[0]}}) |
                 ({in[9:0],  {22{1'b0}}} & {32{ shamt[4] & ~shamt[3] &  shamt[2] &  shamt[1] & ~shamt[0]}}) |
                 ({in[8:0],  {23{1'b0}}} & {32{ shamt[4] & ~shamt[3] &  shamt[2] &  shamt[1] &  shamt[0]}}) |
                 ({in[7:0],  {24{1'b0}}} & {32{ shamt[4] &  shamt[3] & ~shamt[2] & ~shamt[1] & ~shamt[0]}}) |
                 ({in[6:0],  {25{1'b0}}} & {32{ shamt[4] &  shamt[3] & ~shamt[2] & ~shamt[1] &  shamt[0]}}) |
                 ({in[5:0],  {26{1'b0}}} & {32{ shamt[4] &  shamt[3] & ~shamt[2] &  shamt[1] & ~shamt[0]}}) |
                 ({in[4:0],  {27{1'b0}}} & {32{ shamt[4] &  shamt[3] & ~shamt[2] &  shamt[1] &  shamt[0]}}) |
                 ({in[3:0],  {28{1'b0}}} & {32{ shamt[4] &  shamt[3] &  shamt[2] & ~shamt[1] & ~shamt[0]}}) |
                 ({in[2:0],  {29{1'b0}}} & {32{ shamt[4] &  shamt[3] &  shamt[2] & ~shamt[1] &  shamt[0]}}) |
                 ({in[1:0],  {30{1'b0}}} & {32{ shamt[4] &  shamt[3] &  shamt[2] &  shamt[1] & ~shamt[0]}}) |
                 ({in[0],    {31{1'b0}}} & {32{ shamt[4] &  shamt[3] &  shamt[2] &  shamt[1] &  shamt[0]}}))) |
                 ({32{control[1]}} &
                 ((in                    & {32{~shamt[4] & ~shamt[3] & ~shamt[2] & ~shamt[1] & ~shamt[0]}}) |
                 ({ext, in[31:1]}        & {32{~shamt[4] & ~shamt[3] & ~shamt[2] & ~shamt[1] &  shamt[0]}}) |
                 ({{2{ext}}, in[31:2]}   & {32{~shamt[4] & ~shamt[3] & ~shamt[2] &  shamt[1] & ~shamt[0]}}) |
                 ({{3{ext}}, in[31:3]}   & {32{~shamt[4] & ~shamt[3] & ~shamt[2] &  shamt[1] &  shamt[0]}}) |
                 ({{4{ext}}, in[31:4]}   & {32{~shamt[4] & ~shamt[3] &  shamt[2] & ~shamt[1] & ~shamt[0]}}) |
                 ({{5{ext}}, in[31:5]}   & {32{~shamt[4] & ~shamt[3] &  shamt[2] & ~shamt[1] &  shamt[0]}}) |
                 ({{6{ext}}, in[31:6]}   & {32{~shamt[4] & ~shamt[3] &  shamt[2] &  shamt[1] & ~shamt[0]}}) |
                 ({{7{ext}}, in[31:7]}   & {32{~shamt[4] & ~shamt[3] &  shamt[2] &  shamt[1] &  shamt[0]}}) |
                 ({{8{ext}}, in[31:8]}   & {32{~shamt[4] &  shamt[3] & ~shamt[2] & ~shamt[1] & ~shamt[0]}}) |
                 ({{9{ext}}, in[31:9]}   & {32{~shamt[4] &  shamt[3] & ~shamt[2] & ~shamt[1] &  shamt[0]}}) |
                 ({{10{ext}}, in[31:10]} & {32{~shamt[4] &  shamt[3] & ~shamt[2] &  shamt[1] & ~shamt[0]}}) |
                 ({{11{ext}}, in[31:11]} & {32{~shamt[4] &  shamt[3] & ~shamt[2] &  shamt[1] &  shamt[0]}}) |
                 ({{12{ext}}, in[31:12]} & {32{~shamt[4] &  shamt[3] &  shamt[2] & ~shamt[1] & ~shamt[0]}}) |
                 ({{13{ext}}, in[31:13]} & {32{~shamt[4] &  shamt[3] &  shamt[2] & ~shamt[1] &  shamt[0]}}) |
                 ({{14{ext}}, in[31:14]} & {32{~shamt[4] &  shamt[3] &  shamt[2] &  shamt[1] & ~shamt[0]}}) |
                 ({{15{ext}}, in[31:15]} & {32{~shamt[4] &  shamt[3] &  shamt[2] &  shamt[1] &  shamt[0]}}) |
                 ({{16{ext}}, in[31:16]} & {32{ shamt[4] & ~shamt[3] & ~shamt[2] & ~shamt[1] & ~shamt[0]}}) |
                 ({{17{ext}}, in[31:17]} & {32{ shamt[4] & ~shamt[3] & ~shamt[2] & ~shamt[1] &  shamt[0]}}) |
                 ({{18{ext}}, in[31:18]} & {32{ shamt[4] & ~shamt[3] & ~shamt[2] &  shamt[1] & ~shamt[0]}}) |
                 ({{19{ext}}, in[31:19]} & {32{ shamt[4] & ~shamt[3] & ~shamt[2] &  shamt[1] &  shamt[0]}}) |
                 ({{20{ext}}, in[31:20]} & {32{ shamt[4] & ~shamt[3] &  shamt[2] & ~shamt[1] & ~shamt[0]}}) |
                 ({{21{ext}}, in[31:21]} & {32{ shamt[4] & ~shamt[3] &  shamt[2] & ~shamt[1] &  shamt[0]}}) |
                 ({{22{ext}}, in[31:22]} & {32{ shamt[4] & ~shamt[3] &  shamt[2] &  shamt[1] & ~shamt[0]}}) |
                 ({{23{ext}}, in[31:23]} & {32{ shamt[4] & ~shamt[3] &  shamt[2] &  shamt[1] &  shamt[0]}}) |
                 ({{24{ext}}, in[31:24]} & {32{ shamt[4] &  shamt[3] & ~shamt[2] & ~shamt[1] & ~shamt[0]}}) |
                 ({{25{ext}}, in[31:25]} & {32{ shamt[4] &  shamt[3] & ~shamt[2] & ~shamt[1] &  shamt[0]}}) |
                 ({{26{ext}}, in[31:26]} & {32{ shamt[4] &  shamt[3] & ~shamt[2] &  shamt[1] & ~shamt[0]}}) |
                 ({{27{ext}}, in[31:27]} & {32{ shamt[4] &  shamt[3] & ~shamt[2] &  shamt[1] &  shamt[0]}}) |
                 ({{28{ext}}, in[31:28]} & {32{ shamt[4] &  shamt[3] &  shamt[2] & ~shamt[1] & ~shamt[0]}}) |
                 ({{29{ext}}, in[31:29]} & {32{ shamt[4] &  shamt[3] &  shamt[2] & ~shamt[1] &  shamt[0]}}) |
                 ({{30{ext}}, in[31:30]} & {32{ shamt[4] &  shamt[3] &  shamt[2] &  shamt[1] & ~shamt[0]}}) |
                 ({{31{ext}}, in[31]}    & {32{ shamt[4] &  shamt[3] &  shamt[2] &  shamt[1] &  shamt[0]}})));
endmodule

module alu(A,B,F,Y,overflow,zero);
	input [31:0] A, B;
	input [3:0] F;
	output [31:0] Y;
	output overflow, zero;
	
	wire cout, zero_lgc;
	wire [31:0] S, shift, Bout;
	
	assign Bout = F[2] ? ~B : B;
	assign zero_lgc = ~(|S) & ~F[1] & ~F[0] |
	                   S[31] & ~F[1] & F[0] |
	                   ~overflow & F[1] & ~F[0];
	prefixAdder pa1(A[15:0],Bout[15:0],F[2],S[15:0],cout);
	prefixAdder pa2(A[31:16],Bout[31:16],cout,S[31:16],overflow);
	shifter s1(A, B[4:0], F[1:0], shift);
	
	assign Y = (~{32{F[3]}} &
	           ((~{32{F[1]}} & ~{32{F[0]}} & A & Bout) |
	           (~{32{F[1]}} &  {32{F[0]}} & (A | Bout)) |
	           ( {32{F[1]}} & ~{32{F[0]}} & S) |
	           ( {{31{1'b0}}, F[1] & F[0] & S[31]}))) |
	           ({32{F[3]}} &
	           ((~{32{F[1]}} & ~{32{F[0]}} & (A ^ Bout)) |
	           (~{32{F[1]}} &  {32{F[0]}} & shift) |
	           ( {32{F[1]}} & ~{32{F[0]}} & shift) |
	           (~{32{F[2]}} & {32{F[1]}} &  {32{F[0]}} & shift) |
	           ( {{31{1'b0}}, F[2] & F[1] & F[0] & ~overflow})));
	assign zero = F[3] ? ~zero_lgc : zero_lgc;
endmodule

module aluControl(
    input [1:0] aluOp,
    input [3:0] func,
    output [3:0] aluY
    );
    
    //1XX100 | 001 | 101
    assign aluY[3] = (aluOp[1] & ~func[1] & (func[2] | func[0])) |
                     (aluOp[1] & ~func[2] & func[1] & func[0]) |
                     (aluOp[0] & func[0] & (~func[1] | func[2]));
    //X1XXXX | 1X1000
    assign aluY[2] = ( aluOp[0]) |
                     ( aluOp[1] & ~func[2] & ~func[0] & (func[3] ^ func[1]));
    //00XXXX | X1XXXX | 1XX000
    assign aluY[1] = (~aluOp[1] & ~aluOp[0]) |
                     ( aluOp[0] & func[2] & func[1]) |
                     ( aluOp[1] & ~func[2] & ~func[0] & (~func[1] | ~func[3])) |
                     ( aluOp[1] &  func[2] & ~func[1] & func[0]);
    //1X0110
    assign aluY[0] = (aluOp[1] & ~func[3] & func[1] & ~func[0]) |
                     (aluOp[1] & ~func[1] & func[0] & ~(func[3] ^ func[2])) |
                     (aluOp[0] & func[2] & ~func[1]);
endmodule

module control(
    input [6:0] opcode,
    output [1:0] aluOp,
    output aluBSrc,
    output branch,
    output mem_ren,
    output mem_wen,
    output reg_wen,
    output mem_to_reg,
    output imm_load,
    output aluASrc,
    output jump,
    output jumpr
    );
    
    // R-format: 0110011
    // I-format: 0010011
    // ld: 0000011
    // sd: 0100011
    // beq: 1100011
    
    assign aluOp = {~opcode[6] & opcode[4] & ~opcode[3] & ~opcode[2] & opcode[1] & opcode[0], 
                     opcode[6] & opcode[5] & ~opcode[4] & ~opcode[3] & ~opcode[2] & opcode[1] & opcode[0]};
    assign aluBSrc = ~opcode[6] & ~opcode[3] & ~opcode[2] & opcode[1] & opcode[0] & (~opcode[5] | ~opcode[4]);
    assign aluASrc = ~opcode[6] & ~opcode[5] & opcode[4] & ~opcode[3] & opcode[2] & opcode[1] & opcode[0] |
                     opcode[6] & opcode[5] & ~opcode[4] & opcode[2] & opcode[1] & opcode[0];
    assign branch = opcode[6] & opcode[5] & ~opcode[4] & ~opcode[3] & ~opcode[2] & opcode[1] & opcode[0];
    assign jump = opcode[6] & opcode[5] & ~opcode[4] & opcode[2] & opcode[1] & opcode[0];
    assign jumpr = opcode[6] & opcode[5] & ~opcode[4] & ~opcode[3] & opcode[2] & opcode[1] & opcode[0];
    assign mem_ren = ~opcode[6] & ~opcode[5] & ~opcode[4] & ~opcode[3] & ~opcode[2] & opcode[1] & opcode[0];
    assign mem_wen = ~opcode[6] & opcode[5] & ~opcode[4] & ~opcode[3] & ~opcode[2] & opcode[1] & opcode[0];
    assign reg_wen = ~opcode[6] & ~opcode[3] & ~opcode[2] & opcode[1] & opcode[0] & (~opcode[5] | opcode[4]) |
                     ~opcode[6] & opcode[5] & opcode[4] & ~opcode[3] & opcode[2] & opcode[1] & opcode[0] |
                     opcode[6] & opcode[5] & ~opcode[4] & opcode[2] & opcode[1] & opcode[0];
    assign mem_to_reg = ~opcode[6] & ~opcode[5] & ~opcode[4] & ~opcode[3] & ~opcode[2] & opcode[1] & opcode[0];
    assign imm_load = ~opcode[6] & opcode[5] & opcode[4] & ~opcode[3] & opcode[2] & opcode[1] & opcode[0];
endmodule

module registers(
    input clk,
    input rst_n,
    input [4:0] RR1_addr,
    input [4:0] RR2_addr,
    input [4:0] WR_addr,
    input [31:0] WR_data,
    input wen,
    output [31:0] RR1_data,
    output [31:0] RR2_data
    );
    
    reg [31:0] regfile [31:0];
    reg [31:0] RR1, RR2;
    
    always @(posedge clk, negedge rst_n) begin
        if (~rst_n) begin
            regfile[0] <= 0;
            regfile[1] <= 0;
            regfile[2] <= 0;
            regfile[3] <= 0;
            regfile[4] <= 0;
            regfile[5] <= 0;
            regfile[6] <= 0;
            regfile[7] <= 0;
            regfile[8] <= 0;
            regfile[9] <= 0;
            regfile[10] <= 0;
            regfile[11] <= 0;
            regfile[12] <= 0;
            regfile[13] <= 0;
            regfile[14] <= 0;
            regfile[15] <= 0;
            regfile[16] <= 0;
            regfile[17] <= 0;
            regfile[18] <= 0;
            regfile[19] <= 0;
            regfile[20] <= 0;
            regfile[21] <= 0;
            regfile[22] <= 0;
            regfile[23] <= 0;
            regfile[24] <= 0;
            regfile[25] <= 0;
            regfile[26] <= 0;
            regfile[27] <= 0;
            regfile[28] <= 0;
            regfile[29] <= 0;
            regfile[30] <= 0;
            regfile[31] <= 0;
        end
        else if (|WR_addr & wen) begin
            regfile[WR_addr] <= WR_data;
        end
    end
    
    assign RR1_data = regfile[RR1_addr];
    assign RR2_data = regfile[RR2_addr];
endmodule

module immGen(
    input [31:0] IF_insn,
    output [31:0] ID_imm
    );
    
    wire [31:0] i_type, s_type, sb_type, u_type, uj_type;
    
    assign i_type = {32{~IF_insn[6] & ~IF_insn[5] & ~IF_insn[3] & ~IF_insn[2] & IF_insn[1] & IF_insn[0] |
                    IF_insn[6] & IF_insn[5] & ~IF_insn[4] & ~IF_insn[3] & IF_insn[2] & IF_insn[1] & IF_insn[0]}};
    assign s_type = {32{~IF_insn[6] & IF_insn[5] & ~IF_insn[4] & ~IF_insn[3] & ~IF_insn[2] & IF_insn[1] & IF_insn[0]}};
    assign sb_type = {32{IF_insn[6] & IF_insn[5] & ~IF_insn[4] & ~IF_insn[3] & ~IF_insn[2] & IF_insn[1] & IF_insn[0]}};
    assign u_type = {32{~IF_insn[6] & IF_insn[4] & ~IF_insn[3] & IF_insn[2] & IF_insn[1] & IF_insn[0]}};
    assign uj_type = {32{IF_insn[6] & IF_insn[5] & ~IF_insn[4] & IF_insn[3] & IF_insn[2] & IF_insn[1] & IF_insn[0]}};
    
    assign ID_imm = (i_type &
                    (((({32{IF_insn[14]}} & {32{IF_insn[13]}}) | ~{32{IF_insn[12]}}) & {{20{IF_insn[31]}}, IF_insn[31:20]}) |
                    (~{32{IF_insn[14]}} & {32{IF_insn[13]}} & {32{IF_insn[12]}} & {{20{1'b0}}, IF_insn[31:20]}) |
                    (~{32{IF_insn[13]}} &  {32{IF_insn[12]}} & {{27{1'b0}}, IF_insn[24:20]}))) |
                    (s_type & 
                    ({{20{1'b0}}, IF_insn[31:25], IF_insn[11:7]})) |
                    (u_type &
                    {IF_insn[31:12], 12'b0}) |
                    (uj_type &
                    {{12{IF_insn[31]}}, IF_insn[19:12], IF_insn[20], IF_insn[30:21], 1'b0}) |
                    (sb_type &
                    {{20{IF_insn[31]}}, IF_insn[7], IF_insn[30:25], IF_insn[11:8], 1'b0});
                    
endmodule

module forwardingUnit(
    input [4:0] ID_RR1_addr_reg,
    input [4:0] ID_RR2_addr_reg,
    input [4:0] ID_RR1_addr,
    input [4:0] ID_RR2_addr,
    input [4:0] EX_WR_addr,
    input EX_reg_wen,
    input MEM_reg_wen,
    input [4:0] MEM_WR_addr,
    output [1:0] ForwardA,
    output [1:0] ForwardB,
    output regfileFwdA,
    output regfileFwdB
    );
    
    wire fwdAlgc10, fwdAlgc01, fwdBlgc10, fwdBlgc01;
    
    assign fwdAlgc10 = EX_reg_wen & |EX_WR_addr & ~(|(EX_WR_addr ^ ID_RR1_addr_reg));
    assign fwdAlgc01 = MEM_reg_wen & |MEM_WR_addr & ~((EX_reg_wen & |EX_WR_addr) & ~(|(EX_WR_addr ^ ID_RR1_addr_reg))) & ~(|(MEM_WR_addr ^ ID_RR1_addr_reg));
    assign fwdBlgc10 = EX_reg_wen & |EX_WR_addr & ~(|(EX_WR_addr ^ ID_RR2_addr_reg));
    assign fwdBlgc01 = MEM_reg_wen & |MEM_WR_addr & ~((EX_reg_wen & |EX_WR_addr) & ~(|(EX_WR_addr ^ ID_RR2_addr_reg))) & ~(|(MEM_WR_addr ^ ID_RR2_addr_reg));
    
    assign ForwardA = (2'b10 & {2{fwdAlgc10}}) | (2'b01 & {2{fwdAlgc01}});
    assign ForwardB = (2'b10 & {2{fwdBlgc10}}) | (2'b01 & {2{fwdBlgc01}});
    assign regfileFwdA = ~(|(MEM_WR_addr ^ ID_RR1_addr)) & MEM_reg_wen;
    assign regfileFwdB = ~(|(MEM_WR_addr ^ ID_RR2_addr)) & MEM_reg_wen;
endmodule

module hazardDetection(
    input ID_mem_ren,
    input [4:0] ID_WR_addr,
    input [4:0] ID_RR1_addr,
    input [4:0] ID_RR2_addr,
    input ID_branch,
    input EX_branch,
    input ID_jump,
    input EX_jump,
    input PC_src,
    input MEM_PC_src,
    output ID_ctrl_sel,
    output PC_wen,
    output IF_wen
    );
    
    wire if_result;
    
    assign if_result = ID_mem_ren & (~(|(ID_WR_addr ^ ID_RR1_addr)) | ~(|(ID_WR_addr ^ ID_RR2_addr))) | ID_branch | ID_jump;
    
    assign ID_ctrl_sel = if_result | MEM_PC_src | EX_branch | EX_jump;
    assign PC_wen = ~(if_result | (EX_branch | EX_jump) & ~PC_src);
    assign IF_wen = ~(if_result | EX_branch | EX_jump);
endmodule

module ldstGen(
    input [31:0] r_data,
    input [31:0] dmem_data,
    input [2:0] func,
    input [1:0] offset,
    output [31:0] load,
    output [31:0] store,
    output [3:0] byte_en
    );
    
    assign load = {32{func[1]}} & dmem_data |
                  {32{~func[1] & func[0]}} & 
                      ({32{~offset[1] & ~offset[0]}} & {{16{~func[2] & dmem_data[15]}}, dmem_data[15:0]} |
                      {32{~offset[1] & offset[0]}} & {{16{~func[2] & dmem_data[23]}}, dmem_data[23:8]} |
                      {32{offset[1] & ~offset[0]}} & {{16{~func[2] & dmem_data[31]}}, dmem_data[31:16]}) |
                  {32{~func[1] & ~func[0]}} & 
                      ({32{~offset[1] & ~offset[0]}} & {{24{~func[2] & dmem_data[7]}}, dmem_data[7:0]} |
                      {32{~offset[1] & offset[0]}} & {{24{~func[2] & dmem_data[15]}}, dmem_data[15:8]} |
                      {32{offset[1] & ~offset[0]}} & {{24{~func[2] & dmem_data[23]}}, dmem_data[23:16]} |
                      {32{offset[1] & offset[0]}} & {{24{~func[2] & dmem_data[31]}}, dmem_data[31:24]});
                      
    assign store = {32{func[1]}} & r_data |
                  {32{~func[1] & func[0]}} & 
                      ({32{~offset[1] & ~offset[0]}} & {{16{1'b0}}, r_data[15:0]} |
                      {32{~offset[1] & offset[0]}} & {{8{1'b0}}, r_data[15:0], {8{1'b0}}} |
                      {32{offset[1] & ~offset[0]}} & {r_data[15:0], {16{1'b0}}}) |
                  {32{~func[1] & ~func[0]}} & 
                      ({32{~offset[1] & ~offset[0]}} & {{24{1'b0}}, r_data[7:0]} |
                      {32{~offset[1] & offset[0]}} & {{16{1'b0}}, r_data[7:0], {8{1'b0}}} |
                      {32{offset[1] & ~offset[0]}} & {{8{1'b0}}, r_data[7:0], {16{1'b0}}} |
                      {32{offset[1] & offset[0]}} & {r_data[7:0], {24{1'b0}}});
                      
    assign byte_en = {4{func[1]}} & 4'b1111 |
                     {4{~func[1] & func[0]}} &
                         ({4{~offset[1] & ~offset[0]}} & 4'b0011 |
                         {4{~offset[1] & offset[0]}} & 4'b0110 |
                         {4{offset[1] & ~offset[0]}} & 4'b1100) |
                     {4{~func[1] & ~func[0]}} & 
                         ({4{~offset[1] & ~offset[0]}} & 4'b0001 |
                         {4{~offset[1] & offset[0]}} & 4'b0010 |
                         {4{offset[1] & ~offset[0]}} & 4'b0100 |
                         {4{offset[1] & offset[0]}} & 4'b1000);
endmodule

module mux31_32bit(
    input [31:0] A,
    input [31:0] B,
    input [31:0] C,
    input [1:0] sel,
    output [31:0] out
    );
    
    assign out = (A & ~{32{sel[1]}} & ~{32{sel[0]}}) |
                 (B & ~{32{sel[1]}} &  {32{sel[0]}}) |
                 (C &  {32{sel[1]}} & ~{32{sel[0]}});
endmodule

module mux21_32bit(
    input [31:0] A,
    input [31:0] B,
    input sel,
    output [31:0] out
    );
   
    assign out = (A & ~{32{sel}}) | (B & {32{sel}});
endmodule

module cpu(
    input rst_n,
    input clk,
    output [31:0] imem_addr,
    input [31:0] imem_insn,
    output [31:0] dmem_addr,
    inout [31:0] dmem_data,
    output dmem_wen,
    output [3:0] byte_en
    );
    
    integer reg_print = 0;
    integer PC_print = 0;
    
    initial begin
        reg_print = $fopen("regoutput.txt", "w");
        PC_print = $fopen("pcoutput.txt", "w");
        #1000
        $fclose(reg_print);
        $fclose(PC_print);
    end
    
    reg [31:0] PC;
    wire PC_wen;
    wire [31:0] PC_inc;
    wire PC_carry;
    wire PC_overflow;
    wire PC_src;
    wire [31:0] PC_next;
    wire [31:0] PC_branch;
    wire PC_branch_carry;
    wire PC_oor;
    
    wire [3:0] aluY;
    
    reg [31:0] IF_insn;
    wire IF_wen;
    reg [31:0] IF_PC;
    
    wire [1:0] aluOp;
    wire aluSrc, branch, mem_ren, mem_wen, reg_wen, mem_to_reg, imm_load, aluASrc, jump, jumpr;
    reg [1:0] ID_aluOp;
    reg ID_aluSrc, ID_branch, ID_mem_ren, ID_mem_wen, ID_reg_wen, ID_mem_to_reg, ID_imm_load, ID_aluASrc, ID_jump, ID_jumpr;
    wire ID_ctrl_sel;
    wire [4:0] ID_RR1_addr;
    wire [4:0] ID_RR2_addr;
    reg [4:0] ID_RR1_addr_reg;
    reg [4:0] ID_RR2_addr_reg;
    wire [31:0] ID_RR1_data;
    wire [31:0] ID_RR2_data;
    reg [31:0] ID_RR1_data_reg;
    reg [31:0] ID_RR2_data_reg;
    wire [31:0] ID_FwdA, ID_FwdB;
    reg [4:0] ID_WR_addr;
    wire [31:0] ID_imm;
    reg [31:0] ID_imm_reg;
    reg [3:0] ID_aluFunc;
    reg [31:0] ID_PC;
    reg [2:0] ID_memFunc;
    
    wire [1:0] ForwardA, ForwardB;
    wire regfileFwdA, regfileFwdB;
    wire [31:0] aluAfwd, aluA, aluBfwd, aluBimm, aluB;
    reg EX_branch, EX_mem_ren, EX_mem_wen, EX_reg_wen, EX_mem_to_reg, EX_jump;
    reg [4:0] EX_WR_addr;
    wire [31:0] EX_WR_data;
    reg [31:0] EX_WR_data_reg;
    wire EX_zero;
    reg EX_zero_reg;
    wire EX_overflow;
    reg [31:0] EX_RR2_data;
    reg [31:0] EX_PC_branch;
    reg [2:0] EX_memFunc;
    wire [31:0] EX_imm;
    wire [1:0] EX_ldst_offset;
    reg [1:0] EX_ldst_offset_reg;
    wire [31:0] EX_result;
    wire [31:0] EX_branchA;
    
    reg MEM_reg_wen, MEM_mem_to_reg;
    reg [4:0] MEM_WR_addr;
    reg [31:0] MEM_WR_data_reg;
    wire [31:0] MEM_WR_data;
    reg [31:0] MEM_data_reg;
    wire [31:0] MEM_store_data;
    wire [31:0] MEM_load_data;
    reg MEM_PC_src;
    
    //IF: Instruction fetch from memory
    prefixAdder pcpa1(PC[15:0], 4, 1'b0, PC_inc[15:0], PC_carry);
    prefixAdder pcpa2(PC[31:16], 0, PC_carry, PC_inc[31:16], PC_overflow);
    always @(posedge clk, negedge rst_n) begin
        if (~rst_n) begin
            PC <= 0;
            IF_insn <= 0;
            IF_PC <= 0;
        end
        else begin
            if (PC_wen) PC <= PC_next;
            $fwrite(PC_print, "%x\n", PC);
            if (IF_wen) begin
                IF_insn <= imem_insn;
                IF_PC <= PC;
            end
        end
    end
    assign PC_next = {32{~PC_src}} & PC_inc | {32{PC_src}} & EX_PC_branch;
    assign imem_addr = PC;
    
    //ID: Instruction decode & register read
    hazardDetection hd1(ID_mem_ren, ID_WR_addr, ID_RR1_addr, ID_RR2_addr, ID_branch, EX_branch, ID_jump, EX_jump, PC_src, MEM_PC_src, ID_ctrl_sel, PC_wen, IF_wen);
    control c1(IF_insn[6:0], aluOp, aluSrc, branch, mem_ren, mem_wen, reg_wen, mem_to_reg, imm_load, aluASrc, jump, jumpr);
    immGen ig1(IF_insn, ID_imm);
    mux21_32bit regfileFwdMuxA1(ID_RR1_data, MEM_WR_data, regfileFwdA, ID_FwdA);
    mux21_32bit regfileFwdMuxB1(ID_RR2_data, MEM_WR_data, regfileFwdB, ID_FwdB);
    registers regfile1(clk, rst_n, ID_RR1_addr, ID_RR2_addr, MEM_WR_addr, MEM_WR_data, MEM_reg_wen, ID_RR1_data, ID_RR2_data);
    always @(posedge clk, negedge rst_n) begin
        if (~rst_n) begin
            ID_RR1_data_reg <= 0;
            ID_RR2_data_reg <= 0;
            ID_RR1_addr_reg <= 0;
            ID_RR2_addr_reg <= 0;
            ID_WR_addr <= 0;
            ID_imm_reg <= 0;
            ID_aluOp <= 0;
            ID_aluFunc <= 0;
            ID_memFunc <= 0;
            ID_reg_wen <= 0;
            ID_mem_wen <= 0;
            ID_aluSrc <= 0;
            ID_branch <= 0;
            ID_mem_ren <= 0;
            ID_mem_to_reg <= 0;
            ID_PC <= 0;
            ID_imm_load <= 0;
            ID_aluASrc <= 0;
            ID_jump <= 0;
            ID_jumpr <= 0;
        end
        else begin
            ID_RR1_data_reg <= ID_RR1_data;
            ID_RR2_data_reg <= ID_RR2_data;
            ID_RR1_addr_reg <= ID_RR1_addr;
            ID_RR2_addr_reg <= ID_RR2_addr;
            ID_WR_addr <= IF_insn[11:7];
            ID_imm_reg <= ID_imm;
            ID_aluOp <= aluOp;
            ID_aluFunc <= {IF_insn[30] & (~aluSrc | (aluSrc & IF_insn[12] & ((~IF_insn[14] & ~IF_insn[13]) | (IF_insn[14] & ~IF_insn[13])))), IF_insn[14], IF_insn[13], IF_insn[12]};
            ID_memFunc <= IF_insn[14:12];
            ID_reg_wen <= reg_wen & ~ID_ctrl_sel;
            ID_mem_wen <= mem_wen & ~ID_ctrl_sel;
            ID_aluSrc <= aluSrc;
            ID_branch <= branch;
            ID_mem_ren <= mem_ren;
            ID_mem_to_reg <= mem_to_reg;
            ID_RR1_data_reg <= ID_FwdA;
            ID_RR2_data_reg <= ID_FwdB;
            ID_PC <= IF_PC;
            ID_imm_load <= imm_load;
            ID_aluASrc <= aluASrc;
            ID_jump <= jump;
            ID_jumpr <= jumpr;
        end
    end
    assign ID_RR1_addr = IF_insn[19:15];
    assign ID_RR2_addr = IF_insn[24:20];
    
    //EX: Execute operation or calculate address
    forwardingUnit fw1(ID_RR1_addr_reg, ID_RR2_addr_reg, ID_RR1_addr, ID_RR2_addr, EX_WR_addr, EX_reg_wen, 
        MEM_reg_wen, MEM_WR_addr, ForwardA, ForwardB, regfileFwdA, regfileFwdB);
    mux31_32bit aluAmux1(ID_RR1_data_reg, MEM_WR_data, EX_WR_data_reg, ForwardA, aluAfwd);
    mux31_32bit aluBmux1(ID_RR2_data_reg, MEM_WR_data, EX_WR_data_reg, ForwardB, aluBfwd);
    mux21_32bit aluApc1(aluAfwd, ID_PC, ID_aluASrc, aluA);
    mux21_32bit aluBimm1(aluBfwd, EX_imm, ID_aluSrc, aluBimm);
    mux21_32bit aluBjump1(aluBimm, 4, ID_jump, aluB);
    aluControl ac1(ID_aluOp, ID_aluFunc, aluY);
    alu alumodule1(aluA, aluB, aluY, EX_WR_data, EX_overflow, EX_zero);
    prefixAdder branchAdder1(EX_branchA[15:0], ID_imm_reg[15:0], 1'b0, PC_branch[15:0], PC_branch_carry);
    prefixAdder branchAdder2(EX_branchA[31:16], ID_imm_reg[31:16], PC_branch_carry, PC_branch[31:16], PC_oor);
    always @(posedge clk, negedge rst_n) begin
        if (~rst_n) begin
            EX_WR_addr <= 0;
            EX_WR_data_reg <= 0;
            EX_zero_reg <= 0;
            EX_reg_wen <= 0;
            EX_mem_wen <= 0;
            EX_branch <= 0;
            EX_mem_ren <= 0;
            EX_mem_to_reg <= 0;
            EX_RR2_data <= 0;
            EX_PC_branch <= 0;
            EX_memFunc <= 0;
            EX_ldst_offset_reg <= 0;
            EX_jump <= 0;
        end
        else begin
            EX_WR_addr <= ID_WR_addr;
            EX_WR_data_reg <= EX_result;
            EX_zero_reg <= EX_zero;
            EX_reg_wen <= ID_reg_wen;
            EX_mem_wen <= ID_mem_wen;
            EX_branch <= ID_branch;
            EX_mem_ren <= ID_mem_ren;
            EX_mem_to_reg <= ID_mem_to_reg;
            EX_RR2_data <= aluBfwd;
            EX_PC_branch <= PC_branch;
            EX_memFunc <= ID_memFunc;
            EX_ldst_offset_reg <= EX_ldst_offset;
            EX_jump <= ID_jump;
        end
    end
    assign EX_imm = ID_imm_reg & {32{~(ID_mem_wen | ID_mem_ren)}} | {ID_imm_reg[31:2], 2'b00} & {32{(ID_mem_wen | ID_mem_ren)}};
    assign EX_ldst_offset = ID_imm_reg[1:0];
    assign EX_result = ID_imm_load ? ID_imm_reg : EX_WR_data;
    assign EX_branchA = ID_jumpr ? aluAfwd : ID_PC;
    
    //MEM: Access memory operand
    ldstGen ls1(EX_RR2_data, dmem_data, EX_memFunc, EX_ldst_offset_reg, MEM_load_data, MEM_store_data, byte_en);
    always @(posedge clk, negedge rst_n) begin
        if (~rst_n) begin
            MEM_WR_addr <= 0;
            MEM_WR_data_reg <= 0;
            MEM_reg_wen <= 0;
            MEM_mem_to_reg <= 0;
            MEM_data_reg <= 0;
            MEM_PC_src <= 0;
        end
        else begin
            MEM_WR_addr <= EX_WR_addr;
            MEM_WR_data_reg <= EX_WR_data_reg;
            MEM_reg_wen <= EX_reg_wen;
            MEM_mem_to_reg <= EX_mem_to_reg;
            MEM_data_reg <= MEM_load_data;
            MEM_PC_src <= PC_src;
        end
    end
    assign PC_src = EX_branch & EX_zero_reg | EX_jump;
    assign dmem_addr = EX_WR_data_reg;
    assign dmem_data = EX_mem_wen ? MEM_store_data : 32'bZ;
    assign dmem_wen = EX_mem_wen;
    
    //WB: Write result back to register
    mux21_32bit wbmux1(MEM_WR_data_reg, MEM_data_reg, MEM_mem_to_reg, MEM_WR_data);
    always @(posedge clk, negedge rst_n) begin
        if (~rst_n) begin
        
        end
        else begin
            if (MEM_reg_wen & |MEM_WR_addr)
                $fwrite(reg_print, "%d\t%x\n", MEM_WR_addr, MEM_WR_data);
        end   
    end
endmodule