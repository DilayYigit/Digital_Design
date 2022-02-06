`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/24/2020 02:55:48 PM
// Design Name: 
// Module Name: clock
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module clock(input logic clk, output logic slowClock);
 logic [28:0]clkCntr;
 always @(posedge clk)
 begin
 if(clkCntr >= 199999999)
 begin
 clkCntr <= 0;
 slowClock <= 0;
 end
 else
 begin
 clkCntr <= clkCntr + 1;
 slowClock <= 1;
 end
 end
endmodule

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/24/2020 02:56:25 PM
// Design Name: 
// Module Name: fsm
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module fsm(input logic clk, SA, SB, reset,output logic [2:0] LA,[2:0] LB);
 typedef enum logic [2:0] {S0, S1, S2, S3,S4,S5,S6,S7} statetype;
 statetype [2:0] state, nextstate;
 parameter red = 3'b001;
 parameter yellow = 3'b011;
 parameter green = 3'b111;
 always_ff @(posedge clk, posedge reset)
 if(reset)
 state <= S0;
 else
 state <= nextstate;
 always_comb
 case (state)
 S0:
 if(SA)
 nextstate = S0;
 else
 nextstate = S1;
 S1:
 nextstate = S2;
 S2:
 nextstate = S3;
 S3:
 nextstate = S4;
 S4:
 if(SB)
 nextstate = S4;
 else
 nextstate = S5;
 S5:
 nextstate = S6;
 S6:
 nextstate = S7;
 S7:
 nextstate = S0;
 endcase
 always_comb
 case(state)
 S0:
 begin
 LA = green;
 LB = red;
 end
 S1:
 begin
 LA = yellow;
 LB = red;
 end
 S2:
 begin
 LA = red;
 LB = red;
 end
 S3:
 begin
 LA = red;
 LB = yellow;
 end
 S4:
 begin
 LA = red;
 LB = green;
 end
 S5:
 begin
 LA = red;
 LB = yellow;
 end
 S6:
 begin
 LA = red;
 LB = red;
 end
 S7:
 begin
 LA = yellow;
 LB = red;
 end
 endcase
 endmodule 

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/24/2020 02:57:23 PM
// Design Name: 
// Module Name: sim
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module sim(
    );
    logic clk, reset, SA, SB;
     logic [2:0] LA,LB;
     fsm testbench(clk,SA,SB,reset,LA,LB);
     initial
     begin
     reset = 1; #100
     reset = 0;
     SA=0;SB=0; #100;
     SA=0;SB=1; #100;
     SA=1;SB=0; #100;
     SA=1;SB=1; #100;
    
     reset = 1;
     SA=0;SB=0; #50;
     SA=0;SB=1; #50;
     SA=1;SB=0; #50;
     SA=1;SB=1; #50;
     end
     always
     begin
     clk <= 1; #5;
     clk <= 0; #5;
     end 
endmodule




