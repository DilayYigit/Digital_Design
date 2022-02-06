module full_adder (x1,x2,Cin,S,Cout);
input x1,x2,Cin;
output S, Cout;
wire d1, d2, d3;
xor  firstXor (d1, x1, x2);
xor secondXor (S, d1, Cin);
and firstAnd (d2, x1, x2);
and secondAnd(d3, d1, Cin);
or orGate (Cout, d2, d3);
endmodule 

module twoBit_adder(input x1, input x2,input x3,input x4, input Cin, output S0, output S1,output Cout);
  wire c1;
  
  full_adder firstAdder (x1, x2, cin, S0, c1);
  full_adder secondAdder (x3, x4, c1, S1, Cout);
    
endmodule

module full_adder (x1,x2,Cin,S,Cout);
input x1,x2,Cin;
output S, Cout;
wire d1, d2, d3;
xor  firstXor (d1, x1, x2);
xor secondXor (S, d1, Cin);
and firstAnd (d2, x1, x2);
and secondAnd(d3, d1, Cin);
or orGate (Cout, d2, d3);
endmodule 

module twoBit_adder(input x1, input x2,input x3,input x4, input Cin, output S0, output S1,output Cout);
  wire c1;
  
  full_adder firstAdder (x1, x2, cin, S0, c1);
  full_adder secondAdder (x3, x4, c1, S1, Cout);
    
endmodule


module full_adder_struc(
input x1, x2, Cin,
output S, Cout
    );
    wire d1, d2, d3;

endmodule
 
module full_substractor(
    input x1, x2, Bin,
    output D, Bout
    );
    assign D = x1 ^ x2 ^ Bin;
    assign Bout = (~x1&x2)|~(x1^x2) & Bin;
endmodule

module sim(
    );
    logic x1; logic x2; logic Cin;
    logic S; logic Cout;
    full_adder testbench1 (x1, x2, Cin, S, Cout);
    initial begin 
    #100;
    #10 x1=0; x2=0; Cin=0;
    #10 x1=0; x2=0; Cin=1;
    #10 x1=0; x2=1; Cin=0;
    #10 x1=0; x2=1; Cin=1;
    #10 x1=1; x2=0; Cin=0;
    #10 x1=1; x2=0; Cin=1;
    #10 x1=1; x2=1; Cin=0;
    #10 x1=1; x2=1; Cin=1;
    end
     initial $monitor($time,"x1=%d,x2=%d,Cin=%d, S=%d, Cout=%d", x1,x2,Cin,S,Cout);
endmodule


module sim2(
    );
    logic x1; logic x2; logic Bin;
    logic D; logic Bout;
    full_substractor testbench2 (x1, x2, Bin, D, Bout);
    initial begin 
    #100;
    #10 x1=0; x2=0; Bin=0;
    #10 x1=0; x2=0; Bin=1;
    #10 x1=0; x2=1; Bin=0;
    #10 x1=0; x2=1; Bin=1;
    #10 x1=1; x2=0; Bin=0;
    #10 x1=1; x2=0; Bin=1;
    #10 x1=1; x2=1; Bin=0;
    #10 x1=1; x2=1; Bin=1;
    end
     initial $monitor($time,,"x1=%d,x2=%d,Bin=%d, D=%d, Bout=%d", x1,x2,Bin,D,Bout);
endmodule

module sim3(
    );
    logic x1, x2, x3, x4, Cin;
    logic S0, S1, Cout;
    twoBit_adder testbench3 (x1,x2,x3,x4,Cin,S0,S1,Cout);
    initial begin
      #100;
      #10 x1=0;  x2=0; x3=0;  x4=0; Cin=0;
      #10 x1=0;  x2=0; x3=0;  x4=1; Cin=0;
      #10 x1=0;  x2=0; x3=1;  x4=0; Cin=0;
      #10 x1=0;  x2=0; x3=1;  x4=1; Cin=0;
      #10 x1=0;  x2=1; x3=0;  x4=0; Cin=0;
      #10 x1=0;  x2=1; x3=0;  x4=1; Cin=0;
      #10 x1=0;  x2=1; x3=1;  x4=0; Cin=0;
      #10 x1=0;  x2=1; x3=1;  x4=1; Cin=0;
      #10 x1=1;  x2=1; x3=0;  x4=0; Cin=0;
      #10 x1=1;  x2=0; x3=0;  x4=1; Cin=0;
      #10 x1=1;  x2=0; x3=1;  x4=0; Cin=0;
      #10 x1=1;  x2=0; x3=1;  x4=1; Cin=0;
      #10 x1=1;  x2=1; x3=0;  x4=0; Cin=0;
      #10 x1=1;  x2=1; x3=0;  x4=1; Cin=0;
      #10 x1=1;  x2=1; x3=1;  x4=0; Cin=0;
      #10 x1=1;  x2=1; x3=1;  x4=1; Cin=0;
      #10 x1=0;  x2=0; x3=0;  x4=0; Cin=1;
      #10 x1=0;  x2=0; x3=0;  x4=1; Cin=1;
      #10 x1=0;  x2=0; x3=1;  x4=0; Cin=1;
      #10 x1=0;  x2=0; x3=1;  x4=1; Cin=1;
      #10 x1=0;  x2=1; x3=0;  x4=0; Cin=1;
      #10 x1=0;  x2=1; x3=0;  x4=1; Cin=1;
      #10 x1=0;  x2=1; x3=1;  x4=0; Cin=1;
      #10 x1=0;  x2=1; x3=1;  x4=1; Cin=1;
      #10 x1=1;  x2=1; x3=0;  x4=0; Cin=1;
      #10 x1=1;  x2=0; x3=0;  x4=1; Cin=1;
      #10 x1=1;  x2=0; x3=1;  x4=0; Cin=1;
      #10 x1=1;  x2=0; x3=1;  x4=1; Cin=1;
      #10 x1=1;  x2=1; x3=0;  x4=0; Cin=1;
      #10 x1=1;  x2=1; x3=0;  x4=1; Cin=1;
      #10 x1=1;  x2=1; x3=1;  x4=0; Cin=1;
      #10 x1=1;  x2=1; x3=1;  x4=1; Cin=1;
    end
endmodule



