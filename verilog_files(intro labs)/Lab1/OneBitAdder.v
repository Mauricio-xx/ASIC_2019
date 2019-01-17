module onebitadder (
	input a, b, cin,
	output sum, cout
	);
assign sum = !a*!b*cin+!a*b*!cin;
assign cout = b*cin+a*b+a*cin;
endmodule

