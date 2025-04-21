%This function takes a string input, converts it into binary and 
%returns a vector of numbers (1s and 0s) for the binary representation 
%of the string. Note that each character is 8 bits, so the length of the
%returned vector = 8 x the number of characters in the string (including
%space and punctuation.
function res = StringToBits(string)
res=dec2bin(string, 8);
sz = size(res);
res = reshape(res, sz(1)*sz(2),1);
res = double(str2num(res));
end