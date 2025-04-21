%This script takes as input a vector of 1s and 0s (numerical)
%and returns a string based on the ASCII representation of characters.

%The input vector binary needs to have length that is a multiple of 8

function res = BitsToStrings(binary)
string = '';
sz_binary = length(binary);
binary_chars = reshape(binary, sz_binary/8, 8);
binary_chars = binary_chars*2.^([7:-1:0]');
string = char(binary_chars); %turns it back into ascii
res = string';
end
        
    