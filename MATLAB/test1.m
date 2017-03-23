A = [1 1 1 1; 1 1 1 1; 1 1 1 1; 1 1 1 1];

for i = 1:4
   if i > 2
      A(2:3,i) = NaN; 
   end
end