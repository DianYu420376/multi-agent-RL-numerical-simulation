function C = scalar_cell_mult(a, B)
N = length(B);
C = cell(1,N);
for i = 1:N
    C{i} = a*B{i};
end
end