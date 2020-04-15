function C = vector_cell_mult(v, B)
N = length(B);
assert(length(v) == N)
C = cell(1,N);
for i = 1:N
    C{i} = v(i)*B{i};
end
end