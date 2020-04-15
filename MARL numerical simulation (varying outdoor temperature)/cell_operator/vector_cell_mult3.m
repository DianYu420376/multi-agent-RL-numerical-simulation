function C = vector_cell_mult3(v, B)
N = length(B)/3;
assert(length(v) == N)
C = cell(1,3*N);
for i = 1:N
    C{i} = v(i)*B{i};
    C{N+i} = v(i)*B{N+i};
    C{2*N+i} = v(i)*B{2*N+i};
end
end