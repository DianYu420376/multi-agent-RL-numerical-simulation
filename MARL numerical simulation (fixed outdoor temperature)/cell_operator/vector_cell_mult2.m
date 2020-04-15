function C = vector_cell_mult2(v, B)
N = length(B)-1;
assert(length(v) == N)
C = cell(1,N+1);
for i = 1:N
    C{i} = v(i)*B{i};
end
C{N+1} = v.*B{N+1};
end