function C = cell_minus(A,B)
N = length(A);
assert(length(B) == N)
C = cell(1,N);
for i = 1:N
    Ai = A{i};
    Bi = B{i};
    assert(norm(size(Ai) - size(Bi)) == 0)
    C{i} = Ai - Bi;
end
end