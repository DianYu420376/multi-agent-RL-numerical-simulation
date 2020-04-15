function g = zero_cell(K)
N = length(K);
g = cell(1,N);
for i = 1:N
    g{i} = zeros(size(K{i}));
end
end