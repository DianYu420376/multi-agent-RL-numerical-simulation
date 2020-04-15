function num = numel_cell(K)
l = length(K);
num = 0;
for i = 1:l
    num = numel(K{i}) + num;
end
end