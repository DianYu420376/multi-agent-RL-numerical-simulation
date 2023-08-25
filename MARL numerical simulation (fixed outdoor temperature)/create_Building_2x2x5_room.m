A = zeros(20);
%%
for i = 1:4
    A(i,i+1) = 1; A(i+1, i) = 1;
    A(i,i) = A(i,i) - 1; A(i+1, i+1) = A(i+1,i+1)-1;
end
for i = 6:9
    A(i,i+1) = 1; A(i+1, i) = 1;
    A(i,i) = A(i,i) - 1; A(i+1, i+1) = A(i+1,i+1)-1;
end
for i = 11:14
    A(i,i+1) = 1; A(i+1, i) = 1;
    A(i,i) = A(i,i) - 1; A(i+1, i+1) = A(i+1,i+1)-1;
end
for i = 16:19
    A(i,i+1) = 1; A(i+1, i) = 1;
    A(i,i) = A(i,i) - 1; A(i+1, i+1) = A(i+1,i+1)-1;
end
%%
for i = 1:5
    A(i,i+5) = 1; A(i+5, i) = 1;
    A(i,i) = A(i,i) - 1; A(i+5, i+5) = A(i+5,i+5)-1;
end
for i = 11:15
    A(i,i+5) = 1; A(i+5, i) = 1;
    A(i,i) = A(i,i) - 1; A(i+5, i+5) = A(i+5,i+5)-1;
end
%%
for i = 1:5
    A(i,i+10) = 1; A(i+10, i) = 1;
    A(i,i) = A(i,i) - 1; A(i+10, i+10) = A(i+10,i+10)-1;
end
for i = 6:10
    A(i,i+10) = 1; A(i+10, i) = 1;
    A(i,i) = A(i,i) - 1; A(i+10, i+10) = A(i+10,i+10)-1;
end
%%
W = abs(0.5*A./diag(A));
W = W';
A = A-eye(20);
%%
B = eye(20);
in_door_gain = ones(20,1);
x_set = 22;
xo = 30;
save('Building-2x2x5-room.mat', 'A','B','W',"xo","x_set","in_door_gain")
