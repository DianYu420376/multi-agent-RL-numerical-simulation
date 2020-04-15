clear all
load('History/deterministic_run3.mat')
%% specifying sample_estimator and cost_estimator
sample_generator = @()sample_uniform_sphere(K0, W, TJ);
cost_estimator = @(K) estimate_cost(K, W, x0, Black_Box_Env, TJ);
A_tilde = eye(N) + delta_t*A; B_tilde = delta_t*B;
Sigma_w = eye(N) * delta_t * sigma^2; Q = eye(N); R = alpha*eye(N);
d = delta_t*(xo+in_door_gain);theta = x_set*ones(N,1);
cost_tester = @(K) test_calculate_cost(A_tilde, B_tilde,d,theta, Sigma_w, Q, R, K);
gradient_tester = @(K) test_calculate_gradient(A_tilde, B_tilde,d,theta, Sigma_w, Q, R, K);
K = K0;

TG = 5000;
stepsize = 0.00007; args.stepsize = stepsize;
cost_lst = [];
for i = 1:TG
    cost = cost_tester(K);
    disp(cost)
    cost_lst = [cost_lst, cost];
    %g = gradient_tester(K);
    g = estimate_gradient(cost_estimator, sample_generator, K, args);
    K = cell_add(K, scalar_cell_mult(-stepsize, g));
end
figure(1);plot(cost_lst);


%% Look at the performance of the K obtained form deterministic gd
K_mat = blkdiag(K{1:end-1});
b = K{end};
mu = zeros(size(x0));J_real = 0;
x_traj = zeros(4, 1000);
c_traj = zeros(4, 1000);
x = x0;
for t = 1:1000
    u = K_mat*x + b;
    [c, x] = Black_Box_Env(x, u);
    x_traj(:,t) = x; c_traj(:,t) = c;
    J_real = J_real + sum(c);
    mu = (t-1)/t*W*mu + 1/t*c;
end
J = (length(K0)-1)*mu/N;
J_real = J_real/1000/N;
figure(2);plot(c_traj');title('cost')
figure(3);plot(x_traj');title('temperature')