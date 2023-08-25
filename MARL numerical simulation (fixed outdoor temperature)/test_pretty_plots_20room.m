% test the algorithm
addpath('cell_operator')
clear all
%% specifying hyperparameter:
% Model parameter: A, B, W, sigma, alpha, delta_t
load('Building-20-room.mat')
%xo = 0;
%in_door_gain = zeros(4,1);
C = 200;
sigma = 2.5; alpha = 0.01; delta_t = 60; N = 20;
Black_Box_Env = @(x, u) Building(x,xo,x_set,in_door_gain, u, A, B, sigma, alpha, delta_t, C);

% initial state and intial controller
x0 = xo*ones(N,1);
K0 = {}; for i = 1:N; K0{i} = zeros(2,1); end % K0{N+1} = zeros(N,1);%-xo*ones(N,1);

%%
TJ = 1000;
TB = 1;
TG = 18001;
r = 0.5;
stepsize = 0.000003;
args.TB = TB; args.r = r;args.TG = TG; args.stepsize = stepsize;
%% specifying sample_estimator and cost_estimator
sample_generator = @()sample_uniform_sphere(K0, W, TJ);
cell_2_mat = @(K)cell2mat(K); mat_2_policy = @(K_mat,x) mat_2_policy(K_mat,x);
controller_converter = {cell_2_mat, mat_2_policy};
cost_estimator = @(K) estimate_cost(K, W, x0, Black_Box_Env, TJ, controller_converter);
A_tilde = eye(N) + 1/C*delta_t*A; B_tilde = 1/C*delta_t*B;
Sigma_w = eye(N) * 1/C*1/C*delta_t * sigma^2; Q = eye(N); R = alpha*eye(N);
d = 1/C*delta_t*(xo+in_door_gain);theta = x_set*ones(N,1);
cost_tester = @(K) test_calculate_cost(A_tilde, B_tilde,d,theta, Sigma_w, Q, R, K);

K = K0;
K_history = cell(6,1);
x_history = cell(6,1);
c_history = cell(6,1);
u_history = cell(6,1);
cost_lst = [];
for i = 1:TG
if mod(i,3000) == 1
    idx = (i-1)/3000 + 1;
    K_history{idx,1} = K;
    [x_traj, c_traj, u_traj] = generate_traj(K, Black_Box_Env,x0, 100, controller_converter);
    x_history{idx,1} = x_traj;
    c_history{idx,1} = c_traj;
    u_history{idx,1} = u_traj;
end
cost = cost_tester(K);
disp(cost)
cost_lst = [cost_lst, cost];
g = estimate_gradient(cost_estimator, sample_generator, K, args);
%g = estimate_gradient_coordinate(cost_estimator,sample_generator, K, args);
K = cell_add(K, scalar_cell_mult(-stepsize, g));
%K_mat = blkdiag(K{:});
end
plot(cost_lst)

%%
timeline = 1:length(x_traj);
figure
for i = 1:6
%subplot(1,3,i)
a = figure;
x_traj = x_history{i,1};
timeline1 = 0:length(x_traj);
x_traj = [xo*ones(20,1),x_traj];
hold on
grid on
%plot(timeline, xo*ones(size(timeline)),'--')
plot(timeline1, x_traj', 'LineWidth',4)
plot(timeline1,x_set*ones(size(x_traj,2),1), 'b --','LineWidth',4)
set(gca,'FontSize',20)
set(gca,'position',[0.2, 0.2, 0.7, 0.75])
%title(['Iteration ' num2str((i-1)*50)])
xlabel('Time/min', 'Interpreter', 'latex')
ylabel('Temperature/$^o$C','Interpreter', 'latex')
if i == 1
ylim([20 35])
else
    ylim([20,31])
end
hold off
saveas(a, ['figures/20rooms-',num2str(i),'.eps'],'epsc')
end
