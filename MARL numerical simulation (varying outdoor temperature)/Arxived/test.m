% test the algorithm
addpath('cell_operator')
addpath('test functions')
clear all
%% specifying hyperparameter:
% Model parameter: A, B, W, sigma, alpha, delta_t
rng(1)
load('Building-4-room-changing-outside-temperature.mat')
sigma = 2.5; alpha = 0.01; delta_t = 60; N = 4;
Black_Box_Env = @(x, u, xo, x_set) Building(x,xo,x_set,in_door_gain, u, A, B, sigma, alpha, delta_t, C);

% initial state and intial controller
x0 = xo_lst(1)*ones(N,1);
K0 = {}; 
for i = 1:N
    K0{i} = zeros(1);K0{N+i} = zeros(1);K0{2*N+i} = zeros(1); 
end
%% Algorithm Hyperparameters
TJ = 300;
TB = 1;
record_flag = 50;
TG = record_flag*5 + 1;
r = 0.5;
stepsize = 0.0000005;
args.TB = TB; args.r = r;args.TG = TG; args.stepsize = stepsize;
%% Running the algorithm
sample_generator = @()sample_uniform_sphere(K0, W, TJ);
cost_tester = @(K) test_calculate_expected_cost(K, xo_lst,x_set,in_door_gain, A, B, sigma, alpha, delta_t, C);
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Temporal script for testing whether I have written the cost_tester
% correctly

%cost = cost_tester(K0);
%cost_lst = [];
%for i = 1:length(xo_lst)
%    TJ = 300;
%    xo_lst_temp = xo_lst(i)*ones(1,TJ);
%    x0 = xo_lst(i)*ones(N,1);
%    for batch = 1:10
%[x_traj, c_traj, u_traj] = generate_traj_real_time(K0, Black_Box_Env,x0,xo_lst_temp, x_set);
%cost_est = mean(mean(c_traj))*4;
%cost_lst = [cost_lst,cost_est];
%    end
%end
%disp(cost)
%disp(mean(cost_lst))

% Test end! It is correct! TJ better chosen greater than 300 to get
% accurate estimation results.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load('temperature_data.mat')
xo_history = test_temperature(test_temperature > 26);%xo_history = repmat(xo_history,[2,1]);
K = K0;
K_history = cell(6,1);
x_history = cell(6,1);
c_history = cell(6,1);
u_history = cell(6,1);
cost_lst = [];
for t = 1:TG
    if mod(t,record_flag) == 1
    idx = (t-1)/record_flag + 1;
    K_history{idx,1} = K;
    [x_traj, c_traj, u_traj] = generate_traj_real_time(K, Black_Box_Env,x0,xo_history, x_set);
    x_history{idx,1} = x_traj;
    c_history{idx,1} = c_traj;
    u_history{idx,1} = u_traj;
    end
    L = length(xo_lst);
    for i = 1:L
        %j = randi(length(xo_lst),1);
        %xo = xo_lst(j);
        xo = xo_lst(i);
        cost_estimator = @(K) estimate_cost(K, W, x0, xo, x_set, Black_Box_Env, TJ);
        cost = cost_tester(K);
        disp(cost)
        cost_lst = [cost_lst, cost];
        g = estimate_gradient(cost_estimator, sample_generator, K, args);
        K = cell_add(K, scalar_cell_mult(-stepsize, g));
    end
end

%%
%timeline = 1:length(x_history{1,1});
figure(1)
for i = 1:6
timeline = 1:599;
subplot(2,3,i)
x_traj = x_history{i,1};
x_traj = x_traj(:,timeline);
hold on
plot(timeline, xo_history(timeline), '--')
plot(timeline, x_traj', 'LineWidth',1)
plot(timeline,x_set*ones(size(x_traj,2),1), 'b --')
set(gca,'FontSize',18)
title(['Iteration ' num2str((i-1)*record_flag)])
xlabel('Time/min')
ylabel('Temperature/^oC')
if i == 1
ylim([20 35])
else
    ylim([18,30])
end
hold off
end

%% Test the performance of the final controller on real time data
load('temperature_data.mat')
xo_history = test_temperature(test_temperature > 26);

[x_traj, c_traj, u_traj] = generate_traj_real_time(K, Black_Box_Env,x0,xo_history, x_set);
disp('Cost for controller K')
disp(sum(mean(c_traj,2)));
figure(2)
plot(x_traj')
hold on
plot(xo_history, '--')
plot(x_set*ones(size(xo_history)), '--')
hold off

%% Comparing this with the controller of the form u = Kx + b
K_comparison = load('K_comparison.mat');
K_comparison = K_comparison.K;

[x_traj, c_traj, u_traj] = generate_traj_real_time_comparing(K_comparison, Black_Box_Env,x0,xo_history, x_set);
disp('Cost for comparison controller K_comparison')
disp(sum(mean(c_traj,2)));
figure(3)
plot(x_traj')
hold on
plot(xo_history, '--')
plot(x_set*ones(size(xo_history)), '--')
hold off