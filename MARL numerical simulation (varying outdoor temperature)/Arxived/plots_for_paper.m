%% Test the performance of the final controller on real time data
load('temperature_data.mat')
load('K_online.mat')
xo_history = test_temperature(test_temperature > 26);

Black_Box_Env = @(x, u, xo, x_set) Building(x,xo,x_set,in_door_gain, u, A, B, sigma, alpha, delta_t, C);


[x_traj, c_traj, u_traj] = generate_traj_real_time(K, Black_Box_Env,x0,xo_history, x_set);
disp('Cost for controller K')
disp(sum(mean(c_traj,2)));
figure
subplot(1,3,3)
plot(x_traj')
hold on
plot(xo_history, '--')
plot(x_set*ones(size(xo_history)), '--')
hold off
title('Setting 3')

%% Test the performance of the final controller on real time data
load('temperature_data.mat')
load('K.mat')
xo_history = test_temperature(test_temperature > 26);

[x_traj, c_traj, u_traj] = generate_traj_real_time(K, Black_Box_Env,x0,xo_history, x_set);
disp('Cost for controller K')
disp(sum(mean(c_traj,2)));
subplot(1,3,2)
plot(x_traj')
hold on
plot(xo_history, '--')
plot(x_set*ones(size(xo_history)), '--')
hold off
title('Setting 2')

%% Comparing this with the controller of the form u = Kx + b
K_comparison = load('K_comparison.mat');
K_comparison = K_comparison.K;

[x_traj, c_traj, u_traj] = generate_traj_real_time_comparing(K_comparison, Black_Box_Env,x0,xo_history, x_set);
disp('Cost for comparison controller K_comparison')
disp(sum(mean(c_traj,2)));
subplot(1,3,1)
plot(x_traj')
hold on
plot(xo_history, '--')
plot(x_set*ones(size(xo_history)), '--')
hold off
title('Setting 1')
