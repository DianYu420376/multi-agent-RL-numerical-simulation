function [c,x] = Building(x,xo,x_set,in_door_gain, u, A, B, sigma, alpha, delta_t, C)
% ---- Inputs:
% x: State variable T_1, ..., T_N
% u: Control variable
% A,B,b,sigma: Dynamical parameters
%   The system is given by
%   \dot{x} = Ax + Bu + b + sigma*randn(size(x))
% x_set, u_set, alpha: Reward parameters
%   The reward is given by
%   c = (x - x_set).^2 + alpha*(u - u_set).^2
% delta_t: Stepsize for discretization
% ---- Outputs:
% x: The state variable at next time step
% c: (A vector of size N*1) rewards observed by each agent
delta_x = A*x + B*u + in_door_gain +  xo;
c = (x-x_set).^2 + alpha*u.^2;
x = x + 1/C*delta_t*delta_x + 1/C*sqrt(delta_t)*sigma*randn(size(x));
end
