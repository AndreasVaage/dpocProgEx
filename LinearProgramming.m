function [ J_opt, u_opt_ind ] = LinearProgramming( P, G )
%LINEARPROGRAMMING Value iteration
%   Solve a stochastic shortest path problem by linear programming.
%
%   [J_opt, u_opt_ind] = LinearProgramming(P, G) computes the optimal cost
%   and the optimal control input for each state of the state space.
%
%   Input arguments:
%
%       P:
%           A (MN x MN x L) matrix containing the transition probabilities
%           between all states in the state space for all attainable
%           control inputs. The entry P(i, j, l) represents the transition
%           probability from state i to state j if control input l is
%           applied.
%
%       G:
%           A (MN x L) matrix containing the stage costs of all states in
%           the state space for all attainable control inputs. The entry
%           G(i, l) represents the cost if we are in state i and apply
%           control input l.
%
%   Output arguments:
%
%       J_opt:
%       	A (1 x MN) matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (1 x MN) matrix containing the indices of the optimal control
%       	inputs for each element of the state space.

% Remove termination (zero) state from P and G
zeroInputIdx = 1;
targetIdx = -1;
for i=1:size(G,1)
   if P(i,i,zeroInputIdx) > 0.999999
    targetIdx = i;
   end
end
P(targetIdx,:,:) = [];
P(:,targetIdx,:) = [];
G(targetIdx,:) = [];

% Set up needed variables
n_states = size(G,1);
n_inputs = size(G,2);
mu = zeros(1,n_states);

% Adjustable params
alpha = 1.0;

% Set up Linear program
f = -ones(n_states,1);
A = zeros(n_inputs*n_states,n_states);
b = zeros(n_inputs*n_states,1);
for i=1:n_inputs
    A(n_states*(i-1)+1:n_states*i,:) = eye(n_states)-alpha*P(:,:,i);
    b(n_states*(i-1)+1:n_states*i) = G(:,i);
end

% Remove infinite costs before running LP
tf = b==Inf;
b(tf) = [];
A(tf,:) = [];

disp('Running linear program ...');
J_opt = linprog(f,A,b)';

% We know that we now have the optimal cost-to-go
% One iteration of policy improvement gives the optimal policy
for i=1:n_states
    P_i = squeeze(P(i,:,:));
    [~,mu(i)] = min(G(i,:) + alpha*J_opt*P_i);
end

disp('Linear porgram complete!');

% Zero input in termination state
% Cost 0 in termination state
J_opt = [J_opt(1:targetIdx-1), 0, J_opt(targetIdx:end)];
u_opt_ind = [mu(1:targetIdx-1), zeroInputIdx, mu(targetIdx:end)];

end
