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

zeroInputIdx = 1;
targetIdx = -1;
for i=1:size(G,1)
       if P(i,i,zeroInputIdx) > 0.999999
        targetIdx = i;
       end
end

disp(targetIdx);

P(targetIdx,:,:) = [];
P(:,targetIdx,:) = [];
G(targetIdx,:) = [];

n_states = size(G,1);
n_inputs = size(G,2);
alpha=1.0;

f = -ones(n_states,1);
M = zeros(n_inputs*n_states,n_states);
h = zeros(n_inputs*n_states,1);
for i=1:n_inputs
    M(n_states*(i-1)+1:n_states*i,:) = eye(n_states)-alpha*P(:,:,i);
    h(n_states*(i-1)+1:n_states*i) = G(:,i);
end

% Remove infinite costs before running LP
tf = h==Inf;
h(tf) = [];
M(tf,:) = [];

J = linprog(f,M,h)';
J_opt = [J(1:targetIdx-1), 0, J(targetIdx:end)];

% TODO: replace next line with finding policy
u_opt_ind = ones(1,n_states); %temporary ones

u_opt_ind = [u_opt_ind(1:targetIdx-1), 1, u_opt_ind(targetIdx:end)];

end
