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
J_opt = linprog(f,M,h);
% Need to find the actuation!:
u_opt_ind = ones(1,n_states);

end
