function [ J_opt, u_opt_ind ] = ValueIteration( P, G )
%VALUEITERATION Value iteration
%   Solve a stochastic shortest path problem by value iteration.
%
%   [J_opt, u_opt_ind] = ValueIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
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

% Setting up needed variables
n_states = size(G,1); 
J_update = zeros(1,n_states);
mu = zeros(1,n_states);
delta = Inf;

% Adjustable params
a = 0.99;
cost_to_go_change_tol = 0.000000001;
J = zeros(1,n_states); % init cost guess (default: zero cost)

disp('Running value iteration ...');
while any(delta(:) > cost_to_go_change_tol)
   for i=1:n_states
        P_i = squeeze(P(i,:,:));
        [J_update(i),mu(i)] = min(G(i,:) + a*J*P_i);
   end
   delta = abs(J_update-J);
   J=J_update;
end

disp('Value iteration complete!');
J_opt = J;
u_opt_ind = mu;

end

