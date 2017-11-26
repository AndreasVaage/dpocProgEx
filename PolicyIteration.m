function [ J_opt, u_opt_ind ] = PolicyIteration( P, G )
%POLICYITERATION Value iteration
%   Solve a stochastic shortest path problem by policy iteration.
%
%   [J_opt, u_opt_ind] = PolicyIteration(P, G) computes the optimal cost and
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
J = zeros(1,n_states);
J_update = zeros(1,n_states);
P_given_mu = zeros(n_states,n_states);
g_given_mu = zeros(n_states,1);
zeroInputIdx = 1;

% Adjustable params
alpha = 1.0;
mu = zeros(1,n_states);
mu(:) = zeroInputIdx; % init policy guess (default: zero input)

disp('Running policy iteration ...');
while true
    % Policy evaluation
    for i=1:n_states
       P_given_mu(i,:) = P(i,:,mu(i));
       g_given_mu(i) = G(i,mu(i)); 
    end
    J_updateT = (eye(n_states) - alpha*P_given_mu)\g_given_mu;
    J_update = J_updateT';
    
    if isequal(J,J_update)
        disp('Cost-to-go not changing.');
        break;
    end
    J = J_update;
    
    % Policy improvement
    for i=1:n_states
        P_i = squeeze(P(i,:,:));
        [~,mu(i)] = min(G(i,:) + alpha*J_update*P_i);
    end
end

disp('Policy iteration complete!');
J_opt = J;
u_opt_ind = mu;

end

