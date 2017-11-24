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

% put your code here

%Test first iteration against Exam 2015 Ex.3:
% clear all; close all; clc;
% n_states = 3;
% n_inputs = 2;
% P = zeros(n_states,n_states,n_inputs);
% P(:,:,1) = [0.2 0.4 0.4; 0.4 0.6 0; 0.4 0.6 0];
% P(:,:,2) = [0.4 0.2 0.4; 0 0.6 0.4; 0.6 0.4 0];
% G = [16 8; 10 8; 1 2];
% J0 = [10,10,10];
% a = 0.5;

%Test against exercise 2.3 on problem set 2
% clear all; close all; clc;
% n_states = 2;
% n_inputs = 2;
% G = [-4 -6; 5 3];
% P = zeros(2,2,2);
% P(:,:,1) = [0.8 0.2; 0.7 0.3];
% P(:,:,2) = [0.5 0.5; 0.4 0.6];
% a = 0.99;
% J0 = [0,0];

iter_no=1;
tol = 0.00001;
delta = Inf;
J = J0;
J_update = zeros(1,n_states);
mu = zeros(1,n_inputs);
disp('Running value iteration ...');

% Actual value iteration
while delta > tol
   for i=1:n_states
        P_i = squeeze(P(i,:,:)); %P_i(j,u)=P(i,j,u)
        [J_update(i),mu(i)] = min(G(i,:) + J*P_i); %alt: a*J*P_I
   end;
   
   delta = abs(J_update-J);
   J=J_update;
   iter_no = iter_no + 1; 
end

disp('Value iteration complete!');
disp('Result of value iteration:');
disp('--------------------------');
disp('iter_no ='); disp(iter_no);
disp('J ='); disp(J);
disp('mu= '); disp(mu);

end

