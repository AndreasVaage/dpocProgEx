%function [ J_opt, u_opt_ind ] = ValueIteration( P, G )
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

%Test first iteration against Exam 2015 Ex.3
clear all; close all; clc;
n_states = 3;
n_inputs = 2;
P = zeros(n_states,n_states,n_inputs);
P(:,:,1) = [0.2 0.4 0.4; 0.4 0.6 0; 0.4 0.6 0];
P(:,:,2) = [0.4 0.2 0.4; 0 0.6 0.4; 0.6 0.4 0];
G = zeros(n_states,n_inputs);
G = [16 8; 10 8; 1 2];
J = [10,10,10];

iter_no=1;
a=0.5;
J_update = [0,0,0];
policy = [0,0,0];
while iter_no < 100
   for i=1:n_states
        P_i = squeeze(P(i,:,:)); %P_i(j,u)=P(i,j,u)
        disp('J');
        disp(J);
        disp('P_i');
        disp(P_i);
        disp('J*P_i');
        disp(J*P_i);
        disp('total:');
        disp(G(i,:) + a*J*P_i);
        loop_info_text = [' G(i,:)=',num2str(G(i,:)),...
        '   J=',num2str(J,'%3d'), ...
        '   P_i',num2str(P_i,'%3d'), ...
        '   iter_no =',num2str(iter_no,'%3d')];
        [J_update(i) policy(i)] = min(G(i,:) + a*J*P_i);
   end;
   
   J=J_update;
   disp('J_updated:');
   disp(J);
   info_text = ['   J(1)=',num2str(J),...
        '   mu(1)=',num2str(policy(1),'%3d'), ...
        '   mu(2)=',num2str(policy(2),'%3d'), ...
        '   iter_no =',num2str(iter_no,'%3d')];
   disp(info_text);
   iter_no = iter_no + 1; 
end




%end

