function G = ComputeStageCosts( stateSpace, controlSpace, mazeSize, walls, targetCell, holes, resetCell, p_f, c_p, c_r )
%COMPUTESTAGECOSTS Compute stage costs.
% 	Compute the stage costs for all states in the state space for all
%   attainable control inputs.
%
%   G = ComputeStageCosts(stateSpace, controlSpace, disturbanceSpace,
%   mazeSize, walls, targetCell) computes the stage costs for all states in
%   the state space for all attainable control inputs.
%
%   Input arguments:
%
%       stateSpace:
%           A (MN x 2) matrix, where the i-th row represents the i-th
%           element of the state space. Note that the state space also
%           contains the target cell, in order to simplify state indexing.
%
%       controlSpace:
%           A (L x 2) matrix, where the l-th row represents the l-th
%           element of the control space.
%
%       mazeSize:
%           A (1 x 2) matrix containing the width and the height of the
%           maze in number of cells.
%
%   	walls:
%          	A (2K x 2) matrix containing the K wall segments, where the start
%        	and end point of the k-th segment are stored in row 2k-1
%         	and 2k, respectively.
%
%    	targetCell:
%          	A (2 x 1) matrix describing the position of the target cell in
%         	the maze.
%       holes:
%         	A (H x 2) matrix containg the H holes of the maze. Each row
%         	represents the position of a hole.
%
%   	resetCell:
%         	A (1 x 2) matrix describing the position of the reset cell in
%           the maze.
%
%       p_f:
%           The probability of falling into a hole when the ball is
%           traversing through or to a cell with a hole
%       
%       c_p:
%           Every time the ball bounces into a wall or boundary, we get this number 
%            of time steps as penalty.
%       c_r:
%           Every time the ball falls into a hole, the ball is set to the reset cell
%           at the beginning of the next stage and we get this number of time steps
%           as additional penalty.
%
%   Output arguments:
%
%       G:
%           A (MN x L) matrix containing the stage costs of all states in
%           the state space for all attainable control inputs. The entry
%           G(i, l) represents the cost if we are in state i and apply
%           control input l.



% Get the TransitionProbabilities [must be implemented]
% The entry P(i, j, l) represents the transition
% probability from state i to state j if control input l is
% applied.
P = ComputeTransitionProbabilities( stateSpace, controlSpace, ...
    mazeSize, walls, targetCell, holes, resetCell, p_f );

% REMARKS
% What to do about the infinite transition probabilities?
% Always add 1 to the cost, as it is the time step cost
% What to do if there is a hole on the restart square?
 % What to do if restart square is in disturbance's reach?

MN = size(stateSpace,1); 
L = size(controlSpace,1);
G = zeros(MN,L); %or set to infinite intially?

for x_idx=1:MN
   for u_idx=1:size(controlSpace,1)
       x_orig = stateSpace(x_idx,:);
       u = controlSpace(u_idx,:);
       
       x_end_det_input = x_orig+u; % but won't necessarily end up here?
               %handeled by transition probability?
       
       %%%if P(x_orig,x_end_det_input,u) = 0:
        %%%G(x_orig,u) = inf ???
            %%%will there always be a probability of getting
            %%%there during the deterministic part? (if no walls on way)
               
       P_stay_during_disturb = P(x_end_det_input,x_end_det_input,[0,0]);
       P_no_fall_during_det_input = P(x_idx,x_end_det_input,u) ... % TODO not entirely sure about this one
           - P_stay_during_disturb;
       P_fall_during_det_input = 1-P_no_fall_during_det_input;

       P_fall_during_disturb = P(x_end_det_input,resetCell,[0,0]);
       P_hit_wall_no_fall_during_disturb = P_stay_during_disturb - 1/9;
       hole_at_end_state_det_input = true; % TODO is_hole(holes,end_state)
       if hole_at_end_state_det_input
           P_hit_wall_during_disturb = P_hit_wall_no_fall_during_disturb / ...
               (1-p_f);
       else
           P_hit_wall_during_disturb = P_hit_wall_no_fall_during_disturb;
       end

       g_fall_during_det_input = c_r * P_fall_during_det_input;
       g_fall_during_disturb = c_r * P_fall_during_disturb;
       g_hit_wall_during_disturb = c_p * P_hit_wall_during_disturb;
       g_time_step = 1;
       
       g_total = g_time_step + g_fall_during_det_input + ...
           g_fall_during_disturb + g_hit_wall_during_disturb;
       G(x_idx,u_idx) = g_total;
   end % input loop 
end %state loop
end

function is_hole = isHole(x)
    %TODO, should be easy
end




