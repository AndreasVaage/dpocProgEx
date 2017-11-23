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

% REMARKS
% What to do if there is a hole on the restart square?
 % What to do if restart square is in disturbance's reach?

% P(i, j, l) = P(transition from i to j if l applied)
P = ComputeTransitionProbabilities( stateSpace, controlSpace, ...
    mazeSize, walls, targetCell, holes, resetCell, p_f );

MN = size(stateSpace,1); 
L = size(controlSpace,1);
G = zeros(MN,L);
zeroInputIdx = 1;

for xIdx=1:MN
   for uIdx=1:L
       x = stateSpace(xIdx,:);
       u = controlSpace(uIdx,:);
       
       xAfterU = x+u;
       xAfterUIdx = getStateIdx(xAfterU);
       
       if P(xIdx) == 0
           G(xIdx,uIdx) = Inf;
           continue;
       end
       
       P_ArriveAtXAfterUAlsoAfterW = P(xIdx,xAfterUIdx,uIdx);
       P_StayDuringW = P(xAfterUIdx,xAfterUIdx,zeroInputIdx);
       P_NoFallDuringU = P_ArriveAtXAfterUAlsoAfterW - P_StayDuringW; %unsure

       P_FallDuringW = P(xAfterUIdx,resetCell,zeroInputIdx);
       
       P_ZeroW = 1/9;
       P_HitWallNoFall = P_StayDuringW - P_ZeroW;
       P_NoFallWhenHole = 1-p_f;
       if isStateOnHole(xAfterU,holes)
           P_HitWall = P_HitWallNoFall/P_NoFallWhenHole;
       else
           P_HitWall = P_HitWallNoFall;
       end

       gTimeStep = 1;
       gFallDuringU = c_r * (1-P_NoFallDuringU);
       gFallDuringW = c_r * P_FallDuringW;
       gHitWall = c_p * P_HitWall;
       
       gTotal = gTimeStep + gFallDuringU + gFallDuringW + gHitWall;
       G(xIdx,uIdx) = gTotal;
   end 
end

function xOnHole = isStateOnHole(x,holes)
    xOnHole = ismember(x,holes,'rows');
end

function idx = getStateIdx(coordinate)
    if coordinate > 0 && all(coordinate <= mazeSize)
        idx = stateSpace(:,coordinate);
    else
        idx = 0;
    end
end


end








