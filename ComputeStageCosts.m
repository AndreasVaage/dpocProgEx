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

  statesNextToWallTable = createStatesNextToWallsTable(walls, mazeSize);
    resetCellIdx = getStateIdx(resetCell,mazeSize);
    G = zeros(size(stateSpace,1),size(controlSpace,1));
    zeroInputIdx = 1;
    % i = start state,    cell_i = start cell
    % j = intended state,      cell_j = intended cell (end cell before noise)
    % u = controll input, step_u = controll input step 
    
    for i = 1:length(stateSpace)
        cell_i = stateSpace(i,:);
        if cell_i == targetCell
            G(i,:) = Inf;
            G(i,zeroInputIdx) = 0;
            continue
        end
        for u = 1:length(controlSpace)
            step_u = controlSpace(u,:);
            if blockedByWall(cell_i,step_u,statesNextToWallTable,mazeSize)
                G(i,u) = Inf;
                continue
            end
            
            cell_j = cell_i + step_u;
            P_fallDuringU = probOfFalling(cell_i,step_u,p_f,holes);
            P_fallDuringW = 0;
            P_hitWall = 0;
            for step_w = controlSpace(1:9,:)'
                if not(blockedByWall(cell_j,step_w',statesNextToWallTable,mazeSize)) % No Wall
                    P_fallDuringW = P_fallDuringW +  probOfFalling(cell_j,step_w',p_f,holes)/9;
                elseif isStateOnHole(cell_j,holes) % Wall, and wanted state is a hole
                    P_hitWall = P_hitWall + 1/9;
                    P_fallDuringW = P_fallDuringW + p_f/9;
                else % Wall, and wanted state is not a hole
                    P_hitWall = P_hitWall + 1/9;
                end
            end
   
            gTimeStep = 1;
            gFallDuringU = c_r * P_fallDuringU;
            gFallDuringW = c_r * (1-P_fallDuringU)*P_fallDuringW;
            gHitWall = c_p * (1-P_fallDuringU)*P_hitWall;

            G(i,u) = gTimeStep + gFallDuringU + gFallDuringW + gHitWall;
        end
    end
end

function blocked = blockedByWall(start,u,statesNextToWallTable,mazeSize)
% Given start state and control input, returns true of the step is 
% blocked by a wall.
    newState = start;
    number_of_steps = max(abs(u));
    for i = 1:number_of_steps
        state = newState;
        newState = state + u/number_of_steps;
        j = getStateIdx(newState,mazeSize);
        if j == 0 || statesNextToWallTable(getStateIdx(state,mazeSize),j)
            blocked = true;
            return
        end
    end
    blocked = false;
end

function blocked = createStatesNextToWallsTable(walls, mazeSize)
    % Create a MN x MN logical array where only two states lying next to 
    % each other, with a wall between, returns true, rest are false.
    blocked = false(mazeSize(1)*mazeSize(2));
    for k = 1:2:size(walls,1)
        x1 = walls(k,1); y1 = walls(k,2);
        x2 = walls(k+1,1); y2 = walls(k+1,2);
        if ((x1-x2) == 0) % Vertical wall
            for y = (min(y1,y2) + 1):max(y1,y2)
                for shift = [0 1; 0 -1; 0 0; 1 0; -1 0]'
                    oneSideOfWall = getStateIdx([x1,y+shift(1)],mazeSize);
                    otherSideOfWall = getStateIdx([x1+1,y+shift(2)],mazeSize);
                    % Assignes index 0 if out of bound
                    if oneSideOfWall ~= 0 && otherSideOfWall ~= 0 
                        blocked(oneSideOfWall,otherSideOfWall) = true;
                        blocked(otherSideOfWall,oneSideOfWall) = true;
                    end
                end
            end
        elseif ((y1 - y2) == 0) % Horisontal wall
            for x = (min(x1,x2) + 1):max(x1,x2)
                for shift = [0 1; 0 -1; 0 0; 1 0; -1 0]'
                    oneSideOfWall = getStateIdx([x+shift(1),y1],mazeSize);
                    otherSideOfWall = getStateIdx([x+shift(2),y1+1],mazeSize);
                    if oneSideOfWall ~= 0 && otherSideOfWall ~= 0
                        blocked(oneSideOfWall,otherSideOfWall) = true;
                        blocked(otherSideOfWall,oneSideOfWall) = true;
                    end
                end
            end
        end
    end
end
    
function idx = getStateIdx(cell, mazeSize)
% Returns the corresponding state for a given cell. Returns 0 if the cell
% lies outside the board.
    if all(cell > 0) && all(cell <= mazeSize)
        idx = ( cell(1) - 1 ) * mazeSize( 2 ) + cell(2);
    else
        idx = 0;
    end
end

function prob = probOfFalling(start,u,p_f,holes)
% Returns the probability of falling while performing the step u from state
% start.
    prob = 0;
    state = start;
    number_of_steps = max(abs(u));
    for i = 1:number_of_steps
        state = state + u/number_of_steps;
        if isStateOnHole(state,holes) == true
            prob = prob + (1-prob)*p_f;
        end
    end
end
    
function xOnHole = isStateOnHole(x,holes)
% Returns true if a cell x, lies on a hole.
    if isempty(holes)
        xOnHole = false;
    else
        xOnHole = ismember(x,holes,'rows');
    end
end