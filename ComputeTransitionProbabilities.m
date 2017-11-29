function P = ComputeTransitionProbabilities( stateSpace, controlSpace,... 
mazeSize, walls, targetCell, holes, resetCell, p_f )
%COMPUTETRANSITIONPROBABILITIES Compute transition probabilities.
% 	Compute the transition probabilities between all states in the state
%   space for all attainable control inputs.
%
%   P = ComputeTransitionProbabilities(stateSpace, controlSpace,
%   disturbanceSpace, mazeSize, walls, targetCell) computes the transition
%   probabilities between all states in the state space for all attainable
%   control inputs.
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
%       
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
%   Output arguments:
%
%       P:
%           A (MN x MN x L) matrix containing the transition probabilities
%           between all states in the state space for all attainable
%           control inputs. The entry P(i, j, l) represents the transition
%           probability from state i to state j if control input l is
%           applied.

% put your code here
    statesNextToWallTable = createStatesNextToWallsTable(walls, mazeSize);
    resetCellIdx = getStateIdx(resetCell,mazeSize);
    P = zeros(mazeSize(1)*mazeSize(2),mazeSize(1)*mazeSize(2),length(controlSpace));
    
    % i = start state,    cell_i = start cell
    % j = intended state,      cell_j = intended cell (end cell before noise)
    % u = controll input, step_u = controll input step 
    
    for i = 1:length(stateSpace)
        cell_i = stateSpace(i,:);
        if cell_i == targetCell
            P(i,i,:) = 1;
            continue
        end
        for u = 1:length(controlSpace)
            step_u = controlSpace(u,:);
            if blockedByWall(cell_i,step_u,statesNextToWallTable,mazeSize)
                % P(i,:,u) = 0 (zero by default)
                continue
            end
            
            cell_j = cell_i + step_u;
            j = getStateIdx(cell_j,mazeSize);
            P_notFallDuringU = 1-probOfFalling(cell_i,step_u,p_f,holes);
            P_reset = 0;
            P_stayDuringW = 0;

            for step_w = controlSpace(1:9,:)'
                finalState = getStateIdx(cell_j + step_w',mazeSize);
                if not(blockedByWall(cell_j,step_w',statesNextToWallTable,mazeSize)) % No Wall
                    P_fallDuringW = probOfFalling(cell_j,step_w',p_f,holes);
                    P(i,finalState,u) = P_notFallDuringU*(1-P_fallDuringW)/9;
                    P_reset = P_reset + P_notFallDuringU * (P_fallDuringW)/9;
                elseif isStateOnHole(cell_j,holes) % Wall, and wanted state is a hole
                    P_stayDuringW = P_stayDuringW + (1-p_f)/9;
                    P_reset = P_reset + P_notFallDuringU * p_f/9;
                else % Wall, and wanted state is not a hole
                    P_stayDuringW = P_stayDuringW + 1/9;
                end
            end

            P(i,j,u) = P_notFallDuringU * (P_stayDuringW + 1/9);
            P(i,resetCellIdx,u) = P(i,resetCellIdx,u) + P_reset + (1-P_notFallDuringU);
        end
    end
    
    
    % Debug:
    % Plots the probabilities, pauses the program and gives a warning if the
    % sum of probabilities for a given start state and input is not equal
    % to 0 or 1.
    for i = 1:length(stateSpace)
        for u = 1:length(controlSpace) 
            s = sum(P(i,:,u));
            if ((s < 0.99999 && s ~= 0) || s > 1.0001) %|| isequal(stateSpace(startStateIdx)',targetCell))
                warning('Sum of probabilities is not equal 1');
                PlotMazeDebugg( 1, mazeSize, walls, targetCell, holes, resetCell, P,stateSpace,controlSpace,i,u);
                waitforbuttonpress; 
                close all
            end
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
            y = max(y1,y2);
            for shift = [0 1; 0 -1; 0 0; 1 0; -1 0]'
                oneSideOfWall = getStateIdx([x1,y+shift(1)],mazeSize);
                otherSideOfWall = getStateIdx([x1+1,y+shift(2)],mazeSize);
                % Assignes index 0 if out of bound
                if oneSideOfWall ~= 0 && otherSideOfWall ~= 0 
                    blocked(oneSideOfWall,otherSideOfWall) = true;
                    blocked(otherSideOfWall,oneSideOfWall) = true;
                end
            end
        elseif ((y1 - y2) == 0) % Horisontal wall
            x = max(x1,x2);
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

