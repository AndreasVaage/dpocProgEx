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
    statesLiesNextToWallTable = ...
    createOneStepWallTransitionTable(walls, mazeSize);
    stateLiesOnAHole = false(mazeSize);
    for hole = holes'
        stateLiesOnAHole(hole) = true;
    end
    resetCellIdx = getStateIdx(resetCell);
    
    P = zeros(mazeSize(1)*mazeSize(2),mazeSize(1)*mazeSize(2),length(controlSpace));
    size(P)
    for startStateIdx = 1:length(stateSpace)
        startState = stateSpace(startStateIdx);
        for controlInputIdx = 1:length(controlSpace)
            controlInput = controlSpace(controlInputIdx);
            endState = startState + controlInput;
            endStateIdx = getStateIdx(endState);
            resetProb = 0;
            if endStateIdx ~= 0 && blockedByWall(startState,controlInput) == 0
                probOfReachingWantedState = 1-probOfFalling(startState,controlInput);
                
              
                
                for disturbance = [1 0; 1 1; 0 1; -1 1; -1 0; -1 -1; 0 -1; 1 -1; 0 0]'
                    newEndState = endState + disturbance;
                    newEndStateIdx = getStateIdx(newEndState);
                    if newEndStateIdx ~= 0 && blockedByWall(endState,disturbance) == 0
                        probOfFall = probOfFalling(endState,disturbance);
                        P(startStateIdx,newEndStateIdx,controlInputIdx) = (1-probOfFall)/9;
                        resetProb =+ (probOfFall)/9;
                    elseif stateLiesOnAHole(endState)
                        P(startStateIdx,endStateIdx,controlInputIdx) =+ p_f/9;
                        resetProb =+ (1-p_f)/9;
                    else
                        P(startStateIdx,endStateIdx,controlInputIdx) =+ 1/9;
                    end
                end
                
                P(startStateIdx,endStateIdx,controlInputIdx) =... 
                    P(startStateIdx,endStateIdx,controlInputIdx)*probOfReachingWantedState;
                P(startStateIdx,resetCellIdx,controlInputIdx) =+ ...
                    resetProb*(1-probOfReachingWantedState);
            end
        end
    end
    
    
    
    function prob = probOfFalling(start,u)
        prob = 0;
        state = start;
        number_of_steps = int(abs(u));
        for i = 1:number_of_steps
            state = state + u/number_of_steps;
            if stateLiesOnAHole(state) == true
                prob = prob + (1-prob)*p_f;
            end
        end
    end
    
    function blocked = blockedByWall(start,u)
        state = start;
        number_of_steps = int(abs(u));
        for i = 1:number_of_steps
            state = state + u/number_of_steps;
            if statesLiesNextToWallTable(getStateIdx(start),getStateIdx(state))
                blocked = true;
                return
            end
        end
        blocked = false;
    end

    function blocked = createOneStepWallTransitionTable(walls, mazeSize)
        % Create a MN x MN logical array where one-step-transitions through
        % walls are true. Rest is false.
        blocked = false(mazeSize(1)*mazeSize(2));
        for k = 1:2:size(walls,1)
            x1,y1 = walls(k); x2,y2 = walls(k+1);
            if ((x1-x2) == 0) % Vertical wall
                y = max(y1,y2);
                for shift = [0 1; 0 -1; 0 0; 1 0; -1 0]'
                    oneSideOfWall = getStateIdx([x1,y+shift(1)]);
                    otherSideOfWall = getStateIdx([x1+1,y+shift(2)]);
                    if oneSideOfWall ~= 0 && otherSideOfWall ~= 0
                        blocked(oneSideOfWall,otherSideOfWall) = true;
                        blocked(otherSideOfWall,oneSideOfWall) = true;
                    end
                end
            elseif ((y1 - y2) == 0) % Horisontal wall
                x = max(x1,x2);
                for shift = [0 1; 0 -1; 0 0; 1 0; -1 0]'
                    oneSideOfWall = getStateIdx([x+shift(1),y1]);
                    otherSideOfWall = getStateIdx([x+shift(2),y1+1]);
                    if oneSideOfWall ~= 0 && otherSideOfWall ~= 0
                        blocked(oneSideOfWall,otherSideOfWall) = true;
                        blocked(otherSideOfWall,oneSideOfWall) = true;
                    end
                end
            end
        end
    end

    function idx = getStateIdx(coordinate)
        if coordinate > 0 && all(coordinate <= mazeSize)
            idx = stateSpace(:,coordinate);
        else
            idx = 0;
        end
    end
end

