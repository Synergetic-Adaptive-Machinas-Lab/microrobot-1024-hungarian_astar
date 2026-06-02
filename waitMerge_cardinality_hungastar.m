clear; close all; clc;

%% USER SETTINGS / FILE INPUTS

[filename1, pathname1] = uigetfile('*.csv', 'Select start state CSV');
if isequal(filename1, 0)
    error('Start CSV selection canceled.');
end

[filename2, pathname2] = uigetfile('*.csv', 'Select goal state CSV');
if isequal(filename2, 0)
    error('Goal CSV selection canceled.');
end

outDir = uigetdir(pwd, 'Select output folder');
if isequal(outDir, 0)
    error('Output folder selection canceled.');
end

P.start_csv = fullfile(pathname1, filename1);
P.goal_csv  = fullfile(pathname2, filename2);
P.out_dir   = outDir;

P.N = 32;
P.occupied_value = 7;
P.empty_value = 0;

P.row_spacing = sqrt(3)/2;

P.assignment_cost = 'euclidean';   % 'euclidean' or 'astar_length'
P.check_collisions = true;

P.show_figure = true;
P.show_grid = true;
P.marker_size = 85;
P.anim_pause = 0.08;

P.save_video = true;
P.video_name = 'integrated_transition_preview.mp4';
P.frame_rate = 15;

% Visualization
P.show_agent_trajectories = true;
P.show_agent_labels = true;
P.trajectory_line_width = 2.0;

% Dynamic replanning / merge avoidance
P.max_dynamic_steps = 3000;
P.allow_goal_as_temporary_passage = false;
P.deadlock_escape = true;

COLOR.agent = [0.85 0.15 0.10];
COLOR.goal = [0.00 0.65 0.20];
COLOR.centroid = [0.10 0.20 1.00];
COLOR.grid_node = [0.78 0.78 0.78];
COLOR.grid_edge = [0.90 0.90 0.90];

%% COMMAND WINDOW INPUT

P.output_start_index = input('Enter output start index, e.g., 0 or 12: ');

if isempty(P.output_start_index)
    P.output_start_index = 0;
end

assert(isscalar(P.output_start_index) && ...
       isnumeric(P.output_start_index) && ...
       isfinite(P.output_start_index) && ...
       P.output_start_index >= 0 && ...
       floor(P.output_start_index) == P.output_start_index, ...
       'output_start_index must be a nonnegative integer.');

%% READ CSV STATES

fprintf('\n[1] Reading CSV files...\n');

startPayload = readmatrix(P.start_csv);
goalPayload  = readmatrix(P.goal_csv);

assert(isequal(size(startPayload), [P.N, P.N]), ...
    'Start CSV must be %d x %d.', P.N, P.N);

assert(isequal(size(goalPayload), [P.N, P.N]), ...
    'Goal CSV must be %d x %d.', P.N, P.N);

startMap = payloadToPhysicalMap(startPayload, P);
goalMap  = payloadToPhysicalMap(goalPayload, P);

S0_original = mapToCells(startMap, P.occupied_value);
Sg = mapToCells(goalMap, P.occupied_value);

nStart = size(S0_original, 1);
nGoal  = size(Sg, 1);

fprintf('    grid size: %d x %d\n', P.N, P.N);
fprintf('    start active cells: %d\n', nStart);
fprintf('    goal active cells:  %d\n', nGoal);

%% PHASE 1: CARDINALITY MATCHING BY EXPANSION / CONTRACTION

fprintf('\n[2] Cardinality matching phase...\n');

allMaps = {};
phaseLabels = {};

% For identity-preserving visualization.
% Before cardinality matching is complete, identity is undefined.
agentCellsByFrame = {};
agentTrajByFrame = {};

currentCells = S0_original;

allMaps{end+1,1} = cellsToMap(currentCells, P.N, P.N, ...
    P.occupied_value, P.empty_value);
phaseLabels{end+1,1} = 'start';
agentCellsByFrame{end+1,1} = [];
agentTrajByFrame{end+1,1} = [];

if nStart < nGoal
    cardinalityMode = 'expansion';

    while size(currentCells,1) < nGoal
        currentCells = addNearestEmptyCellToCentroid(currentCells, P);

        allMaps{end+1,1} = cellsToMap(currentCells, P.N, P.N, ...
            P.occupied_value, P.empty_value);
        phaseLabels{end+1,1} = 'expansion';
        agentCellsByFrame{end+1,1} = [];
        agentTrajByFrame{end+1,1} = [];
    end

elseif nStart > nGoal
    cardinalityMode = 'contraction';

    while size(currentCells,1) > nGoal
        currentCells = removeFarthestOccupiedCellFromCentroid(currentCells, P);

        allMaps{end+1,1} = cellsToMap(currentCells, P.N, P.N, ...
            P.occupied_value, P.empty_value);
        phaseLabels{end+1,1} = 'contraction';
        agentCellsByFrame{end+1,1} = [];
        agentTrajByFrame{end+1,1} = [];
    end

else
    cardinalityMode = 'no_change';
end

S0_matched = currentCells;
nMatched = size(S0_matched, 1);

fprintf('    cardinality mode: %s\n', cardinalityMode);
fprintf('    matched active cells: %d\n', nMatched);
fprintf('    cardinality phase frames: %d\n', numel(allMaps));

assert(nMatched == nGoal, ...
    'Cardinality matching failed: matched=%d, goal=%d.', nMatched, nGoal);

% From this frame onward, agent identity is fixed.
matchedStartFrame = numel(allMaps);

%% PHASE 2: HUNGARIAN ASSIGNMENT

fprintf('\n[3] Computing Hungarian assignment...\n');

nAgents = nGoal;

switch lower(P.assignment_cost)
    case 'euclidean'
        [assignment, C] = assignByEuclideanDistance(S0_matched, Sg, P);

    case 'astar_length'
        [assignment, C] = assignByAstarLength(S0_matched, Sg, P);

    otherwise
        error('Unknown P.assignment_cost: %s', P.assignment_cost);
end

totalCost = sum(C(sub2ind(size(C), 1:nAgents, assignment)));

fprintf('    assignment cost type: %s\n', P.assignment_cost);
fprintf('    total Hungarian cost: %.6f\n', totalCost);

agentColors = lines(nAgents);
assignedGoalCells = Sg(assignment, :);

% Initial identity state after cardinality matching.
agentTraj = cell(nAgents, 1);
for i = 1:nAgents
    agentTraj{i} = S0_matched(i,:);
end

agentCellsByFrame{matchedStartFrame,1} = S0_matched;
agentTrajByFrame{matchedStartFrame,1} = agentTraj;

%% PHASE 3: BASELINE A* PATH STATS ONLY

fprintf('\n[4] Computing baseline A* paths for statistics...\n');

paths = cell(nAgents, 1);

for i = 1:nAgents
    paths{i} = astarStaggered(S0_matched(i,:), assignedGoalCells(i,:), P);
end

pathLengths = cellfun(@(p) size(p,1)-1, paths);

fprintf('    mean baseline A* path length: %.3f edges\n', mean(pathLengths));
fprintf('    max  baseline A* path length: %d edges\n', max(pathLengths));

%% PHASE 4: DEADLOCK-RESISTANT DYNAMIC A* TRANSITION

fprintf('\n[5] Creating deadlock-resistant dynamic A* transition states...\n');

currentAgentCells = S0_matched;
done = false(nAgents, 1);

stepCounter = 1;
stagnationCounter = 0;

while ~all(done) && stepCounter <= P.max_dynamic_steps

    oldCells = currentAgentCells;
    nextCells = currentAgentCells;

    % Rotating priority prevents one low-index agent from always winning.
    priority = circshift(1:nAgents, [0, -(mod(stepCounter-1, nAgents))]);

    movedThisStep = false(nAgents, 1);

    for pp = 1:nAgents
        i = priority(pp);

        if isequal(nextCells(i,:), assignedGoalCells(i,:))
            done(i) = true;
            continue;
        end

        % Treat already reserved current/next positions of other agents as obstacles.
        blockedCells = nextCells;
        blockedCells(i,:) = [];

        path_i = astarStaggeredBlocked( ...
            nextCells(i,:), assignedGoalCells(i,:), blockedCells, P);

        if size(path_i,1) < 2
            continue;
        end

        desiredCell = path_i(2,:);

        % Additional uniqueness check.
        otherCells = nextCells;
        otherCells(i,:) = [];

        if ~isCellInSet(desiredCell, otherCells)
            nextCells(i,:) = desiredCell;
            movedThisStep(i) = true;
        end
    end

    % Deadlock escape: if nobody moved, force one legal sidestep.
    if ~any(movedThisStep)
        stagnationCounter = stagnationCounter + 1;

        if P.deadlock_escape
            [nextCells, forcedMoved] = forceOneEscapeMove( ...
                currentAgentCells, assignedGoalCells, P);

            if forcedMoved
                movedThisStep(:) = false;
                stagnationCounter = 0;
            else
                warning('No legal escape move exists. Stopping early.');
                break;
            end
        else
            warning('No agent moved at step %d. Stopping early.', stepCounter);
            break;
        end
    else
        stagnationCounter = 0;
    end

    % Final uniqueness validation.
    nUnique = size(unique(nextCells, 'rows'), 1);
    if nUnique < nAgents
        error('Duplicate cell detected at dynamic step %d.', stepCounter);
    end

    currentAgentCells = nextCells;

    for i = 1:nAgents
        agentTraj{i}(end+1,:) = currentAgentCells(i,:);
        done(i) = isequal(currentAgentCells(i,:), assignedGoalCells(i,:));
    end

    allMaps{end+1,1} = cellsToMap(currentAgentCells, P.N, P.N, ...
        P.occupied_value, P.empty_value);
    phaseLabels{end+1,1} = 'dynamic hungarian astar';

    agentCellsByFrame{end+1,1} = currentAgentCells;
    agentTrajByFrame{end+1,1} = agentTraj;

    stepCounter = stepCounter + 1;

    if mod(stepCounter, 100) == 0
        fprintf('    dynamic step %d, reached %d/%d goals\n', ...
            stepCounter, sum(done), nAgents);
    end

    if stagnationCounter > 50
        warning('Repeated stagnation detected. Stopping early.');
        break;
    end
end

if stepCounter > P.max_dynamic_steps
    warning('Dynamic A* reached P.max_dynamic_steps. Some agents may not have reached goal.');
end

nFrames = numel(allMaps);

fprintf('    dynamic A* frames added: %d\n', stepCounter - 1);
fprintf('    agents reached goal: %d / %d\n', sum(done), nAgents);
fprintf('    total output frames: %d\n', nFrames);

%% SHOW INTEGRATED TRANSITION FIGURE

if P.show_figure
    fig = figure('Name', 'Integrated Cardinality + Dynamic Hungarian A* Transition', ...
        'Color', 'w', 'Position', [80 80 900 760]);

    ax = axes(fig);

    if P.save_video
        vw = VideoWriter(fullfile(P.out_dir, P.video_name), 'MPEG-4');
        vw.FrameRate = P.frame_rate;
        open(vw);
    end

    goalXY = cellToXY(Sg, P);

    for k = 1:nFrames
        currentMap = allMaps{k};
        activeCells = mapToCells(currentMap, P.occupied_value);
        activeXY = cellToXY(activeCells, P);

        fileIndex = P.output_start_index + k - 1;

        cla(ax);
        setupAxes(ax, P);

        if P.show_grid
            drawGrid(ax, P, COLOR);
        end

        % Show final goal state in transparent green.
        if ~isempty(goalXY)
            scatter(ax, goalXY(:,1), goalXY(:,2), ...
                P.marker_size, COLOR.goal, 'filled', ...
                'MarkerFaceAlpha', 0.22, ...
                'MarkerEdgeColor', 'none');
        end

        if k < matchedStartFrame || isempty(agentCellsByFrame{k})

            % Before cardinality completion, identity is undefined.
            if ~isempty(activeXY)
                scatter(ax, activeXY(:,1), activeXY(:,2), ...
                    P.marker_size, COLOR.agent, 'filled', ...
                    'MarkerFaceAlpha', 0.95, ...
                    'MarkerEdgeColor', 'k');
            end

        else

            % After cardinality completion, identity is fixed.
            currentAgentCells_k = agentCellsByFrame{k};
            currentAgentTraj_k = agentTrajByFrame{k};

            for i = 1:nAgents
                currentXY = cellToXY(currentAgentCells_k(i,:), P);

                % Assigned final target of agent i.
                goal_i_xy = cellToXY(assignedGoalCells(i,:), P);
                scatter(ax, goal_i_xy(1), goal_i_xy(2), ...
                    P.marker_size * 0.95, agentColors(i,:), ...
                    'o', ...
                    'LineWidth', 1.5, ...
                    'MarkerFaceColor', 'none', ...
                    'MarkerEdgeColor', agentColors(i,:));

                % Actual accumulated trajectory.
                trailXY = cellToXY(currentAgentTraj_k{i}, P);

                if P.show_agent_trajectories && size(trailXY,1) >= 2
                    plot(ax, trailXY(:,1), trailXY(:,2), '-', ...
                        'Color', agentColors(i,:), ...
                        'LineWidth', P.trajectory_line_width);
                end

                % Current agent position.
                scatter(ax, currentXY(1), currentXY(2), ...
                    P.marker_size, agentColors(i,:), 'filled', ...
                    'MarkerFaceAlpha', 0.95, ...
                    'MarkerEdgeColor', 'k');

                if P.show_agent_labels
                    text(ax, currentXY(1), currentXY(2), sprintf('%d', i), ...
                        'HorizontalAlignment', 'center', ...
                        'VerticalAlignment', 'middle', ...
                        'FontSize', 8, ...
                        'FontWeight', 'bold', ...
                        'Color', 'w');
                end
            end
        end

        % Centroid.
        if ~isempty(activeXY)
            centroidXY = mean(activeXY, 1);

            scatter(ax, centroidXY(1), centroidXY(2), ...
                130, COLOR.centroid, 'filled', ...
                'MarkerFaceAlpha', 0.90, ...
                'MarkerEdgeColor', 'k');
        end

        title(ax, sprintf('%s | %03d.csv | frame %d/%d | active = %d / goal = %d', ...
            phaseLabels{k}, fileIndex, k, nFrames, size(activeCells,1), nGoal), ...
            'FontSize', 13, 'FontWeight', 'bold');

        drawnow;

        if P.save_video
            writeVideo(vw, getframe(fig));
        end

        pause(P.anim_pause);
    end

    if P.save_video
        close(vw);
        fprintf('\nSaved preview video to %s\n', fullfile(P.out_dir, P.video_name));
    end
end

%% SAVE CSV SEQUENCE

fprintf('\n[6] Saving numbered CSV sequence...\n');

firstIndex = P.output_start_index;
lastIndex  = P.output_start_index + nFrames - 1;

nDigits = max(3, ceil(log10(lastIndex + 1)));

for k = 1:nFrames
    fileIndex = P.output_start_index + k - 1;

    fname = sprintf(['%0', num2str(nDigits), 'd.csv'], fileIndex);
    payloadOut = physicalMapToPayload(allMaps{k}, P);

    writematrix(payloadOut, fullfile(P.out_dir, fname));
end

fprintf('    saved files: %s.csv to %s.csv\n', ...
    sprintf(['%0', num2str(nDigits), 'd'], firstIndex), ...
    sprintf(['%0', num2str(nDigits), 'd'], lastIndex));

fprintf('    saved to folder: %s\n', P.out_dir);
fprintf('\nDone.\n');

%% ============================================================
%% EXPANSION / CONTRACTION OPERATORS
%% ============================================================

function cellsOut = addNearestEmptyCellToCentroid(cellsIn, P)
    allCells = allCellsRC(P.N);

    if isempty(cellsIn)
        centroidXY = mean(cellToXY(allCells, P), 1);
    else
        activeXY = cellToXY(cellsIn, P);
        centroidXY = mean(activeXY, 1);
    end

    occupiedIds = cellToLinearIndex(cellsIn, P.N);
    allIds = cellToLinearIndex(allCells, P.N);

    isEmpty = ~ismember(allIds, occupiedIds);
    emptyCells = allCells(isEmpty, :);

    if isempty(emptyCells)
        error('No empty cell remains for expansion.');
    end

    emptyXY = cellToXY(emptyCells, P);
    d2 = sum((emptyXY - centroidXY).^2, 2);

    sortTable = [d2, emptyCells(:,1), emptyCells(:,2)];
    [~, idx] = sortrows(sortTable, [1 2 3]);

    newCell = emptyCells(idx(1), :);

    cellsOut = sortrows([cellsIn; newCell], [1 2]);
end

function cellsOut = removeFarthestOccupiedCellFromCentroid(cellsIn, P)
    if isempty(cellsIn)
        error('No occupied cell remains for contraction.');
    end

    if size(cellsIn,1) == 1
        cellsOut = zeros(0,2);
        return;
    end

    activeXY = cellToXY(cellsIn, P);
    centroidXY = mean(activeXY, 1);

    d2 = sum((activeXY - centroidXY).^2, 2);

    sortTable = [-d2, -cellsIn(:,1), -cellsIn(:,2)];
    [~, idx] = sortrows(sortTable, [1 2 3]);

    removeIdx = idx(1);

    cellsOut = cellsIn;
    cellsOut(removeIdx,:) = [];
    cellsOut = sortrows(cellsOut, [1 2]);
end

%% ============================================================
%% PAYLOAD MAP CONVERSION
%% ============================================================

function physicalMap = payloadToPhysicalMap(payload, P)
    physicalMap = zeros(P.N, P.N);

    for physicalCol0 = 0:(P.N-1)
        csvRow = physicalCol0 + 1;

        for physicalRow0 = 0:(P.N-1)
            csvCol = P.N - physicalRow0;
            physicalMap(physicalRow0 + 1, physicalCol0 + 1) = payload(csvRow, csvCol);
        end
    end
end

function payload = physicalMapToPayload(physicalMap, P)
    payload = zeros(P.N, P.N);

    for physicalCol0 = 0:(P.N-1)
        csvRow = physicalCol0 + 1;

        for physicalRow0 = 0:(P.N-1)
            csvCol = P.N - physicalRow0;
            payload(csvRow, csvCol) = physicalMap(physicalRow0 + 1, physicalCol0 + 1);
        end
    end
end

%% ============================================================
%% MAP AND CELL UTILITIES
%% ============================================================

function cells = mapToCells(map, occupiedValue)
    [r, c] = find(abs(map) == occupiedValue);
    cells = sortrows([r, c], [1 2]);
end

function map = cellsToMap(cells, Nrow, Ncol, occupiedValue, emptyValue)
    map = emptyValue * ones(Nrow, Ncol);

    for k = 1:size(cells,1)
        r = cells(k,1);
        c = cells(k,2);

        if r < 1 || r > Nrow || c < 1 || c > Ncol
            error('Cell [%d, %d] is outside the grid.', r, c);
        end

        map(r,c) = occupiedValue;
    end
end

function ids = cellToLinearIndex(cells, N)
    if isempty(cells)
        ids = zeros(0,1);
        return;
    end

    r = cells(:,1);
    c = cells(:,2);

    ids = (c - 1) * N + r;
end

%% ============================================================
%% ASSIGNMENT COSTS
%% ============================================================

function [assignment, C] = assignByEuclideanDistance(S0, Sg, P)
    n = size(S0, 1);
    assert(n == size(Sg,1), 'S0 and Sg must have equal cardinality.');

    xy0 = cellToXY(S0, P);
    xyg = cellToXY(Sg, P);

    C = zeros(n, n);

    for i = 1:n
        for j = 1:n
            d = xy0(i,:) - xyg(j,:);
            C(i,j) = d*d.';
        end
    end

    assignment = munkres(C);
end

function [assignment, C] = assignByAstarLength(S0, Sg, P)
    n = size(S0, 1);
    assert(n == size(Sg,1), 'S0 and Sg must have equal cardinality.');

    C = zeros(n, n);

    for i = 1:n
        for j = 1:n
            path = astarStaggered(S0(i,:), Sg(j,:), P);
            C(i,j) = size(path,1) - 1;
        end
    end

    assignment = munkres(C);
end

%% ============================================================
%% A STAR
%% ============================================================

function path = astarStaggered(startCell, goalCell, P)
    if isequal(startCell, goalCell)
        path = startCell;
        return;
    end

    startId = cellToId(startCell, P);
    goalId  = cellToId(goalCell,  P);
    nNodes  = P.N * P.N;

    openIds = startId;
    isOpen = false(nNodes,1);
    isOpen(startId) = true;

    isClosed = false(nNodes,1);
    parent = zeros(nNodes,1,'int32');

    gScore = inf(nNodes,1);
    fScore = inf(nNodes,1);

    gScore(startId) = 0;
    fScore(startId) = heuristic(startCell, goalCell, P);

    while ~isempty(openIds)
        [~, localIdx] = min(fScore(openIds));
        currentId = openIds(localIdx);
        currentCell = idToCell(currentId, P);

        if currentId == goalId
            path = reconstructPath(parent, currentId, P);
            return;
        end

        openIds(localIdx) = [];
        isOpen(currentId) = false;
        isClosed(currentId) = true;

        nb = staggeredNeighbors(currentCell, P);

        for k = 1:size(nb,1)
            nbId = cellToId(nb(k,:), P);

            if isClosed(nbId)
                continue;
            end

            stepCost = heuristic(currentCell, nb(k,:), P);
            tentativeG = gScore(currentId) + stepCost;

            if tentativeG < gScore(nbId)
                parent(nbId) = currentId;
                gScore(nbId) = tentativeG;
                fScore(nbId) = tentativeG + heuristic(nb(k,:), goalCell, P);

                if ~isOpen(nbId)
                    openIds(end+1) = nbId;
                    isOpen(nbId) = true;
                end
            end
        end
    end

    warning('A* failed from [%d,%d] to [%d,%d]. Returning endpoints only.', ...
        startCell(1), startCell(2), goalCell(1), goalCell(2));

    path = [startCell; goalCell];
end

function path = astarStaggeredBlocked(startCell, goalCell, blockedCells, P)

    if isequal(startCell, goalCell)
        path = startCell;
        return;
    end

    nNodes = P.N * P.N;
    blocked = false(nNodes,1);

    for k = 1:size(blockedCells,1)
        id = cellToId(blockedCells(k,:), P);

        if P.allow_goal_as_temporary_passage
            if ~isequal(blockedCells(k,:), goalCell)
                blocked(id) = true;
            end
        else
            blocked(id) = true;
        end
    end

    startId = cellToId(startCell, P);
    goalId  = cellToId(goalCell,  P);

    blocked(startId) = false;
    blocked(goalId)  = false;

    openIds = startId;
    isOpen = false(nNodes,1);
    isOpen(startId) = true;

    isClosed = false(nNodes,1);
    parent = zeros(nNodes,1,'int32');

    gScore = inf(nNodes,1);
    fScore = inf(nNodes,1);

    gScore(startId) = 0;
    fScore(startId) = heuristic(startCell, goalCell, P);

    while ~isempty(openIds)
        [~, localIdx] = min(fScore(openIds));
        currentId = openIds(localIdx);
        currentCell = idToCell(currentId, P);

        if currentId == goalId
            path = reconstructPath(parent, currentId, P);
            return;
        end

        openIds(localIdx) = [];
        isOpen(currentId) = false;
        isClosed(currentId) = true;

        nb = staggeredNeighbors(currentCell, P);

        for kk = 1:size(nb,1)
            nbId = cellToId(nb(kk,:), P);

            if blocked(nbId) || isClosed(nbId)
                continue;
            end

            stepCost = heuristic(currentCell, nb(kk,:), P);
            tentativeG = gScore(currentId) + stepCost;

            if tentativeG < gScore(nbId)
                parent(nbId) = currentId;
                gScore(nbId) = tentativeG;
                fScore(nbId) = tentativeG + heuristic(nb(kk,:), goalCell, P);

                if ~isOpen(nbId)
                    openIds(end+1) = nbId;
                    isOpen(nbId) = true;
                end
            end
        end
    end

    % If blocked A* fails, stay.
    path = startCell;
end

function h = heuristic(aCell, bCell, P)
    a = cellToXY(aCell, P);
    b = cellToXY(bCell, P);
    h = hypot(a(1)-b(1), a(2)-b(2));
end

function path = reconstructPath(parent, currentId, P)
    ids = currentId;

    while parent(currentId) ~= 0
        currentId = parent(currentId);
        ids = [currentId; ids];
    end

    path = zeros(numel(ids), 2);

    for k = 1:numel(ids)
        path(k,:) = idToCell(ids(k), P);
    end
end

function nb = staggeredNeighbors(cellRC, P)
    r = cellRC(1);
    c = cellRC(2);

    nb = [ ...
        r, c-1;
        r, c+1];

    if mod(r-1, 2) == 0
        diagCols = [c-1, c];
    else
        diagCols = [c, c+1];
    end

    nb = [ ...
        nb;
        r-1, diagCols(1);
        r-1, diagCols(2);
        r+1, diagCols(1);
        r+1, diagCols(2)];

    valid = nb(:,1) >= 1 & nb(:,1) <= P.N & ...
            nb(:,2) >= 1 & nb(:,2) <= P.N;

    nb = nb(valid,:);
end

%% ============================================================
%% DYNAMIC REPLANNING HELPERS
%% ============================================================

function tf = isCellInSet(cellRC, cellSet)
    if isempty(cellSet)
        tf = false;
        return;
    end

    tf = any(cellSet(:,1) == cellRC(1) & cellSet(:,2) == cellRC(2));
end

function [nextCells, moved] = forceOneEscapeMove(currentCells, goalCells, P)

    nAgents = size(currentCells, 1);
    nextCells = currentCells;
    moved = false;

    occupiedIds = zeros(nAgents, 1);
    for i = 1:nAgents
        occupiedIds(i) = cellToId(currentCells(i,:), P);
    end

    % Move the agent farthest from its goal first.
    distToGoal = zeros(nAgents, 1);
    for i = 1:nAgents
        xy = cellToXY(currentCells(i,:), P);
        gy = cellToXY(goalCells(i,:), P);
        distToGoal(i) = norm(xy - gy);
    end

    [~, order] = sort(distToGoal, 'descend');

    for idx = 1:nAgents
        i = order(idx);

        if isequal(currentCells(i,:), goalCells(i,:))
            continue;
        end

        nb = staggeredNeighbors(currentCells(i,:), P);

        bestCell = [];
        bestScore = inf;

        otherOccupiedIds = occupiedIds;
        otherOccupiedIds(i) = [];

        for k = 1:size(nb,1)
            nbId = cellToId(nb(k,:), P);

            if ismember(nbId, otherOccupiedIds)
                continue;
            end

            nbXY = cellToXY(nb(k,:), P);
            gXY  = cellToXY(goalCells(i,:), P);
            score = norm(nbXY - gXY);

            if score < bestScore
                bestScore = score;
                bestCell = nb(k,:);
            end
        end

        if ~isempty(bestCell)
            nextCells(i,:) = bestCell;
            moved = true;
            return;
        end
    end
end

%% ============================================================
%% COORDINATE UTILITIES
%% ============================================================

function xy = cellToXY(cells, P)
    if isempty(cells)
        xy = zeros(0,2);
        return;
    end

    r0 = cells(:,1) - 1;
    c0 = cells(:,2) - 1;

    x = c0 + 0.5 * mod(r0, 2);
    y = r0 * P.row_spacing;

    xy = [x, y];
end

function id = cellToId(cellRC, P)
    r = cellRC(1);
    c = cellRC(2);
    id = (c-1)*P.N + r;
end

function cellRC = idToCell(id, P)
    r = mod(id-1, P.N) + 1;
    c = floor((id-1)/P.N) + 1;
    cellRC = [r, c];
end

function cells = allCellsRC(N)
    cells = zeros(N*N, 2);
    k = 0;

    for r = 1:N
        for c = 1:N
            k = k + 1;
            cells(k,:) = [r, c];
        end
    end
end

%% ============================================================
%% PLOTTING UTILITIES
%% ============================================================

function setupAxes(ax, P)
    hold(ax, 'on');
    axis(ax, 'equal');
    box(ax, 'on');
    set(ax, 'YDir', 'reverse');

    xlim(ax, [-0.7, P.N + 1.2]);
    ylim(ax, [-0.7, (P.N-1)*P.row_spacing + 1.2]);

    xlabel(ax, 'staggered x');
    ylabel(ax, 'staggered y');
end

function drawGrid(ax, P, COLOR)
    cells = allCellsRC(P.N);
    xy = cellToXY(cells, P);

    scatter(ax, xy(:,1), xy(:,2), 16, COLOR.grid_node, 'filled', ...
        'MarkerFaceAlpha', 0.60, ...
        'MarkerEdgeColor', 'none');

    for k = 1:size(cells,1)
        rc = cells(k,:);
        nb = staggeredNeighbors(rc, P);
        xy0 = cellToXY(rc, P);

        for j = 1:size(nb,1)
            if cellToId(nb(j,:), P) <= cellToId(rc, P)
                continue;
            end

            xy1 = cellToXY(nb(j,:), P);

            plot(ax, [xy0(1), xy1(1)], [xy0(2), xy1(2)], ...
                '-', 'Color', COLOR.grid_edge, 'LineWidth', 0.45);
        end
    end
end

%% ============================================================
%% MUNKRES
%% ============================================================

function assignment = munkres(costMat)
    [n, m] = size(costMat);
    assert(n == m, 'munkres: cost matrix must be square.');

    C = double(costMat);
    tol = 1e-12;

    C = C - min(C, [], 2);
    C = C - min(C, [], 1);

    starred = false(n,n);
    primed  = false(n,n);
    rowCovered = false(n,1);
    colCovered = false(n,1);

    for i = 1:n
        for j = 1:n
            if abs(C(i,j)) <= tol && ...
                    ~any(starred(i,:)) && ...
                    ~any(starred(:,j))
                starred(i,j) = true;
            end
        end
    end

    colCovered = any(starred, 1).';

    iterLimit = max(1000, n*n*20);
    iter = 0;

    while sum(colCovered) < n && iter < iterLimit
        iter = iter + 1;
        foundZero = false;

        for i = 1:n
            if rowCovered(i)
                continue;
            end

            for j = 1:n
                if colCovered(j)
                    continue;
                end

                if abs(C(i,j)) <= tol
                    primed(i,j) = true;
                    starCol = find(starred(i,:), 1);

                    if isempty(starCol)
                        path = [i, j];

                        while true
                            starRow = find(starred(:, path(end,2)), 1);

                            if isempty(starRow)
                                break;
                            end

                            path(end+1,:) = [starRow, path(end,2)];

                            primeCol = find(primed(starRow,:), 1);
                            path(end+1,:) = [starRow, primeCol];
                        end

                        for k = 1:size(path,1)
                            starred(path(k,1), path(k,2)) = ...
                                ~starred(path(k,1), path(k,2));
                        end

                        primed(:) = false;
                        rowCovered(:) = false;
                        colCovered = any(starred, 1).';

                        foundZero = true;
                        break;

                    else
                        rowCovered(i) = true;
                        colCovered(starCol) = false;

                        foundZero = true;
                        break;
                    end
                end
            end

            if foundZero
                break;
            end
        end

        if ~foundZero
            uncovered = C(~rowCovered, ~colCovered);
            mn = min(uncovered(:));

            C(rowCovered, :) = C(rowCovered, :) + mn;
            C(:, ~colCovered) = C(:, ~colCovered) - mn;
        end
    end

    if iter >= iterLimit
        warning('munkres reached iteration limit.');
    end

    assignment = zeros(1,n);

    for i = 1:n
        j = find(starred(i,:), 1);

        if isempty(j)
            error('munkres failed to assign row %d.', i);
        end

        assignment(i) = j;
    end
end