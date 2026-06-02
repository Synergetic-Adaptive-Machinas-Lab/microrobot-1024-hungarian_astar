clear; close all; clc;

%% USER SETTINGS

P.N = 32;
P.occupied_value = 7;
P.empty_value = 0;

P.row_spacing = sqrt(3)/2;

P.assignment_cost = 'euclidean';   % 'euclidean' or 'astar_length'
P.check_collisions = true;

P.show_figure = true;
P.show_grid = true;
P.marker_size = 85;
P.anim_pause = 0;

P.save_video = true;
P.video_name = 'multistate_transition_preview.mp4';
P.frame_rate = 15;

% Hold frames for user-input states only.
% Example: if this is 5, each selected START/GOAL state is repeated
% 5 times in the final CSV sequence.
P.input_state_hold_frames = 5;

P.show_agent_trajectories = true;
P.show_agent_labels = true;
P.trajectory_line_width = 2.0;

COLOR.agent = [0.85 0.15 0.10];
COLOR.goal = [0.00 0.65 0.20];
COLOR.centroid = [0.10 0.20 1.00];
COLOR.grid_node = [0.78 0.78 0.78];
COLOR.grid_edge = [0.90 0.90 0.90];

%% SEQUENTIAL FILE INPUT

fprintf('\n[1] Select CSV states sequentially...\n');

stateFiles = {};
stateNames = {};

% Select start CSV
[startFile, startPath] = uigetfile('*.csv', 'Select START state CSV');
if isequal(startFile, 0)
    error('Start CSV selection canceled.');
end

stateFiles{end+1,1} = fullfile(startPath, startFile);
stateNames{end+1,1} = startFile;

fprintf('    START: %s\n', startFile);

% Select goal CSVs sequentially
goalCount = 0;

while true
    titleStr = sprintf('Select GOAL %d CSV, or Cancel to finish', goalCount + 1);
    [goalFile, goalPath] = uigetfile('*.csv', titleStr);

    if isequal(goalFile, 0)
        break;
    end

    goalCount = goalCount + 1;

    stateFiles{end+1,1} = fullfile(goalPath, goalFile);
    stateNames{end+1,1} = goalFile;

    fprintf('    GOAL %d: %s\n', goalCount, goalFile);
end

assert(goalCount >= 1, ...
    'You must select at least one goal CSV after the start CSV.');

nStates = numel(stateFiles);

fprintf('\n    total selected states: %d\n', nStates);
fprintf('    interpreted order:\n');

for k = 1:nStates
    if k == 1
        fprintf('      %02d: START  = %s\n', k, stateNames{k});
    else
        fprintf('      %02d: GOAL %d = %s\n', k, k-1, stateNames{k});
    end
end

%% OUTPUT FOLDER

outDir = uigetdir(pwd, 'Select output folder for generated CSV sequence');
if isequal(outDir, 0)
    error('Output folder selection canceled.');
end

P.out_dir = outDir;

%% OUTPUT START INDEX

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

%% READ SELECTED STATE CSV FILES

fprintf('\n[2] Reading selected state CSV files...\n');

stateMaps = cell(nStates, 1);
stateCells = cell(nStates, 1);

for k = 1:nStates
    payload = readmatrix(stateFiles{k});

    assert(isequal(size(payload), [P.N, P.N]), ...
        'CSV %s must be %d x %d.', stateNames{k}, P.N, P.N);

    physicalMap = payloadToPhysicalMap(payload, P);
    cells = mapToCells(physicalMap, P.occupied_value);

    stateMaps{k} = physicalMap;
    stateCells{k} = cells;

    fprintf('    state %02d active cells: %d | %s\n', ...
        k, size(cells,1), stateNames{k});

end

%% BUILD FULL MULTISTATE SEQUENCE

fprintf('\n[3] Building multistate transition sequence...\n');

allMaps = {};
phaseLabels = {};
segmentIds = {};

agentCellsByFrame = {};
agentTrajByFrame = {};
goalCellsByFrame = {};
assignedGoalByFrame = {};
agentColorsByFrame = {};

for seg = 1:(nStates-1)

    fprintf('\n============================================================\n');
    fprintf('Segment %d / %d: %s -> %s\n', ...
        seg, nStates-1, stateNames{seg}, stateNames{seg+1});
    fprintf('============================================================\n');

    S_start = stateCells{seg};
    S_goal  = stateCells{seg+1};

    [segMaps, segLabels, segAgentCells, segAgentTraj, ...
        segGoalCells, segAssignedGoals, segAgentColors] = ...
        buildOneTransition(S_start, S_goal, seg, P);

    % Avoid duplicated boundary frame.
    % Segment 1 keeps the original start.
    % Segment 2 and after remove the first frame because it equals previous goal.
    if seg == 1
        idxStart = 1;
    else
        idxStart = 2;
    end

    for k = idxStart:numel(segMaps)

        % Repeat only user-input state frames:
        %   k == 1             : selected source state of this segment
        %   k == numel(segMaps): selected target state of this segment
        %
        % For seg > 1, k == 1 is skipped by idxStart = 2, so intermediate
        % selected GOAL states are held only once as the final frame of
        % the previous segment.
        isInputStateFrame = (k == 1) || (k == numel(segMaps));

        if isInputStateFrame
            nRepeat = P.input_state_hold_frames;
        else
            nRepeat = 1;
        end

        for rep = 1:nRepeat
            allMaps{end+1,1} = segMaps{k};

            if isInputStateFrame
                phaseLabels{end+1,1} = sprintf('%s_hold_%d', segLabels{k}, rep);
            else
                phaseLabels{end+1,1} = segLabels{k};
            end

            segmentIds{end+1,1} = seg;

            agentCellsByFrame{end+1,1} = segAgentCells{k};
            agentTrajByFrame{end+1,1} = segAgentTraj{k};
            goalCellsByFrame{end+1,1} = segGoalCells{k};
            assignedGoalByFrame{end+1,1} = segAssignedGoals{k};
            agentColorsByFrame{end+1,1} = segAgentColors{k};
        end
    end
end

nFrames = numel(allMaps);

fprintf('\n[4] Full multistate sequence built.\n');
fprintf('    total selected states: %d\n', nStates);
fprintf('    total transitions: %d\n', nStates - 1);
fprintf('    total output frames: %d\n', nFrames);

%% SHOW / SAVE VIDEO

if P.show_figure
    fprintf('\n[5] Showing integrated multistate transition figure...\n');

    fig = figure('Name', 'Multistate Cardinality + Hungarian A* Transition', ...
        'Color', 'w', 'Position', [80 80 920 780]);

    ax = axes(fig);

    if P.save_video
        vw = VideoWriter(fullfile(P.out_dir, P.video_name), 'MPEG-4');
        vw.FrameRate = P.frame_rate;
        open(vw);
    end

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

        goalCells = goalCellsByFrame{k};

        if ~isempty(goalCells)
            goalXY = cellToXY(goalCells, P);

            scatter(ax, goalXY(:,1), goalXY(:,2), ...
                P.marker_size, COLOR.goal, 'filled', ...
                'MarkerFaceAlpha', 0.20, ...
                'MarkerEdgeColor', 'none');
        end

        currentAgentCells = agentCellsByFrame{k};
        currentAgentTraj = agentTrajByFrame{k};
        assignedGoalCells = assignedGoalByFrame{k};
        agentColors = agentColorsByFrame{k};

        if isempty(currentAgentCells)

            % Cardinality phase before identity is fixed.
            if ~isempty(activeXY)
                scatter(ax, activeXY(:,1), activeXY(:,2), ...
                    P.marker_size, COLOR.agent, 'filled', ...
                    'MarkerFaceAlpha', 0.95, ...
                    'MarkerEdgeColor', 'k');
            end

        else

            nAgents = size(currentAgentCells, 1);

            for i = 1:nAgents
                currentXY = cellToXY(currentAgentCells(i,:), P);

                goal_i_xy = cellToXY(assignedGoalCells(i,:), P);

                scatter(ax, goal_i_xy(1), goal_i_xy(2), ...
                    P.marker_size * 0.95, agentColors(i,:), ...
                    'o', ...
                    'LineWidth', 1.5, ...
                    'MarkerFaceColor', 'none', ...
                    'MarkerEdgeColor', agentColors(i,:));

                if P.show_agent_trajectories
                    trailXY = cellToXY(currentAgentTraj{i}, P);

                    if size(trailXY,1) >= 2
                        plot(ax, trailXY(:,1), trailXY(:,2), '-', ...
                            'Color', agentColors(i,:), ...
                            'LineWidth', P.trajectory_line_width);
                    end
                end

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

        if ~isempty(activeXY)
            centroidXY = mean(activeXY, 1);

            scatter(ax, centroidXY(1), centroidXY(2), ...
                130, COLOR.centroid, 'filled', ...
                'MarkerFaceAlpha', 0.90, ...
                'MarkerEdgeColor', 'k');
        end

        title(ax, sprintf('segment %d/%d | %s | %03d.csv | frame %d/%d | active = %d', ...
            segmentIds{k}, nStates-1, phaseLabels{k}, fileIndex, ...
            k, nFrames, size(activeCells,1)), ...
            'FontSize', 13, 'FontWeight', 'bold');

        drawnow;

        if P.save_video
            writeVideo(vw, getframe(fig));
        end

        pause(P.anim_pause);
    end

    if P.save_video
        close(vw);
        fprintf('    saved preview video to: %s\n', fullfile(P.out_dir, P.video_name));
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
%% ONE TRANSITION BUILDER
%% ============================================================

function [segMaps, segLabels, segAgentCells, segAgentTraj, ...
    segGoalCells, segAssignedGoals, segAgentColors] = ...
    buildOneTransition(S_start_original, S_goal, segId, P)

    segMaps = {};
    segLabels = {};
    segAgentCells = {};
    segAgentTraj = {};
    segGoalCells = {};
    segAssignedGoals = {};
    segAgentColors = {};

    nStart = size(S_start_original, 1);
    nGoal  = size(S_goal, 1);

    fprintf('    start active cells: %d\n', nStart);
    fprintf('    goal active cells:  %d\n', nGoal);

    %% PHASE 1: CARDINALITY MATCHING

    currentCells = S_start_original;

    segMaps{end+1,1} = cellsToMap(currentCells, P.N, P.N, ...
        P.occupied_value, P.empty_value);

    segLabels{end+1,1} = sprintf('seg%d_start', segId);
    segAgentCells{end+1,1} = [];
    segAgentTraj{end+1,1} = [];
    segGoalCells{end+1,1} = S_goal;
    segAssignedGoals{end+1,1} = [];
    segAgentColors{end+1,1} = [];

    if nStart < nGoal
        cardinalityMode = 'expansion';

        while size(currentCells,1) < nGoal
            currentCells = addNearestEmptyCellToCentroid(currentCells, P);

            segMaps{end+1,1} = cellsToMap(currentCells, P.N, P.N, ...
                P.occupied_value, P.empty_value);

            segLabels{end+1,1} = sprintf('seg%d_expansion', segId);
            segAgentCells{end+1,1} = [];
            segAgentTraj{end+1,1} = [];
            segGoalCells{end+1,1} = S_goal;
            segAssignedGoals{end+1,1} = [];
            segAgentColors{end+1,1} = [];
        end

    elseif nStart > nGoal
        cardinalityMode = 'contraction';

        while size(currentCells,1) > nGoal
            currentCells = removeFarthestOccupiedCellFromCentroid(currentCells, P);

            segMaps{end+1,1} = cellsToMap(currentCells, P.N, P.N, ...
                P.occupied_value, P.empty_value);

            segLabels{end+1,1} = sprintf('seg%d_contraction', segId);
            segAgentCells{end+1,1} = [];
            segAgentTraj{end+1,1} = [];
            segGoalCells{end+1,1} = S_goal;
            segAssignedGoals{end+1,1} = [];
            segAgentColors{end+1,1} = [];
        end

    else
        cardinalityMode = 'no_change';
    end

    S_start_matched = currentCells;
    nAgents = size(S_start_matched, 1);

    fprintf('    cardinality mode: %s\n', cardinalityMode);
    fprintf('    matched active cells: %d\n', nAgents);

    assert(nAgents == nGoal, ...
        'Cardinality matching failed: matched=%d, goal=%d.', nAgents, nGoal);

    matchedStartFrame = numel(segMaps);

    %% PHASE 2: HUNGARIAN ASSIGNMENT

    switch lower(P.assignment_cost)
        case 'euclidean'
            [assignment, C] = assignByEuclideanDistance(S_start_matched, S_goal, P);

        case 'astar_length'
            [assignment, C] = assignByAstarLength(S_start_matched, S_goal, P);

        otherwise
            error('Unknown P.assignment_cost: %s', P.assignment_cost);
    end

    totalCost = sum(C(sub2ind(size(C), 1:nAgents, assignment)));

    fprintf('    assignment cost type: %s\n', P.assignment_cost);
    fprintf('    total Hungarian cost: %.6f\n', totalCost);

    assignedGoalCells = S_goal(assignment, :);
    agentColors = lines(max(nAgents, 1));

    %% PHASE 3: A* PATHS

    paths = cell(nAgents, 1);

    for i = 1:nAgents
        paths{i} = astarStaggered(S_start_matched(i,:), assignedGoalCells(i,:), P);
    end

    pathLengths = cellfun(@(p) size(p,1)-1, paths);

    fprintf('    mean A* path length: %.3f edges\n', mean(pathLengths));
    fprintf('    max  A* path length: %d edges\n', max(pathLengths));

    %% STORE IDENTITY AT MATCHED START FRAME

    agentTraj = cell(nAgents, 1);

    for i = 1:nAgents
        agentTraj{i} = S_start_matched(i,:);
    end

    segAgentCells{matchedStartFrame,1} = S_start_matched;
    segAgentTraj{matchedStartFrame,1} = agentTraj;
    segAssignedGoals{matchedStartFrame,1} = assignedGoalCells;
    segAgentColors{matchedStartFrame,1} = agentColors;

    %% PHASE 4: A* TRANSIENT FRAMES

    maxSteps = max(cellfun(@(p) size(p,1), paths));

    % step = 1 is S_start_matched, already saved.
    for step = 2:maxSteps
        currentAstarCells = zeros(nAgents, 2);

        for i = 1:nAgents
            path_i = paths{i};

            idx = min(step, size(path_i, 1));
            currentAstarCells(i,:) = path_i(idx,:);

            trajIdx = min(step, size(path_i, 1));
            agentTraj{i} = path_i(1:trajIdx, :);
        end

        segMaps{end+1,1} = cellsToMap(currentAstarCells, P.N, P.N, ...
            P.occupied_value, P.empty_value);

        segLabels{end+1,1} = sprintf('seg%d hungarian astar', segId);

        segAgentCells{end+1,1} = currentAstarCells;
        segAgentTraj{end+1,1} = agentTraj;
        segGoalCells{end+1,1} = S_goal;
        segAssignedGoals{end+1,1} = assignedGoalCells;
        segAgentColors{end+1,1} = agentColors;

        if P.check_collisions
            nUnique = size(unique(currentAstarCells, 'rows'), 1);

            if nUnique < nAgents
                warning('Segment %d A* frame %d has overlapping agents: unique=%d, agents=%d.', ...
                    segId, step-1, nUnique, nAgents);
            end
        end
    end

    fprintf('    segment frames: %d\n', numel(segMaps));
end

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