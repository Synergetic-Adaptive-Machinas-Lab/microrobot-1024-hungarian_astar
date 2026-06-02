clear; close all; clc;

%% USER SETTINGS / INPUTS

[filename1, pathname1] = uigetfile('*.csv', 'Select start state CSV');
if isequal(filename1, 0)
    error('Start CSV selection canceled.');
end

outDir = uigetdir(pwd, 'Select output folder');
if isequal(outDir, 0)
    error('Output folder selection canceled.');
end

P.start_csv = fullfile(pathname1, filename1);
P.out_dir   = outDir;

P.N = 32;
P.occupied_value = 7;
P.empty_value = 0;

P.row_spacing = sqrt(3)/2;

P.show_figure = true;
P.show_grid = true;
P.marker_size = 85;
P.anim_pause = 0.08;

P.save_video = false;
P.video_name = 'expansion_contraction_preview.mp4';
P.frame_rate = 15;

COLOR.agent = [0.85 0.15 0.10];
COLOR.centroid = [0.10 0.20 1.00];
COLOR.grid_node = [0.78 0.78 0.78];
COLOR.grid_edge = [0.90 0.90 0.90];

%% COMMAND WINDOW INPUTS

goalCount = input('Enter goal number of active cells, e.g., 12: ');

assert(isscalar(goalCount) && ...
       isnumeric(goalCount) && ...
       isfinite(goalCount) && ...
       goalCount >= 0 && ...
       goalCount <= P.N * P.N && ...
       floor(goalCount) == goalCount, ...
       'goalCount must be an integer between 0 and 1024.');

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

%% READ START CSV

fprintf('\n[1] Reading start CSV...\n');

startPayload = readmatrix(P.start_csv);

assert(isequal(size(startPayload), [P.N, P.N]), ...
    'Start CSV must be %d x %d.', P.N, P.N);

startMap = payloadToPhysicalMap(startPayload, P);
currentCells = mapToCells(startMap, P.occupied_value);

startCount = size(currentCells, 1);

fprintf('    grid size: %d x %d\n', P.N, P.N);
fprintf('    start active cells: %d\n', startCount);
fprintf('    goal active cells:  %d\n', goalCount);

if startCount < goalCount
    mode = 'expansion';
elseif startCount > goalCount
    mode = 'contraction';
else
    mode = 'no_change';
end

fprintf('    transition mode: %s\n', mode);

%% GENERATE TRANSITION STATES

fprintf('\n[2] Generating transition states...\n');

maps = {};
counts = [];

maps{end+1,1} = cellsToMap(currentCells, P.N, P.N, ...
    P.occupied_value, P.empty_value);
counts(end+1,1) = size(currentCells,1);

switch mode
    case 'expansion'
        while size(currentCells,1) < goalCount
            currentCells = addNearestEmptyCellToCentroid(currentCells, P);

            maps{end+1,1} = cellsToMap(currentCells, P.N, P.N, ...
                P.occupied_value, P.empty_value);
            counts(end+1,1) = size(currentCells,1);
        end

    case 'contraction'
        while size(currentCells,1) > goalCount
            currentCells = removeFarthestOccupiedCellFromCentroid(currentCells, P);

            maps{end+1,1} = cellsToMap(currentCells, P.N, P.N, ...
                P.occupied_value, P.empty_value);
            counts(end+1,1) = size(currentCells,1);
        end

    case 'no_change'
        % Only save the initial state.
end

nFrames = numel(maps);

fprintf('    number of output states: %d\n', nFrames);
fprintf('    first active count: %d\n', counts(1));
fprintf('    last active count:  %d\n', counts(end));

%% SHOW FIGURE

if P.show_figure
    fig = figure('Name', 'Expansion / Contraction Preview', ...
        'Color', 'w', 'Position', [80 80 900 760]);

    ax = axes(fig);

    if P.save_video
        vw = VideoWriter(fullfile(P.out_dir, P.video_name), 'MPEG-4');
        vw.FrameRate = P.frame_rate;
        open(vw);
    end

    for k = 1:nFrames
        currentMap = maps{k};
        activeCells = mapToCells(currentMap, P.occupied_value);
        activeXY = cellToXY(activeCells, P);

        fileIndex = P.output_start_index + k - 1;

        cla(ax);
        setupAxes(ax, P);

        if P.show_grid
            drawGrid(ax, P, COLOR);
        end

        if ~isempty(activeXY)
            scatter(ax, activeXY(:,1), activeXY(:,2), ...
                P.marker_size, COLOR.agent, 'filled', ...
                'MarkerFaceAlpha', 0.95, ...
                'MarkerEdgeColor', 'k');

            centroidXY = mean(activeXY, 1);

            scatter(ax, centroidXY(1), centroidXY(2), ...
                140, COLOR.centroid, 'filled', ...
                'MarkerFaceAlpha', 0.95, ...
                'MarkerEdgeColor', 'k');
        end

        title(ax, sprintf('%s | %03d.csv | frame %d/%d | active = %d / goal = %d', ...
            mode, fileIndex, k, nFrames, size(activeCells,1), goalCount), ...
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

fprintf('\n[3] Saving numbered CSV sequence...\n');

firstIndex = P.output_start_index;
lastIndex  = P.output_start_index + nFrames - 1;

nDigits = max(3, ceil(log10(lastIndex + 1)));

for k = 1:nFrames
    fileIndex = P.output_start_index + k - 1;

    fname = sprintf(['%0', num2str(nDigits), 'd.csv'], fileIndex);
    payloadOut = physicalMapToPayload(maps{k}, P);

    writematrix(payloadOut, fullfile(P.out_dir, fname));
end

fprintf('    saved files: %s.csv to %s.csv\n', ...
    sprintf(['%0', num2str(nDigits), 'd'], firstIndex), ...
    sprintf(['%0', num2str(nDigits), 'd'], lastIndex));

fprintf('    saved to folder: %s\n', P.out_dir);
fprintf('\nDone.\n');

%% EXPANSION / CONTRACTION OPERATORS

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

    % Tie-breaking:
    % 1) smallest distance to centroid
    % 2) smaller row index
    % 3) smaller column index
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

    % Tie-breaking:
    % 1) largest distance from centroid
    % 2) larger row index
    % 3) larger column index
    sortTable = [-d2, -cellsIn(:,1), -cellsIn(:,2)];
    [~, idx] = sortrows(sortTable, [1 2 3]);

    removeIdx = idx(1);

    cellsOut = cellsIn;
    cellsOut(removeIdx,:) = [];
    cellsOut = sortrows(cellsOut, [1 2]);
end

%% PAYLOAD MAP CONVERSION

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

%% MAP AND CELL UTILITIES

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

%% COORDINATE UTILITIES

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

%% PLOTTING UTILITIES

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

function id = cellToId(cellRC, P)
    r = cellRC(1);
    c = cellRC(2);
    id = (c-1)*P.N + r;
end