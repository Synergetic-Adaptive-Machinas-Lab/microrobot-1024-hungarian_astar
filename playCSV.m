%% playCSV.m

clear; close all; clc;

%% USER SETTINGS

P.N = 32;
P.occupied_value = 7;

P.row_spacing = sqrt(3)/2;
P.marker_size = 85;
P.anim_pause = 0.08;

P.show_grid = true;
P.show_trajectory = true;

P.save_video = false;
P.video_name = 'csv_transition_player.mp4';
P.frame_rate = 15;

COLOR.agent = [0.85 0.15 0.10];
COLOR.grid_node = [0.78 0.78 0.78];
COLOR.grid_edge = [0.90 0.90 0.90];
COLOR.trajectory = [0.10 0.10 0.10];

%% SELECT CSV FOLDER

csvDir = uigetdir(pwd, 'Select folder containing numbered CSV files');

if isequal(csvDir, 0)
    error('CSV folder selection canceled.');
end

files = dir(fullfile(csvDir, '*.csv'));
names = {files.name};

isFrame = false(size(names));

for i = 1:numel(names)
    [~, base, ext] = fileparts(names{i});
    isFrame(i) = strcmp(ext, '.csv') && ~isempty(regexp(base, '^\d+$', 'once'));
end

files = files(isFrame);

if isempty(files)
    error('No numbered CSV files found. Expected files like 000.csv, 001.csv, 002.csv.');
end

frameNums = zeros(numel(files), 1);

for i = 1:numel(files)
    [~, base, ~] = fileparts(files(i).name);
    frameNums(i) = str2double(base);
end

[frameNums, idx] = sort(frameNums);
files = files(idx);

nFrames = numel(files);

fprintf('\nSelected folder: %s\n', csvDir);
fprintf('Number of frames: %d\n', nFrames);
fprintf('First frame: %s\n', files(1).name);
fprintf('Last frame:  %s\n', files(end).name);

%% INITIALIZE FIGURE

fig = figure('Name', '1024 Grid CSV Player with Trajectory', ...
    'Color', 'w', 'Position', [80 80 900 760]);

ax = axes(fig);

if P.save_video
    vw = VideoWriter(fullfile(csvDir, P.video_name), 'MPEG-4');
    vw.FrameRate = P.frame_rate;
    open(vw);
end

%% TRAJECTORY STORAGE

trajectoryXY = [];

%% PLAY CSV FRAMES

for k = 1:nFrames
    framePath = fullfile(csvDir, files(k).name);

    payload = readmatrix(framePath);

    assert(isequal(size(payload), [P.N, P.N]), ...
        'CSV frame %s is not %d x %d.', files(k).name, P.N, P.N);

    physicalMap = payloadToPhysicalMap(payload, P);
    activeCells = mapToCells(physicalMap, P.occupied_value);
    activeXY = cellToXY(activeCells, P);

    if ~isempty(activeXY)
        trajectoryXY = [trajectoryXY; activeXY];
    end

    cla(ax);
    setupAxes(ax, P);

    if P.show_grid
        drawGrid(ax, P, COLOR);
    end

    if P.show_trajectory && ~isempty(trajectoryXY)
        scatter(ax, trajectoryXY(:,1), trajectoryXY(:,2), ...
            18, COLOR.trajectory, 'filled', ...
            'MarkerFaceAlpha', 0.22, ...
            'MarkerEdgeColor', 'none');
    end

    if ~isempty(activeXY)
        scatter(ax, activeXY(:,1), activeXY(:,2), ...
            P.marker_size, COLOR.agent, 'filled', ...
            'MarkerFaceAlpha', 0.95, ...
            'MarkerEdgeColor', 'k');
    end

    title(ax, sprintf('1024 Grid CSV Player | %s | frame %d/%d | active = %d', ...
        files(k).name, k, nFrames, size(activeCells,1)), ...
        'FontSize', 13, 'FontWeight', 'bold');

    drawnow;

    if P.save_video
        writeVideo(vw, getframe(fig));
    end

    pause(P.anim_pause);
end

if P.save_video
    close(vw);
    fprintf('\nSaved video to %s\n', fullfile(csvDir, P.video_name));
end

fprintf('\nDone.\n');

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

%% MAP UTILITIES

function cells = mapToCells(map, occupiedValue)
    [r, c] = find(abs(map) == occupiedValue);
    cells = sortrows([r, c], [1 2]);
end

%% PLOTTING

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
        'MarkerFaceAlpha', 0.60, 'MarkerEdgeColor', 'none');

    for k = 1:size(cells,1)
        rc = cells(k,:);
        nb = staggeredNeighbors(rc, P.N);
        xy0 = cellToXY(rc, P);

        for j = 1:size(nb,1)
            if cellToId(nb(j,:), P.N) <= cellToId(rc, P.N)
                continue;
            end

            xy1 = cellToXY(nb(j,:), P);

            plot(ax, [xy0(1), xy1(1)], [xy0(2), xy1(2)], ...
                '-', 'Color', COLOR.grid_edge, 'LineWidth', 0.45);
        end
    end
end

%% GRID UTILITIES

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

function nb = staggeredNeighbors(cellRC, N)
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

    valid = nb(:,1) >= 1 & nb(:,1) <= N & ...
            nb(:,2) >= 1 & nb(:,2) <= N;

    nb = nb(valid,:);
end

function id = cellToId(cellRC, N)
    id = (cellRC(2)-1)*N + cellRC(1);
end