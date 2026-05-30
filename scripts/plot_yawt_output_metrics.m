clear; 
clc; 
close all;

% ============================================================
% Plot YAWT exported tracks and centerline metrics
% ============================================================
% This script DOES NOT recompute tip-to-tip distance, curvature, or signed
% velocity. It only plots the values already exported in the Excel workbook.
%
% It also removes bad centroid points where PositionX = 0 and PositionY = 0
% by converting those x,y coordinates to NaN. This prevents MATLAB from
% drawing artificial diagonal lines through (0,0).
%
% Usage:
%   1. Put the full Excel file path below, or leave it as "" to select it.
%   2. Press Run.

xlsxFile = "/Users/sk3526/Desktop/yawt_movies/yawt/S4_N2-0_0001_cropped/PROC_2026-05-29-224515/S4_N2-0_0001_cropped_tracks.xlsx";
% Example:
% xlsxFile = "/Users/sk3526/Desktop/yawt_movies/yawt/S4_N2-0_0001_cropped/PROC_2026-05-29-221903/S4_N2-0_0001_cropped_tracks.xlsx";

plot_yawt_exported_centerline_metrics(xlsxFile);


% ============================================================
% Local functions are below
% ============================================================

function plot_yawt_exported_centerline_metrics(xlsxFile)
    % Main plotting function.
    % Creates separate paginated 2 x 4 figures for:
    %   1. XY tracks
    %   2. Tip-to-tip distance
    %   3. Centerline curvature
    %   4. Signed velocity

    if nargin < 1 || strlength(string(xlsxFile)) == 0
        [fileName, folderName] = uigetfile("*.xlsx", "Choose YAWT tracks workbook");
        if isequal(fileName, 0)
            error("No workbook selected.");
        end
        xlsxFile = fullfile(folderName, fileName);
    end

    if ~isfile(xlsxFile)
        error("Could not find this workbook: %s", string(xlsxFile));
    end

    % Read exported YAWT sheets.
    tracks = readtable(xlsxFile, "Sheet", "Tracks", "VariableNamingRule", "preserve");
    centerlines = readtable(xlsxFile, "Sheet", "Centerlines", "VariableNamingRule", "preserve");

    % Find track columns.
    wormColTracks  = findVar(tracks, ["WormID", "WormId", "Worm", "TrackID", "TrackId"]);
    frameColTracks = findVar(tracks, ["Frame", "FrameNumber", "FrameID", "FrameId"]);
    xColTracks     = findVar(tracks, ["PositionX", "X", "CentroidX", "CenterX", "x"]);
    yColTracks     = findVar(tracks, ["PositionY", "Y", "CentroidY", "CenterY", "y"]);

    % Find centerline metric columns.
    wormColCL      = findVar(centerlines, ["WormID", "WormId", "Worm", "TrackID", "TrackId"]);
    frameColCL     = findVar(centerlines, ["Frame", "FrameNumber", "FrameID", "FrameId"]);
    tipCol         = findVar(centerlines, ["TipToTipDistance", "Tip_to_tip_distance", "Tip to tip distance", "TipTipDistance"]);
    curvatureCol   = findVar(centerlines, ["CenterlineCurvature", "Curvature", "MeanCurvature", "Centerline curvature"]);
    velocityCol    = findVar(centerlines, ["SignedVelocityPxPerSec", "SignedVelocity", "Velocity", "Signed velocity", "SignedVelocityPixelsPerSecond"]);

    % Replace invalid centroid locations, where x = 0 and y = 0, with NaN.
    % This removes artificial diagonal lines in the track plots.
    [tracks, badFrameKeys, nBadXY] = replaceZeroXYWithNaN(tracks, wormColTracks, frameColTracks, xColTracks, yColTracks);

    % Optional but useful: if the centroid was invalid at a worm-frame pair,
    % also break the metric traces at those same frames. This does not
    % recompute anything; it only avoids plotting metric values during frames
    % where the track position was clearly invalid.
    metricCols = [tipCol, curvatureCol, velocityCol];
    [centerlines, nBadMetricRows] = convertMetricsToNaNForBadFrames(centerlines, wormColCL, frameColCL, metricCols, badFrameKeys);

    fprintf("Loaded: %s\n", string(xlsxFile));
    fprintf("Converted %d track rows with centroid (0,0) to NaN.\n", nBadXY);
    fprintf("Converted %d matching centerline metric rows to NaN.\n", nBadMetricRows);

    % Get all worm IDs found in either sheet.
    wormIds = unique([asNumeric(tracks.(wormColTracks)); asNumeric(centerlines.(wormColCL))]);
    wormIds = sort(wormIds(isfinite(wormIds)));

    if isempty(wormIds)
        error("No valid worm IDs were found.");
    end

    % Plot separate 2 x 4 panel figures.
    plotTrackPages(tracks, wormIds, wormColTracks, frameColTracks, xColTracks, yColTracks, xlsxFile);

    plotMetricPages(centerlines, wormIds, wormColCL, frameColCL, tipCol, ...
        "Tip-to-tip distance", "Tip-to-tip distance (pixels)", xlsxFile, []);

    plotMetricPages(centerlines, wormIds, wormColCL, frameColCL, curvatureCol, ...
        "Centerline curvature", "Curvature (rad/pixel)", xlsxFile, []);

    plotMetricPages(centerlines, wormIds, wormColCL, frameColCL, velocityCol, ...
        "Signed velocity", "Signed velocity (pixels/second)", xlsxFile, 0);
end

function plotTrackPages(tbl, wormIds, wormCol, frameCol, xCol, yCol, xlsxFile)
    wormsPerFigure = 8;
    nPages = ceil(numel(wormIds) / wormsPerFigure);

    wormAll  = asNumeric(tbl.(wormCol));
    frameAll = asNumeric(tbl.(frameCol));
    xAll     = asNumeric(tbl.(xCol));
    yAll     = asNumeric(tbl.(yCol));

    for page = 1:nPages
        idx = (page - 1) * wormsPerFigure + 1 : min(page * wormsPerFigure, numel(wormIds));

        figure("Name", sprintf("YAWT tracks page %d", page), "Color", "w");
        tiledlayout(2, 4, "TileSpacing", "compact", "Padding", "compact");

        for panel = 1:numel(idx)
            wormId = wormIds(idx(panel));
            nexttile;

            rows = wormAll == wormId;
            frame = frameAll(rows);
            x = xAll(rows);
            y = yAll(rows);

            % Sort by frame so that the track is drawn in time order.
            goodFrame = isfinite(frame);
            frame = frame(goodFrame);
            x = x(goodFrame);
            y = y(goodFrame);
            [~, order] = sort(frame);
            x = x(order);
            y = y(order);

            plot(x, y, "-", "LineWidth", 1.4);
            hold on;

            goodXY = isfinite(x) & isfinite(y);
            if any(goodXY)
                firstGood = find(goodXY, 1, "first");
                lastGood  = find(goodXY, 1, "last");

                plot(x(firstGood), y(firstGood), "go", ...
                    "MarkerFaceColor", "g", "MarkerSize", 5);
                plot(x(lastGood), y(lastGood), "ro", ...
                    "MarkerFaceColor", "r", "MarkerSize", 5);
            end

            hold off;
            set(gca, "YDir", "reverse");
            grid on;
            title(sprintf("Worm %g", wormId), "Interpreter", "none");
            xlabel("X (pixels)");
            ylabel("Y (pixels)");
        end

        sgtitle(sprintf("Tracks: %s", shortName(xlsxFile)), "Interpreter", "none");
    end
end

function plotMetricPages(tbl, wormIds, wormCol, frameCol, valueCol, figTitle, yLabelText, xlsxFile, referenceLine)
    wormsPerFigure = 8;
    nPages = ceil(numel(wormIds) / wormsPerFigure);

    wormAll  = asNumeric(tbl.(wormCol));
    frameAll = asNumeric(tbl.(frameCol));
    valueAll = asNumeric(tbl.(valueCol));

    for page = 1:nPages
        idx = (page - 1) * wormsPerFigure + 1 : min(page * wormsPerFigure, numel(wormIds));

        figure("Name", sprintf("%s page %d", figTitle, page), "Color", "w");
        tiledlayout(2, 4, "TileSpacing", "compact", "Padding", "compact");

        for panel = 1:numel(idx)
            wormId = wormIds(idx(panel));
            nexttile;

            rows = wormAll == wormId;
            frame = frameAll(rows);
            value = valueAll(rows);

            goodFrame = isfinite(frame);
            frame = frame(goodFrame);
            value = value(goodFrame);

            [frame, order] = sort(frame);
            value = value(order);

            plot(frame, value, "-", "LineWidth", 1.2);

            if ~isempty(referenceLine)
                hold on;
                yline(referenceLine, "k--", "LineWidth", 0.8);
                hold off;
            end

            grid on;
            title(sprintf("Worm %g", wormId), "Interpreter", "none");
            xlabel("Frame");
            ylabel(yLabelText);
        end

        sgtitle(sprintf("%s: %s", figTitle, shortName(xlsxFile)), "Interpreter", "none");
    end
end

function [tbl, badFrameKeys, nBadXY] = replaceZeroXYWithNaN(tbl, wormCol, frameCol, xCol, yCol)
    worm  = asNumeric(tbl.(wormCol));
    frame = asNumeric(tbl.(frameCol));
    x     = asNumeric(tbl.(xCol));
    y     = asNumeric(tbl.(yCol));

    badXY = isfinite(x) & isfinite(y) & x == 0 & y == 0;
    nBadXY = nnz(badXY);

    x(badXY) = NaN;
    y(badXY) = NaN;

    tbl.(xCol) = x;
    tbl.(yCol) = y;

    badFrameKeys = makeWormFrameKeys(worm(badXY), frame(badXY));
    badFrameKeys = unique(badFrameKeys);
end

function [tbl, nBadMetricRows] = convertMetricsToNaNForBadFrames(tbl, wormCol, frameCol, metricCols, badFrameKeys)
    if isempty(badFrameKeys)
        nBadMetricRows = 0;
        return;
    end

    worm = asNumeric(tbl.(wormCol));
    frame = asNumeric(tbl.(frameCol));
    rowKeys = makeWormFrameKeys(worm, frame);
    badRows = ismember(rowKeys, badFrameKeys);
    nBadMetricRows = nnz(badRows);

    metricCols = string(metricCols);
    for i = 1:numel(metricCols)
        colName = metricCols(i);
        values = asNumeric(tbl.(colName));
        values(badRows) = NaN;
        tbl.(colName) = values;
    end
end

function keys = makeWormFrameKeys(worm, frame)
    worm = double(worm(:));
    frame = double(frame(:));
    keys = strings(size(worm));

    good = isfinite(worm) & isfinite(frame);
    keys(good) = compose("%.15g__%.15g", worm(good), frame(good));
    keys(~good) = "";
end

function name = findVar(tbl, candidates)
    vars = string(tbl.Properties.VariableNames);
    normVars = normalizeVarName(vars);
    normCandidates = normalizeVarName(string(candidates));

    for i = 1:numel(normCandidates)
        match = find(normVars == normCandidates(i), 1);
        if ~isempty(match)
            name = vars(match);
            return;
        end
    end

    error("Could not find any of these columns: %s", strjoin(string(candidates), ", "));
end

function out = normalizeVarName(in)
    out = lower(string(in));
    out = regexprep(out, "[^a-z0-9]", "");
end

function values = asNumeric(values)
    if isnumeric(values)
        values = double(values);
        return;
    end

    if iscell(values)
        values = string(values);
    elseif iscategorical(values)
        values = string(values);
    end

    values = str2double(string(values));
end

function label = shortName(filePath)
    [~, baseName, ext] = fileparts(char(filePath));
    label = string(baseName) + string(ext);
end
