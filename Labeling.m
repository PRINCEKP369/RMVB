clc;
clear;
close all;

%% ==========================================
% LOAD DATA
%% ==========================================

% load('mvdr_data.mat');   % mvdr_data [N x 180]

mvdr_data = randn(5,180);

[N, num_angles] = size(mvdr_data);

%% ==========================================
% OUTPUT LABELS
%% ==========================================

labels = zeros(N, num_angles);

%% ==========================================
% PARAMETERS
%% ==========================================

gaussian_sigma = 2;

%% ==========================================
% MAIN LOOP
%% ==========================================

for i = 1:N

    clf;

    x = mvdr_data(i, :);



    plot(1:num_angles, x, 'LineWidth', 2);

    grid on;
    hold on;

    xlabel('Angle Bin');
    ylabel('Normalized Power');

    title({
        sprintf('Sample %d / %d', i, N)
        'Left Click = Add Peak'
        'Right Click = Remove Last Peak'
        'Press ENTER = Finish'
    });

    % --------------------------------------
    % Interactive Selection
    % --------------------------------------

    peaks = [];

    while true

        [x_click, ~, button] = ginput(1);

        % ----------------------------------
        % ENTER pressed
        % ----------------------------------
        if isempty(button)
            break;
        end

        % ----------------------------------
        % LEFT CLICK -> ADD PEAK
        % ----------------------------------
        if button == 1

            p = round(x_click);

            if p >= 1 && p <= num_angles

                peaks(end+1) = p;

                % Immediate visual feedback
                plot(p, x(p), ...
                    'ro', ...
                    'MarkerSize', 12, ...
                    'LineWidth', 2);

                drawnow;
            end
        end

        % ----------------------------------
        % RIGHT CLICK -> REMOVE LAST PEAK
        % ----------------------------------
        if button == 3

            if ~isempty(peaks)

                peaks(end) = [];

                % Redraw everything
                clf;

                plot(1:num_angles, x, ...
                    'LineWidth', 2);

                grid on;
                hold on;

                xlabel('Angle Bin');
                ylabel('Normalized Power');

                title({
                    sprintf('Sample %d / %d', i, N)
                    'Left Click = Add Peak'
                    'Right Click = Remove Last Peak'
                    'Press ENTER = Finish'
                });

                % Replot remaining peaks
                for k = 1:length(peaks)

                    plot(peaks(k), x(peaks(k)), ...
                        'ro', ...
                        'MarkerSize', 12, ...
                        'LineWidth', 2);
                end

                drawnow;
            end
        end

    end

    % --------------------------------------
    % CREATE SOFT LABELS
    % --------------------------------------

    label = zeros(1, num_angles);

    angles = 1:num_angles;

    for k = 1:length(peaks)

        p = peaks(k);

        gaussian_label = exp( ...
            -0.5 * ((angles - p)/gaussian_sigma).^2);

        label = label + gaussian_label;
    end

    % Normalize

    if max(label) > 0
        label = label ./ max(label);
    end

    labels(i, :) = label;

    % --------------------------------------
    % Save Every 100 Samples
    % --------------------------------------

    if mod(i,100) == 0

        save('manual_labels.mat', ...
            'labels', ...
            '-v7.3');

        fprintf('Saved at sample %d\n', i);
    end

end

% ==========================================
% FINAL SAVE
% ==========================================

save('manual_labels.mat', ...
    'labels', ...
    '-v7.3');

fprintf('\nAnnotation Completed.\n');
