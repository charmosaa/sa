clear; clc; close all;

% const
v_max = 0.5;
w_max = 1.0;
w_min = - 1.0;

% robot starting position
cx = 0;
cy = 0;
ct = 0;

% params
dt = 0.03;
v = 0.5;
w = -1;
side_length = 4.0;

% arrays to track history for plotting
x_h = cx;
y_h = cy;

% setup plot
figure;
hold on; 
grid on;
title('Simulated Robot Trajectory');
xlabel('X [m]');
ylabel('Y [m]');
axis([-1 5 -1 5])

for side = 1:4

    % straight line movement
    vc = min(v, v_max);
    wc = 0;

    error_history = [];

    while StopCondition(side, cx, cy, side_length)
        tic;

        [cx, cy, ct] = SimDiffRob(cx, cy, ct, dt, vc, wc);
        
        % save positions
        x_h(end+1) = cx;
        y_h(end+1) = cy;
        
        % update the plot
        plot(x_h, y_h, 'y-', 'LineWidth', 2);

        % error detection
        switch side
            case 1
                e = abs(cx - 0);
            case 2
                e = abs(cy - 4);
            case 3
                e = abs(cx - 4);
            case 4
                e = abs(cy - 0);
        end

        error_history(end+1) = e;

        elapsed_time = toc;

        time_to_pause = dt - elapsed_time;
        if time_to_pause > 0
            pause(time_to_pause);
        end
    end

    % rotation movement
    vc = 0;
    wc = max(min(w, w_max), w_min);

    while abs(ct) < (pi/2) * (side)
        tic;

        [cx, cy, ct] = SimDiffRob(cx, cy, ct, dt, vc, wc);

        elapsed_time = toc;

        time_to_pause = dt - elapsed_time;
        if time_to_pause > 0
            pause(time_to_pause);
        end
    end

    % RMSE for the current side
    rmse = sqrt(mean(error_history.^2));
    fprintf('RMSE for side %d: %.4f\n', side, rmse);
end