[pos, obs] = ExtractPathScans('writelog_2026_03_09_18_20_36.log', 45/180*pi);

cellSize = 0.1;

all_x = cell2mat(obs.x);
all_y = cell2mat(obs.y);

% map boundaries
minX = min(all_x); maxX = max(all_x);
minY = min(all_y); maxY = max(all_y);

cols = ceil((maxX - minX) / cellSize);
rows = ceil((maxY - minY) / cellSize);

gridMap = zeros(rows, cols);

for i = 1:length(obs.x)
    scan_x = obs.x{i};
    scan_y = obs.y{i};

    valid = ~isnan(scan_x) & ~isnan(scan_y);
    scan_x = scan_x(valid);
    scan_y = scan_y(valid);

    % calculate column and row
    idx_x = ceil((scan_x - minX) / cellSize);
    idx_y = ceil((scan_y - minY) / cellSize);
    
    % check boundaries
    idx_x = max(1, min(idx_x, cols));
    idx_y = max(1, min(idx_y, rows));
    
    % count laser
    for j = 1:length(idx_x)
        gridMap(idx_y(j), idx_x(j)) = gridMap(idx_y(j), idx_x(j)) + 1;
    end
end


% visualization 3D map
gridMapNorm = gridMap / max(gridMap(:));

[X, Y] = meshgrid(1:cols, 1:rows);

figure;
surf(X, Y, gridMapNorm);
shading interp;         
colormap(jet);
colorbar;

view(3);
grid on;
title('Mapa de obstaculos 3D');
xlabel('X');
ylabel('Y');
zlabel('Probabilidad');

% save in .mat format
save('mapa_obstaculos_final.mat', 'gridMapNorm', 'X', 'Y', 'cellSize', 'minX', 'minY');
fprintf('Mapa guardado correctamente en .mat\n');