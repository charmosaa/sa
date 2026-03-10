[pos obs] = ExtractPathScans('writelog_2026_03_09_18_20_36.log', 45/180*pi)

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
    
    % no 0 in matlab?
    idx_x(idx_x == 0) = 1;
    idx_y(idx_y == 0) = 1;
    
    % count laser
    for j = 1:length(idx_x)
        gridMap(idx_y(j), idx_x(j)) = gridMap(idx_y(j), idx_x(j)) + 1;
    end
end


% visualization 2D map
gridMapNorm = gridMap / max(gridMap(:));

figure;
imagesc(gridMapNorm);

colormap(flipud(gray)); 

colorbar;

title('Mapa con obstaculos');
xlabel('X');
ylabel('Y');