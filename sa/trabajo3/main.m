%% 0. Setup
load('mapa_obstaculos_final.mat');
threshold = 0.03; 
mapBin = gridMapNorm > threshold;
map = binaryOccupancyMap(mapBin, 1/cellSize);

figure;
show(map);
title('Binarized Occupancy Map');

%% 1. Path planning without obstacles
start1World = [2, 15];
goal1World = [12, 15];
start1Grid = world2grid(map, start1World);
goal1Grid = world2grid(map, goal1World);

planner1 = plannerAStarGrid(map);
[path1Grid, debugInfo1] = plan(planner1, start1Grid, goal1Grid);
path1World = grid2world(map, path1Grid);

plotAndSave(map, path1World, start1World, goal1World, ...
            '1. Ruta A* sin obstáculos', '1_imagen.png', '1_resultado', debugInfo1);

%% 2. Path planning with obstacles
start2World = [1, 2];
goal2World = [12, 12];
start2Grid = world2grid(map, start2World);
goal2Grid = world2grid(map, goal2World);

planner2 = plannerAStarGrid(map);
[path2Grid, debugInfo2] = plan(planner2, start2Grid, goal2Grid);
path2World = grid2world(map, path2Grid);

plotAndSave(map, path2World, start2World, goal2World, ...
            '2. Ruta A* con obstáculos', '2_imagen.png', '2_resultado', debugInfo2);

%% 3. Inflating obstacle boundaries
mapInflated = copy(map);
vehicleRadius = 0.2;
inflate(mapInflated, vehicleRadius);

start3World = [1, 2];
goal3World = [12, 12];
% Other points tested for point 5 validation:
% start3World = [12, 2]; goal3World = [1, 13];
% start3World = [12, 2]; goal3World = [12, 12];
% start3World = [15.5, 5.5]; goal3World = [1, 1];

start3Grid = world2grid(mapInflated, start3World);
goal3Grid = world2grid(mapInflated, goal3World);

planner3 = plannerAStarGrid(mapInflated);
[path3Grid, debugInfo3] = plan(planner3, start3Grid, goal3Grid);
path3World = grid2world(mapInflated, path3Grid);

plotAndSave(mapInflated, path3World, start3World, goal3World, ...
            '3. Ruta A* con obstáculos inflados', '3_imagen.png', '3_resultado', debugInfo3);

distance2 = sum(sqrt(sum(diff(path2World).^2, 2)));
distance3 = sum(sqrt(sum(diff(path3World).^2, 2)));
fprintf('Distancia de la ruta 2 (sin inflar): %.2f m\n', distance2);
fprintf('Distancia de la ruta 3 (inflada): %.2f m\n', distance3);

%% 4. Influence of cell size
% Large cells
cellSizeLarge = 0.2;
mapBinLarge = imresize(mapBin, 0.1/cellSizeLarge, 'nearest');
mapLarge = binaryOccupancyMap(mapBinLarge, 1/cellSizeLarge);

start4LargeGrid = world2grid(mapLarge, start3World);
goal4LargeGrid = world2grid(mapLarge, goal3World);
planner4Large = plannerAStarGrid(mapLarge);

tic;
[path4LargeGrid, debugInfo4Large] = plan(planner4Large, start4LargeGrid, goal4LargeGrid);
timeLarge = toc;
fprintf('Tiempo de cómputo para celdas GRANDES (0.2m): %.4f s\n', timeLarge);

% Small cells
cellSizeSmall = 0.02;
mapBinSmall = imresize(mapBin, 0.1/cellSizeSmall, 'nearest');
mapSmall = binaryOccupancyMap(mapBinSmall, 1/cellSizeSmall);

start4SmallGrid = world2grid(mapSmall, start3World);
goal4SmallGrid = world2grid(mapSmall, goal3World);
planner4Small = plannerAStarGrid(mapSmall);

tic;
[path4SmallGrid, debugInfo4Small] = plan(planner4Small, start4SmallGrid, goal4SmallGrid);
timeSmall = toc;
fprintf('Tiempo de cómputo para celdas PEQUEÑAS (0.02m): %.4f s\n', timeSmall);

% Plots
path4LargeWorld = grid2world(mapLarge, path4LargeGrid);
path4SmallWorld = grid2world(mapSmall, path4SmallGrid);

figure;
subplot(1,2,1);
show(mapLarge);
hold on;
plot(path4LargeWorld(:,1), path4LargeWorld(:,2), 'c-', 'LineWidth', 2);
title('Celdas grandes (0.2m)');

subplot(1,2,2);
show(mapSmall);
hold on;
plot(path4SmallWorld(:,1), path4SmallWorld(:,2), 'y-', 'LineWidth', 2);
title('Celdas pequeñas (0.02m)');

exportgraphics(gcf, '4_imagen.png', 'Resolution', 300);

%% 5. Smoothed trajectory
step = 8; 
indices = 1:step:size(path3World, 1);
if indices(end) ~= size(path3World, 1)
    indices = [indices, size(path3World, 1)];
end
controlPoints = path3World(indices, :);

t = 1:size(controlPoints, 1);
tSmooth = 1:0.2:size(controlPoints, 1);
pathSmoothX = spline(t, controlPoints(:,1), tSmooth);
pathSmoothY = spline(t, controlPoints(:,2), tSmooth);
pathSmooth = [pathSmoothX', pathSmoothY'];

collisions = checkOccupancy(map, pathSmooth);
numCollisions = sum(collisions);

figure;
show(map);
hold on;
h1 = plot(path3World(:,1), path3World(:,2), 'c-', 'LineWidth', 2.5);
h2 = plot(pathSmooth(:,1), pathSmooth(:,2), 'y-', 'LineWidth', 2.5);
h3 = plot(start3World(1), start3World(2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
h4 = plot(goal3World(1), goal3World(2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

title('5. Trayectoria suavizada');
legend([h1, h2, h3, h4], 'Ruta original', 'Ruta suavizada', 'Start', 'Meta');

save('5_resultado.mat', 'pathSmooth');
exportgraphics(gcf, '5_imagen.png', 'Resolution', 300);
fprintf('Ruta suavizada. Colisiones: %d\n', numCollisions);

%% 6. Comparison with another method (PRM)
prm = mobileRobotPRM;
prm.Map = mapInflated;
prm.NumNodes = 120;
prm.ConnectionDistance = 4;

start6World = [1, 2];
goal6World = [12, 12];
pathPrm = findpath(prm, start6World, goal6World);

figure;
show(prm);
hold on;
if ~isempty(pathPrm)
    plot(pathPrm(:,1), pathPrm(:,2), 'm-', 'LineWidth', 3);
end
plot(start6World(1), start6World(2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot(goal6World(1), goal6World(2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

title('6. Ruta obtenida con el algoritmo PRM');
exportgraphics(gcf, '6_imagen.png', 'Resolution', 300);

%% --- HELPER FUNCTIONS ---
function plotAndSave(mapObj, pathWorld, startW, goalW, titleStr, imgName, saveVarName, debugInfo)
    % Helper function to plot maps, paths, and save results
    figure;
    show(mapObj);
    hold on;
    
    if ~isempty(pathWorld)
        h1 = plot(pathWorld(:,1), pathWorld(:,2), 'c-', 'LineWidth', 2);
    else
        h1 = [];
    end
    
    h2 = plot(startW(1), startW(2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
    h3 = plot(goalW(1), goalW(2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    
    title(titleStr);
    if ~isempty(h1)
        legend([h1, h2, h3], 'Ruta A*', 'Start', 'Meta');
    else
        legend([h2, h3], 'Start', 'Meta');
    end
    
    % Save .mat and .png
    if exist('debugInfo', 'var')
        save(sprintf('%s.mat', saveVarName), 'pathWorld', 'debugInfo');
    else
        save(sprintf('%s.mat', saveVarName), 'pathWorld');
    end
    
    exportgraphics(gcf, imgName, 'Resolution', 300);
    fprintf('Guardado imagen: %s\n', imgName);
end