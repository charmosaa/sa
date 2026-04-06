%% 0. Inicio
load('mapa_obstaculos_final.mat');
umbral = 0.03; 
mapaBinarizada = gridMapNorm > umbral;
mapa = binaryOccupancyMap(mapaBinarizada, 1/cellSize);
figure;
show(mapa);
title('Binarized Occupancy Map');

%% 1. Planificación sin obstáculos
start1_world = [2, 15];
goal1_world = [12, 15];

start1_grid = world2grid(mapa, start1_world);
goal1_grid = world2grid(mapa, goal1_world);

planner1 = plannerAStarGrid(mapa);
[path1_grid, debugInfo1] = plan(planner1, start1_grid, goal1_grid);

path1_world = grid2world(mapa, path1_grid);

figure;
show(mapa);
hold on;
plot(path1_world(:,1), path1_world(:,2), 'c-', 'LineWidth', 2);
plot(start1_world(1), start1_world(2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot(goal1_world(1), goal1_world(2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
title('1. Ruta A* sin obstáculos');
legend('', 'Start', 'Meta');

save('actividad_1_resultado.mat', 'path1_world', 'debugInfo1');
saveas(gcf, 'actividad_1_imagen.png');
fprintf('Guardado imagen para el punto 1.\n');

%% 2. Planificación con obstáculos
start2_world = [1, 2];
goal2_world = [12, 12];

start2_grid = world2grid(mapa, start2_world);
goal2_grid = world2grid(mapa, goal2_world);

planner2 = plannerAStarGrid(mapa);
[path2_grid, debugInfo2] = plan(planner2, start2_grid, goal2_grid);

path2_world = grid2world(mapa, path2_grid);

figure;
show(mapa);
hold on;
plot(path2_world(:,1), path2_world(:,2), 'c-', 'LineWidth', 2);
plot(start2_world(1), start2_world(2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot(goal2_world(1), goal2_world(2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
title('2. Ruta A* con obstáculos');
legend('', 'Start', 'Meta');

save('actividad_2_resultado.mat', 'path2_world', 'debugInfo2');
saveas(gcf, 'actividad_2_imagen.png');
fprintf('Guardado imagen para el punto 2.\n');

%% 3. Inflando los contornos de los obstáculos
mapaInflada = copy(mapa);

radio_vehiculo = 0.2;
inflate(mapaInflada, radio_vehiculo);

start3_world = [1, 2];
goal3_world = [12, 12];
% otros puntos que se ha probado para comprobar actividad 5
% start3_world = [12, 2];
% goal3_world = [1, 13];
% start3_world = [12, 2];
% goal3_world = [12, 12];
% start3_world = [15.5, 5.5];
% goal3_world = [1, 1];


start3_grid = world2grid(mapaInflada, start3_world);
goal3_grid = world2grid(mapaInflada, goal3_world);

planner3 = plannerAStarGrid(mapaInflada);
[path3_grid, debugInfo3] = plan(planner3, start3_grid, goal3_grid);

path3_world = grid2world(mapaInflada, path3_grid);

figure;
show(mapaInflada);
hold on;
plot(path3_world(:,1), path3_world(:,2), 'c-', 'LineWidth', 2);
plot(start3_world(1), start3_world(2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot(goal3_world(1), goal3_world(2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
title('3. Ruta A* con obstáculos inflados');
legend('', 'Start', 'Meta');

save('actividad_3_resultado.mat', 'path3_world', 'debugInfo3');
saveas(gcf, 'actividad_3_imagen.png');
fprintf('Guardado imagen para el punto 3.\n');

distancia2 = sum(sqrt(sum(diff(path2_world).^2, 2)));
distancia3 = sum(sqrt(sum(diff(path3_world).^2, 2)));
fprintf('Distancia de la ruta 2 (sin inflar): %.2f m\n', distancia2);
fprintf('Distancia de la ruta 3 (inflada): %.2f m\n', distancia3);


%% 4. Influencia del tamaño de las celdas
cellSize_grande = 0.2;
mapaBinarizada_grande = imresize(mapaBinarizada, 0.1/cellSize_grande, 'nearest');
mapa_gran = binaryOccupancyMap(mapaBinarizada_grande, 1/cellSize_grande);

start4_grande_grid = world2grid(mapa_gran, start3_world);
goal4_grande_grid = world2grid(mapa_gran, goal3_world);
planner4_grande = plannerAStarGrid(mapa_gran);

tic;
[path4_grande_grid, debugInfo4_grande] = plan(planner4_grande, start4_grande_grid, goal4_grande_grid);
tiempo_grandes = toc;
fprintf('Tiempo de cómputo para celdas GRANDES (0.2m): %.4f segundos\n', tiempo_grandes);

cellSize_peq = 0.02;
mapaBinarizada_pequena = imresize(mapaBinarizada, 0.1/cellSize_peq, 'nearest');
mapa_peq = binaryOccupancyMap(mapaBinarizada_pequena, 1/cellSize_peq);

start4_pequena_grid = world2grid(mapa_peq, start3_world);
goal4_pequena_grid = world2grid(mapa_peq, goal3_world);
planner4_pequena = plannerAStarGrid(mapa_peq);

tic;
[path4_pequena_grid, debugInfo4_pequena] = plan(planner4_pequena, start4_pequena_grid, goal4_pequena_grid);
tiempo_pequenas = toc;
fprintf('Tiempo de cómputo para celdas PEQUEÑAS (0.02m): %.4f segundos\n', tiempo_pequenas);

path4_grande_world = grid2world(mapa_gran, path4_grande_grid);
path4_pequena_world = grid2world(mapa_peq, path4_pequena_grid);

figure;
subplot(1,2,1);
show(mapa_grande);
hold on;
if ~isempty(path4_grande_world)
    plot(path4_grande_world(:,1), path4_grande_world(:,2), 'c-', 'LineWidth', 2);
end
title('Celdas Grandes (0.2m)');

subplot(1,2,2);
show(mapa_pequena);
hold on;
if ~isempty(path4_pequena_world)
    plot(path4_pequena_world(:,1), path4_pequena_world(:,2), 'y-', 'LineWidth', 2);
end
title('Celdas Pequeñas (0.02m)');

exportgraphics(gcf, 'actividad_4_imagen.png', 'Resolution', 300);


%% 5. Trayectoria suavizada
paso = 8; 
indices = 1:paso:size(path3_world, 1);

if indices(end) ~= size(path3_world, 1)
    indices = [indices, size(path3_world, 1)];
end

puntos_control = path3_world(indices, :);

t = 1:size(puntos_control, 1);
t_smooth = 1:0.2:size(puntos_control, 1);

path_smooth_x = spline(t, puntos_control(:,1), t_smooth);
path_smooth_y = spline(t, puntos_control(:,2), t_smooth);
path_smooth = [path_smooth_x', path_smooth_y'];

colisiones = checkOccupancy(mapa, path_smooth);
num_colisiones = sum(colisiones);

figure;
show(mapa);
hold on;
h1 = plot(path3_world(:,1), path3_world(:,2), 'c-', 'LineWidth', 2.5);
h2 = plot(path_smooth(:,1), path_smooth(:,2), 'y-', 'LineWidth', 2.5);
h3 = plot(start3_world(1), start3_world(2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
h4 = plot(goal3_world(1), goal3_world(2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

title('5. Trayectoria suavizada');
legend([h1, h2, h3, h4], 'Ruta original', 'Ruta suavizada', 'Start', 'Meta');

save('actividad_5_resultado.mat', 'path_smooth');
exportgraphics(gcf, 'actividad_5_imagen.png', 'Resolution', 300);

fprintf('Ruta suavizada. Colisiones: %d\n', num_colisiones);


%% 6. Comparación con otro método (PRM - Probabilistic Roadmap)
prm = mobileRobotPRM;
prm.Map = mapaInflada;
prm.NumNodes = 120;
prm.ConnectionDistance = 4;

start6 = [1, 2];
goal6 = [12, 12];

path_prm = findpath(prm, start6, goal6);

figure;
show(prm);
hold on;
if ~isempty(path_prm)
    plot(path_prm(:,1), path_prm(:,2), 'm-', 'LineWidth', 3);
end
plot(start6(1), start6(2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot(goal6(1), goal6(2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

title('6. Ruta obtenida con el algoritmo PRM');
exportgraphics(gcf, 'actividad_6_imagen.png', 'Resolution', 300);