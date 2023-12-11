function uavSmart
    % Define start and goal states
    startState = [0 0 0 deg2rad(0)];
    goalState = [50 50 10 deg2rad(45)];

    % Obstacle and drone settings
    numObstacles = 70;
    obstacleRadius = 1;
    lidarRadius = 5;
    droneRadius = 1;
    safetyMargin = 1;

    % Random and deterministic obstacles
    obstaclePoints = rand(numObstacles, 3) * 50;
    deterministicObstacles = [25 25 5; 30 30 7];
    obstaclePoints = [obstaclePoints; deterministicObstacles];

    % Linear interpolation for visualization
    numPoints = 100;
    x = linspace(startState(1), goalState(1), numPoints);
    y = linspace(startState(2), goalState(2), numPoints);
    z = linspace(startState(3), goalState(3), numPoints);
    waypoints = [x' y' z'];

    % Visualization setup
    figure;
    dronePlot = plot3(startState(1), startState(2), startState(3), 'ro', 'MarkerSize', 10);
    hold on;
    traveledPlot = plot3(startState(1), startState(2), startState(3), 'r-');
    obstaclePlot = plot3(obstaclePoints(:,1), obstaclePoints(:,2), obstaclePoints(:,3), 'kx', 'MarkerSize', 10);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Drone Path Animation');
    grid on;
    axis equal;

    % Initialize drone position
    dronePosition = startState(1:3);

    % Main loop for path animation
    for k = 1:length(waypoints)
        % Implement obstacle avoidance
        [dronePosition, adjusted] = advancedObstacleAvoidance(dronePosition, waypoints(k, :), obstaclePoints, lidarRadius, droneRadius, safetyMargin);

        % Update drone position in visualization
        updateDronePosition(dronePosition, dronePlot, traveledPlot);

        % Control animation speed
        pause(0.05);
    end
end

function [newPosition, adjusted] = advancedObstacleAvoidance(currentPosition, targetPosition, obstacles, lidarRadius, droneRadius, safetyMargin)
    % Calculate distances to all obstacles
    distancesToObstacles = sqrt(sum((obstacles - currentPosition).^2, 2));
    
    % Find potential collision obstacles
    potentialCollisions = find(distancesToObstacles < lidarRadius);
    adjusted = false;
    newPosition = currentPosition; % Initialize newPosition

    % If there are potential collisions, plot avoidance
    if ~isempty(potentialCollisions)
        for collisionIndex = potentialCollisions'
            obstaclePos = obstacles(collisionIndex, :);
            avoidanceCurve = calculateAvoidanceCurve(currentPosition, obstaclePos, lidarRadius);

            if isCurveSafe(avoidanceCurve, obstacles, droneRadius + safetyMargin)
                % Update position along the avoidance curve
                newPosition = avoidanceCurve(end, :); % Assuming the end of the curve is the new position
                adjusted = true;
                break; % Exit loop after successful avoidance
            end
        end
    end

    % Continue on the default path if no adjustment was needed
    if ~adjusted
        newPosition = targetPosition;
    end
end

function updateDronePosition(newPosition, dronePlot, traveledPlot)
    set(dronePlot, 'XData', newPosition(1), 'YData', newPosition(2), 'ZData', newPosition(3));
    set(traveledPlot, 'XData', [get(traveledPlot, 'XData'), newPosition(1)], ...
                     'YData', [get(traveledPlot, 'YData'), newPosition(2)], ...
                     'ZData', [get(traveledPlot, 'ZData'), newPosition(3)]);
    drawnow;
end

function avoidanceCurve = calculateAvoidanceCurve(currentPosition, obstaclePosition, lidarRadius)
    % Simple avoidance curve calculation
    obstacleVector = obstaclePosition - currentPosition;
    theta = linspace(0, pi, 10);
    semiCircleRadius = lidarRadius * 1.5;
    avoidanceCurve = zeros(length(theta), 3);

    for i = 1:length(theta)
        avoidanceCurve(i, :) = currentPosition + ...
                               semiCircleRadius * [cos(theta(i)), sin(theta(i)), 0] + ...
                               obstacleVector * 0.5;
    end
end

function isSafe = isCurveSafe(curve, obstacles, safeDistance)
    isSafe = true;

    for i = 1:size(curve, 1)
        for j = 1:size(obstacles, 1)
            distance = norm(curve(i, :) - obstacles(j, :));
            if distance < safeDistance
                isSafe = false;
                return;
            end
        end
    end
end