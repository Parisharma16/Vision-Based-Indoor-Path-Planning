function compute_shortest_paths_with_specific_paths(adjacencyMatrix, validCentroids, Img)
    numNodes = size(validCentroids, 1);

    disp('Centroid Coordinates:');
    for i = 1:numNodes
        fprintf('Centroid %d: (%.2f, %.2f)\n', i, validCentroids(i, 1), validCentroids(i, 2));
    end
    
    
    numNodes = size(validCentroids, 1);

    % Compute all-pairs shortest paths using Floyd-Warshall algorithm
    [distance, next] = floydWarshall(adjacencyMatrix, validCentroids);

    % Create a table of distances
    distanceTable = array2table(distance, 'VariableNames', strcat('Node', string(1:numNodes)), 'RowNames', strcat('Node', string(1:numNodes)));

    % Display the table
    disp('Distance Table:');
    disp(distanceTable);

    % Create a white image of the same size as the original image
    whiteImage = 255 * ones(size(Img), 'uint8');  % Ensure whiteImage matches Img size and type

    % Plotting the valid centroids with respective names on the image
    figure;
    imshow(Img); hold on;
    plot(validCentroids(:, 1), validCentroids(:, 2), 'r*', 'MarkerSize', 10);

    for i = 1:numNodes
        text(validCentroids(i, 1), validCentroids(i, 2), sprintf('%d', i), 'Color', 'black', 'FontSize', 12);
    end

    % Plot the shortest paths for each connected component
    visitedNodes = false(1, numNodes);
    components = {};
    for i = 1:numNodes
        if ~visitedNodes(i)
            component = dfsComponent(adjacencyMatrix, i, visitedNodes);
            components{end+1} = component;
            visitedNodes(component) = true;
        end
    end

    for k = 1:length(components)
        component = components{k};
        for i = 1:length(component)
            for j = i+1:length(component)
                startNode = component(i);
                endNode = component(j);
                if next(startNode, endNode) ~= 0
                    u = startNode;
                    while u ~= endNode
                        v = next(u, endNode);
                        pt1 = validCentroids(u, :);
                        pt2 = validCentroids(v, :);
                        [pathX, pathY] = interpolatePath(pt1, pt2);
                        plot(pathX, pathY, 'b-', 'LineWidth', 1);  % Decreased line width
                        u = v;
                    end
                end
            end
        end
    end

    
    paths = [];

    for p = 1:size(paths, 1)
        startNode = paths(p, 1);
        endNode = paths(p, 2);
        u = startNode;
        while u ~= endNode
            v = next(u, endNode);
            pt1 = validCentroids(u, :);
            pt2 = validCentroids(v, :);
            [pathX, pathY] = interpolatePath(pt1, pt2);
            plot(pathX, pathY, 'b-', 'LineWidth', 1);  % Decreased line width
            u = v;
        end
    end

        % Call to display_specific_shortest_path for nodes 14 to 8
    


    % Save the final image with centroids, names, and shortest paths
    saveas(gcf, 'ShortestPath.png');  % Save the current figure

    % Close the figure to prevent multiple figures from being saved
    close(gcf);

    %disp('Shortest paths image saved as ShortestPath.png');
end

function [component] = dfsComponent(adjacencyMatrix, startNode, visited)
    stack = [startNode];
    component = [];
    while ~isempty(stack)
        node = stack(end);
        stack(end) = [];
        if ~visited(node)
            visited(node) = true;
            component = [component node];
            neighbors = find(adjacencyMatrix(node, :) == 1);
            stack = [stack neighbors(~visited(neighbors))];
        end
    end
end

function [distance, next] = floydWarshall(adjacencyMatrix, centroids)
    numNodes = size(adjacencyMatrix, 1);
    distance = inf(numNodes);
    next = zeros(numNodes);

    for i = 1:numNodes
        distance(i, i) = 0;
        for j = find(adjacencyMatrix(i, :))
            distance(i, j) = norm(centroids(i, :) - centroids(j, :));
            next(i, j) = j;
        end
    end

    for k = 1:numNodes
        for i = 1:numNodes
            for j = 1:numNodes
                if distance(i, k) + distance(k, j) < distance(i, j)
                    distance(i, j) = distance(i, k) + distance(k, j);
                    next(i, j) = next(i, k);
                end
            end
        end
    end
end

function [pathX, pathY] = interpolatePath(pt1, pt2)
    numPoints = 50;
    t = linspace(0, 1, numPoints);
    pathX = round(pt1(1) + (pt2(1) - pt1(1)) * t);
    pathY = round(pt1(2) + (pt2(2) - pt1(2)) * t);
end
