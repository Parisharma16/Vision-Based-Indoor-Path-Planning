function display_specific_shortest_path(startNode, endNode, next, validCentroids, newImg)
    % Plot the centroids on the new image
    figure;
    imshow(newImg); hold on;
    plot(validCentroids(:, 1), validCentroids(:, 2), 'r*', 'MarkerSize', 10);

    for i = 1:size(validCentroids, 1)
        text(validCentroids(i, 1), validCentroids(i, 2), sprintf('%d', i), 'Color', 'black', 'FontSize', 12);
    end

    % Plot the specific shortest path
    u = startNode;
    while u ~= endNode
        v = next(u, endNode);
        pt1 = validCentroids(u, :);
        pt2 = validCentroids(v, :);
        [pathX, pathY] = interpolatePath(pt1, pt2);
        plot(pathX, pathY, 'b-', 'LineWidth', 2);  % Path marked in blue
        u = v;
    end

    hold off;

    % Save the image
    saveas(gcf, 'SpecificShortestPath.png');
    close(gcf);

    disp('Specific shortest path image saved as SpecificShortestPath.png');
end


function [pathX, pathY] = interpolatePath(pt1, pt2)
    numPoints = 50;
    t = linspace(0, 1, numPoints);
    pathX = round(pt1(1) + (pt2(1) - pt1(1)) * t);
    pathY = round(pt1(2) + (pt2(2) - pt1(2)) * t);
end
