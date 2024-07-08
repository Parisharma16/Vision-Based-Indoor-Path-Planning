function [adjacencyMatrix, validCentroids] = create_adjacency_matrix(Img, semanobj, seman, mask)
    a = rgb2gray(semanobj);
    stats = regionprops(a);
    areas = [stats.Area];
    centroids = cat(1, stats.Centroid);
    mxar = max(areas);
    indx = find(areas == mxar);
    cen = centroids(indx, :);
    cen = floor(cen);

    cen(cen < 1) = 1;
    cen(cen(:, 1) > size(a, 1), 1) = size(a, 1);
    cen(cen(:, 2) > size(a, 2), 2) = size(a, 2);

    pxvalue = a(cen(1), cen(2));

    a(a ~= indx) = 0;
    ind = find(a == indx);
    a(ind) = 1;

    % New map
    newmap = mask .* a;
    newmap = imbinarize(newmap);

    semanedit(:, :, 1) = seman(:, :, 1) .* a;
    semanedit(:, :, 2) = seman(:, :, 2) .* a;
    semanedit(:, :, 3) = seman(:, :, 3) .* a;

    grsem = rgb2gray(semanedit);
    seg = unique(grsem);

    nnum = 1;
    nodes = [];
    for i = 1:numel(seg)
        reg = seg(i);
        if reg ~= 0
            [rows, cols] = find(grsem == reg);
            regarea(nnum) = numel(rows);
            region_image = zeros(size(grsem));
            region_image(sub2ind(size(grsem), rows, cols)) = 1;
            vacreg{nnum} = region_image;
            min_row = min(rows);
            max_row = max(rows);
            min_col = min(cols);
            max_col = max(cols);
            bbox(nnum, :) = [min_row, min_col, max_col - min_col + 1, max_row - min_row + 1];
            centroid_row = (min_row + max_row) / 2;
            centroid_col = (min_col + max_col) / 2;
            reg_bboxcen{nnum} = [centroid_row, centroid_col];
            regcen = [mean(cols), mean(rows)];
            nodes = [nodes; regcen];
            nnum = nnum + 1;
        end
    end

    % Exclude centroids on objects by using mask
    validCentroids = [];
    for i = 1:size(nodes, 1)
        node = nodes(i, :);
        if mask(floor(node(2)), floor(node(1))) == 0
            validCentroids = [validCentroids; node];
        end
    end

    
    % Create adjacency matrix
    numNodes = size(validCentroids, 1);
    adjacencyMatrix = zeros(numNodes);

    % Populate adjacency matrix using interpolated paths
    for i = 1:numNodes
        for j = i+1:numNodes
            pt1 = validCentroids(i, :);
            pt2 = validCentroids(j, :);

            [pathX, pathY] = interpolatePath(pt1, pt2);
            if ~checkPathIntersection(pathX, pathY, mask)
                adjacencyMatrix(i, j) = 1;
                adjacencyMatrix(j, i) = 1;
            end
        end
    end

    

    % Display valid centroids
    disp('Valid Centroids:');
    disp(validCentroids);

    % Plot room image with centroids
    figure;
    imshow(Img);
    hold on;
    plot(validCentroids(:, 1), validCentroids(:, 2), 'r*', 'MarkerSize', 10);
    for i = 1:size(validCentroids, 1)
        text(validCentroids(i, 1), validCentroids(i, 2), num2str(i), 'Color', 'black', 'FontSize', 12);
    end

    % Save image with centroids
    saveas(gcf, 'FloorMapWithCentroids.png');
    hold off;
    close(gcf);
end

function [pathX, pathY] = interpolatePath(pt1, pt2)
    numPoints = 50;
    t = linspace(0, 1, numPoints);
    pathX = round(pt1(1) + (pt2(1) - pt1(1)) * t);
    pathY = round(pt1(2) + (pt2(2) - pt1(2)) * t);
end

function intersects = checkPathIntersection(pathX, pathY, mask)
    intersects = false;
    pathX = round(pathX);
    pathY = round(pathY);

    for k = 1:length(pathX)
        x = pathX(k);
        y = pathY(k);

        if x >= 1 && x <= size(mask, 2) && y >= 1 && y <= size(mask, 1)
            if mask(y, x) > 0
                intersects = true;
                return;
            end
        end
    end
end
