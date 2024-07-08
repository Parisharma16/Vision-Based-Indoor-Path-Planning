% Load images
Img = imread('/MATLAB Drive/Projects/DC_Final/first27June/RGB_LVL2.png');
imshow(Img);
semanobj = imread('/MATLAB Drive/Projects/DC_Final/first27June/SEMOBJ_LVL2.png');
seman = imread('/MATLAB Drive/Projects/DC_Final/first27June/SEMINST_LVL2.png');
mask = imread('/MATLAB Drive/Projects/DC_Final/first27June/MASK_LVL2.png');


% Create adjacency matrix with hardcoded centroids and get valid centroids
[adjacencyMatrix, validCentroids] = create_adjacency_matrix(Img, semanobj, seman, mask);


% Compute shortest paths and display results
compute_shortest_paths_with_specific_paths(adjacencyMatrix, validCentroids, Img);