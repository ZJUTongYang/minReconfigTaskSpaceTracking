function solution = lineSolver(nodes)

% We use a row array to mark a solution:
% [currentcost, c1, c2, ..., cn]
solution = zeros(1, size(nodes, 1)+1);
for step = 1:size(nodes, 1)
solution = solveDP(nodes, solution, step);
end

mincost = min(solution(:, 1));
solution = solution(solution(:, 1) == mincost, :);

end

function newsolution = solveDP(nodes, solution, step)
color = nodes(step).possible_color_;
if isempty(color)
    newsolution = solution;
    newsolution(:, step+1) = -1;
    return ;
end
n = size(solution, 1);
newsolution = repmat(solution, [size(color, 1), 1]);
for i = 1:size(color, 1)
    newsolution((i-1)*n+1:i*n, step+1) = color(i);
end
newsolution(:, 1) = newsolution(:, 1) + 1*(newsolution(:, step) ~= newsolution(:, step+1));

%% We only preserve the least-cost solutions for each frontier state
preserve = zeros(size(newsolution, 1), 1);
for i = 1:size(color, 1)
    cost = newsolution((i-1)*n+1:i*n, 1);
    mincost = min(cost);
    preserve((i-1)*n+1:i*n, 1) = (cost == mincost);
end
newsolution = newsolution(preserve == 1, :);

end
