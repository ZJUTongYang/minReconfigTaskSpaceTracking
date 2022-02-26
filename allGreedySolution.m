function solution = allGreedySolution(nodes)

% We use a row array to mark a solution:
% [currentcost, c1, c2, ..., cn]
solution = zeros(1, size(nodes, 1)+1);
for step = 1:size(nodes, 1)
solution = solveDP(nodes, solution, step);
end

% mincost = min(solution(:, 1));
% solution = solution(solution(:, 1) == mincost, :);

end

function newsolution = solveDP(nodes, solution, step)
color = nodes(step).possible_color_;
if isempty(color)
    newsolution = solution;
    newsolution(:, step+1) = -1;
    return ;
end

% n = size(solution, 1);
newsolution = zeros(size(solution, 1)*size(color, 1), size(solution, 2));
count = 1;
for i = 1:size(solution, 1)
    if step ~= 1 && any(color == solution(i, step))
        newsolution(count, :) = solution(i, :);
        newsolution(count, step+1) = solution(i, step);
        count = count + 1;
    else
        for j = 1:size(color, 1)
            newsolution(count, :) = solution(i, :);
            newsolution(count, step+1) = color(j);
            newsolution(count, 1) = solution(i, 1) + 1;
            count = count + 1;
        end
    end
end
newsolution = newsolution(1:count-1, :);


%% We only preserve the least-cost solutions for each frontier state
% newsolution = sortrows(newsolution, [step+1, 1]);
% preserve = zeros(size(newsolution, 1), 1);
% preserve(1, 1) = 1;
% for i = 2:size(newsolution, 1)
%     if newsolution(i, step+1) == newsolution(i-1, step+1)
%         if preserve(i-1, 1) ~= 0 && newsolution(i, 1) == newsolution(i-1, 1)
%             preserve(i, 1) = 1;
%         end
%     else
%         preserve(i, 1) = 1;
%     end
% end
% newsolution = newsolution(preserve == 1, :);

end
