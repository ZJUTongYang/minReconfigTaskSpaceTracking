function defineStates(vertices, epsilon_cont)

colorindex = 1;% The index of the next colour to be assigned
seed = [];

frontier = [];

adj_matrix = [];

i = 1;
while i <= size(vertices, 1)
    if isempty(vertices(i).gIK_)
        i = i + 1;
        frontier = [];
        adj_matrix = [];
        continue;
    end
    
    if isempty(frontier)
        % Find the first uncoloured IK to be the seed
        for j = 1:size(vertices(i).gIK_, 1)
                vertices(i).gIK_(j, 1) = colorindex + (j-1);
        end
        colorindex = colorindex + size(vertices(i).gIK_, 1);
        frontier = vertices(i).gIK_;
        i = i + 1;
        continue;
    end

    % Find the nearest row to the seed
    m = size(frontier, 1);
    n = size(vertices(i).gIK_, 1);
    adj_matrix = zeros(m, n);
    for j = 1:m
        for k = 1:n
            adj_matrix(j, k) = max(abs(wrapToPi(frontier(j, 2:end) - vertices(i).gIK_(k, 2:end))));
        end
    end

    %% Match configurations
    cycle = min(m, n);
    while cycle > 0
        mm = min(min(adj_matrix));
        if mm == 999
            break;
        end
        [row, column] = find(adj_matrix == mm);
        row = row(1);
        column = column(1);
        if adj_matrix(row, column) < epsilon_cont
            vertices(i).gIK_(column, 1) = frontier(row, 1);
            adj_matrix(row, :) = 999;
            adj_matrix(:, column) = 999;
        end
        cycle = cycle - 1;
    end

    %% Assign left-out configurations
    for j = 1:size(vertices(i).gIK_, 1)
        if vertices(i).gIK_(j, 1) == 0
            vertices(i).gIK_(j, 1) = colorindex;
            colorindex = colorindex + 1;
        end
    end

    frontier = vertices(i).gIK_;
    i = i + 1;
end


end
