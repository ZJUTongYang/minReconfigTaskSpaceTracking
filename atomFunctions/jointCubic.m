function result = jointCubic(source)
% X, Y, Z, T: 1*n array, the data that requires interpolation
% t: 1*n array, the required timestamp

% We adjust within pi rad
motion = zeros(size(source, 1), size(source, 2));
motion(:, 7) = source(:, 7);
motion(1, :) = source(1, :);
for i = 2:size(source, 1)
    motion(i, 1:6) = motion(i-1, 1:6) + wrapToPi(source(i, 1:6) - motion(i-1, 1:6));
end

source = motion;

speed = 0.4;
t = zeros(size(source, 1), 1);
for i = 2:size(source, 1)
    t(i) = t(i-1) + norm(wrapToPi(source(i, 1:5) - source(i-1, 1:5)))/speed;
end

N = size(source, 1)*2;
t_result = linspace(0, floor(t(end))+1, N)';

result = zeros(size(t_result, 1), 7);

result(:, 1) = spline(t, source(:, 1), t_result);
result(:, 2) = spline(t, source(:, 2), t_result);
result(:, 3) = spline(t, source(:, 3), t_result);
result(:, 4) = spline(t, source(:, 4), t_result);
result(:, 5) = spline(t, source(:, 5), t_result);
result(:, 6) = spline(t, source(:, 6), t_result);

cur = 1;
i = 1;
while i <= size(result, 1)
    if t_result(i) > t(end)
        result(i, 7) = source(end, 7);
        i = i + 1;
    elseif t_result(i) >= t(cur) && t_result(i) < t(cur+1)
        result(i, 7) = source(cur, 7);
        i = i + 1;
    else
        cur = cur + 1;
    end
end



