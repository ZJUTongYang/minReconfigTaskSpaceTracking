function [xf, yf, zf] = cubic3D(X, Y, Z, T, t)
% X, Y, Z, T: 1*n array, the data that requires interpolation
% t: 1*n array, the required timestamp

xf = spline(T, X, t);
yf = spline(T, Y, t);
zf = spline(T, Z, t);

