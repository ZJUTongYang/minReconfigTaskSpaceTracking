function q_sols = analyticUR5IK(T, q6_des)
% T: 4*4 tform matrix
% q_sols: maximum 8*6 matrix

use_gripper = false;

if nargin == 1
    q6_des = 0;
end


ZERO_THRESH = 0.00000001;
d1 =  0.089159;
a2 = -0.42500;
a3 = -0.39225;
d4 =  0.10915;
d5 =  0.09465;
if use_gripper 
%     d6 = 0.15; % for main_3
    d6 = 0.27;
else
    d6 =  0.0823;
end

num_sols = 1;
q_sols = [];

T02 = -T(1, 1); T00 = T(1, 2); T01 = T(1, 3); T03 = -T(1, 4);
T12 = -T(2, 1); T10 = T(2, 2); T11 = T(2, 3); T13 = -T(2, 4);
T22 = T(3, 1); T20 = -T(3, 2); T21 = -T(3, 3); T23 = T(3, 4);

%% Shoulder rotate joint (q1)
q1 = zeros(1, 2);
A = d6*T12 - T13;
B = d6*T02 - T03;
R = A*A + B*B;
if abs(A) < ZERO_THRESH
    if abs(abs(d4) - abs(B)) < ZERO_THRESH
        div = -sign(d4)*sign(B);
    else
        div = -d4/B;
    end
    arcsin = asin(div);
    if abs(arcsin) < ZERO_THRESH
        arcsin = 0.0;
    end
    if arcsin < 0.0
        q1(1, 1) = arcsin + 2.0*pi;
    else
        q1(1, 1) = arcsin;
    end
    q1(1, 2) = pi - arcsin;
elseif abs(B) < ZERO_THRESH
    if abs(abs(d4) - abs(A)) < ZERO_THRESH
        div = sign(d4)*sign(A);
    else
        div = d4/A;
    end
    arccos = acos(div);
    q1(1, 1) = arccos;
    q1(1, 2) = 2.0*pi - arccos;
    
elseif d4*d4 > R
    % There is no solution
    return ;
else
    arccos = acos(d4 / sqrt(R)) ;
    arctan = atan2(-B, A);
    pos = arccos + arctan;
    neg = -arccos + arctan;
    if abs(pos) < ZERO_THRESH
        pos = 0.0;
    end
    if abs(neg) < ZERO_THRESH
        neg = 0.0;
    end
    if pos >= 0.0
        q1(1, 1) = pos;
    else
        q1(1, 1) = 2.0*pi + pos;
    end
    if neg >= 0.0
        q1(1, 2) = neg;
    else
        q1(1, 2) = 2.0*pi + neg;
    end
end

%% Wrist 2 joint (q5)
q5 = zeros(2, 2);
for i = 1:2
    numer = (T03*sin(q1(i)) - T13*cos(q1(i))-d4);
    if abs(abs(numer) - abs(d6)) < ZERO_THRESH
        div = sign(numer) * sign(d6);
    else
        div = numer / d6;
    end
    arccos = acos(div);
    q5(i, 1) = arccos;
    q5(i, 2) = 2.0*pi - arccos;
end

for i = 1:2
    for j = 1:2
        c1 = cos(q1(i));
        s1 = sin(q1(i));
        c5 = cos(q5(i, j));
        s5 = sin(q5(i, j));
        %% wrist 3 joint (q6)
        if abs(s5) < ZERO_THRESH
            q6 = q6_des;
        else
            q6 = atan2(sign(s5)*-(T01*s1 - T11*c1), ...
                sign(s5)*(T00*s1 - T10*c1));
            if abs(q6) < ZERO_THRESH
                q6 = 0.0;
            end
            if q6 < 0.0
                q6 = q6 + 2.0*pi;
            end
        end
        
        q2 = zeros(1, 2);
        q3 = zeros(1, 2);
        q4 = zeros(1, 2);
        %% RRR joints (q2,q3,q4)
        c6 = cos(q6);
        s6 = sin(q6);
        x04x = -s5*(T02*c1 + T12*s1) - c5*(s6*(T01*c1 + T11*s1) - c6*(T00*c1 + T10*s1));
        x04y = c5*(T20*c6 - T21*s6) - T22*s5;
        p13x = d5*(s6*(T00*c1 + T10*s1) + c6*(T01*c1 + T11*s1)) - d6*(T02*c1 + T12*s1) + T03*c1 + T13*s1;
        p13y = T23 - d1 - d6*T22 + d5*(T21*c6 + T20*s6);
        
        c3 = (p13x*p13x + p13y*p13y - a2*a2 - a3*a3) / (2.0*a2*a3);
        if abs(abs(c3) - 1.0) < ZERO_THRESH
            c3 = sign(c3);
        elseif abs(c3) > 1.0
            continue;
        end
        arccos = acos(c3);
        q3(1, 1) = arccos;
        q3(1, 2) = 2.0*pi - arccos;
        denom = a2*a2 + a3*a3 + 2*a2*a3*c3;
        s3 = sin(arccos);
        A = (a2 + a3*c3);
        B = a3*s3;
        
        q2(1, 1) = atan2((A*p13y - B*p13x) / denom, (A*p13x + B*p13y) / denom);
        q2(1, 2) = atan2((A*p13y + B*p13x) / denom, (A*p13x - B*p13y) / denom);
        c23_0 = cos(q2(1)+q3(1));
        s23_0 = sin(q2(1)+q3(1));
        c23_1 = cos(q2(2)+q3(2));
        s23_1 = sin(q2(2)+q3(2));
        
        q4(1, 1) = atan2(c23_0*x04y - s23_0*x04x, x04x*c23_0 + x04y*s23_0);
        q4(1, 2) = atan2(c23_1*x04y - s23_1*x04x, x04x*c23_1 + x04y*s23_1);
        
        for k = 1:2
            if abs(q2(k)) < ZERO_THRESH
                q2(k) = 0.0;
            elseif q2(k) < 0.0
                q2(k) = q2(k) + 2.0*pi;
            end
            if abs(q4(k)) < ZERO_THRESH
                q4(k) = 0.0;
            elseif q4(k) < 0.0
                q4(k) = q4(k) + 2.0*pi;
            end
            
            q_sols(num_sols, 1) = q1(i);
            q_sols(num_sols, 2) = q2(k);
            q_sols(num_sols, 3) = q3(k);
            q_sols(num_sols, 4) = q4(k);
            q_sols(num_sols, 5) = q5(i, j);
            q_sols(num_sols, 6) = q6;
            num_sols = num_sols + 1;
        end
    end
end

end% endfunciton