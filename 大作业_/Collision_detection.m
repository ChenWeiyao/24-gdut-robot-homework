function result = Collision_detection (P1, P2, C, r)

% 判断空间线段与球是否相交
% 输入参数：
% P1, P2: 线段的两个端点坐标，3x1向量
% C: 球心坐标，3x1向量
% r: 球的半径
% 输出参数：
% result: 相交返回1，否则返回0

D = P2 - P1;
L = norm(cross(C-P1, D)) / norm(D);
if L > r
    result = 0;
else
    t = dot(C-P1, D) / dot(D, D);
    if t < 0
        Q = P1;
    elseif t > 1
        Q = P2;
    else
        Q = P1 + t*D;
    end
    d = norm(C - Q);
    if d > r
        result = 0;
    else
        result = 1;
    end
end
end
