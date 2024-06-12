function Connect3D_column(p1, p2, radius, color)
    % 提取 x, y, z 值
    p1 = p1(1:3);
    p2 = p2(1:3);
    
    % 计算两点之间的距离
    len = norm(p2 - p1);
    
    % 创建一个单位圆柱
    [X, Y, Z] = cylinder([radius radius]);
    
    % 缩放 Z 轴以匹配所需的长度
    Z = Z * len;
    
    % 将圆柱旋转和平移到正确的位置
    [az, el, ~] = cart2sph(p2(1) - p1(1), p2(2) - p1(2), p2(3) - p1(3));
    rotm = eul2rotm([az -pi/2 -el]);
    XYZ = rotm * [X(:)'; Y(:)'; Z(:)'];
    X = reshape(XYZ(1, :) + p1(1), size(X));
    Y = reshape(XYZ(2, :) + p1(2), size(Y));
    Z = reshape(XYZ(3, :) + p1(3), size(Z));
    
    % 绘制圆柱
    h = surf(X, Y, Z);
    set(h, 'FaceColor', color, 'EdgeColor', 'none');
end