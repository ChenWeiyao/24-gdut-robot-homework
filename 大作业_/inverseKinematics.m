function IK_muzhi = inverseKinematics(targetPosition, targetOrientation, Link)
    % targetPosition: 目标位置
    % targetOrientation: 目标方向
    % Link: DH参数

    % 提取出链接长度和偏距
    % a2 = Link(3).dx;
    % a3 = Link(4).dx;
    % d1 = Link(2).dz;
    % d4 = Link(5).dz;

    % 计算出目标位置的坐标
    x = targetPosition(1);
    y = targetPosition(2);
    z = targetPosition(3);

    % 计算关节角度
    % 计算关节角度
    

    % 检查关节角度是否在允许的范围内
    q2 = max(min(q2, 35 * ToRad), -15 * ToRad);
    q3 = max(min(q3, 0 * ToRad), -60 * ToRad);
    q4 = max(min(q4, 0 * ToRad), -50 * ToRad);
    q5 = max(min(q5, 0 * ToRad), -90 * ToRad);

    % 返回关节角度
    IK_muzhi = [q1, q2, q3, q4, q5];
end