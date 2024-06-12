function h = DrawSphere_End(x, y, z, R)
    % Create a unit sphere
    [X, Y, Z] = sphere;

    % Scale and translate the unit sphere to the desired location and size
    X = R*X + x;
    Y = R*Y + y;
    Z = R*Z + z;

    % Plot the sphere
    h = surf(X, Y, Z, 'FaceColor', 'b', 'EdgeColor', 'none');
    axis([-200,200,-200,200,-200,300]);
end