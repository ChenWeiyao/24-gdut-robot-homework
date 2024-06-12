function h = drawSphere(x, y, z, R)
    % Create a unit sphere
    [X, Y, Z] = sphere;

    % Scale and translate the unit sphere to the desired location and size
    X = R*X + x;
    Y = R*Y + y;
    Z = R*Z + z;

    % Plot the sphere
    h = surf(X, Y, Z, 'FaceColor', 'g', 'EdgeColor', 'none');
    axis([-400,400,-400,400,-200,500]);
end