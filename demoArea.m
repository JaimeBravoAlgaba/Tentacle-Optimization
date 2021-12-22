    a = 1/10.*[4.046932052213479,2.436623648727864,1.449614448815573,2.067288390760513];
    dh = dhMatrix(a);   % Denavit-Hartemberg Matrix.
    [robot, endEffector] = createRobot(dh); % Creation of the tentacle and getting the endEffector name for future uses.
    robot.DataFormat = 'column';
    ik = inverseKinematics('RigidBodyTree',robot); % IKinematic solver.

    weights = [0 0 0 1 1 1];
    initialguess = robot.homeConfiguration;
    
    nPoints = 1000;

    x = zeros(nPoints, 1);
    y = x;
    z = x;

    for i = 1:nPoints
        config = robot.randomConfiguration;        
        currTform = getTransform(robot, config, endEffector);
        currPos = tform2trvec(currTform);
        x(i)= currPos(1);
        y(i)= currPos(2);
        z(i)= currPos(3);
    end
    figure;
    scatter(x, y, 'b');
    axis equal;
    K = boundary([x, y], 0.6);

    figure;
    scatter(x(K), y(K), 'r');
    axis equal;
    area = polyarea(x(K), y(K));

    count = length(K);
    qs = zeros(count, length(a));
    for i=1:count
        targetTform = trvec2tform([x(K(i)) y(K(i)) 0]);
        qSol = ik(endEffector,targetTform,weights,initialguess);
        qs(i, :) = qSol;
        initialguess = qSol;
    end

    figure
    show(robot,qs(1,:)');
    view(2);
    ax = gca;
    ax.Projection = 'orthographic';
    hold on;
    plot(x(K), y(K),'k');
    axis([-2 2 -2 2]); 

    framesPerSecond = 7;
    r = rateControl(framesPerSecond);
    for i = 1:count
        show(robot,qs(i,:)','PreservePlot',false);
        drawnow
        waitfor(r);
    end 

