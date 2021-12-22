%% Inicializations:
    global nSections totalLength delta weights;
    nSections = 4; %Number of sections in a robot.

    totalLength = 10;
    minLength = 0.05*totalLength;

    delta = 0.5;
    weights = [0 0 0 1 1 1]; % Tolerances for the iKine calculations.
    
    iters = 0;
    generation = 0;

    maxGenerations = 50;
    populationSize = 50;
%% Restrictions: A·x <= b, Aeq·x = beq
    % No ineq
    A = [];
    b = [];
    
    % All lengths must sum up "totalLength"
    Aeq = ones(1, nSections);
    beq = totalLength;

    % Lower bound: all must be positive and non-zero
    lb = minLength * ones(1, nSections);
    ub = [];

%% Execution:
    options = optimoptions('ga', 'PlotFcn', @gaplotbestf, 'MaxGenerations', maxGenerations, 'PopulationSize', populationSize);
    %sol = ga(@evaluationCinManip, nSections, A, b, Aeq, beq, lb, ub, [], [], options);
    %sol = ga(@evaluationDynManip, nSections, A, b, Aeq, beq, lb, ub, [], [], options);
    sol = ga(@evaluationCond, nSections, A, b, Aeq, beq, lb, ub, [], [], options);
    %sol = ga(@evaluationPoint, nSections, A, b, Aeq, beq, lb, ub, [], [], options);
    %sol = ga(@evaluationArea, nSections, A, b, Aeq, beq, lb, ub, [], [], options);

%% Evaluation:
function r = evaluationCinManip(a)

    dh = dhMatrix(a);   % Denavit-Hartemberg Matrix.

    [robot, endEffector] = createRobot(dh); % Creation of the tentacle and getting the endEffector name for future uses.
    
    nPoints = 1000; % Number of points to study.
    m = 0;  % Mean Manipulability.

    for i = 1:nPoints            
        config = robot.randomConfiguration;
        J = robot.geometricJacobian(config, endEffector);
        Jv = J(4:5, :);
        m = m + yoshikawa(Jv); % Calculating the manipulability parameter in the current position.
    end

    r = nPoints / m;
end

function r = evaluationDynManip(a)

    dh = dhMatrix(a);   % Denavit-Hartemberg Matrix.

    [robot, endEffector] = createRobot(dh); % Creation of the tentacle and getting the endEffector name for future uses.
    robot.DataFormat = 'column';

    nPoints = 100; % Number of points to study.
    m = 0;  % Mean Dynamic Manipulability.

    parfor i = 1:nPoints            
        config = robot.randomConfiguration;
        J = robot.geometricJacobian(config, endEffector);
        Jv = J(4:5, :);
        M = robot.massMatrix(config);
        m = m + dynManip(Jv, M); % Calculating the manipulability parameter in the current position.
    end

    r = nPoints / m;
end

function r = evaluationCond(a)

    dh = dhMatrix(a);   % Denavit-Hartemberg Matrix.

    [robot, endEffector] = createRobot(dh); % Creation of the tentacle and getting the endEffector name for future uses.
    
    nPoints = 100; % Number of points to study.
    m = 0;  % Mean Condition Number.

    parfor i = 1:nPoints            
        config = robot.randomConfiguration;
        J = robot.geometricJacobian(config, endEffector);
        Jv = J(4:5, :);
        m = m + cond(Jv);
    end

    r = m / nPoints;
end

function r = evaluationPoint(a)
    global  totalLength delta weights;

    dh = dhMatrix(a);   % Denavit-Hartemberg Matrix.

    [robot, endEffector] = createRobot(dh); % Creation of the tentacle and getting the endEffector name for future uses.

    ik = inverseKinematics('RigidBodyTree',robot); % IKinematic solver.                
    initialguess = robot.homeConfiguration;

    targetPos = [0.5, 0.5, 0];
    targetTform = trvec2tform(targetPos);
    
    configSol = ik(endEffector,targetTform,weights,initialguess); % Solves the the inverse kinematics.

    J = robot.geometricJacobian(configSol, endEffector);
    Jv = J(4:5, :);
    m = yoshikawa(Jv); % Calculating the manipulability parameter in the current position.

    r = 1 / m;
end

function r = evaluationArea(a)
    dh = dhMatrix(a);   % Denavit-Hartemberg Matrix.
    [robot, endEffector] = createRobot(dh); % Creation of the tentacle and getting the endEffector name for future uses.

    nPoints = 100;
    x = -1*ones(nPoints, 1);
    y = x;
    
    % Random point cloud generation:
    for i = 1:nPoints
        while x(i) == -1
            config = robot.randomConfiguration;        
            currTform = getTransform(robot, config, endEffector);
            currPos = tform2trvec(currTform);
            if currPos(2) >= 0
                x(i)= currPos(1);
                y(i)= currPos(2);
            end
        end
    end
    k = boundary(x, y);    
    r = 1/polyarea(x(k), y(k));
end