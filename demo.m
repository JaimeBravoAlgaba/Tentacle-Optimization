nSections = 3; % Number of sections in the tentacle.

a = [0.6, 0.3, 0.1]; % Vector with the sections' lengths.

if length(a) ~= nSections
    logService("WARN", 'Dimension of "a" differs from the number of sections.');
    return;
end

dh = dhMatrix(a);   % Denavit-Hartemberg Matrix.

[robot, endEffector] = createRobot(dh); % Creation of the tentacle and getting the endEffector name for future uses.
robot.DataFormat = 'column';

ik = inverseKinematics('RigidBodyTree',robot); % IKinematic solver.

targetPos = [1 0 0];
targetTform = trvec2tform(targetPos);
weights = [0 0 0 1 1 1];
initialguess = robot.homeConfiguration;

configSol = ik(endEffector,targetTform,weights,initialguess); % Solves the the inverse kinematics.

currTform = getTransform(robot,configSol,endEffector);
currPos = tform2trvec(currTform);

errPos = norm(currPos - targetPos); % Calculating the position error.

J = robot.geometricJacobian(configSol, endEffector);
Jv = J(4:5, :);
mv = yoshikawa(Jv); % Calculating the manipulability parameter in the current position.