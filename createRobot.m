function [robot, endEffector] = createRobot(dh)
%CREATEROBOT Creates a Robot given its DH Matrix.
    robot = rigidBodyTree;
    dimensions = size(dh);
    nSections = dimensions(1);
    jointAmplitude = pi/1;

    body = rigidBody("body1");
    jnt = rigidBodyJoint('jnt1','revolute');
    jnt.PositionLimits = [-jointAmplitude/2 jointAmplitude/2];
    
    setFixedTransform(jnt,dh(1,:),'dh');
    body.Joint = jnt;
    
    addBody(robot,body,'base');

    for i=2:nSections
        body = rigidBody("body" + int2str(i));
        jnt = rigidBodyJoint("jnt" + int2str(i),'revolute');
        jnt.PositionLimits = [-jointAmplitude/2 jointAmplitude/2];
        
        setFixedTransform(jnt,dh(i,:) ,'dh');
        body.Joint = jnt;
        
        addBody(robot,body,"body" + int2str(i-1));     
    end

    endEffector = "body" + int2str(nSections);
end

