path = [0 0; 7 7; 7 17; 0 27; -7 17; -7 7; 0 0];
robotInitialLocation = path(1,:);
initialOrientation = 0.78;
robotCurrentPose = [robotInitialLocation initialOrientation]';
sampleTime = 0.001;
vizRate = rateControl(2/sampleTime);

k = 1;
realPoseY = 0; realPoseX = 0;
realPoseY(k) = 0; realPoseX(k) = 0;
realW1 = 0; realW2 = 0;
realW1(k) = 0; realW2(k) = 0;

figure

i =1;
while ( i < 7 )
    eold = 0;
    E = 0;
    E1 = 0;
    eold1 = 0;   
    theta = myTheta(robotCurrentPose, path, i);
    w1 = 0;
    w2 = 0;
    eold1a = 0;
    eold2a = 0;
    
    while( norm(robotCurrentPose(1:2)' - path(i+1,:)) > 0.1 )
                
        [v, w, E, eold, E1, eold1] = controller(robotCurrentPose', [path(i+1,:),theta], E, eold, E1, eold1);
        [C, w1, w2, eold1a, eold2a] = myKinematics(v, w, theta, w1, w2, eold1a, eold2a);
        robotCurrentPose = myPose(robotCurrentPose, C, sampleTime)
        
        k = k + 1;
        realPoseX(k) = robotCurrentPose(1);
        realPoseY(k) = robotCurrentPose(2);
        realW1(k) = w1;
        realW2(k) = w2;
        
    hold off
    plot(path(:,1), path(:,2),"k--d")
    hold all
    
    % Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0];
    plot(realPoseX,realPoseY);
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "View","3D", "FrameSize", 2);
    light;
    xlim([-13 13])
    ylim([-5 35])  
    waitfor(vizRate);
    end
   i = i+1;
end

function [w1, w2] = getWheelSpeeds(v, w)
    d = 1;
    r = 1;
    w1 = (v - w*d)/r;
    w2 = (v + w*d)/r;
 end
 
function [v, w, E, eold, E1, eold1] = controller(curPose, finalPose, E, eold, E1, eold1)
    e = (finalPose(1:2) - curPose(1:2));
    edot = e - eold ;
    E = E + e;
    U = 200*e + 1*edot + 1*E;
    eold = e; 
    
    e1 = (finalPose(3) - curPose(3));
    edot1 = e1 - eold1 ;
    E1 = E1 + e1;
    U1 = 10000*e1 + 10*edot1 + 0.001*E1;
    eold1 = e1; 
    v = sqrt(U(1)*U(1) + U(2)*U(2));
    w = U1;
end
 
function [wfinal, eold] = myAlphaCheck(wcur,w, eold)
    sampleTime = 0.001  ;
    e = (w - wcur);
    edot = e-eold;
    U = 0.1*e + 10*edot ;
    U = min(max(-100,U), 350) ;
    wfinal = wcur + U*sampleTime ; 
    eold = e;
end

function [C, w1cur, w2cur, eold1, eold2] = myKinematics(v, w, theta, w1cur, w2cur, eold1, eold2)
    [w1, w2] = getWheelSpeeds(v, w);
    [w1cur, eold1] = myAlphaCheck(w1cur , w1, eold1);
    [w2cur, eold2] = myAlphaCheck(w2cur , w2, eold2);
    d = 1;
    r = 1;
    A = [cos(theta) -sin(theta) 0 ;sin(theta) cos(theta) 0 ; 0 0 1];
    B = [r*0.5*(w1cur+w2cur); 0; r/d*0.5*(w2cur-w1cur)];
    C = mtimes(A ,B);
end

function CurrentPose = myPose(robotCurrentPose, C, sampleTime)
    CurrentPose = robotCurrentPose + C*sampleTime;
end

function theta = myTheta(robotCurrentPose, path, i)
    if (i == 1 || i == 2 )
        theta = atan((robotCurrentPose(2)-path(i+1,2))/(robotCurrentPose(1)-path(i+1,1)));
    elseif( i == 6)
        theta = 2*pi + atan((robotCurrentPose(2)-path(i+1,2))/(robotCurrentPose(1)-path(i+1,1)));
    else
        theta = pi + atan((robotCurrentPose(2)-path(i+1,2))/(robotCurrentPose(1)-path(i+1,1)));
    end
end
