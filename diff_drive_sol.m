path = [0 0; 7 7; 7 17; 0 27; -7 17; -7 7; 0 0; 0 0];
robotInitialLocation = path(1,:);
initialOrientation = 0;
robotCurrentPose = [robotInitialLocation initialOrientation]';

sampleTime = 0.001;

k = 1;
realPoseY = 0; realPoseX = 0;
realPoseY(k) = 0; realPoseX(k) = 0;
W1(k) = 0;
W2(k) = 0;
realW1 = 0; realW2 = 0;
theta = 0;

i =1;
goal = path(i,:);
while ( i <=8 )
    eold = [0 0];
    E = [0 0];
    [vref, wref, E, eold] = controller(robotCurrentPose, goal, E, eold);
    [w1ref, w2ref] =  getWheelSpeeds(vref,wref);
    [realW1, realW2] = dynamicModel(realW1, realW2, w1ref, w2ref);
    [v, w] = kinematicModel(realW1, realW2);
    robotCurrentPose = motionModel(robotCurrentPose, v, w);
    
    %disp(robotCurrentPose)
    k = k + 1;
    realPoseX(k) = robotCurrentPose(1);
    realPoseY(k) = robotCurrentPose(2);
    W1(k) = realW1;
    W2(k) = realW2;
    if(getDist(robotCurrentPose, goal) < 0.001)
        eold = [0 0];
        E = [0 0];
        disp(robotCurrentPose)
        goal = path(i,:);
        i = i+1;
        
    end
    
end
plot(realPoseX,realPoseY);

function [w1, w2] = getWheelSpeeds(v, w)
    d = 2;

    r = 0.5;
    
    w1 = (v + w*d)/r;
    w2 = (v - w*d)/r;
end
 

function dist = getDist(currPose, goal)
    dist = sqrt((goal(1)-currPose(1))^2 + (goal(2)-currPose(2))^2);

end


function [v, w, E, eold] = controller(curPose, finalPose, E, eold)
    vkp = 0.19;
    vkd = 0.01;
    vki = 0.01;

    wkp = 1.5;
    wkd = 0.3;
    wki = 0.1;
    
    e(1) = getDist(curPose, finalPose);
    e(2) = atan2(finalPose(2)-curPose(2), finalPose(1)-curPose(1)) - curPose(3);
    e(2) = wrapToPi(e(2));
    
    edot = e - eold ;
    E = E + e;
    eold = e;
    v = (vkp*e(1) + vkd*edot(1) + vki*E(1));
    w = (wkp*e(2) + wkd*edot(2) + wki*E(2));
    
end

function [v, w] = kinematicModel(w1, w2)
    d = 2;

    r = 0.5;
    v = 0.5*r*(w1+w2);
    w = 0.5*r*(w1-w2)/d;
end
 
function curPose = motionModel(curPose, v, w)
    sampleTime = 0.01;
    A = [cos(curPose(3)) -sin(curPose(3)) 0 ;sin(curPose(3)) cos(curPose(3)) 0 ; 0 0 1];
    B = [v; 0; w];
    C = mtimes(A ,B);
    curPose = curPose + C*sampleTime;
    curPose(3) = wrapToPi(curPose(3));
end

function [realW1, realW2] = dynamicModel(realW1, realW2, w1ref, w2ref)
   sampleTime = 0.01;
   Kp = 10;
   alpha_max = 5;
   W_max = 10;
   
   error1 =  w1ref - realW1;
   error2 =  w2ref - realW2;
   
   
   alpha1 = Kp*error1;
   alpha2 = Kp*error2;
   
   if abs(alpha1) > alpha_max
       alpha1 = alpha_max*abs(alpha1)/alpha1;
   end
   
   if abs(alpha2) > alpha_max
       alpha2 = alpha_max*abs(alpha2)/alpha2;
   end
   
   
   realW1 = realW1 + alpha1*sampleTime;
   realW2 = realW2 + alpha2*sampleTime;
   
   if abs(realW1) > W_max
       realW1 = W_max*abs(realW1)/realW1;
   end    
   
   if abs(realW2) > W_max
       realW2 = W_max*abs(realW2)/realW2;
   end    
   
end
