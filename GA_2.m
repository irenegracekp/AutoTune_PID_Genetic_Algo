function J = GA_2(Gain)

global target_roll
global val_roll
global target_yaw
global val_yaw 
global target_pitch 
global val_pitch 
global dialect
global gcsNode
global Atti_1
global Atti_2
global tr;
global tp;
global ty;
global vr;
global vp;
global vy;
global k;

msg = createmsg(dialect,"PARAM_SET");
msg.Payload.target_system(:) = 1;
msg.Payload.target_component(:) = 1;
msg.Payload.param_id(1:13) = "MC_ROLLRATE_P";
msg.Payload.param_value(:) = Gain(1);
msg.Payload.param_type(:) = 9;
sendudpmsg(gcsNode,msg,"127.0.0.1",14580)

msg = createmsg(dialect,"PARAM_SET");
msg.Payload.target_system(:) = 1;
msg.Payload.target_component(:) = 1;
msg.Payload.param_id(1:13) = "MC_ROLLRATE_I";
msg.Payload.param_value(:) = Gain(2);
msg.Payload.param_type(:) = 9;
sendudpmsg(gcsNode,msg,"127.0.0.1",14580)

msg = createmsg(dialect,"PARAM_SET");
msg.Payload.target_system(:) = 1;
msg.Payload.target_component(:) = 1;
msg.Payload.param_id(1:13) = "MC_ROLLRATE_D";
msg.Payload.param_value(:) = Gain(3);
msg.Payload.param_type(:) = 9;
sendudpmsg(gcsNode,msg,"127.0.0.1",14580)

msg = createmsg(dialect,"PARAM_SET");
msg.Payload.target_system(:) = 1;
msg.Payload.target_component(:) = 1;
msg.Payload.param_id(1:14) = "MC_PITCHRATE_P";
msg.Payload.param_value(:) = Gain(4);
msg.Payload.param_type(:) = 9;
sendudpmsg(gcsNode,msg,"127.0.0.1",14580)

msg = createmsg(dialect,"PARAM_SET");
msg.Payload.target_system(:) = 1;
msg.Payload.target_component(:) = 1;
msg.Payload.param_id(1:14) = "MC_PITCHRATE_I";
msg.Payload.param_value(:) = Gain(5);
msg.Payload.param_type(:) = 9;
sendudpmsg(gcsNode,msg,"127.0.0.1",14580)

msg = createmsg(dialect,"PARAM_SET");
msg.Payload.target_system(:) = 1;
msg.Payload.target_component(:) = 1;
msg.Payload.param_id(1:14) = "MC_PITCHRATE_D";
msg.Payload.param_value(:) = Gain(6);
msg.Payload.param_type(:) = 9;
sendudpmsg(gcsNode,msg,"127.0.0.1",14580)

msg = createmsg(dialect,"PARAM_SET");
msg.Payload.target_system(:) = 1;
msg.Payload.target_component(:) = 1;
msg.Payload.param_id(1:12) = "MC_YAWRATE_P";
msg.Payload.param_value(:) = Gain(7);
msg.Payload.param_type(:) = 9;
sendudpmsg(gcsNode,msg,"127.0.0.1",14580)

msg = createmsg(dialect,"PARAM_SET");
msg.Payload.target_system(:) = 1;
msg.Payload.target_component(:) = 1;
msg.Payload.param_id(1:12) = "MC_YAWRATE_I";
msg.Payload.param_value(:) = Gain(8);
msg.Payload.param_type(:) = 9;
sendudpmsg(gcsNode,msg,"127.0.0.1",14580)

msg = createmsg(dialect,"PARAM_SET");
msg.Payload.target_system(:) = 1;
msg.Payload.target_component(:) = 1;
msg.Payload.param_id(1:12) = "MC_YAWRATE_D";
msg.Payload.param_value(:) = Gain(9);
msg.Payload.param_type(:) = 9;
sendudpmsg(gcsNode,msg,"127.0.0.1",14580)
pause(1);

MSE = 0; 
d = 0;
i = 2; 
l = 1;
t_now_1 = 0;
t_now_2 = 0;
t0 = clock;
while etime(clock, t0) < 1
    if isempty(latestmsgs(Atti_1,1))
        continue
    else
        k1 = latestmsgs(Atti_1,1).Payload;
    end
    
    if isempty(latestmsgs(Atti_2,1))
        continue
    else
        k2 = latestmsgs(Atti_2,1).Payload; 
    end
    
    if(ismember(k2.time_boot_ms,t_now_2) && ismember(k1.time_boot_ms,t_now_1)) 
        p = 0;
    else
        l = l+1;
        p = 1;
        t_now_2(l) = k2.time_boot_ms ;
        t_now_1(l) = k1.time_boot_ms ;
    end 
    d(l) = t_now_1(l) - t_now_2(l);
    if abs(d(l)) < 15 && p == 1
        target_roll(i-1) = k2.body_roll_rate ;
        val_roll(i-1) = k1.rollspeed ;
        target_yaw(i-1) = k2.body_yaw_rate ;
        val_yaw(i-1) = k1.yawspeed ;
        target_pitch(i-1) = k2.body_pitch_rate ;
        val_pitch(i-1) = k1.pitchspeed ;
        
        addpoints(tr,k,target_roll(i-1));
        addpoints(vr,k,val_roll(i-1));
        
        addpoints(tp,k,target_pitch(i-1));
        addpoints(vp,k,val_pitch(i-1));
        
        addpoints(ty,k,target_yaw(i-1));
        addpoints(vy,k,val_yaw(i-1));
        drawnow limitrate
        
        MEAN = MSE(i-1)*(i-1) + (target_roll(i-1) - val_roll(i-1))^2 + (target_yaw(i-1) - val_yaw(i-1))^2 + (target_pitch(i-1) - val_pitch(i-1))^2;
        MSE(i) = MEAN/i ;
        i = i+1;
        k = k + 1; 
    end   
end
J = MSE(i-1);


