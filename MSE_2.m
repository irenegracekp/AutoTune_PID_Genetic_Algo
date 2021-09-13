time_start = latestmsgs(Atti_1,1).Payload.time_boot_ms ;
MSE = 0;
MEAN = 0;
time_start_2 = latestmsgs(Atti_1,1).Payload.time_boot_ms ;
d = 0;
i = 2; 
l = 1;
dt = 0;
t_now_1 = 0;
t_now_2 = 0;
t0 = clock;
while etime(clock, t0) < 5
    
    k1 = latestmsgs(Atti_1,1).Payload;
    k2 = latestmsgs(Atti_2,1).Payload; 
    
    if(ismember(k2.time_boot_ms,t_now_2) && ismember(k1.time_boot_ms,t_now_1)) 
        p = 0;
    else
        l = l+1;
        p = 1;
        t_now_2(l) = k2.time_boot_ms ;
        t_now_1(l) = k1.time_boot_ms ;
    end
    t = t_now_1(l);
    d(l) = t_now_1(l) - t_now_2(l);
    if abs(d(l)) < 15 && p == 1plot
        target = k2.body_roll_rate ;
        val = k1.rollspeed ;
        MEAN = MSE(i-1)*(i-1) + (target - val)^2  ;
        MSE(i) = MEAN/i ;
        i = i+1;
    end   
    
end
MSE(1) = MSE(2);
disconnect(gcsNode);



