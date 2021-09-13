time_start = latestmsgs(Atti_1,1).Payload.time_boot_ms ;
MSE = 0;
MEAN = 0;
time_start_2 = latestmsgs(Atti_1,1).Payload.time_boot_ms ;
d = 0;
i = 1; 
l = 2;
dt = 0;

while i < 50
    
    k1 = latestmsgs(Atti_1,1).Payload;
    k2 = latestmsgs(Atti_2,1).Payload; 
    t_now_2 = k2.time_boot_ms ;
    t_now_1 = k1.time_boot_ms ;
    d(l) = t_now_1 - t_now_2;
 
    if abs(d(l)) < 30''
        t = t_now_1;
        target = k2.body_roll_rate ;
        val = k1.rollspeed ;
        MEAN = MSE(l-1)*dt + (target - val)^2 ;
        m(i) = MEAN;
        i = i+1;
    end  
    m(l) = MEAN;    
    dt = double(t_now_1 - time_start);
    d_t(l) = dt;
    MSE(l) = MEAN/dt ;  
    l = l+1;
end
MSE(1) = MSE(2);
disconnect(gcsNode);