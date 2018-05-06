% https://hobbyking.com/en_us/hobbyking-x-car-brushless-power-system-4000kv-60a.html
label = "BLDC_sys_4000KV_60A";
link = 'https://hobbyking.com/en_us/hobbyking-x-car-brushless-power-system-4000kv-60a.html';

I = 60; % [A]
Ipeak = 360; % [A]
R = 0.0006; %[R]
V = 7.2; %[V] 2-3 cells Lipo

kv = 4000; %[RPM/V]
Turns = 8.5; %[?] 

save(label)
