% https://hobbyking.com/nl_nl/trackstar-1-8th-2050kv-brushless-sensorless-motor.html
label = 'BLDC_mot_2060KV_140A';
link = 'https://hobbyking.com/nl_nl/trackstar-1-8th-2050kv-brushless-sensorless-motor.html';

% electro
I = 140; % [A]
V = 15; % [V] input voltage
P = 2100; %[W]

kv = 2050; % 4000; %[RPM/V]
R = 0.0006; %[Ohm] rough estimate

save(label)