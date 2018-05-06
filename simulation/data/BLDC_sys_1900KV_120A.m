% https://hobbyking.com/nl_nl/trackstar-1-8th-2050kv-brushless-sensorless-motor.html
label = 'BLDC_sys_1900KV_120A';
link = 'https://hobbyking.com/nl_nl/turnigy-trackstar-waterproof-1-8-brushless-power-system-1900kv-120a.html';

% electro
I = 70; % [A] %A
IPeak = 120; %[A]
nCellMax = 4; %[- 4 cell Lipo]
P = 1470; %[W] (unknown)

% motor specs
kv = 1900; % ; %[RPM/V]
R = 0.0100; %[Ohm] rough estimate
save(label)