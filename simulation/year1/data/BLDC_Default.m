% https://hobbyking.com/nl_nl/trackstar-1-8th-2050kv-brushless-sensorless-motor.html
label = 'BLDC_Default';
link = 'https:';

% electro
I = 45; % [A] %45A controller, 60A motor
nCellMax = 3; %[2 - 3 cell Lipo]
P = nan; %[W] (unknown)

% motor specs
kv = 	3650; % 4000; %[RPM/V]
R = 0.0129; %[Ohm] rough estimate

save(label)