%% BODE PLOTS SETTINGS ------------------------------------------------
s = tf('s');
fmin = 0;              % [Hz]
fmax = 1e6;            % [Hz]
w = 2*pi*logspace(log10(fmin), log10(fmax), 2000);
opts = bodeoptions;
opts.FreqUnits = 'Hz';
opts.Grid = 'on';
opts.PhaseWrapping='on';
set(groot, 'defaultLineLineWidth', 1.6);
set(groot, 'defaultAxesColorOrder', [
    0.00 0.45 0.74;   % blue
    0.85 0.33 0.10;   % orange
    0.93 0.69 0.13;   % yellow
    0.49 0.18 0.56;   % purple
    0.47 0.67 0.19;   % green
    0.30 0.75 0.93;   % cyan
]);
pidopts=pidtuneOptions;


%% System Data

Vin = 35; %[V]

L_buck= 22e-6;
Co_buck = 10e-6; %355e-9;

f0_buck = 1/(2*pi*sqrt(L_buck*Co_buck));

Gbuck_d_i = 1/(s*L_buck) * Vin;
Gbuck_i_v = 1/(s*Co_buck);

figure;
bode(Gbuck_d_i,opts);
title('Buck');
hold on;
grid on;

%% Current Loop
Reg_I_PM = 65;
Reg_I_BW = 3e3;
pidopts.PhaseMargin = Reg_I_PM;
[Reg_I_PAR, info] = pidtune(Gbuck_d_i,'pi', 2*pi*Reg_I_BW, pidopts);
Reg_I_TF = Reg_I_PAR.Kp+Reg_I_PAR.Ki/s;

T_buck_i = Gbuck_d_i*Reg_I_TF;

figure(Name='Buck Current Loop');
margin(Gbuck_d_i,opts);hold on;
margin(T_buck_i,opts); 
grid on;
title('Buck Current Loop Gain T(s)');
%% STEP

G_buck_i = T_buck_i/(1+T_buck_i);

figure;
step(G_buck_i,600e-6);
title('Buck Current Step Response');
grid on;

%% Voltage Loop
Reg_V_PM = 65;
Reg_V_BW = 1e3;
pidopts.PhaseMargin = Reg_V_PM;
[Reg_V_PAR, info] = pidtune(Gbuck_i_v,'pi', 2*pi*Reg_V_BW, pidopts);
Reg_V_TF = Reg_V_PAR.Kp+Reg_V_PAR.Ki/s;

T_buck_v = Gbuck_i_v*Reg_V_TF;
Gbuck_v = T_buck_v/(1+T_buck_v);

figure(Name='Buck Voltage Loop');
margin(Gbuck_i_v,opts);hold on;
margin(T_buck_v,opts); 
grid on;
title('Buck Voltage Loop Gain T(s)');

figure;
step(Gbuck_v,10e-3);
title('Buck Voltage Step Response');
grid on;

%% LINEAR REG 

Ro_lin = 5;
Co_lin = 10e-6;
%CV
Glin_v = Ro_lin/(1+(s*Ro_lin*Co_lin));
Reg_V_lin_PM = 70;
Reg_V_lin_BW = 500;
pidopts.PhaseMargin = Reg_V_lin_PM;

[Reg_V_lin_PAR, info] = pidtune(Glin_v,'pi', 2*pi*Reg_V_lin_BW, pidopts);
Reg_V_lin_TF = Reg_V_lin_PAR.Kp+Reg_V_lin_PAR.Ki/s;

%CC
Glin_i = s*Co_lin/Ro_lin * (1+s*Co_lin*Ro_lin)/(1+(s*Co_lin/Ro_lin)+(s*Co_lin)^2);
Glin_i = Glin_i *1/s;
Reg_I_lin_PM = 70;
Reg_I_lin_BW = 250;
pidopts.PhaseMargin = Reg_I_lin_PM;
[Reg_I_lin_PAR, info] = pidtune(Glin_i,'pi', 2*pi*Reg_I_lin_BW, pidopts);
Reg_I_lin_TF = Reg_I_lin_PAR.Kp+Reg_I_lin_PAR.Ki/s;

T_lin_i = Glin_i*Reg_I_lin_TF;
figure(Name='Linear Reg Current Loop');
margin(Glin_i,opts);hold on;
margin(T_lin_i,opts); 
grid on;
title('Linear Reg Current Loop Gain T(s)');