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
Reg_I_BW = 50e3;
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
Reg_V_BW = 15e3;
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

gm = 1200e-6;
Vref = 0.8;
%comp = piCompExtract(Reg_V_PAR.Kp,Reg_V_PAR.Ki,10e3,[],true);
comp = type2_ota_extract(Reg_V_PAR.Kp,Reg_V_PAR.Ki,gm,0.8,30,2,true);
fprintf("\n---Buck Voltage Loop---\n");
fprintf("Rc = %.2f Ohm\n", comp.Rc);
fprintf("Cc = %.2e F\n", comp.Cc);
fprintf("Cp = %.2e F\n", comp.Cp);

%% LINEAR REG 

Ro_lin = 5;
Co_lin = 10e-6;
%CV
Glin_v = Ro_lin/(1+(s*Ro_lin*Co_lin));
Reg_V_lin_PM = 60;
Reg_V_lin_BW = 1e3;
pidopts.PhaseMargin = Reg_V_lin_PM;

[Reg_V_lin_PAR, info] = pidtune(Glin_v,'pi', 2*pi*Reg_V_lin_BW, pidopts);
Reg_V_lin_TF = Reg_V_lin_PAR.Kp+Reg_V_lin_PAR.Ki/s;

comp = piCompExtract(Reg_V_lin_PAR.Kp, Reg_V_lin_PAR.Ki, 1e3, [], true);
fprintf("\n---Linear Regulator CV---\n");
fprintf("Rf = %.2f Ohm\n", comp.Rf);
fprintf("Cf = %.2e F\n", comp.Cf);



%% CC
Glin_i = s*Co_lin/Ro_lin * (1+s*Co_lin*Ro_lin)/(1+(s*Co_lin/Ro_lin)+(s*Co_lin)^2);
Glin_i = Glin_i *1/s;
Reg_I_lin_PM = 60;
Reg_I_lin_BW = 3e3;
pidopts.PhaseMargin = Reg_I_lin_PM;
[Reg_I_lin_PAR, info] = pidtune(Glin_i,'pid', 2*pi*Reg_I_lin_BW, pidopts);
Reg_I_lin_TF = Reg_I_lin_PAR.Kp+Reg_I_lin_PAR.Ki/s++Reg_I_lin_PAR.Kd*s;

T_lin_i = Glin_i*Reg_I_lin_TF;
figure(Name='Linear Reg Current Loop');
margin(Glin_i,opts);hold on;
margin(T_lin_i,opts); 
grid on;
title('Linear Reg Current Loop Gain T(s)');

comp = piCompExtract(Reg_I_lin_PAR.Kp, Reg_I_lin_PAR.Ki, 1e3, [], true);
fprintf("\n---Linear Regulator CC---\n");
fprintf("Rf = %.2f Ohm\n", comp.Rf);
fprintf("Cf = %.2e F\n", comp.Cf);

%% 
function comp = piCompExtract(Kp, Ki, Ri, f_pole, round_to_e24)
% TYPE-II Compensator (Voltage-Mode/OPAMP) from PI gains
% Kp, Ki  : from pidtune
% Ri      : chosen input resistor (Ohm)
% f_pole  : optional HF pole frequency (Hz) (set [] if none)
% round_to_e24 : true/false

s = tf('s');

% --- Ideal component calculation ---

Rf_ideal = Kp * Ri;
Cf_ideal = 1/(Ki * Ri);

% --- Optional HF pole capacitor ---
if ~isempty(f_pole)
    Cp_ideal = 1/(2*pi*Rf_ideal*f_pole);
else
    Cp_ideal = 0;
end

% --- Rounding (optional) ---
if round_to_e24
    Rf = round_E24(Rf_ideal);
    Cf = round_E24(Cf_ideal);
    Cp = round_E24(Cp_ideal);
else
    Rf = Rf_ideal;
    Cf = Cf_ideal;
    Cp = Cp_ideal;
end

% --- Ideal transfer function ---
Gc_ideal = (Rf_ideal/Ri)*(1 + 1/(s*Rf_ideal*Cf_ideal));

if Cp_ideal > 0
    Gc_ideal = Gc_ideal * 1/(1 + s*Rf_ideal*Cp_ideal);
end

% --- Real transfer function ---
Gc_real = (Rf/Ri)*(1 + 1/(s*Rf*Cf));

if Cp > 0
    Gc_real = Gc_real * 1/(1 + s*Rf*Cp);
end

% --- Frequencies ---
fz_real = 1/(2*pi*Rf*Cf);

if Cp > 0
    fp_real = 1/(2*pi*Rf*Cp);
else
    fp_real = NaN;
end

% --- Return struct ---
comp.Ri = Ri;
comp.Rf = Rf;
comp.Cf = Cf;
comp.Cp = Cp;

comp.Rf_ideal = Rf_ideal;
comp.Cf_ideal = Cf_ideal;
comp.Cp_ideal = Cp_ideal;

comp.Gc_ideal = Gc_ideal;
comp.Gc_real  = Gc_real;

comp.fz_real = fz_real;
comp.fp_real = fp_real;

end

function comp = type2_ota_extract(Kp, Ki, gm, Vref, VLDO_max, Vheadroom,round_to_e24)

% TYPE-II Compensator for OTA (e.g. LM5148)
% Includes:
% - Compensation network (Rc, Cc, Cp)
% - Feedback divider
% - VLDO / VBUCK mixing network
%
% Kp, Ki      : PI gains from pidtune
% gm          : OTA transconductance (A/V)
% Vref        : FB reference voltage (e.g. 0.8V)
% VLDO_max    : maximum LDO voltage
% Vheadroom   : desired headroom
% round_to_e24: true/false

s = tf('s');


% 1️ OTA COMP NETWORK


% From:
% Gc(s) = gm * Rc * (1 + 1/(s Rc Cc))

Rc_ideal = Kp / gm;
Cc_ideal = 1 / (Ki * Rc_ideal);

% Optional HF pole at 10x crossover (example choice)
fz = 1/(2*pi*Rc_ideal*Cc_ideal);
fp = 10 * fz;

Cp_ideal = 1/(2*pi*Rc_ideal*fp);

if round_to_e24
    Rc = round_E24(Rc_ideal);
    Cc = round_E24(Cc_ideal);
    Cp = round_E24(Cp_ideal);
else
    Rc = Rc_ideal;
    Cc = Cc_ideal;
    Cp = Cp_ideal;
end

Zcomp = Rc*(1 + 1/(s*Rc*Cc)) / (1 + s*Rc*Cp);
Gc_real = gm * Zcomp;

% 1 FEEDBACK DIVIDER (TRACKING)


% Desired:
% VBUCK = VLDO + Vheadroom

Vbuck_max = VLDO_max + Vheadroom;

% Choose lower resistor first
Rbot = 10e3;

% Effective upper resistor from Vbuck
Rtop = Rbot*(Vbuck_max/Vref - 1);

if round_to_e24
    Rbot = round_E24(Rbot);
    Rtop = round_E24(Rtop);
end

% 3 MIXING RESISTOR (VLDO injection)

% Add resistor Rmix from VLDO to FB
% Solve approximate ratio so:
% VBUCK = VLDO + Vheadroom

% Simple approximation:
% Rmix ≈ Rtop * (Vref / Vheadroom)

Rmix_ideal = Rtop * (Vref / Vheadroom);

if round_to_e24
    Rmix = round_E24(Rmix_ideal);
else
    Rmix = Rmix_ideal;
end

% Return structure

comp.Rc = Rc;
comp.Cc = Cc;
comp.Cp = Cp;

comp.Rtop = Rtop;
comp.Rbot = Rbot;
comp.Rmix = Rmix;

comp.Gc_real = Gc_real;

comp.fz = 1/(2*pi*Rc*Cc);
comp.fp = 1/(2*pi*Rc*Cp);

end


function val = round_E96(x)


if x == 0
    val = 0;
    return
end

E96 = [100 102 105 107 110 113 115 118 121 124 ...
       127 130 133 137 140 143 147 150 154 158 ...
       162 165 169 174 178 182 187 191 196 200 ...
       205 210 215 221 226 232 237 243 249 255 ...
       261 267 274 280 287 294 301 309 316 324 ...
       332 340 348 357 365 374 383 392 402 412 ...
       422 432 442 453 464 475 487 499 511 523 ...
       536 549 562 576 590 604 619 634 649 665 ...
       681 698 715 732 750 768 787 806 825 845 ...
       866 887 909 931 953 976];

decade = 10^floor(log10(x));
normalized = x/decade;

[~, idx] = min(abs(E96/100 - normalized));
val = E96(idx)/100 * decade;

end

function val = round_E24(x)

if x == 0
    val = 0;
    return
end

E24 = [1.0 1.1 1.2 1.3 1.5 1.6 1.8 2.0 2.2 2.4 2.7 3.0 ...
       3.3 3.6 3.9 4.3 4.7 5.1 5.6 6.2 6.8 7.5 8.2 9.1];

decade = 10^floor(log10(x));
normalized = x/decade;

idx = find(E24 <= normalized, 1, 'last');

if isempty(idx)
    idx = length(E24);
    decade = decade/10;
end

val = E24(idx) * decade;

end