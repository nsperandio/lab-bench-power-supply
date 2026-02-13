%% System Data

Vin = 35; %[V]

L_buck= 22e-6; %[H]
Co_buck = 10e-6; %355e-9; %[L]
fsw = 400e3; %[Hz]


Reg_I_PAR.Kp = 0.012;
Reg_I_PAR.Ki = 94.4;

Reg_V_PAR.Kp = 0.57;
Reg_V_PAR.Ki = 166.8;

Ro_lin = 5;
Co_lin = 10e-6;

Reg_V_lin_PAR.Kp = 0.010;
Reg_V_lin_PAR.Ki = 637;

Reg_I_lin_PAR.Kp = 0.010;
Reg_I_lin_PAR.Ki = 1200;