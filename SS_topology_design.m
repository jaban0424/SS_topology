
% v1.1 [2026/04/07 03:00] 시뮬레이션 기능을 SS_run_simulation_260407.m 로 분리하고 파라미터 셋업용으로만 작동하도록 수정됨.
% v1.2 [2026/04/14 21:25] 기타오류수정
% commit habbit
%% ===================== Params =====================
SS_Duty = 0.5;
SS_DB = 0.01;
SS_Freq = 85e3;
SS_AglFreq = 2*pi*SS_Freq;

SS_Vin = 650;

SS_driv_Von = 15;
SS_driv_Voff = -5;

SS_MSFT_Ciss = 10.1e-9;
SS_MSFT_Crss = 43e-12;
SS_MSFT_Coss = 500e-12;

SS_Lp = 212e-6;
SS_Ls = 155e-6;
SS_k = 0.2;
SS_M = SS_k * sqrt(SS_Lp * SS_Ls);

SS_delta = 0.8; %%%% 디튜닝!!!!!

% SS_Cp = 16.537e-9;
% SS_Cs = 22.619e-9;
SS_Cp = 1/(SS_AglFreq^2 * SS_Lp);
SS_Cs = 1/(SS_AglFreq^2 * SS_Ls) * (1 - SS_delta);

SS_R1 = 10;
SS_R2 = 70.135;
SS_R3 = 31.23; % to make 22kW

SS_DesiredPo = 22e3;
SS_R_DesiredPo = pi^2 / 8 * SS_DesiredPo * (SS_AglFreq * SS_M)^2 / SS_Vin^2; %%% <-이거 안됨!!@!@ 이상함

SS_R = SS_R3;
SS_Rac = SS_R * 8 / pi^2;

SS_L_filter = 10e-6; % 이거 안 쓰일 수도
SS_C_filter = 10e-6;

SS_SoftStarting_delayed = 0.01e-3;
SS_SoftStarting_TransientTime = 0.2e-3;
SS_SoftStarting_StartPwmTime = 0.01e-3;
SS_SoftStarting_StopPwmTime = 3 e-3; %%%%%%%%%%%%%%%%

SS_SoftStarting_slope = SS_Vin / SS_SoftStarting_TransientTime;

SS_SoftStarting_StartPwmTBCTR = SS_SoftStarting_slope * SS_SoftStarting_StartPwmTime;
SS_SoftStarting_StopPwmTBCTR = SS_SoftStarting_slope * SS_SoftStarting_StopPwmTime;

SS_SoftStarting_MeasurePeriod = 1e-3;
SS_SoftStarting_TimeToStartMeasure = SS_SoftStarting_StopPwmTime - SS_SoftStarting_MeasurePeriod; %%%%%%%%%%%%%%%%%%
SS_SoftStarting_TimeToStopMeasure  = SS_SoftStarting_StopPwmTime; %%%%%%%%%%%%%%

SS_TotalSimulTime = SS_SoftStarting_StopPwmTime + 0.1e-3; %%%%%%%%%%%%%

%% ===================== Ideal (ZPA-based) =====================
SS_Ideal_Vp  = SS_Vin + ( SS_Lp * SS_Rac * SS_Vin / ( SS_AglFreq * SS_M ^ 2 )) * 1j ;
SS_Ideal_Vs  = SS_Ls * SS_Vin / SS_M + ( SS_Rac * SS_Vin / ( SS_AglFreq * SS_M )) * 1j ;
SS_Ideal_Ip  = SS_Rac * SS_Vin / ( SS_AglFreq * SS_M ) ^ 2 ;
SS_Ideal_Is  = SS_Vin / ( SS_AglFreq * SS_M ) * 1j ;
SS_Ideal_Vac = SS_Rac * SS_Vin / ( SS_AglFreq * SS_M ) * 1j; 

SS_Ideal_Po = SS_Rac * SS_Vin^2 / (SS_AglFreq * SS_M)^2;

% DC 쪽 이상값 (가정: 정류+필터 후 Vo_dc ≈ Vac_rms)
SS_Ideal_Vo_dc = sqrt(2) * abs(SS_Ideal_Vac);        % 내 뇌피셜?
SS_Ideal_Io_dc = SS_Ideal_Vo_dc / SS_R;    % (DC) in A

% fprintf('\n===== SS Ideal Calculation Result =====\n')
% fprintf('\n--- Complex form (a + bj) ---\n')
% fprintf('SS_Ideal_Vp  = %.3f %+.3fj V\n', real(SS_Ideal_Vp),  imag(SS_Ideal_Vp));
% fprintf('SS_Ideal_Vs  = %.3f %+.3fj V\n', real(SS_Ideal_Vs),  imag(SS_Ideal_Vs));
% fprintf('SS_Ideal_Ip  = %.3f %+.3fj A\n', real(SS_Ideal_Ip),  imag(SS_Ideal_Ip));
% fprintf('SS_Ideal_Is  = %.3f %+.3fj A\n', real(SS_Ideal_Is),  imag(SS_Ideal_Is));
% fprintf('SS_Ideal_Vac = %.3f %+.3fj V\n', real(SS_Ideal_Vac), imag(SS_Ideal_Vac));
% 
% fprintf('\n--- Phasor form (Magnitude ∠ Phase) ---\n')
% fprintf('SS_Ideal_Vp  = %.3f Vrms ∠ %.3f deg\n', abs(SS_Ideal_Vp),  angle(SS_Ideal_Vp)*180/pi);
% fprintf('SS_Ideal_Vs  = %.3f Vrms ∠ %.3f deg\n', abs(SS_Ideal_Vs),  angle(SS_Ideal_Vs)*180/pi);
% fprintf('SS_Ideal_Ip  = %.3f Arms ∠ %.3f deg\n', abs(SS_Ideal_Ip),  angle(SS_Ideal_Ip)*180/pi);
% fprintf('SS_Ideal_Is  = %.3f Arms ∠ %.3f deg\n', abs(SS_Ideal_Is),  angle(SS_Ideal_Is)*180/pi);
% fprintf('SS_Ideal_Vac = %.3f Vrms ∠ %.3f deg\n', abs(SS_Ideal_Vac), angle(SS_Ideal_Vac)*180/pi);
% 
% fprintf('\n--- DC assumption (rectifier+filter) ---\n')
% fprintf('SS_Ideal_Vo_dc = %.3f V  (DC)\n', SS_Ideal_Vo_dc);
% fprintf('SS_Ideal_Io_dc = %.3f A  (DC)\n', SS_Ideal_Io_dc);
% fprintf('\n=======================================\n')

%% ===================== End of Parameter Definitions =====================