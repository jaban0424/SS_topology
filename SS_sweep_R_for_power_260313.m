%% SS_sweep_R_power_parsim_simple.m
% SS_R을 10~100 (15포인트)로 스윕하면서,
% parpool + parsim 으로 병렬 시뮬레이션을 돌리고
% - Supply input power: Pin = avg(V0*I0)
% - DC output power   : Pout = avg(Vo*Io)
% - Ideal power       : Pideal = 8/pi^2 * SS_Rac * SS_Vin^2 / (SS_AglFreq * SS_M)^2
% 를 한 번에 플롯한다.
%
% 불필요한 자동화/옵션은 전부 제거하고,
% - parpool 시작
% - parsim 실행
% - 후처리 + 플롯
% - parpool 정리
% 만 남긴 단순 버전.

%% (A) 기본 설정
modelName = 'SS_design';
numworkers = 5;

R_min = 5;
R_max = 100;
Npts  = 15;
R_list = linspace(R_min, R_max, Npts);

% 측정 윈도우 (SS_topology 값 있으면 사용)
SS_TotalSimulTime = local_get_base_var('SS_TotalSimulTime', 2.5e-3);
tStart = local_get_base_var('SS_SoftStarting_TimeToStartMeasure', 1e-3);
tEnd   = local_get_base_var('SS_SoftStarting_TimeToStopMeasure',  2e-3);

vInName  = 'V0';
iInName  = 'I0';
vOutName = 'Vo';
iOutName = 'Io';

%% (B) 병렬 풀 시작 (간단 버전)
% 이미 풀 있으면 그대로 사용, 없으면 local 프로파일로 워커 4개 생성
if isempty(gcp('nocreate'))
    parpool('local', numworkers);  %#ok<PARPLO>
end

%% (C) 이상적 전력 계산에 필요한 파라미터
SS_Vin = local_get_base_var('SS_Vin', 380);

if evalin('base','exist("SS_AglFreq","var")')
    SS_AglFreq = evalin('base','SS_AglFreq');
elseif evalin('base','exist("SS_Freq","var")')
    SS_AglFreq = 2*pi*evalin('base','SS_Freq');
else
    SS_AglFreq = 2*pi*85e3;
end

if evalin('base','exist("SS_M","var")')
    SS_M = evalin('base','SS_M');
else
    SS_Lp = local_get_base_var('SS_Lp', 211e-6);
    SS_Ls = local_get_base_var('SS_Ls', 155e-6);
    SS_k  = local_get_base_var('SS_k',  0.2);
    SS_M = SS_k * sqrt(SS_Lp * SS_Ls);
end

SS_Rac_list = R_list * 8 / pi^2;
P_ideal_W = (8 / pi^2) .* SS_Rac_list .* (SS_Vin.^2) ./ ( (SS_AglFreq * SS_M).^2 );

%% (D) SimulationInput 구성
load_system(modelName);
set_param(modelName,'StopTime',num2str(SS_TotalSimulTime));
set_param(modelName,'FastRestart','off');

simIn(Npts,1) = Simulink.SimulationInput(modelName);
for k = 1:Npts
    R = R_list(k);
    simIn(k) = Simulink.SimulationInput(modelName) ...
        .setVariable('SS_R',   R) ...
        .setVariable('SS_Rac', R * 8 / pi^2);
end

%% (E) parsim 실행
fprintf('\n==================== SWEEP (parsim) SS_R: %g ~ %g (%d pts) ====================\n', ...
    R_min, R_max, Npts);
fprintf('Window: %.6f ~ %.6f s\n', tStart, tEnd);
fprintf('Signals: Pin=(%s*%s), Pout=(%s*%s)\n', vInName, iInName, vOutName, iOutName);
fprintf('----------------------------------------------------------------------------------\n');
fprintf('%8s | %14s | %14s | %14s | %9s\n', 'SS_R', 'P_in [W]', 'P_out [W]', 'P_loss [W]', 'Eff[%]');
fprintf(repmat('-',1,44)); fprintf('\n');

out = parsim(simIn, ...
    'UseParallel', true, ...
    'UseFastRestart', 'off', ...
    'TransferBaseWorkspaceVariables', 'on', ...
    'ShowProgress', 'on');

%% (F) 후처리: Pin/Pout 계산
P_in_W  = NaN(size(R_list));
P_out_W = NaN(size(R_list));

for k = 1:Npts
    R = R_list(k);

    if isprop(out(k),'ErrorMessage') && out(k).ErrorMessage ~= ""
        fprintf('%8.2f | %14s | %14s\n', R, 'N/A', 'N/A');
        fprintf('          [parsim error @ k=%d, SS_R=%.2f] %s\n', k, R, out(k).ErrorMessage);
        continue;
    end

    logs = [];
    if isprop(out(k),'logsout') && ~isempty(out(k).logsout)
        logs = out(k).logsout;
    end
    if isempty(logs)
        fprintf('%8.2f | %14s | %14s\n', R, 'N/A', 'N/A');
        continue;
    end

    vInEl  = local_get_element_by_name(logs, vInName);
    iInEl  = local_get_element_by_name(logs, iInName);
    vOutEl = local_get_element_by_name(logs, vOutName);
    iOutEl = local_get_element_by_name(logs, iOutName);

    if isempty(vInEl) || isempty(iInEl) || isempty(vOutEl) || isempty(iOutEl)
        fprintf('%8.2f | %14s | %14s\n', R, 'N/A', 'N/A');
        continue;
    end

    [Pin, okIn]   = local_calc_avg_power(vInEl.Values,  iInEl.Values,  tStart, tEnd);
    [Pout, okOut] = local_calc_avg_power(vOutEl.Values, iOutEl.Values, tStart, tEnd);

    if okIn,  P_in_W(k)  = Pin;  end
    if okOut, P_out_W(k) = Pout; end

    if okIn && okOut
        Ploss = Pin - Pout;
        effPct = NaN;
        if Pin > 0
            effPct = Pout / Pin * 100;
        end
        fprintf('%8.2f | %14.3f | %14.3f | %14.3f | %9.2f\n', R, Pin, Pout, Ploss, effPct);
    elseif okIn
        fprintf('%8.2f | %14.3f | %14s | %14s | %9s\n', R, Pin, 'N/A', 'N/A', 'N/A');
    elseif okOut
        fprintf('%8.2f | %14s | %14.3f | %14s | %9s\n', R, 'N/A', Pout, 'N/A', 'N/A');
    else
        fprintf('%8.2f | %14s | %14s | %14s | %9s\n', R, 'N/A', 'N/A', 'N/A', 'N/A');
    end
end

fprintf(repmat('-',1,76)); fprintf('\n');
fprintf('==================================================================================\n');

%% (G) 플롯
figure('Name','SS_R sweep (parsim simple): supply vs dc output vs ideal');
plot(R_list, P_in_W/1e3,  '-o', 'LineWidth', 1.5); hold on;
plot(R_list, P_out_W/1e3, '-s', 'LineWidth', 1.5);
plot(R_list, P_ideal_W/1e3, '-k', 'LineWidth', 1.5);
grid on;
xlabel('SS\_R [\Omega]');
ylabel('Power [kW]');
title('Power vs SS\_R (15 simulations, windowed average)');
legend('Supply input power (V0*I0)', 'DC output power (Vo*Io)', 'Ideal power (formula)', 'Location', 'best');

%% (H) 메모리/풀 정리
clear simIn out logs vInEl iInEl vOutEl iOutEl Pin Pout;
delete(gcp('nocreate'));

%% 로컬 함수들
function v = local_get_base_var(name, defaultValue)
    if evalin('base',sprintf('exist("%s","var")',name))
        v = evalin('base',name);
    else
        v = defaultValue;
    end
end

function el = local_get_element_by_name(logs, targetName)
    el = [];
    try
        el = logs.get(targetName);
        return;
    catch
    end
    for i = 1:logs.numElements
        e = logs.getElement(i);
        if strcmp(e.Name, targetName)
            el = e;
            return;
        end
    end
end

function [Pavg, ok] = local_calc_avg_power(vTs, iTs, tStart, tEnd)
    ok = false;
    Pavg = NaN;
    [t, v, i] = local_align_times(vTs, iTs, tStart, tEnd);
    if isempty(t) || numel(t) < 2
        return;
    end
    T = t(end) - t(1);
    if T <= 0
        return;
    end
    Pavg = trapz(t, v .* i) / T;
    ok = true;
end

function [tOut, vOut, iOut] = local_align_times(vTs, iTs, tStart, tEnd)
    tV = vTs.Time(:);
    v  = vTs.Data(:);
    tI = iTs.Time(:);
    i  = iTs.Data(:);

    if ~isvector(v) || ~isvector(i)
        error('local_align_times: 벡터가 아닌(다차원) 신호는 지원하지 않습니다.');
    end

    idxV = (tV >= tStart) & (tV <= tEnd);
    idxI = (tI >= tStart) & (tI <= tEnd);

    tVw = tV(idxV);
    vw  = v(idxV);
    tIw = tI(idxI);
    iw  = i(idxI);

    if numel(tVw) < 2 || numel(tIw) < 2
        tOut = []; vOut = []; iOut = [];
        return;
    end

    iInterp = interp1(tIw, iw, tVw, 'linear', 'extrap');
    tOut = tVw;
    vOut = vw;
    iOut = iInterp;
end

