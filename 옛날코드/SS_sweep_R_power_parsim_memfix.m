%% SS_sweep_R_power_parsim_memfix.m
% 병렬(parsim) SS_R 스윕 + 메모리 관리 강화 버전
%
% 핵심:
% - 케이스별로 Pin/Pout 숫자만 추출(윈도우 적분)하고, 후처리 루프에서 임시변수 즉시 clear
% - 스윕 종료 후 parpool을 자동 종료(delete gcp)해서 워커 메모리(예: 1.2GB/worker) 반환
% - 워커 수(numWorkers)를 제한해서 총 메모리 사용량을 줄임
%
% 플롯: 
% - Supply input power: avg(V0*I0)
% - DC output power   : avg(Vo*Io)
% - Ideal power (black): P_ideal = 8/pi^2 * SS_Rac * SS_Vin^2 / (SS_AglFreq * SS_M)^2

%% (A) 사용자 설정
modelName = 'SS_design';

R_min = 10;
R_max = 100;
Npts  = 15;
R_list = linspace(R_min, R_max, Npts);

% 병렬/메모리 설정
numWorkers = 4;        % 메모리 부담 크면 줄이기 (예: 10 -> 4) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
autoClosePool = true;  % 끝나면 parpool 종료해서 메모리 반환
autoStartPool = false; % true면 이 스크립트가 parpool을 직접 시작(환경에 따라 오래 걸릴 수 있음)
poolMode = 'threads';  % 'threads'(가벼움) | 'processes'(기본) | 'none'

% SS_topology.m에서 쓰는 설정값이 base에 있으면 그대로 사용
SS_TotalSimulTime = local_get_base_var('SS_TotalSimulTime', 2.5e-3);
tStart = local_get_base_var('SS_SoftStarting_TimeToStartMeasure', 1e-3);
tEnd   = local_get_base_var('SS_SoftStarting_TimeToStopMeasure',  2e-3);

% 측정 신호명
vInName  = 'V0';
iInName  = 'I0';
vOutName = 'Vo';
iOutName = 'Io';

%% (B) 이상적 전력(검은색 선) 계산
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

%% (C) 결과 배열
P_in_W  = NaN(size(R_list));
P_out_W = NaN(size(R_list));

%% (D) 모델 로드 및 SimulationInput 구성
load_system(modelName);
set_param(modelName,'StopTime',num2str(SS_TotalSimulTime));
set_param(modelName,'FastRestart','off');

simIn(numel(R_list),1) = Simulink.SimulationInput(modelName);
for k = 1:numel(R_list)
    R = R_list(k);
    simIn(k) = Simulink.SimulationInput(modelName) ...
        .setVariable('SS_R',   R) ...
        .setVariable('SS_Rac', R * 8 / pi^2);
end

%% (E) 병렬 풀 준비 (선택)
% parpool(Processes)는 최초 1회가 오래 걸리거나(캐시/Simulink 로딩),
% 이미 존재하는 pool을 종료/재시작하면서 "멈춘 것처럼" 보일 수 있습니다.
% 그래서 기본은 autoStartPool=false로 두고, 이미 떠있는 pool이 있으면 그걸 사용합니다.
useParallel = false;
try
    p = gcp('nocreate');
    if ~isempty(p)
        useParallel = true;
    elseif autoStartPool && ~strcmpi(poolMode,'none')
        if strcmpi(poolMode,'threads')
            parpool('threads', numWorkers); %#ok<PARPLO>
        else
            parpool(numWorkers); %#ok<PARPLO> % processes
        end
        useParallel = true;
    end
catch
    useParallel = false;
end

%% (F) 실행 (가능하면 parsim 병렬, 아니면 직렬 sim 폴백)
fprintf('\n==================== SWEEP (parsim) SS_R: %g ~ %g (%d pts) ====================\n', ...
    R_min, R_max, Npts);
fprintf('Window: %.6f ~ %.6f s\n', tStart, tEnd);
fprintf('Signals: Pin=(%s*%s), Pout=(%s*%s)\n', vInName, iInName, vOutName, iOutName);
fprintf('----------------------------------------------------------------------------------\n');
fprintf('%8s | %14s | %14s\n', 'SS_R', 'P_in [W]', 'P_out [W]');
fprintf(repmat('-',1,44)); fprintf('\n');

if useParallel
    out = parsim(simIn, ...
        'UseParallel', true, ...
        'UseFastRestart', 'off', ...
        'TransferBaseWorkspaceVariables', 'on', ...
        'ShowProgress', 'on');
else
    % parpool 시작이 오래 걸리거나 실패하면, 그래도 결과는 뽑아주기(느리지만 멈추진 않음)
    out(numel(R_list),1) = Simulink.SimulationOutput;
    for k = 1:numel(R_list)
        out(k) = sim(simIn(k));
    end
end

%% (G) 후처리 (숫자만 추출)
for k = 1:numel(R_list)
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
        clear logs vInEl iInEl vOutEl iOutEl
        continue;
    end

    [Pin, okIn]   = local_calc_avg_power(vInEl.Values,  iInEl.Values,  tStart, tEnd);
    [Pout, okOut] = local_calc_avg_power(vOutEl.Values, iOutEl.Values, tStart, tEnd);

    if okIn,  P_in_W(k)  = Pin;  end
    if okOut, P_out_W(k) = Pout; end

    if okIn && okOut
        fprintf('%8.2f | %14.3f | %14.3f\n', R, Pin, Pout);
    elseif okIn
        fprintf('%8.2f | %14.3f | %14s\n', R, Pin, 'N/A');
    elseif okOut
        fprintf('%8.2f | %14s | %14.3f\n', R, 'N/A', Pout);
    else
        fprintf('%8.2f | %14s | %14s\n', R, 'N/A', 'N/A');
    end

    % 임시변수 즉시 폐기(클라이언트 메모리 절약)
    clear logs vInEl iInEl vOutEl iOutEl Pin Pout okIn okOut
end

fprintf(repmat('-',1,44)); fprintf('\n');
fprintf('==================================================================================\n');

% SimulationOutput/SimulationInput 자체도 더 이상 필요 없으면 제거(클라이언트 메모리 절약)
clear simIn
clear out

%% (H) 플롯
figure('Name','SS_R sweep (parsim): supply vs dc output vs ideal');
plot(R_list, P_in_W/1e3,  '-o', 'LineWidth', 1.5); hold on;
plot(R_list, P_out_W/1e3, '-s', 'LineWidth', 1.5);
plot(R_list, P_ideal_W/1e3, '-k', 'LineWidth', 1.5);
grid on;
xlabel('SS\_R [\Omega]');
ylabel('Power [kW]');
title('Power vs SS\_R (15 simulations, windowed average)');
legend('Supply input power (V0*I0)', 'DC output power (Vo*Io)', 'Ideal power (formula)', 'Location', 'best');

%% (I) 워커 종료(메모리 반환)
if autoClosePool
    try
        delete(gcp('nocreate'));
    catch
    end
end

%% 로컬 함수
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

