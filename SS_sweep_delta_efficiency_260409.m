%% SS_sweep_delta_efficiency_260409.m
% =========================================================================
% [버전 기록]
% v1.1 : (2026-04-09 13:55:00)
%        - 턴오프 전류(Vi 하강에지 시점의 Ip) 계산 및 Figure 3 추가
% v1.0 : (2026-04-09 13:30:00)
%        - SS_delta를 스윕하면서 SS_design.slx 시뮬레이션을 병렬 수행
%        - SS_power_analysis 로직을 내장하여 각 delta별 효율, 인버터 스위치 손실, 정류기 손실 계산
%        - Figure 1: delta에 따른 전체 효율(DC출력/DC입력) 그래프
%        - Figure 2: delta에 따른 인버터 스위치 손실 & 정류기 손실 겹치기 그래프
%        - Figure 3: delta에 따른 턴오프 전류 그래프
% =========================================================================

%% ===================== (A) 사용자 설정 =====================
delta_min = 0;
delta_max = 0.5;
num_points = 41;
delta_vec = linspace(delta_min, delta_max, num_points);

num_workers = 5;
modelName = 'SS_design';

% 정상상태 측정을 위한 윈도우(마지막 N주기)
num_measure_periods = 200;

%% ===================== (B) 파라미터 로드 =====================
fprintf('--- 1. 시스템 파라미터 로드 중 (SS_topology_design) ---\n');
SS_topology_design;

tEnd_measure   = SS_SoftStarting_TimeToStopMeasure;
tEnd_sim       = SS_TotalSimulTime;
tStart_measure = tEnd_measure - num_measure_periods * (1 / SS_Freq);

%% ===================== (C) 병렬 시뮬레이션 구성 =====================
fprintf('\n--- 2. parsim 병렬 시뮬레이션 구성 중 (총 %d개 배치) ---\n', num_points);

if isempty(gcp('nocreate'))
    parpool('local', num_workers);
end

load_system(modelName);
set_param(modelName, 'StopTime', num2str(tEnd_sim));
set_param(modelName, 'FastRestart', 'off');

simIn(num_points, 1) = Simulink.SimulationInput(modelName);

for idx = 1:num_points
    delta = delta_vec(idx);
    Cs_val = 1 / (SS_AglFreq^2 * SS_Ls) * (1 - delta);
    
    simIn(idx) = Simulink.SimulationInput(modelName) ...
        .setVariable('SS_delta', delta) ...
        .setVariable('SS_Cs', Cs_val);
end

fprintf('시뮬레이션 전송 중...\n');
out = parsim(simIn, ...
    'UseParallel', true, ...
    'UseFastRestart', 'off', ...
    'TransferBaseWorkspaceVariables', 'on', ...
    'ShowProgress', 'on');

%% ===================== (D) 결과 후처리: 전력 분석 =====================
fprintf('\n--- 3. 시뮬레이션 결과 후처리 (전력/효율/손실 분석) ---\n');

% 결과 저장 배열
eta_total    = NaN(1, num_points);  % 전체 효율 [%]
P_supply     = NaN(1, num_points);  % 공급 입력 전력 [W]
P_dcout      = NaN(1, num_points);  % DC 출력 전력 [W]
P_sw_total   = NaN(1, num_points);  % 인버터 스위치 4소자 합계 손실 [W]
P_rect_total = NaN(1, num_points);  % 정류기 다이오드 4소자 합계 손실 [W]
I_turnoff    = NaN(1, num_points);  % 턴오프 전류 [A]

% 전력 분석에 필요한 신호 쌍 정의
power_pairs = {
    'V0',  'I0';   % Supply input (idx=1)
    'Vo',  'Io';   % DC output    (idx=2)
};

for idx = 1:num_points
    delta = delta_vec(idx);
    
    % 오류 체크
    if isprop(out(idx),'ErrorMessage') && out(idx).ErrorMessage ~= ""
        fprintf('  [오류] delta=%.4f 시뮬레이션 실패: %s\n', delta, out(idx).ErrorMessage);
        continue;
    end
    
    % logsout 추출
    logs = [];
    if isprop(out(idx),'logsout') && ~isempty(out(idx).logsout)
        logs = out(idx).logsout;
    end
    if isempty(logs)
        fprintf('  [경고] delta=%.4f logsout 없음\n', delta);
        continue;
    end
    
    % --- 공급 입력 전력 (V0 * I0) ---
    [Pavg_supply] = local_calc_power(logs, 'V0', 'I0', tStart_measure, tEnd_measure);
    
    % --- DC 출력 전력 (Vo * Io) ---
    [Pavg_dcout] = local_calc_power(logs, 'Vo', 'Io', tStart_measure, tEnd_measure);
    
    % --- 인버터 스위치 손실 (Vds4 * Id4, 1소자 * 4) ---
    [Pavg_sw1] = local_calc_power(logs, 'Vds4', 'Id4', tStart_measure, tEnd_measure);
    
    % --- 정류기 다이오드 손실 (Vdi * Idi, 1소자 * 4) ---
    [Pavg_rect1] = local_calc_power(logs, 'Vdi', 'Idi', tStart_measure, tEnd_measure);
    
    % --- 턴오프 전류 (Vi 하강에지 시점의 Ip) ---
    Ip_to = local_calc_turnoff_current(logs, 'Vi', 'Ip', tStart_measure, tEnd_measure);
    
    % 결과 저장
    P_supply(idx) = Pavg_supply;
    P_dcout(idx)  = Pavg_dcout;
    I_turnoff(idx) = Ip_to;
    
    if ~isnan(Pavg_supply) && ~isnan(Pavg_dcout) && Pavg_supply > 0
        eta_total(idx) = Pavg_dcout / Pavg_supply * 100;
    end
    
    if ~isnan(Pavg_sw1)
        P_sw_total(idx) = 4 * Pavg_sw1;
    end
    
    if ~isnan(Pavg_rect1)
        P_rect_total(idx) = 4 * Pavg_rect1;
    end
    
    fprintf('  delta=%.4f | η=%.2f%%, P_sw=%.1fW, P_rect=%.1fW, I_to=%.2fA\n', ...
        delta, eta_total(idx), P_sw_total(idx), P_rect_total(idx), Ip_to);
end

%% ===================== (E) 그래프 출력 =====================
fprintf('\n--- 4. 그래프 출력 ---\n');

% Figure 1: 효율 그래프
figure('Name', 'Efficiency vs Delta', 'Color', 'w', 'Position', [100 300 800 500]);
plot(delta_vec, eta_total, '-bo', 'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor', 'b');
grid on;
xlabel('\delta (Detuning)');
ylabel('Efficiency \eta [%]');
title('전체 효율 (P_{DC,out} / P_{supply}) vs Detuning \delta');
ylim([0 105]);

% Figure 2: 스위치 손실 & 정류기 손실
figure('Name', 'Device Losses vs Delta', 'Color', 'w', 'Position', [150 250 800 500]);
plot(delta_vec, P_sw_total, '-rs', 'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor', 'r'); hold on;
plot(delta_vec, P_rect_total, '-g^', 'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor', 'g');
grid on;
xlabel('\delta (Detuning)');
ylabel('Power Loss [W]');
title('소자 손실 vs Detuning \delta');
legend('인버터 스위치 손실 (4소자 합)', '정류기 다이오드 손실 (4소자 합)', 'Location', 'best');

% Figure 3: 턴오프 전류
figure('Name', 'Turn-off Current vs Delta', 'Color', 'w', 'Position', [200 200 800 500]);
plot(delta_vec, I_turnoff, '-kd', 'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor', [0.5 0.5 0.5]);
grid on;
xlabel('\delta (Detuning)');
ylabel('Turn-off Current I_{p,turnoff} [A]');
title('턴오프 전류 (Vi 하강에지 시점 I_p 평균) vs Detuning \delta');
yline(0, 'r--', 'LineWidth', 1.5); % ZVS 기준선
legend('턴오프 전류', 'ZVS 기준 (0 A)', 'Location', 'best');

%% ===================== (F) 메모리 정리 =====================
fprintf('\n--- 5. 메모리 정리 ---\n');
clear simIn out logs;
delete(gcp('nocreate'));
fprintf('완료.\n');

%% ===================== Local Functions =====================

function Pavg = local_calc_power(logs, vName, iName, tStart, tEnd)
    % logsout에서 V, I 신호를 추출하여 평균 유효전력을 계산
    Pavg = NaN;
    
    vTs = local_get_ts(logs, vName);
    iTs = local_get_ts(logs, iName);
    
    if isempty(vTs) || isempty(iTs)
        return;
    end
    
    % 시간축 정렬
    tV = vTs.Time(:); v = vTs.Data(:);
    tI = iTs.Time(:); i = iTs.Data(:);
    
    idxV = (tV >= tStart) & (tV <= tEnd);
    idxI = (tI >= tStart) & (tI <= tEnd);
    
    tVw = tV(idxV); vw = v(idxV);
    tIw = tI(idxI); iw = i(idxI);
    
    if numel(tVw) < 2 || numel(tIw) < 2
        return;
    end
    
    % 전압 시간축 기준으로 전류 보간
    iInterp = interp1(tIw, iw, tVw, 'linear', 'extrap');
    
    T = tVw(end) - tVw(1);
    if T <= 0
        return;
    end
    
    Pavg = trapz(tVw, vw .* iInterp) / T;
end

function ts = local_get_ts(logs, targetName)
    ts = [];
    try
        el = logs.get(targetName);
    catch
        el = [];
    end
    if isempty(el)
        for i = 1:logs.numElements
            e = logs.getElement(i);
            if strcmp(e.Name, targetName)
                el = e; break;
            end
        end
    end
    if isempty(el), return; end
    if isa(el, 'timeseries')
        ts = el; return;
    end
    if isobject(el) && isprop(el, 'Values')
        v = el.Values;
        if isa(v, 'timeseries')
            ts = v; return;
        end
    end
end

function Ip_turnoff_avg = local_calc_turnoff_current(logs, viName, ipName, tStart, tEnd)
    % Vi의 하강에지(양→음) 시점에서의 Ip 값을 추출하여 평균 턴오프 전류 반환
    Ip_turnoff_avg = NaN;
    
    Vi_ts = local_get_ts(logs, viName);
    Ip_ts = local_get_ts(logs, ipName);
    
    if isempty(Vi_ts) || isempty(Ip_ts)
        return;
    end
    
    % 시간축 정렬
    tV = Vi_ts.Time(:); vi = Vi_ts.Data(:);
    tI = Ip_ts.Time(:); ip = Ip_ts.Data(:);
    
    idxV = (tV >= tStart) & (tV <= tEnd);
    idxI = (tI >= tStart) & (tI <= tEnd);
    
    tVw = tV(idxV); vi_w = vi(idxV);
    tIw = tI(idxI); ip_w = ip(idxI);
    
    if numel(tVw) < 10 || numel(tIw) < 10
        return;
    end
    
    % Ip를 Vi 시간축으로 보간
    ip_interp = interp1(tIw, ip_w, tVw, 'linear', 'extrap');
    
    % Vi가 양수→0 이하로 떨어지는 하강에지 검출
    idx_falling = find((vi_w(1:end-1) > 0) & (vi_w(2:end) <= 0));
    
    if isempty(idx_falling)
        return;
    end
    
    % 해당 시점의 Ip 추출 및 평균
    Ip_turnoff_avg = mean(ip_interp(idx_falling));
end
