%% SS_sweep_delta_TurnOffCurrent.m
% =========================================================================
% [버전 기록]
% v1.0 : (이전 버전) 디튜닝 파라미터(delta) 스윕에 따른 수식/시뮬레이션 비교 스크립트 작성.
% v1.1 : (현재 버전 - 2026-04-07 02:55:16) SS_topology_design.m 에서 변수를 자동으로 로드하도록 개선.
%
% [설명] 
% 2차측 디튜닝 파라미터(delta)를 스윕하면서:
% 1) 이론식 기반 턴오프 전류 계산 (SS_delta_TurnOffCurrent_... 로직)
% 2) Simulink 시뮬레이션 기반(parsim 병렬처리) 턴오프 전류 관측
% 위 두 결과를 하나의 창에 통합하여 플롯합니다.
% =========================================================================

% clear; clc; close all;

%% ===================== (A) 사용자 설정 파라미터 =====================
% 스윕할 delta 범위
delta_min = 0;
delta_max = 0.05;

% 총 배치 수 (결과 포인트 개수)
num_points = 11; 

% 병렬 처리 워커(Core) 수 설정
num_workers = 5;

% 턴오프 전류 관측을 위해 평균을 낼 마지막 주기 수
num_turnoff_periods = 20;

% 시뮬레이션 모델 이름 지정
modelName = 'SS_design';

%% ===================== (B) 기본 파라미터 로드 및 계산 =====================
% SS_topology_design.m 직접 실행하여 파라미터 동적 로드
runSim_backup = exist('runSim','var');
if runSim_backup
    runSim_orig = runSim;
end
runSim = 0; % 시뮬레이션 돌지 않도록 설정
SS_topology_design;
if runSim_backup
    runSim = runSim_orig;
else
    clear runSim;
end
clear runSim_backup runSim_orig;

% 시뮬레이션 시간 정보 확보
tEnd_measure = SS_SoftStarting_TimeToStopMeasure;
tEnd_sim     = SS_TotalSimulTime;

% delta 스펙 벡터 생성
delta_vec = linspace(delta_min, delta_max, num_points);


%% ===================== (C) 1. 이론식(수식) 기반 턴오프 전류 계산 =====================
fprintf('\n--- 1. 수식 기반 턴오프 전류 계산 ---\n');

% 기존 수식에서 지정된 정전압 및 레벨값 (필요시 변경 가능)
UAB = SS_Vin;  % 입력측 레벨
uab = 850;     % 출력측 레벨 (기존 코드 기준)

f0 = SS_Freq;
omega = 2*pi*f0;
L = SS_Lp;
k = SS_k;
phi1_rad = deg2rad(90);

m_max_ref = 1001;
m_all = 1:2:m_max_ref;
m_h   = 3:2:m_max_ref;
A = 2*sqrt(2)/pi;

Ip_formula_turnoff = NaN(size(delta_vec));

for idx = 1:length(delta_vec)
    delta = delta_vec(idx);
    
    if abs(1-delta) < 1e-12
        continue;
    end
    
    % Is,m 계산 및 phi2(delta)
    Is_1st_term = @(m) sqrt(Lp/Ls) .* (A .* UAB ./ (omega*L .* ( (m.^2 - 1) .* (m.^2 .* (1-delta) - 1) ./ (m.^2 .* k .* (1-delta)) - m.^2 .* k ))) .* exp(-1j*pi/2);
    
    Is_1 = Is_1st_term(1);
    Is_h = Is_1st_term(m_h);
    Is1_rms = abs(Is_1);
    Ish_sum = sum(abs(Is_h));
    
    ratio = Ish_sum / Is1_rms;
    phi2_rad = asin(min(max(ratio, -1), 1));
    phi_rad = phi1_rad + phi2_rad;
    
    % Ip,m 계산 (새 공식 적용)
    Ip_term = @(m) ...
        (A .* UAB ./ (omega*L .* ( m.^2 - 1 - (m.^4 .* k.^2 .* (1-delta)) ./ (m.^2 .* (1-delta) - 1) ))) .* exp(-1j*pi/2) ...
        + ...
        (A .* uab ./ (omega*L .* ( ((m.^2 - 1) ./ (m .* k)) .* (m - 1 ./ (m .* (1-delta))) - m.^2 .* k ))) .* exp(1j*(phi_rad + pi/2));
        
    Ip_tot = sum(Ip_term(m_all));
    Ip_formula_turnoff(idx) = -imag(Ip_tot); % 허수성분이 턴오프 전류 관여
end
fprintf(' => 수식 기반 delta = %.4f ~ %.4f 계산 완료.\n', delta_min, delta_max);


%% ===================== (D) 2. 병렬 시뮬레이션 기반 턴오프 전류 측정 =====================
fprintf('\n--- 2. 시뮬레이션 기반(parsim 병렬) 턴오프 전류 측정 ---\n');

% 병렬 풀 가동 (기존 풀이 없을 경우 워커 개수 맞춰 생성)
if isempty(gcp('nocreate'))
    parpool('local', num_workers);
end

% Simulink 준비
load_system(modelName);
set_param(modelName,'StopTime',num2str(tEnd_sim));
set_param(modelName,'FastRestart','off');

% SimulationInput 배열 병렬 설계
simIn(num_points,1) = Simulink.SimulationInput(modelName);

for idx = 1:num_points
    delta = delta_vec(idx);
    
    % delta가 바뀌면 2차측 튜닝 커패시터 Cs도 영향을 받음 (SS_topology.m 방식 준용)
    Cs_val = 1 / (SS_AglFreq^2 * SS_Ls) * (1 - delta);
    
    % delta 와 SS_Cs 를 다이내믹하게 덮어쓰도록 묶음
    simIn(idx) = Simulink.SimulationInput(modelName) ...
        .setVariable('SS_delta', delta) ...
        .setVariable('SS_Cs', Cs_val);
end

fprintf('시뮬레이션 전송 중... (총 %d 개 배치)\n', num_points);
out = parsim(simIn, ...
    'UseParallel', true, ...
    'UseFastRestart', 'off', ...
    'TransferBaseWorkspaceVariables', 'on', ...
    'ShowProgress', 'on');


%% ===================== (E) 병렬 시뮬레이션 결괏값 후처리 (떨어지는 에지 추출) =====================
fprintf('\n--- 시뮬레이션 결과 후처리 및 관측 ---\n');

Ip_sim_turnoff = NaN(size(delta_vec));
t_turnoff_start = tEnd_measure - num_turnoff_periods * (1/SS_Freq);

for idx = 1:num_points
    delta = delta_vec(idx);
    
    % 만약 시뮬레이션 문제로 에러 발생한 경우
    if isprop(out(idx),'ErrorMessage') && out(idx).ErrorMessage ~= ""
        fprintf('  [오류] delta=%.4f 처리 실패: %s\n', delta, out(idx).ErrorMessage);
        continue;
    end
    
    % logsout 로그 확인
    logs = [];
    if isprop(out(idx),'logsout') && ~isempty(out(idx).logsout)
        logs = out(idx).logsout;
    end
    if isempty(logs)
        fprintf('  [경고] delta=%.4f 관측값(logsout) 확인 불가.\n', delta);
        continue;
    end
    
    % 필요한 Vi, Ip 검출
    Vi_el = local_get_element_by_name(logs, 'Vi');
    Ip_el = local_get_element_by_name(logs, 'Ip');
    if isempty(Vi_el) || isempty(Ip_el)
        continue;
    end
    
    Vi_ts = Vi_el.Values;
    Ip_ts = Ip_el.Values;
    
    % 보간하여 시간 정렬
    [t_to, v_i_to, i_p_to] = local_align_times(Vi_ts, Ip_ts, t_turnoff_start, tEnd_measure);
    if isempty(t_to)
        continue;
    end
    
    % Vi 구형파 하강 에지 (Positive -> Negative) 순간을 인덱스로 서칭
    idx_falling = find((v_i_to(1:end-1) > 0) & (v_i_to(2:end) <= 0));
    
    if ~isempty(idx_falling)
        Ip_turnoff_avg = mean(i_p_to(idx_falling));
        Ip_sim_turnoff(idx) = Ip_turnoff_avg;
        fprintf('  delta=%.4f 처리 완료 (관측 턴오프 전류: %8.3f A)\n', delta, Ip_turnoff_avg);
    else
        fprintf('  [경고] delta=%.4f 데이터에 하강 에지 턴오프 구간이 포착되지 않음.\n', delta);
    end
end


%% ===================== (F) 통합 플롯 (수식 vs 모델 시뮬레이션) =====================
figure('Name', 'Turn-Off Current vs \delta: Formula & Simulation', 'Color', 'w', 'Position', [150 150 800 500]);
plot(delta_vec, Ip_formula_turnoff, '-b', 'LineWidth', 2, 'DisplayName', '이론수식 계산결과 (-Imag(I_{p,total}))');
hold on;
plot(delta_vec, Ip_sim_turnoff, '-ro', 'LineWidth', 2, 'MarkerFaceColor','r', 'MarkerSize', 6, 'DisplayName', '시뮬레이션 관측값 (평균 턴오프 전류)');
grid on;
xlabel('\delta = \DeltaC_s / C_{s,res} (디튜닝 수치)');
ylabel('Turn-off Current [A]');
title(sprintf('디튜닝 수치(\\delta)에 따른 턴오프 전류 변화 비교\n(마지막 %d주기 평균 적용)', num_turnoff_periods));
legend('Location', 'best');


%% ===================== 로컬 함수 모음 =====================
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

function [tOut, vOut, iOut] = local_align_times(vTs, iTs, tStart, tEnd)
    tV = vTs.Time(:); v = vTs.Data(:);
    tI = iTs.Time(:); i = iTs.Data(:);
    
    if ~isvector(v) || ~isvector(i)
        error('local_align_times: 다차원(벡터/행렬) 신호는 지원하지 않습니다.');
    end
    
    idxV = (tV >= tStart) & (tV <= tEnd);
    idxI = (tI >= tStart) & (tI <= tEnd);
    
    tVw = tV(idxV); vw = v(idxV);
    tIw = tI(idxI); iw = i(idxI);
    
    if numel(tVw) < 2 || numel(tIw) < 2
        tOut = []; vOut = []; iOut = [];
        return;
    end
    
    iInterp = interp1(tIw, iw, tVw, 'linear', 'extrap');
    tOut = tVw; vOut = vw; iOut = iInterp;
end
