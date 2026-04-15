           %% SS_sweep_delta_for_phi_Sumulink_260407.m
% =========================================================================
% [버전 기록]
% v1.1 : (2026-04-07 21:30:00)
%        - 통합 비교 스펙트럼(3D)을 위해 각 차수별 위상 데이터(Ism_sim_ang_all) 추출 및 저장 로직 추가
% v1.0 : (2026-04-07 03:09:12) delta 스윕에 따른 수식(\phi) 해와 
%        Simulink(parsim) 기반 Vac, Vi 간의 기본파 상대 위상차를 비교 관측.
% =========================================================================

% 외부 작업공간 변수 유지를 위해 clear 제거

%% ===================== (A) 사용자 설정 파라미터 =====================
if exist('delta_list', 'var')
    delta_vec = delta_list;
    num_points = length(delta_vec);
else
    delta_min = 0;
    delta_max = 0.5;
    num_points = 11; % 스윕 배치의 개수
    delta_vec = linspace(delta_min, delta_max, num_points);
end

% 병렬 처리 워커 수 설정
num_workers = 5;

% 시뮬레이션 모델 및 로깅 이름
modelName = 'SS_design';
refName = 'Vi';
tgtName = 'Vac';

% 기준 윈도우 계산 주기 (정상 상태 확인을 위한 마지막 20주기)
num_turnoff_periods = 200;

%% ===================== (B) 파라미터 로드 파트 =====================
fprintf('--- 1. 시스템 파라미터 동적 로드 중 (SS_topology_design) ---\n');
runSim_backup = exist('runSim','var');
if runSim_backup
    runSim_orig = runSim;
end
runSim = 0; % 파라미터만 셋업
SS_topology_design;
if runSim_backup
    runSim = runSim_orig;
else
    clear runSim;
end
clear runSim_backup runSim_orig;

tEnd_measure = SS_SoftStarting_TimeToStopMeasure;
tEnd_sim     = SS_TotalSimulTime;

% tStart_measure (마지막 N주기 윈도우 설정)
tStart_measure = tEnd_measure - num_turnoff_periods * (1 / SS_Freq);

%% =
% 
% ==================== (C) 병렬 시뮬레이션 기반 \phi 추출 =====================
fprintf('\n--- 2. 시뮬레이션 기반(parsim 병렬) Vac 위상차 관측 ---\n');

if isempty(gcp('nocreate'))
    parpool('local', num_workers);
end

load_system(modelName);
set_param(modelName, 'StopTime', num2str(tEnd_sim));
set_param(modelName, 'FastRestart', 'off');

simIn(num_points, 1) = Simulink.SimulationInput(modelName);

for idx = 1:num_points
    delta = delta_vec(idx);
    
    % delta가 바뀌면 2차측 튜닝 커패시터 Cs 값에 영향을 미침
    Cs_val = 1 / (SS_AglFreq^2 * SS_Ls) * (1 - delta);
    
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

%% ===================== (D) 로그 후처리 및 페이저 계산 =====================
fprintf('\n--- 시뮬레이션 결과 후처리 및 기준 페이저 추출 ---\n');

phi_sim_deg = NaN(size(delta_vec));

for idx = 1:num_points
    delta = delta_vec(idx);
    
    if isprop(out(idx),'ErrorMessage') && out(idx).ErrorMessage ~= ""
        fprintf('  [오류] delta=%.4f 처리 실패.\n', delta);
        continue;
    end
    
    logs = [];
    if isprop(out(idx),'logsout') && ~isempty(out(idx).logsout)
        logs = out(idx).logsout;
    end
    if isempty(logs), continue; end
    
    % Vi, Vac 추출
    Vi_ts  = local_get_timeseries_by_name(logs, refName);
    Vac_ts = local_get_timeseries_by_name(logs, tgtName);
    
    % 추가: 2차측 AC 전류 Is(또는 Iac) 추출 (Simulink 모델 내 로깅된 이름에 맞춤)
    Is_ts = local_get_timeseries_by_name(logs, 'Iac');
    if isempty(Is_ts)
        Is_ts = local_get_timeseries_by_name(logs, 'Is');
    end
    if isempty(Is_ts)
        Is_ts = local_get_timeseries_by_name(logs, 'I0'); 
    end
    
    if isempty(Vi_ts) || isempty(Vac_ts)
        continue;
    end
    
    % 특정 측정 윈도우 내에서 기본파 페이저 추출
    Vi_Ph  = local_calc_f0_phasor_only(Vi_ts, SS_Freq, tStart_measure, tEnd_measure);
    Vac_Ph = local_calc_f0_phasor_only(Vac_ts, SS_Freq, tStart_measure, tEnd_measure);
    
    % 상대 위상 계산 (Vi 기준)
    phi_meas_rad = angle(Vac_Ph) - angle(Vi_Ph);
    phi_meas_deg = rad2deg(phi_meas_rad);
    
    % 180도 부근(근사적으로 0~360 등으로) 튀는 문제를 잡기 위한 wrapTo180
    phi_meas_deg = wrapTo180(phi_meas_deg); 
    
    phi_sim_deg(idx) = phi_meas_deg;
    fprintf('  delta=%.4f 처리 완료 | Vi-Vac 상대 위상차: %8.3f [deg]\n', delta, phi_meas_deg);
    
    % 푸리에 분석으로 각 고조파 성분의 크기 추출
    if ~isempty(Is_ts)
        if ~exist('m_list', 'var')
            m_list = 1:2:101; 
        end
        % 초기화 (첫 스텝에서)
        if ~exist('Ism_sim_mag_all', 'var') || size(Ism_sim_mag_all,2) ~= num_points
            Ism_sim_mag_all = zeros(length(m_list), num_points);
            Ism_sim_ang_all = zeros(length(m_list), num_points);
        end
        
        % 주어진 m_list에 대해 고조파 페이저 추출
        for m_idx = 1:length(m_list)
            m = m_list(m_idx);
            % local_calc_f0_phasor_only 를 고조파 주파수(m * f0)로 응용 계산
            harmonic_phasor = local_calc_f0_phasor_only(Is_ts, m * SS_Freq, tStart_measure, tEnd_measure);
            % f0의 m배 주파수에 해당하는 peak 페이저이므로, RMS로 변환
            Ism_sim_mag_all(m_idx, idx) = abs(harmonic_phasor) / sqrt(2);
            % 위상 각도 (rad) 저장
            Ism_sim_ang_all(m_idx, idx) = angle(harmonic_phasor);
        end
    end
end


%% ===================== (E) 통합 플롯 시각화 =====================
figure('Name', 'Detuning \delta vs \phi Phase Shift', 'Color', 'w', 'Position', [200 200 700 500]);
plot(delta_vec, phi_sim_deg, '-rx', 'LineWidth', 2, 'MarkerSize', 8, 'DisplayName', '시뮬레이션 관측 \phi (Vac - Vi 기본파 위상차)');
grid on; hold on;
xlabel('\delta = \DeltaC_s / C_{s,res} (디튜닝 수치)');
ylabel('Relative Phase \phi [deg]');
title(sprintf('디튜닝 수치(\\delta)에 따른 2차측 상대 위상(\\phi) 변화 검증\n(마지막 %d주기 AC페이저 측정)', num_turnoff_periods));
legend('Location', 'best');

%% ===================== (F) 메모리/풀 정리 =====================
clear simIn out logs Vi_ts Vac_ts Vi_Ph Vac_Ph;
delete(gcp('nocreate'));

%% ===================== Local Functions =====================
function ts = local_get_timeseries_by_name(logs, targetName)
    ts = [];
    try el = logs.get(targetName); catch, el=[]; end
    if isempty(el)
        for i = 1:logs.numElements
            e = logs.getElement(i);
            if strcmp(e.Name, targetName), el = e; break; end
        end
    end
    if isempty(el), return; end
    if isa(el, 'timeseries'), ts = el; return; end
    if isobject(el) && isprop(el, 'Values')
        v = el.Values;
        if isa(v, 'timeseries'), ts = v; return; end
    end
end

function X = local_calc_f0_phasor_only(ts, f0, tStart, tEnd)
    t = ts.Time(:);
    x = ts.Data(:);
    idx = (t >= tStart) & (t <= tEnd);
    t = t(idx); x = x(idx);
    
    w0  = 2*pi*f0;
    xac = x - mean(x);
    ref = exp(-1j*w0*t);

    T = t(end) - t(1);
    X = (2/T) * trapz(t, xac .* ref);
end
