%% SS_sweep_Lp_Ls_Stress_260408.m
% =========================================================================
% [버전 기록]
% v1.0 : (2026-04-08 00:43) 처음 생성됨. 파라미터 Lp, Ls의 2D Sweep을 통해
%                           주요 소자(Cp, Cs, Lp, Ls)의 전류/전압 스트레스를 추출 및 시각화.
% =========================================================================
% [설명] 
% SS_Lp와 SS_Ls를 동시에 변경(Sweep)하면서 각 소자의 전류 및 전압 스트레스(Peak)를
% 3D 플롯으로 관찰하는 스크립트입니다. parsim을 활용하여 병렬 시뮬레이션을 수행합니다.
% =========================================================================

clear; clc; close all;

%% 1. 파라미터 환경 구성
fprintf('파라미터 초기화 중 (SS_topology_design)...\n');
SS_topology_design; % Base Workspace에 파라미터를 로드

%% 2. Sweep 범위 설정 (사용자 조절 가능)
% Lp 범위 및 쪼갤 개수
Lp_min = 70e-6;     % 70uH
Lp_max = 230e-6;    % 230uH
Lp_points = 5;      % Lp 포인트 개수 (예: 5개)

% Ls 범위 및 쪼갤 개수
Ls_min = 70e-6;     % 70uH
Ls_max = 230e-6;    % 230uH
Ls_points = 5;      % Ls 포인트 개수 (예: 5개)

vec_Lp = linspace(Lp_min, Lp_max, Lp_points);
vec_Ls = linspace(Ls_min, Ls_max, Ls_points);

[grid_Lp, grid_Ls] = meshgrid(vec_Lp, vec_Ls);
num_simulations = numel(grid_Lp);

modelName = 'SS_design';

%% 3. 시뮬레이션 입력 배열(Simulink.SimulationInput) 생성
fprintf('Sweep 시작: Lp(%.1f~%.1fuH), Ls(%.1f~%.1fuH) (총 %d 번 시뮬레이션)\n', ...
    Lp_min*1e6, Lp_max*1e6, Ls_min*1e6, Ls_max*1e6, num_simulations);

load_system(modelName);
simIn(1:num_simulations) = Simulink.SimulationInput(modelName);

for idx = 1:num_simulations
    cur_Lp = grid_Lp(idx);
    cur_Ls = grid_Ls(idx);
    
    % 변경된 Lp, Ls에 연동되는 파라미터 재계산
    cur_M  = SS_k * sqrt(cur_Lp * cur_Ls);
    cur_Cp = 1 / (SS_AglFreq^2 * cur_Lp);
    cur_Cs = 1 / (SS_AglFreq^2 * cur_Ls) * (1 - SS_delta);
    
    % 워크스페이스 변수 반영
    simIn(idx) = simIn(idx).setVariable('SS_Lp', cur_Lp);
    simIn(idx) = simIn(idx).setVariable('SS_Ls', cur_Ls);
    simIn(idx) = simIn(idx).setVariable('SS_M',  cur_M);
    simIn(idx) = simIn(idx).setVariable('SS_Cp', cur_Cp);
    simIn(idx) = simIn(idx).setVariable('SS_Cs', cur_Cs);
    
    % 시뮬레이션 옵션
    simIn(idx) = simIn(idx).setModelParameter('StopTime', num2str(SS_TotalSimulTime));
end

%% 4. parsim 병렬 시뮬레이션 실행 (병렬 풀 사용)
fprintf('parsim을 통한 병렬 시뮬레이션 실행 중...\n');
out = parsim(simIn, 'ShowProgress', 'on', 'TransferBaseWorkspaceVariables', 'on');
fprintf('시뮬레이션 완료.\n');

%% 5. 결과 분석 및 스트레스(Peak값) 추출
% 데이터를 저장할 행렬 초기화
stress_V_Cp = zeros(size(grid_Lp));
stress_I_Cp = zeros(size(grid_Lp));

stress_V_Cs = zeros(size(grid_Lp));
stress_I_Cs = zeros(size(grid_Lp));

stress_V_Lp = zeros(size(grid_Lp));
stress_I_Lp = zeros(size(grid_Lp));

stress_V_Ls = zeros(size(grid_Lp));
stress_I_Ls = zeros(size(grid_Lp));

tStart = SS_SoftStarting_TimeToStartMeasure;
tEnd   = SS_SoftStarting_TimeToStopMeasure;

fprintf('스트레스 결과 추출 중...\n');

for idx = 1:num_simulations
    simOut = out(idx);
    
    if isempty(simOut.ErrorMessage)
        logs = simOut.logsout;
        
        try
            % TimeSeries 요소 가져오기
            ts_Vi  = get_ts_from_logsout(logs, 'Vi');
            ts_Vp  = get_ts_from_logsout(logs, 'Vp');
            ts_Ip  = get_ts_from_logsout(logs, 'Ip');
            ts_Vs  = get_ts_from_logsout(logs, 'Vs');
            ts_Is  = get_ts_from_logsout(logs, 'Is');
            ts_Vac = get_ts_from_logsout(logs, 'Vac');
            
            % 공통 Time vector로 보간(Interpolation) - 가변 스텝 불일치 방지
            t_common = linspace(tStart, tEnd, 10000); % 충분히 촘촘한 고정 윈도우(약 10000 샘플)
            
            % 고유한 시간값(Unique)만 추출하여 오류 방지
            [tVi, iVi] = unique(ts_Vi.Time);
            [tVp, iVp] = unique(ts_Vp.Time);
            [tIp, iIp] = unique(ts_Ip.Time);
            [tVs, iVs] = unique(ts_Vs.Time);
            [tIs, iIs] = unique(ts_Is.Time);
            [tVac, iVac] = unique(ts_Vac.Time);
            
            val_Vi  = interp1(tVi, ts_Vi.Data(iVi), t_common, 'linear', 'extrap');
            val_Vp  = interp1(tVp, ts_Vp.Data(iVp), t_common, 'linear', 'extrap');
            val_Ip  = interp1(tIp, ts_Ip.Data(iIp), t_common, 'linear', 'extrap');
            val_Vs  = interp1(tVs, ts_Vs.Data(iVs), t_common, 'linear', 'extrap');
            val_Is  = interp1(tIs, ts_Is.Data(iIs), t_common, 'linear', 'extrap');
            val_Vac = interp1(tVac, ts_Vac.Data(iVac), t_common, 'linear', 'extrap');
            
            % 측정 대상 신호의 연산
            % Cp: 전압 = Vi - Vp, 전류 = Ip
            V_Cp = val_Vi - val_Vp;
            I_Cp = val_Ip;
            
            % Lp: 전압 = Vp, 전류 = Ip
            V_Lp = val_Vp;
            I_Lp = val_Ip;
            
            % Cs: 전압 = Vac - Vs, 전류 = Is
            V_Cs = val_Vac - val_Vs;
            I_Cs = val_Is;
            
            % Ls: 전압 = Vs, 전류 = Is
            V_Ls = val_Vs;
            I_Ls = val_Is;
            
            % Peak 스트레스 추출
            stress_V_Cp(idx) = max(abs(V_Cp));
            stress_I_Cp(idx) = max(abs(I_Cp));
            
            stress_V_Lp(idx) = max(abs(V_Lp));
            stress_I_Lp(idx) = max(abs(I_Lp));
            
            stress_V_Cs(idx) = max(abs(V_Cs));
            stress_I_Cs(idx) = max(abs(I_Cs));
            
            stress_V_Ls(idx) = max(abs(V_Ls));
            stress_I_Ls(idx) = max(abs(I_Ls));
            
        catch ME
            warning('idx=%d 시뮬레이션의 logsout 파싱 에러: %s', idx, ME.message);
        end
    else
        warning('idx=%d 시뮬레이션 에러 발생: %s', idx, simOut.ErrorMessage);
    end
end

%% 6. 그래프 출력 (4개 Figure, 8개 3D Plot)
% X, Y축 단위를 uH로 스케일링
grid_Lp_uH = grid_Lp * 1e6;
grid_Ls_uH = grid_Ls * 1e6;

components = {'Cp', 'Lp', 'Cs', 'Ls'};
stress_V = {stress_V_Cp, stress_V_Lp, stress_V_Cs, stress_V_Ls};
stress_I = {stress_I_Cp, stress_I_Lp, stress_I_Cs, stress_I_Ls};

% uH 단위로 라벨 표시를 위한 변수
x_label = 'Lp (\muH)';
y_label = 'Ls (\muH)';

for c = 1:4
    fig = figure('Color', 'w', 'Position', [100+50*c, 100+50*c, 1000, 450], ...
           'Name', [components{c} ' Stress Analysis']);
    
    % subplot 1: 전압 스트레스 (Voltage Stress)
    subplot(1, 2, 1);
    surf(grid_Lp_uH, grid_Ls_uH, stress_V{c});
    xlabel(x_label, 'FontWeight', 'bold'); 
    ylabel(y_label, 'FontWeight', 'bold'); 
    zlabel('Voltage Level (V, Peak)', 'FontWeight', 'bold');
    title([components{c} ' Voltage Stress']);
    colorbar;
    view(45, 30);
    
    % subplot 2: 전류 스트레스 (Current Stress)
    subplot(1, 2, 2);
    surf(grid_Lp_uH, grid_Ls_uH, stress_I{c});
    xlabel(x_label, 'FontWeight', 'bold'); 
    ylabel(y_label, 'FontWeight', 'bold'); 
    zlabel('Current Level (A, Peak)', 'FontWeight', 'bold');
    title([components{c} ' Current Stress']);
    colorbar;
    view(45, 30);
    
    % 그림 상단 타이틀
    sgtitle(sprintf('Stress Profiles for %s Component', components{c}), ...
        'FontSize', 14, 'FontWeight', 'bold');
end

fprintf('시뮬레이션 및 데이터 시각화가 완료되었습니다.\n');

%% Helper Function
function ts = get_ts_from_logsout(logs, name)
    el = [];
    try
        el = logs.get(name);
    catch
    end
    
    if isempty(el)
        for i = 1:logs.numElements
            e = logs.getElement(i);
            if strcmp(e.Name, name)
                el = e; break;
            end
        end
    end
    
    if isempty(el)
        error('Cannot find %s in logsout', name);
    end
    
    if isa(el, 'timeseries')
        ts = el;
    elseif isobject(el) && isprop(el, 'Values') && isa(el.Values, 'timeseries')
        ts = el.Values;
    else
        error('Data for %s is not a timeseries', name);
    end
end
