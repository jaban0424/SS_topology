%% SS_find_R_for_power_260407.m
% =========================================================================
% 파일명: SS_find_R_for_power_260407.m
% 작성일: 2026-04-07 21:22
% 설명: 사용자가 지정한 목표 전력(Target Power)을 출력할 수 있는 저항(SS_R)을 
%       찾아주는 반복적 그리드 서치(Iterative Grid Search) 프로그램.
% 
% [수정 및 버전 기록]
% 2026-04-07 21:33: - 목표 전력을 입력전력(Pin) 또는 출력전력(Pout) 중
%                     선택하여 탐색할 수 있는 기능 추가 (Target_Type 변수 신설).
% 2026-04-07 21:22: - 기존 SS_sweep_R_for_power_260313.m 코드 기반으로 신규 생성.
%                   - Ideal Power 공식을 역산하여 기준점(R_base) 계산 로직 추가.
%                   - 초기 탐색 범위를 [R_base - 1, R_base + 35]로 설정.
%                   - 오차(실제 전력 - 목표 전력)의 부호가 바뀌는 두 구간을 추출하여
%                     탐색 범위를 좁히며 N등분 시뮬레이션을 반복(배치 단위)하는 기능 구현.
%                   - 최적화된 저항(Best R) 도출 및 Workspace 출력 기능 추가.
% =========================================================================

%% (A) 사용자 및 기본 설정
Target_Power_W = 22e3; % 목표 전력 (기본값: 22kW)
Target_Type = 'Pout';  % 'Pin' (입력전력) 또는 'Pout' (출력전력) 중 선택
NumPointsPerBatch = 6; % 배치당 저항 탐색 점 개수 (n등분 포인트)
MaxBatches = 4;        % 최대 반복 탐색 배치 횟수

modelName = 'SS_design';
numworkers = 5;

% 측정 윈도우 (SS_topology 값 있으면 사용)
SS_TotalSimulTime = local_get_base_var('SS_TotalSimulTime', 2.5e-3);
tStart = local_get_base_var('SS_SoftStarting_TimeToStartMeasure', 1e-3);
tEnd   = local_get_base_var('SS_SoftStarting_TimeToStopMeasure',  2e-3);

vInName  = 'V0';
iInName  = 'I0';
vOutName = 'Vo';
iOutName = 'Io';

%% (B) 병렬 풀 시작
if isempty(gcp('nocreate'))
    parpool('local', numworkers);  %#ok<PARPLO>
end

%% (C) 이상적 전력 계산에 필요한 파라미터 및 R_base 도출
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
    SS_M  = SS_k * sqrt(SS_Lp * SS_Ls);
end

% Ideal Power 공식: P_ideal = (8 / pi^2)^2 * R * (SS_Vin^2) / (SS_AglFreq * SS_M)^2
% 이를 R에 대해 역산하여 기준 저항값(R_base) 산출:
EqConst = ((8 / pi^2)^2) * (SS_Vin^2) / ( (SS_AglFreq * SS_M)^2 );
R_base = Target_Power_W / EqConst;

% 초기 탐색 범위: [R_base - 1, R_base + 35]
R_min = max(0.1, R_base - 1); % R은 물리적인 저항이므로 최소 0보다 크게 설정
R_max = R_base + 35;

fprintf('\n================== 반복적 최적 저항 탐색 시작 ==================\n');
fprintf('사용자 목표 전력 (Target Power) : %.2f kW (%s 기준)\n', Target_Power_W / 1e3, Target_Type);
fprintf('Ideal 식을 통한 기준 저항 (R_base) : %.3f Ω\n', R_base);
fprintf('초기 탐색 범위 지정: [%.3f, %.3f] Ω\n', R_min, R_max);
fprintf('=================================================================\n');

%% (D) 다중 배치 반복 탐색(Grid Search)
load_system(modelName);
set_param(modelName,'StopTime',num2str(SS_TotalSimulTime));
set_param(modelName,'FastRestart','off');

Best_R = NaN;
Best_Power = NaN;
Min_Error = inf;

for batch = 1:MaxBatches
    fprintf('\n>>> [BATCH %d / %d] 시작\n', batch, MaxBatches);
    fprintf('>>> 현재 탐색 범위: [%.3f, %.3f] Ω (%d points)\n', R_min, R_max, NumPointsPerBatch);
    
    R_list = linspace(R_min, R_max, NumPointsPerBatch);
    simIn(NumPointsPerBatch,1) = Simulink.SimulationInput(modelName);
    
    for k = 1:NumPointsPerBatch
        R = R_list(k);
        simIn(k) = Simulink.SimulationInput(modelName) ...
            .setVariable('SS_R',   R) ...
            .setVariable('SS_Rac', R * 8 / pi^2);
    end
    
    % parsim 실행 (병렬 시뮬레이션)
    out = parsim(simIn, ...
        'UseParallel', true, ...
        'UseFastRestart', 'off', ...
        'TransferBaseWorkspaceVariables', 'on', ...
        'ShowProgress', 'on');
        
    P_in_W  = NaN(size(R_list));
    P_out_W = NaN(size(R_list));
    
    fprintf('----------------------------------------------------------------------\n');
    fprintf('%8s | %14s | %14s | %16s\n', 'SS_R [Ω]', 'P_in [W]', 'P_out [W]', sprintf('Error(%s-Target)', Target_Type));
    fprintf(repmat('-',1,70)); fprintf('\n');
    
    for k = 1:NumPointsPerBatch
        R = R_list(k);
        
        if isprop(out(k),'ErrorMessage') && out(k).ErrorMessage ~= ""
            fprintf('%8.2f | %14s | %14s | %16s\n', R, 'Error', 'Error', 'Error');
            continue;
        end
        
        logs = [];
        if isprop(out(k),'logsout') && ~isempty(out(k).logsout)
            logs = out(k).logsout;
        end
        if isempty(logs)
            fprintf('%8.2f | %14s | %14s | %16s\n', R, 'No Logs', 'No Logs', 'N/A');
            continue;
        end
        
        vInEl  = local_get_element_by_name(logs, vInName);
        iInEl  = local_get_element_by_name(logs, iInName);
        vOutEl = local_get_element_by_name(logs, vOutName);
        iOutEl = local_get_element_by_name(logs, iOutName);
        
        if isempty(vInEl) || isempty(iInEl) || isempty(vOutEl) || isempty(iOutEl)
            fprintf('%8.2f | %14s | %14s | %16s\n', R, 'Data Missing', 'Data Missing', 'N/A');
            continue;
        end
        
        % 전력 계산
        [Pin, okIn]   = local_calc_avg_power(vInEl.Values,  iInEl.Values,  tStart, tEnd);
        [Pout, okOut] = local_calc_avg_power(vOutEl.Values, iOutEl.Values, tStart, tEnd);
        
        if okIn,  P_in_W(k)  = Pin;  end
        if okOut, P_out_W(k) = Pout; end
        
        if strcmp(Target_Type, 'Pin') && okIn
            err = Pin - Target_Power_W;
            fprintf('%8.3f | %14.3f | %14.3f | %16.3f\n', R, Pin, Pout, err);
            
            % 최적값(Best) 기록 갱신 여부
            if abs(err) < Min_Error
                Min_Error = abs(err);
                Best_R = R;
                Best_Power = Pin;
            end
        elseif strcmp(Target_Type, 'Pout') && okOut
            err = Pout - Target_Power_W;
            fprintf('%8.3f | %14.3f | %14.3f | %16.3f\n', R, Pin, Pout, err);
            
            % 최적값(Best) 기록 갱신 여부
            if abs(err) < Min_Error
                Min_Error = abs(err);
                Best_R = R;
                Best_Power = Pout;
            end
        else
            fprintf('%8.3f | %14s | %14s | %16s\n', R, 'Fail', 'Fail', 'N/A');
        end
    end
    fprintf(repmat('-',1,70)); fprintf('\n');
    
    % 오차의 부호가 바뀌는 구간 찾기 (Zero-Crossing)
    if strcmp(Target_Type, 'Pin')
        ErrorList = P_in_W - Target_Power_W;
    else
        ErrorList = P_out_W - Target_Power_W;
    end
    signErr = sign(ErrorList);
    
    % NaN 무시 및 유효한 데이터만 필터링
    validIdx = find(~isnan(signErr));
    if length(validIdx) < 2
        fprintf('유효한 시뮬레이션 결과가 부족하여 배치를 중단합니다.\n');
        break;
    end
    
    crossIdx = -1;
    for vi = 1:length(validIdx)-1
        i1 = validIdx(vi);
        i2 = validIdx(vi+1);
        if signErr(i1) ~= signErr(i2)
            crossIdx = i1;
            break;
        end
    end
    
    % 다음 배치를 위한 범위(R_min, R_max) 조정
    if crossIdx ~= -1
        % 교차점 발견
        next_r_idx = validIdx(find(validIdx == crossIdx) + 1);
        R_min_new = min(R_list(crossIdx), R_list(next_r_idx));
        R_max_new = max(R_list(crossIdx), R_list(next_r_idx));
        fprintf('>>> 교차점 발견! (오차 부호 변화 구간)\n');
        fprintf('>>> 다음 탐색은 [%.3f, %.3f] 구간으로 축소 진행합니다.\n', R_min_new, R_max_new);
        R_min = R_min_new;
        R_max = R_max_new;
    else
        % 교차점이 없다면, 단순 단조 증가/감소 상태라고 가정하고 범위 이동
        [~, mIdx] = min(abs(ErrorList(validIdx)));
        best_idx = validIdx(mIdx);
        if ErrorList(best_idx) > 0
            % 모든 전력이 목표보다 높은 경우 (저항을 더 낮춰야 함)
            fprintf('>>> 모든 결과가 목표 전력보다 초과했습니다 (Pout > Target).\n');
            fprintf('>>> 탐색 범위를 아래로 이동합니다.\n');
            span = R_max - R_min;
            R_max = R_list(validIdx(1));
            R_min = max(0.1, R_list(validIdx(1)) - span);
        else
            % 모든 전력이 목표보다 낮은 경우 (저항을 더 높여야 함)
            fprintf('>>> 모든 결과가 목표 전력보다 미달했습니다 (Target > Pout).\n');
            fprintf('>>> 탐색 범위를 위로 이동합니다.\n');
            span = R_max - R_min;
            R_min = R_list(validIdx(end));
            R_max = R_list(validIdx(end)) + span;
        end
    end
    
    % 범위가 충분히 좁아졌으면 조기 종료 판단 (0.05옴 이내)
    if (R_max - R_min) < 0.05
        fprintf('\n>>> 탐색 범위 구간이 0.05 Ω 미만으로 충분히 좁혀졌습니다. 배치를 조기 종료합니다.\n');
        break;
    end
end

fprintf('\n==================== 탐색 완료 ====================\n');
if isnan(Best_R)
    fprintf('오류: 유효한 해결책을 찾지 못했습니다.\n');
else
    fprintf('목표 전력(Target) : %.2f kW (%s 기준)\n', Target_Power_W/1e3, Target_Type);
    fprintf('찾아낸 최적 R     : %.3f Ω\n', Best_R);
    fprintf('예상되는 전력     : %.3f kW (오차: %.2f W)\n', Best_Power/1e3, Best_Power - Target_Power_W);
    
    % 워크스페이스에 최종 결과 변수로 저장
    assignin('base', 'SS_R_opt', Best_R);
    if strcmp(Target_Type, 'Pin')
        assignin('base', 'SS_Pin_opt', Best_Power);
        fprintf('\n[안내] Workspace에 SS_R_opt, SS_Pin_opt 변수로 최적 결과를 저장했습니다.\n');
    else
        assignin('base', 'SS_Pout_opt', Best_Power);
        fprintf('\n[안내] Workspace에 SS_R_opt, SS_Pout_opt 변수로 최적 결과를 저장했습니다.\n');
    end
end
fprintf('===================================================\n\n');

%% (E) 메모리 정리
clear simIn out logs vInEl iInEl vOutEl iOutEl Pin Pout;

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
