%% SS_sweep_R_power.m
% SS_R을 스윕(10~100, 15포인트)하면서 각 케이스의
% - 공급 입력 전력: P_in  = avg( V0 * I0 )
% - DC 출력 전력  : P_out = avg( Vo * Io )
% 을 윈도우 구간에서 계산하여 누적하고, 2개의 꺾은선 그래프로 플롯한다.
%
% 메모리 절약 포인트:
% - 각 시뮬레이션마다 logsout(timeseries)는 즉시 파워만 계산 후 폐기(clear)
% - 결과는 (R, Pin, Pout) 숫자 배열만 저장
%
% 전제:
% - 모델 `SS_design` 에서 V0, I0, Vo, Io 가 logsout 에 해당 이름으로 로깅됨

%% (A) 사용자 설정
modelName = 'SS_design';

R_min = 10;
R_max = 100;
Npts  = 15;
R_list = linspace(R_min, R_max, Npts);

% SS_topology.m에서 쓰는 설정값이 base에 있으면 그대로 사용, 없으면 기본값 사용
if evalin('base','exist("SS_TotalSimulTime","var")')
    SS_TotalSimulTime = evalin('base','SS_TotalSimulTime');
else
    SS_TotalSimulTime = 2.5e-3;
end

if evalin('base','exist("SS_SoftStarting_TimeToStartMeasure","var")')
    tStart = evalin('base','SS_SoftStarting_TimeToStartMeasure');
else
    tStart = 1e-3;
end

if evalin('base','exist("SS_SoftStarting_TimeToStopMeasure","var")')
    tEnd = evalin('base','SS_SoftStarting_TimeToStopMeasure');
else
    tEnd = 2e-3;
end

% 측정에 사용할 신호명
vInName  = 'V0';
iInName  = 'I0';
vOutName = 'Vo';
iOutName = 'Io';

%% (B) 결과 저장용 배열
P_in_W  = NaN(size(R_list));
P_out_W = NaN(size(R_list));

%% (C) 모델 로드(1회)
load_system(modelName);
set_param(modelName,'StopTime',num2str(SS_TotalSimulTime));

fprintf('\n==================== SWEEP SS_R: %g ~ %g (%d pts) ====================\n', ...
    R_min, R_max, Npts);
fprintf('Window: %.6f ~ %.6f s\n', tStart, tEnd);
fprintf('Signals: Pin=(%s*%s), Pout=(%s*%s)\n', vInName, iInName, vOutName, iOutName);
fprintf('----------------------------------------------------------------------------------\n');
fprintf('%8s | %14s | %14s\n', 'SS_R', 'P_in [W]', 'P_out [W]');
fprintf(repmat('-',1,44)); fprintf('\n');

%% (D) 스윕 루프
for k = 1:numel(R_list)
    R = R_list(k);

    % SS_topology와 동일하게 SS_R와 SS_Rac를 함께 세팅(모델에서 무엇을 쓰든 안전)
    assignin('base','SS_R', R);
    assignin('base','SS_Rac', R * 8 / pi^2);

    % 시뮬레이션 실행
    simOut = sim(modelName);

    % logsout 확보 (simOut 우선, 없으면 base logsout)
    logs = [];
    if isprop(simOut,'logsout') && ~isempty(simOut.logsout)
        logs = simOut.logsout;
    elseif evalin('base','exist("logsout","var")')
        logs = evalin('base','logsout');
    end

    if isempty(logs)
        fprintf('%8.2f | %14s | %14s\n', R, 'N/A', 'N/A');
        clear simOut logs
        continue;
    end

    % 필요한 4개 신호만 뽑아서 파워 계산 후 숫자만 저장
    vInEl  = local_get_element_by_name(logs, vInName);
    iInEl  = local_get_element_by_name(logs, iInName);
    vOutEl = local_get_element_by_name(logs, vOutName);
    iOutEl = local_get_element_by_name(logs, iOutName);

    if isempty(vInEl) || isempty(iInEl) || isempty(vOutEl) || isempty(iOutEl)
        fprintf('%8.2f | %14s | %14s\n', R, 'N/A', 'N/A');
        clear simOut logs vInEl iInEl vOutEl iOutEl
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

    % 시계열/로그는 즉시 폐기(메모리 절약)
    clear simOut logs vInEl iInEl vOutEl iOutEl
end

fprintf(repmat('-',1,44)); fprintf('\n');
fprintf('==================================================================================\n');

%% (E) 플롯
figure('Name','SS_R sweep: Supply input vs DC output power');
plot(R_list, P_in_W/1e3,  '-o', 'LineWidth', 1.5); hold on;
plot(R_list, P_out_W/1e3, '-s', 'LineWidth', 1.5);
grid on;
xlabel('SS\_R [\Omega]');
ylabel('Power [kW]');
title('Power vs SS\_R (15 simulations, windowed average)');
legend('Supply input power (V0*I0)', 'DC output power (Vo*Io)', 'Location', 'best');

%% (F) 로컬 함수
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
    % Pavg = (1/T) * integral v(t)*i(t) dt  over [tStart, tEnd]
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

