%% SS_power_analysis.m
% [2026/03/09/01:18 cursor ??? model로 마지막 수정.] <- 해당 코드를 편집하는 모델은 이 항목을 계속 업데이트 할 것!!!
% SS 토폴로지 IPT 시스템에서 12개 신호(V0,I0,Vi,Ii,Vp,Ip,Vs,Is,Vac,Iac,Vo,Io)를 이용해
% 6개 지점의 전력을 분석하는 스크립트.
%
% 사용 전제:
%   - Simulink 모델 `SS_design` 에서 각 신호가 logsout 에 이름 그대로 로깅되어 있음
%   - (권장) 먼저 `SS_topology.m` 을 실행하여 SS_Freq, SS_SoftStarting_*, logsout 등을 생성
%
% 이 스크립트는 SS_topology.m 은 수정하지 않고, 별도의 분석만 수행한다.

%% (A) 사용자 설정

% 필요 시, 여기서 SS_topology 를 다시 실행할지 여부
run_SS_topology = 0;   % 필요하면 true 로 바꿔 사용

% 모델 이름 (SS_topology.m 과 동일하게 맞춤)
modelName = 'SS_design';

% 기본 측정 윈도우 / 주파수는 SS_topology 에서 정의한 값을 우선 사용
if evalin('base','exist("SS_Freq","var")')
    f0 = evalin('base','SS_Freq');
else 
    f0 = 85e3;  %#ok<NASGU>  % 기본값
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

%% (B) 필요 시 SS_topology 실행 (옵션)
if run_SS_topology
    % SS_topology 내부에서 시뮬레이션과 로그 통계까지 모두 수행됨
    SS_topology;
end

%% (C) logsout 가져오기
logs = [];

% 1) base workspace에 out 이 있고, logsout 프로퍼티가 있으면 우선 사용
if evalin('base','exist("out","var")')
    out = evalin('base','out');
    if isprop(out,'logsout') && ~isempty(out.logsout)
        logs = out.logsout;
    end
end

% 2) 직접 logsout 변수가 있는 경우
if isempty(logs) && evalin('base','exist("logsout","var")')
    logs = evalin('base','logsout');
end

if isempty(logs)
    error('logsout 을 찾지 못했습니다. SS_design 의 Signal logging 설정이나 SS_topology 실행 여부를 확인하세요.');
end

%% (D) 분석할 6개 전력 지점과 12개 신호 정의
%
% label        : 사람이 읽기 쉬운 설명
% vName, iName : logsout 에 저장된 전압/전류 신호 이름

pairs = {
    'Supply input (V0,I0)',           'V0',   'I0';   % DC 버스(공급) 전력
    'Inverter output (Vi,Ii)',        'Vi',   'Ii';   % 인버터 출력 & 공진 네트워크 입력
    'Primary coil (Vp,Ip)',           'Vp',   'Ip';   % 1차 코일에 인가되는 전력
    'Secondary coil (Vs,Is)',         'Vs',   'Is';   % 2차 코일에서 수신되는 전력
    'Rectifier input AC (Vac,Iac)',   'Vac',  'Iac';  % 정류기 입력 AC 전력
    'DC output (Vo,Io)',              'Vo',   'Io';   % 정류기 출력 DC 전력
};

%% (E) 결과 구조체 초기화
N = size(pairs,1);
results = struct( ...
    'label',  [], ...
    'vName',  [], ...
    'iName',  [], ...
    'Pavg',   NaN, ...
    'Vrms',   NaN, ...
    'Irms',   NaN, ...
    'Sapp',   NaN, ...
    'PF',     NaN);
results = repmat(results, N, 1);

%% (F) 각 지점별 전력 계산
for k = 1:N
    label = pairs{k,1};
    vName = pairs{k,2};
    iName = pairs{k,3};

    % logsout 에서 해당 신호 가져오기
    vEl = local_get_element_by_name(logs, vName);
    iEl = local_get_element_by_name(logs, iName);

    if isempty(vEl) || isempty(iEl)
        fprintf('[경고] "%s" 단계에서 신호 "%s" 또는 "%s" 를 logsout 에서 찾지 못했습니다.\n', ...
            label, vName, iName);
        continue;
    end

    vTs = vEl.Values;
    iTs = iEl.Values;

    % 시간 윈도우 내에서 두 신호를 동일 시간축으로 정렬
    [t, v, i] = local_align_times(vTs, iTs, tStart, tEnd);
    if isempty(t)
        fprintf('[경고] "%s" 단계에서 유효한 데이터가 윈도우 내에 없습니다.\n', label);
        continue;
    end

    T = t(end) - t(1);
    if T <= 0
        fprintf('[경고] "%s" 단계에서 시간 길이가 0 입니다.\n', label);
        continue;
    end

    % 평균 실효 전력 (time-domain)
    Pavg = trapz(t, v .* i) / T;          % [W]

    % RMS (time-domain)
    Vrms = sqrt(trapz(t, v.^2) / T);      % [V]
    Irms = sqrt(trapz(t, i.^2) / T);      % [A]

    % 피상전력 및 PF
    Sapp = Vrms * Irms;                   % [VA]
    PF   = NaN;
    if Sapp > 0
        PF = Pavg / Sapp;
    end

    % 결과 저장
    results(k).label = label;
    results(k).vName = vName;
    results(k).iName = iName;
    results(k).Pavg  = Pavg;
    results(k).Vrms  = Vrms;
    results(k).Irms  = Irms;
    results(k).Sapp  = Sapp;
    results(k).PF    = PF;
end

%% (G) 결과 출력
fprintf('\n==================== POWER ANALYSIS (time-domain, windowed) ====================\n');
% fprintf('Model: %s\n', modelName);
% fprintf('Window: %.6f ~ %.6f s  (%.3f ms)\n', tStart, tEnd, (tEnd - tStart)*1e3);
% fprintf('신호쌍: (V,I) = (V0,I0), (Vi,Ii), (Vp,Ip), (Vs,Is), (Vac,Iac), (Vo,Io)\n');
fprintf('----------------------------------------------------------------------------------\n');
fprintf('%-28s | %12s | %10s | %10s | %12s | %7s\n', ...
    'Stage', 'P_avg [W]', 'Vrms [V]', 'Irms [A]', 'S_app [VA]', 'PF');
fprintf(repmat('-',1,96)); fprintf('\n');

for k = 1:N
    R = results(k);
    if isnan(R.Pavg)
        fprintf('%-28s | %12s | %10s | %10s | %12s | %7s\n', ...
            R.label, 'N/A', 'N/A', 'N/A', 'N/A', 'N/A');
    else
        fprintf('%-28s | %12.3f | %10.3f | %10.3f | %12.3f | %7.3f\n', ...
            R.label, R.Pavg, R.Vrms, R.Irms, R.Sapp, R.PF);
    end
end

% 전체 효율 예시: 공급 입력 대비 DC 출력
idxSupply = 1;   % Supply input (V0,I0)
idxDCout  = 6;   % DC output (Vo,Io)
if ~isnan(results(idxSupply).Pavg) && ~isnan(results(idxDCout).Pavg) && results(idxSupply).Pavg > 0
    eta_total = results(idxDCout).Pavg / results(idxSupply).Pavg * 100;
    fprintf('----------------------------------------------------------------------------------\n');
    fprintf('전체 효율 (DC 출력 / 공급 입력) ≈ %.2f %%\n', eta_total);
end
fprintf('이상 DC 출력 : %.2f kW \n', SS_Ideal_Po/1000);

%% (H) 구간별 전력 손실(인접 지점 차이)
% NOTE:
% - 이 손실은 "각 지점에서 측정된 평균 유효전력(Pavg)"의 차이입니다.
% - 스위치/다이오드 소자 손실과는 별도로, 제어/공진/정류/필터/기생 등 전체 효과가 포함될 수 있습니다.
idxMap = struct( ...
    'Supply',   1, ... % (V0,I0)
    'InvOut',   2, ... % (Vi,Ii)
    'Primary',  3, ... % (Vp,Ip)
    'Secondary',4, ... % (Vs,Is)
    'RectIn',   5, ... % (Vac,Iac)
    'DCOut',    6);    % (Vo,Io)

stagePairs = { ...
    'Supply -> InverterOut', idxMap.Supply,   idxMap.InvOut; ...
    'InverterOut -> Primary',idxMap.InvOut,   idxMap.Primary; ...
    'Primary -> Secondary',  idxMap.Primary,  idxMap.Secondary; ...
    'Secondary -> RectIn',   idxMap.Secondary,idxMap.RectIn; ...
    'RectIn -> DCOut',       idxMap.RectIn,   idxMap.DCOut; ...
    'Supply -> DCOut (total)',idxMap.Supply,  idxMap.DCOut; ...
};

fprintf('\n==================== STAGE LOSSES (ΔPavg, windowed) ====================\n');
fprintf('%-26s | %12s | %12s | %12s | %9s\n', 'Section', 'P_up [W]', 'P_dn [W]', 'Loss [W]', 'Loss[%]');
fprintf(repmat('-',1,82)); fprintf('\n');

for kk = 1:size(stagePairs,1)
    label = stagePairs{kk,1};
    iu = stagePairs{kk,2};
    id = stagePairs{kk,3};

    Pu = results(iu).Pavg;
    Pd = results(id).Pavg;

    if isnan(Pu) || isnan(Pd)
        fprintf('%-26s | %12s | %12s | %12s | %9s\n', label, 'N/A', 'N/A', 'N/A', 'N/A');
        continue;
    end

    lossW = Pu - Pd;
    lossPct = NaN;
    if abs(Pu) > 0
        lossPct = lossW / Pu * 100;
    end

    fprintf('%-26s | %12.3f | %12.3f | %12.3f | %9.2f\n', label, Pu, Pd, lossW, lossPct);
end

fprintf('==================================================================================\n');

%% (I) 스위치 / 정류기 손실 분석

% 인버터 스위치(풀브리지, 소자 4개) : Vds4, Id4  -> 한 소자 손실 * 4
% 정류기 다이오드(풀브리지, 소자 4개) : Vdi, Idi -> 한 소자 손실 * 4

inv_vName = 'Vds4';
inv_iName = 'Id4';
rect_vName = 'Vdi';
rect_iName = 'Idi';

P_sw_1 = NaN;   % 인버터 스위치 1소자 평균손실
P_sw_4 = NaN;   % 인버터 스위치 4소자 합계
P_d_1  = NaN;   % 정류기 다이오드 1소자 평균손실
P_d_4  = NaN;   % 정류기 다이오드 4소자 합계

% --- 인버터 스위치 손실 ---
inv_vEl = local_get_element_by_name(logs, inv_vName);
inv_iEl = local_get_element_by_name(logs, inv_iName);

if isempty(inv_vEl) || isempty(inv_iEl)
    fprintf('[경고] 인버터 스위치 손실 계산용 신호 "%s" 또는 "%s" 를 logsout 에서 찾지 못했습니다.\n', ...
        inv_vName, inv_iName);
else
    vTs = inv_vEl.Values;
    iTs = inv_iEl.Values;

    [t_sw, v_sw, i_sw] = local_align_times(vTs, iTs, tStart, tEnd);
    if isempty(t_sw)
        fprintf('[경고] 인버터 스위치 손실 계산 윈도우 내에 유효 데이터가 없습니다.\n');
    else
        T_sw = t_sw(end) - t_sw(1);
        if T_sw > 0
            % 한 소자 손실 (time-domain 평균)
            P_sw_1 = trapz(t_sw, v_sw .* i_sw) / T_sw;   % [W]
            P_sw_4 = 4 * P_sw_1;                         % 풀브리지 4소자 합계
        end
    end
end

% --- 정류기 다이오드 손실 ---
rect_vEl = local_get_element_by_name(logs, rect_vName);
rect_iEl = local_get_element_by_name(logs, rect_iName);

if isempty(rect_vEl) || isempty(rect_iEl)
    fprintf('[경고] 정류기 다이오드 손실 계산용 신호 "%s" 또는 "%s" 를 logsout 에서 찾지 못했습니다.\n', ...
        rect_vName, rect_iName);
else
    vTs = rect_vEl.Values;
    iTs = rect_iEl.Values;

    [t_d, v_d, i_d] = local_align_times(vTs, iTs, tStart, tEnd);
    if isempty(t_d)
        fprintf('[경고] 정류기 다이오드 손실 계산 윈도우 내에 유효 데이터가 없습니다.\n');
    else
        T_d = t_d(end) - t_d(1);
        if T_d > 0
            % 한 소자 손실 (time-domain 평균)
            P_d_1 = trapz(t_d, v_d .* i_d) / T_d;   % [W]
            P_d_4 = 4 * P_d_1;                      % 풀브리지 4소자 합계
        end
    end
end

% --- 결과 출력 ---
fprintf('\n==================== LOSS ANALYSIS (device-level, windowed) ====================\n');
% fprintf('Window: %.6f ~ %.6f s  (%.3f ms)\n', tStart, tEnd, (tEnd - tStart)*1e3);
% fprintf('신호: 인버터 스위치 (Vds4, Id4), 정류기 다이오드 (Vdi, Idi)\n');
fprintf('----------------------------------------------------------------------------------\n');
fprintf('%-36s | %16s | %16s\n', 'Element', 'P_1device_avg [W]', 'P_4devices_total [W]');
fprintf(repmat('-',1,90)); fprintf('\n');

if isnan(P_sw_1)
    fprintf('%-36s | %16s | %16s\n', 'Inverter switch (full-bridge)', 'N/A', 'N/A');
else
    fprintf('%-36s | %16.3f | %16.3f\n', 'Inverter switch (full-bridge)', P_sw_1, P_sw_4);
end

if isnan(P_d_1)
    fprintf('%-36s | %16s | %16s\n', 'Rectifier diode (full-bridge)', 'N/A', 'N/A');
else
    fprintf('%-36s | %16.3f | %16.3f\n', 'Rectifier diode (full-bridge)', P_d_1, P_d_4);
end

fprintf('==================================================================================\n');

%% (J) 턴오프 전류 (Turn-off Current) 분석
% 턴오프 전류: 1차측 구형파(Vi)가 순간적으로 하강할 때(스위치 턴오프)의 1차측 전류(Ip) 값
% 안정성을 위해 측정 종료 시점(tEnd) 직전 20주기 내의 샘플들을 평균 내어 계산합니다.

if evalin('base','exist("SS_Freq","var")')
    f_op = evalin('base','SS_Freq');
else
    f_op = 85e3;
end
T_op = 1 / f_op;
num_turnoff_periods = 20;

t_turnoff_start = tEnd - num_turnoff_periods * T_op;

Vi_el = local_get_element_by_name(logs, 'Vi');
Ip_el = local_get_element_by_name(logs, 'Ip');

fprintf('\n==================== TURN-OFF CURRENT (Last %d periods) ====================\n', num_turnoff_periods);

if isempty(Vi_el) || isempty(Ip_el)
    fprintf('[경고] 턴오프 전류 계산용 신호 "Vi" 또는 "Ip"를 logsout에서 찾지 못했습니다.\n');
else
    Vi_ts = Vi_el.Values;
    Ip_ts = Ip_el.Values;
    
    [t_to, v_i_to, i_p_to] = local_align_times(Vi_ts, Ip_ts, t_turnoff_start, tEnd);
    if isempty(t_to)
        fprintf('[경고] 지정된 턴오프 측정 구간 내 유효한 데이터가 없습니다.\n');
    else
        % 구형파 Vi가 양수에서 음수(혹은 0 이하)로 팍 꺼지는 순간(Falling Edge)
        % 하강 직전의 점을 턴오프 순간으로 파악합니다.
        idx_falling = find((v_i_to(1:end-1) > 0) & (v_i_to(2:end) <= 0));
        
        if isempty(idx_falling)
            fprintf('[경고] 측정 윈도우 내에서 Vi의 하강 에지(턴오프 순간)를 찾을 수 없습니다.\n');
        else
            % 해당 순간의 Ip 추출
            Ip_turnoff_all = i_p_to(idx_falling);
            Ip_turnoff_avg = mean(Ip_turnoff_all);
            
            fprintf('측정 구간: %.6f s ~ %.6f s (마지막 %d주기 평균)\n', t_turnoff_start, tEnd, num_turnoff_periods);
            fprintf('검출된 하강 에지 횟수: %d 회\n', length(idx_falling));
            fprintf('평균 턴오프 전류 (Ip at Vi drop) : %10.3f A\n', Ip_turnoff_avg);
        end
    end
end
fprintf('==================================================================================\n');

%% (K) 로컬 함수들

function el = local_get_element_by_name(logs, targetName)
    % logsout 객체에서 Name 또는 get(targetName) 으로 요소를 찾아온다.
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
    % 두 timeseries(vTs, iTs)를 동일한 시간축으로 맞춰서 [t, v, i] 반환
    % - 먼저 각 신호의 [tStart, tEnd] 구간만 잘라낸 뒤,
    % - 전압 신호의 시간축을 기준으로 전류를 interp1 로 보간

    tV = vTs.Time(:);
    v  = vTs.Data(:);
    tI = iTs.Time(:);
    i  = iTs.Data(:);

    if ~isvector(v) || ~isvector(i)
        error('local_align_times: 벡터가 아닌 신호(다차원)를 지원하지 않습니다.');
    end

    % 윈도우 내 데이터만 선택
    idxV = (tV >= tStart) & (tV <= tEnd);
    idxI = (tI >= tStart) & (tI <= tEnd);

    tVw = tV(idxV);
    vw  = v(idxV);
    tIw = tI(idxI);
    iw  = i(idxI);

    if numel(tVw) < 2 || numel(tIw) < 2
        tOut = [];
        vOut = [];
        iOut = [];
        return;
    end

    % 전압 시간축 기준으로 전류 보간
    % (보통 동일 솔버 스텝이므로 거의 동일하겠지만, 일반성을 위해 interp1 사용)
    iInterp = interp1(tIw, iw, tVw, 'linear', 'extrap');

    tOut = tVw;
    vOut = vw;
    iOut = iInterp;
end

