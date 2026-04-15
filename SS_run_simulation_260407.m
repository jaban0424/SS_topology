%% SS_run_simulation_260407.m
% =========================================================================
% [버전 기록]
% v1.0 : (2026-04-07) SS_topology_design.m 에 통합되어 있던 시뮬레이션 실행 및 후처리 분석 
%        (Postprocess) 기능을 독립된 스크립트로 분리함.
% =========================================================================
% 설명: SS_topology_design.m의 파라미터를 기반으로 시뮬레이션을 수행하고 AC/DC 분석을 진행합니다.

clear; clc; close all;

%% 1. 파라미터 환경 구성
fprintf('파라미터 초기화 중 (SS_topology_design)...\n');
SS_topology_design; % Base Workspace에 파라미터를 로드

%% 2. User settings & Parameter Mapping
modelName = 'SS_design';

% 기준 위상 신호 (인버터 출력 구형파)
refName = 'Vi';

% 분석 대상 신호들
sigNames = {'V0','I0','Vi','Ii', ...
            'Vp','Ip','Vs','Is', ...
            'Vac','Iac','Vo','Io', ...
            'Vds4','Id4','Vdi','Idi'};
dcNames  = {'V0','I0','Vo','Io'};  % DC signals

f0 = SS_Freq;

tStart = SS_SoftStarting_TimeToStartMeasure;
tEnd   = SS_SoftStarting_TimeToStopMeasure;

% ideal map: AC phasor들 + DC 값들
idealMap = struct( ...
    'Vp',  SS_Ideal_Vp, ...
    'Ip',  SS_Ideal_Ip, ...
    'Vs',  SS_Ideal_Vs, ...
    'Is',  SS_Ideal_Is, ...
    'Vac', SS_Ideal_Vac, ...
    'Vo',  SS_Ideal_Vo_dc, ...   % scalar (DC)
    'Io',  SS_Ideal_Io_dc );     % scalar (DC)

%% 3. 시뮬레이션 실행
fprintf('시뮬레이션 %s 모델 실행 중 (Simulink)...\n', modelName);
load_system(modelName);
set_param(modelName,'StopTime',num2str(SS_TotalSimulTime));
out = sim(modelName);
fprintf('시뮬레이션 완료.\n');

%% 4. 로그아웃 (logsout) 추출 및 데이터 구조 최적화 (Trimming)
logs = [];
if exist('out','var') && isprop(out,'logsout') && ~isempty(out.logsout)
    logs = out.logsout;
elseif evalin('base','exist("logsout","var")')
    logs = evalin('base','logsout');
end
if isempty(logs)
    error('logsout을 찾지 못함. Signal logging 설정 확인.');
end

% 원본 데이터가 과도하게 크므로, tStart ~ tEnd 구간만 축소 유지
fprintf('메모리 절약을 위해 logsout 축소 중 (%.4f ~ %.4f)...\n', tStart, tEnd);
logs = local_trim_logs_dataset(logs, tStart, tEnd);

% 축소된 Dataset만 분석에 사용하고, 원본 logsout 및 out 객체는 즉시 메모리에서 제거
clear out logsout;
evalin('base', 'if exist("logsout","var"), clear logsout; end');
evalin('base', 'if exist("out","var"), clear out; end');

%% 5. 기준 페이저 도출 (Vi fundamental)
refTs = local_get_timeseries_by_name(logs, refName);
if isempty(refTs)
    error('Reference signal "%s" NOT FOUND in logsout. Vi 로깅 이름 확인.', refName);
end
refPh = local_calc_f0_phasor_only(refTs, f0, tStart, tEnd);  % peak phasor
refAng = angle(refPh);                                      % rad

%% 6. 결과 출력 테이블 구성
fprintf('\n==================== LOG STATS (Fixed Window, Vi-referenced) ====================\n');
fprintf('Phase reference: %s (fundamental)\n', refName);
fprintf('Load Resistance: %.2f\n', SS_R);
if exist('SS_delta', 'var')
    fprintf('Detuning (SS_delta): %g\n', SS_delta);
end
fprintf('----------------------------------------------------------------------------------\n');

fprintf('%-10s | %10s | %10s | %10s | %24s | %24s\n', ...
    'Signal','RMS','Peak','PkPk','MeasPhasor(f0,rms∠deg)','IdealPhasor');
fprintf(repmat('-',1,112)); fprintf('\n');

for k = 1:numel(sigNames)
    name = sigNames{k};

    el = local_get_element_by_name(logs, name);
    if isempty(el)
        fprintf('%-10s | %s\n', name, 'NOT FOUND in logsout');
        continue;
    end
    ts = local_extract_timeseries(el);
    if isempty(ts)
        fprintf('%-10s | %s\n', name, 'FOUND but not timeseries');
        continue;
    end

    isDC = any(strcmp(name, dcNames));

    if isDC
        % ---- DC signal handling ----
        [rms_dc, mean_dc] = local_calc_dc_stats(ts, tStart, tEnd);

        % DC는 Peak/PkPk/Phasor 의미 없으니 '-'
        peakStr = '-';
        pk2pkStr = '-';
        measStr = '-';

        % 이상값은 scalar로 표기 + (DC)
        if isfield(idealMap, name)
            ideal_dc = idealMap.(name); % scalar
            idealStr = sprintf('%9.3f (DC)', ideal_dc);
        else
            idealStr = 'N/A';
        end

        % RMS 칸에는 크기만 (DC니까 rms≈mean)
        fprintf('%-10s | %10.3f | %10s | %10s | %24s | %24s\n', ...
            name, rms_dc, peakStr, pk2pkStr, measStr, idealStr);

    else
        % ---- AC signal handling ----
        S = local_calc_stats_window(ts, f0, tStart, tEnd);

        % Vi 기준 상대 위상
        ph_rel = S.phasor_peak_abs * exp(-1j*refAng);
        mag_rms_rel = abs(ph_rel)/sqrt(2);
        ang_deg_rel = angle(ph_rel)*180/pi;

        measStr = sprintf('%9.3f ∠ %7.2f', mag_rms_rel, ang_deg_rel);

        % ideal phasor 표시 (rms∠deg)
        if isfield(idealMap, name)
            ideal = idealMap.(name);
            idealStr = sprintf('%9.3f ∠ %7.2f', abs(ideal), angle(ideal)*180/pi);
        else
            idealStr = 'N/A';
        end

        fprintf('%-10s | %10.3f | %10.3f | %10.3f | %24s | %24s\n', ...
            name, S.rms, S.peak, S.pk2pk, measStr, idealStr);
    end
end

fprintf('==================================================================================\n');

%% ===================== Local Functions =====================

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
            el = e; return;
        end
    end
end

function ts = local_get_timeseries_by_name(logs, targetName)
    ts = [];
    el = local_get_element_by_name(logs, targetName);
    ts = local_extract_timeseries(el);
end

function ts = local_extract_timeseries(x)
    ts = [];
    if isempty(x), return; end
    if isa(x, 'timeseries')
        ts = x; return;
    end
    if isobject(x) && isprop(x, 'Values')
        v = x.Values;
        if isa(v, 'timeseries')
            ts = v; return;
        end
    end
    if isa(x, 'Simulink.SimulationData.Dataset')
        ts = []; return;
    end
end

function S = local_calc_stats_window(ts, f0, tStart, tEnd)
    t = ts.Time(:);
    x = ts.Data(:);
    if ~isvector(x), error('Data가 벡터/행렬 신호임.'); end

    idx = (t >= tStart) & (t <= tEnd);
    t = t(idx); x = x(idx);
    if numel(t) < 10, error('윈도우 데이터가 너무 적음.'); end

    S.rms   = sqrt(mean(x.^2));
    S.peak  = max(abs(x));
    S.pk2pk = max(x) - min(x);

    w0  = 2*pi*f0;
    xac = x - mean(x);            % DC 제거
    ref = exp(-1j*w0*t);

    T = t(end) - t(1);
    X = (2/T) * trapz(t, xac .* ref);   % peak phasor (complex)

    S.phasor_peak_abs = X;
end

function X = local_calc_f0_phasor_only(ts, f0, tStart, tEnd)
    t = ts.Time(:);
    x = ts.Data(:);
    if ~isvector(x), error('Reference Data가 벡터/행렬 신호임.'); end

    idx = (t >= tStart) & (t <= tEnd);
    t = t(idx); x = x(idx);
    if numel(t) < 10, error('Reference 윈도우 데이터가 너무 적음.'); end

    w0  = 2*pi*f0;
    xac = x - mean(x);
    ref = exp(-1j*w0*t);

    T = t(end) - t(1);
    X = (2/T) * trapz(t, xac .* ref);
end

function [rms_dc, mean_dc] = local_calc_dc_stats(ts, tStart, tEnd)
    t = ts.Time(:);
    x = ts.Data(:);
    if ~isvector(x), error('DC Data가 벡터/행렬 신호임.'); end

    idx = (t >= tStart) & (t <= tEnd);
    x = x(idx);
    if numel(x) < 10, error('DC 윈도우 데이터가 너무 적음.'); end

    mean_dc = mean(x);
    rms_dc  = sqrt(mean(x.^2));
end

function newLogs = local_trim_logs_dataset(logs, tStart, tEnd)
% logsout 전체 요소들을 순회하며 tStart~tEnd 구간만 남긴 새 Dataset을 반환
    newLogs = Simulink.SimulationData.Dataset;
    
    for i = 1:logs.numElements
        el = logs.getElement(i);
        
        try
            elName = logs.Names{i};
        catch
            if isprop(el, 'Name')
                elName = el.Name;
            else
                elName = sprintf('Element%d', i);
            end
        end
        
        if isa(el, 'Simulink.SimulationData.Signal')
            if isa(el.Values, 'timeseries')
                newEl = Simulink.SimulationData.Signal;
                newEl.Name = el.Name;
                if isprop(el,'BlockPath'), newEl.BlockPath = el.BlockPath; end
                if isprop(el,'PortType'),  newEl.PortType = el.PortType; end
                if isprop(el,'PortIndex'), newEl.PortIndex = el.PortIndex; end
                
                newEl.Values = local_trim_timeseries(el.Values, tStart, tEnd);
                newLogs = newLogs.addElement(newEl, elName);
            else
                newLogs = newLogs.addElement(el, elName);
            end
        elseif isa(el, 'timeseries')
            ts_trimmed = local_trim_timeseries(el, tStart, tEnd);
            newLogs = newLogs.addElement(ts_trimmed, elName);
        else
            % timeseries가 아닌 element는 (struct, vector 등) 스킵하거나 원본 유지
            newLogs = newLogs.addElement(el, elName);
        end
    end
    
    if newLogs.numElements == 0
        error('Trimming 후 Dataset에 유효한 데이터가 없습니다. (tStart/tEnd 구간 확인)');
    end
end

function newTs = local_trim_timeseries(ts, tStart, tEnd)
% timeseries 내부의 Time/Data를 tStart~tEnd 범위로 잘라서 반환
    t = ts.Time;
    d = ts.Data;
    
    idx = (t >= tStart) & (t <= tEnd);
    if sum(idx) == 0
        error('Trimming 결과 데이터가 없습니다 (Name: %s).', ts.Name);
    end
    
    t_trimmed = t(idx);
    
    % 다차원 Data까지 안전하게 유지하도록 x(idx, :, :, ...) 형태로 처리
    dims = ndims(d);
    colons = repmat({':'}, 1, dims - 1);
    
    if ismatrix(d) && size(d, 2) == 1
        % 간단한 컬럼 벡터 신호
        d_trimmed = d(idx, :);
    elseif isprop(ts, 'IsTimeFirst') && ts.IsTimeFirst
        d_trimmed = d(idx, colons{:});
    else
        d_trimmed = d(colons{:}, idx);
    end
    
    newTs = timeseries(d_trimmed, t_trimmed);
    newTs.Name = ts.Name;
    
    % 기존 메타데이터 유지 가능하면 보존
    try
        if isprop(ts, 'DataInfo')
            newTs.DataInfo = ts.DataInfo;
        end
        if isprop(ts, 'TimeInfo') && isprop(ts.TimeInfo, 'Units')
            newTs.TimeInfo.Units = ts.TimeInfo.Units;
        end
    catch
    end
end
