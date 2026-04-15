% SS_fft_analysis_260405.m
% =========================================================================
% [현재버전] v1.2
% [마지막 수정시각] 2026/04/08 14:05 (현재 시스템 시각 기준)
% 
% [수정 기록]
% v1.0 (2026/04/05) - 최초 작성: 4주기 추출, FFT 스펙트럼, 1/3/5 고조파 분리 및 도식화
% v1.1 (2026/04/05) - 그래프 x축 여백 제거, 스템 플롯 등으로 개선
% v1.2 (2026/04/08) - 관측 주기를 2주기로 축소
%                   - 3번, 4번 Subplot에 1st, 3rd, 5th 하모닉 위상 지점(Peak) 점선 추가
%                   - 구문 오류(Anonymous function 관련) 수정 및 하모닉 계산 범위 확장
% =========================================================================

% == 1. 사용자 설정 ==
targetSignalName = 'Is'; % 분석할 신호 이름
refSignalName = 'Vi';    % 위상 0도 기준 신호 (Vin)

f0 = 85e3;       % 기본 주파수 (Hz)
T = 1 / f0;      % 1주기 시간 (s)

tEnd = 6e-3;     % 측정 끝점
num_periods = 2; % 채취할 주기 수 (2주기로 축소)
tStart = tEnd - num_periods * T;

fprintf('\n대상 신호: "%s"\n', targetSignalName);
fprintf('위상 기준 신호: "%s" (fundamental = 0 deg)\n', refSignalName);
fprintf('분석 구간: %.6f 초 ~ %.6f 초 (총 %d주기)\n', tStart, tEnd, num_periods);

% == 2. 데이터 가져오기 ==
if exist('out', 'var') && isprop(out, 'logsout') && ~isempty(out.logsout)
    logs = out.logsout;
elseif evalin('base', 'exist("logsout", "var")')
    logs = evalin('base', 'logsout');
else
    error('logsout 데이터를 찾을 수 없습니다. 모델 시뮬레이션을 먼저 실행해주세요.');
end

% 대상/기준 신호 추출
target_el = local_get_signal_element(logs, targetSignalName);
ref_el    = local_get_signal_element(logs, refSignalName);

if isempty(target_el)
    error('설정한 신호 "%s"를 logsout에서 찾을 수 없습니다.', targetSignalName);
end
if isempty(ref_el)
    error('기준 신호 "%s"를 logsout에서 찾을 수 없습니다.', refSignalName);
end

% timeseries 변환
ts_target = target_el.Values;
ts_ref    = ref_el.Values;

t_raw = ts_target.Time(:);
x_raw = ts_target.Data(:);
t_ref_raw = ts_ref.Time(:);
x_ref_raw = ts_ref.Data(:);

% 구간 추출
idx = (t_raw >= tStart) & (t_raw <= tEnd);
t_win = t_raw(idx);
x_win = x_raw(idx);

idx_ref = (t_ref_raw >= tStart) & (t_ref_raw <= tEnd);
t_ref_win = t_ref_raw(idx_ref);
x_ref_win = x_ref_raw(idx_ref);

% 보간 (2000 samples per period)
N_samples = num_periods * 2000; 
t_uni = linspace(tStart, tEnd, N_samples)';
x_uni = interp1(t_win, x_win, t_uni, 'pchip');
x_ref_uni = interp1(t_ref_win, x_ref_win, t_uni, 'pchip');

% == 4. 고조파 분석 ==
w0 = 2*pi*f0;
WindowTime = tEnd - tStart;
x_dc = mean(x_uni);
x_ac = x_uni - x_dc;
x_ref_dc = mean(x_ref_uni);
x_ref_ac = x_ref_uni - x_ref_dc;

% 적분 계수는 cos 기준 phasor이므로, check_formulars와 맞추기 위해
% 이후 sin 기준 phasor로 변환해서 사용한다.  sin(wt+phi) = cos(wt+phi-pi/2)
calc_coef = @(n) (2/WindowTime) * trapz(t_uni, x_ac .* exp(-1j * n * w0 * t_uni));
calc_ref_coef = @(n) (2/WindowTime) * trapz(t_uni, x_ref_ac .* exp(-1j * n * w0 * t_uni));

n_list = 1:101;
C_list_cos = zeros(size(n_list));
C_list_sin = zeros(size(n_list));
C_list_rel = zeros(size(n_list));
x_individual_harmonics = zeros(length(t_uni), length(n_list));

ref_fund_cos = calc_ref_coef(1);
ref_fund_sin = 1j * ref_fund_cos;
ref_fund_ang = angle(ref_fund_sin);

fprintf('\n==== [%s] 고조파 분석 결과 (Peak 기준, Vi fundamental = 0 deg 기준) ====\n', targetSignalName);
for i = 1:length(n_list)
    n = n_list(i);
    C_list_cos(i) = calc_coef(n);
    C_list_sin(i) = 1j * C_list_cos(i);
    % Vi fundamental의 위상을 시간원점으로 사용한다.
    % 즉, 각 n차 위상에서 n * angle(Vi_1) 만큼 빼서 "Vin = 0 deg" 축으로 정렬한다.
    C_list_rel(i) = C_list_sin(i) * exp(-1j * n * ref_fund_ang);
    % 실제 파형 복원은 원래 시간축을 유지해야 하므로 절대(sin 기준) 위상을 사용한다.
    x_individual_harmonics(:, i) = abs(C_list_sin(i)) * sin(n * w0 * t_uni + angle(C_list_sin(i)));
    if n <= 9
        fprintf('%2d차 고조파: 최대진폭 %8.4f, 위상 %7.2f도\n', n, abs(C_list_rel(i)), angle(C_list_rel(i))*180/pi);
    end
end

% 기본파와 고조파합 분리
x_fund = x_individual_harmonics(:, 1);
x_harmonics_sum = sum(x_individual_harmonics(:, 2:end), 2); 

% == 5. 결과 그래프 ==
f = figure('Name', sprintf('FFT Analysis: %s', targetSignalName), 'Position', [100, 100, 1200, 800], 'Color', 'w');

% 그래프에 표시하는 위상선은 실제 복원된 파형과 같은 절대(sin 기준) 위상을 사용한다.
t_phase_calc = @(n) -angle(C_list_sin(n)) / (n * w0);
T_n = @(n) (2*pi) / (n * w0);

% (1) 원신호
subplot(2,2,1);
plot(t_uni*1000, x_uni, 'k', 'LineWidth', 1.5);
title(sprintf('[%s] 원래 신호 (%d주기)', targetSignalName, num_periods));
xlabel('Time (ms)'); ylabel('Amplitude');
xlim([tStart, tEnd]*1000); grid on;

% (2) 스펙트럼
subplot(2,2,2);
stem(n_list(1:21), abs(C_list_rel(1:21)), 'b', 'Filled');
title('주파수 스펙트럼 (1~21차, Vi = 0 deg 기준)');
xlabel('Harmonic Order'); ylabel('Magnitude (Peak)'); grid on;

% (3) 개별 고조파 + 위상선
subplot(2,2,3); hold on;
colors = lines(11);
for i = 1:11
    plot(t_uni*1000, x_individual_harmonics(:, i), 'Color', colors(i,:));
end
% 위상선 추가
c_map = {'b', 'g', 'm'};
h_list = [1, 3, 5];
for idx = 1:3
    n = h_list(idx);
    tp_base = t_phase_calc(n);
    % 구간 내의 모든 주기 표시
    tp_vec = tp_base + (ceil((tStart-tp_base)/T_n(n)) + (0:num_periods)) * T_n(n);
    for tp = tp_vec
        if tp >= tStart && tp <= tEnd
            xline(tp*1000, ':', 'Color', c_map{idx}, 'LineWidth', 1.2, 'HandleVisibility', 'off');
        end
    end
end
title('개별 하모닉 (1~11차) 및 위상지점');
xlabel('Time (ms)'); ylabel('Amplitude');
xlim([tStart, tEnd]*1000); grid on;

% (4) 비교 + 위상선
subplot(2,2,4);
plot(t_uni*1000, x_uni, 'k', 'LineWidth', 2); hold on;
plot(t_uni*1000, x_fund, 'b', 'LineWidth', 1.5);
plot(t_uni*1000, x_harmonics_sum, 'r', 'LineWidth', 1.5);
% 위상선 및 레이블 추가
for idx = 1:3
    n = h_list(idx);
    tp_base = t_phase_calc(n);
    tp_vec = tp_base + (ceil((tStart-tp_base)/T_n(n)) + (0:num_periods)) * T_n(n);
    for tp = tp_vec
        if tp >= tStart && tp <= tEnd
            xline(tp*1000, ':', 'Color', c_map{idx}, 'LineWidth', 1.5, 'HandleVisibility', 'off');
            text(tp*1000, max(x_uni)*1.1, sprintf('%dth', n), 'Color', c_map{idx}, 'HorizontalAlignment', 'center');
        end
    end
end
title('원래신호 vs 기본파 vs 고조파합 비교');
xlabel('Time (ms)'); ylabel('Amplitude');
xlim([tStart, tEnd]*1000); grid on;
legend('원래신호', '기본파(1st)', '고조파합', 'Location', 'best');

function signal_el = local_get_signal_element(logs, signalName)
signal_el = [];
try
    signal_el = logs.get(signalName);
catch
end
if isempty(signal_el)
    for i = 1:logs.numElements
        e = logs.getElement(i);
        if strcmp(e.Name, signalName)
            signal_el = e;
            return;
        end
    end
end
end
