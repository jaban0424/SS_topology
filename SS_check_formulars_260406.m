%% SS_check_formulars_260406.m
% 새로 유도된 Ip,m / Is,m 식의 홀수차 급수 검증용
% m = 1,3,5,7,...,101 등 누적합과, 큰 차수까지 더한 reference(준-무한급수)를 비교
%
% =========================================================================
% [버전 기록]
% v1.7 : (2026-04-15 01:30:00) 
%        - 공식 재정립
% v1.6 : (2026-04-08 13:35:00)
%        - 1차측/2차측 시간 파형 플롯에 3고조파 및 5고조파의 위상 지점(Phase Time) 수직 점선 추가
% v1.5 : (2026-04-08 12:14:00)
%        - 1차측/2차측 시간 파형 플롯에 기본파(m=1)의 위상 편이 지점을 나타내는 수직 점선(Phase Time) 추가
% v1.4 : (2026-04-08 12:10:00)
%        - 재구성 파형(Time Domain)의 플롯 X축을 위상(Phase)에서 물리적 시간(Time, us 단위)으로 변경
% v1.3 : (2026-04-08 10:02:00)
%        - 주파수(고조파 차수 m)에 따른 각 항의 위상(Phase Spectrum)을 나타내는 막대그래프(stem) 플롯 추가
% v1.2 : (2026-04-08 09:40:00)
%        - 시간 파형 플롯에 3고조파, 5고조파 개별 파형 스플릿(Plot) 추가 (총 5개 파형)
% v1.1 : (2026-04-08 09:11:00)
%        - 수식을 바탕으로 한 시간 파형(Time Domain) 재구성 플롯 추가 (Ip 및 Is 모두)
% =========================================================================


% clear; clc; 
% close all;

%% ===================== 사용자 파라미터 (SS_topology_design.m 에서 불러오기) =====================
runSim_backup = exist('runSim','var');
if runSim_backup
    runSim_orig = runSim;
end
runSim = 0; % 시뮬레이션이 돌지 않도록 설정
SS_topology_design;
if runSim_backup
    runSim = runSim_orig;
else
    clear runSim;
end
clear runSim_backup runSim_orig;

UAB = SS_Vin;          % [V] 1차측 구형파 레벨값
Lp  = SS_Lp;           % [H] 1차측 인덕턴스
Ls  = SS_Ls;           % [H] 2차측 인덕턴스
L   = SS_Lp;           % [H] 기존 수식 호환용 인덕턴스 L
k   = SS_k;            % 결합계수
delta = SS_delta;      % delta = DeltaC / C
f0  = SS_Freq;         % [Hz] 기준 주파수
omega = SS_AglFreq;    % [rad/s]

% 아래는 SS_topology_design.m 에 명시되지 않아 기존 값을 유지
uab = 849;             % [V] 2차측 구형파 레벨값(정류기 앞단 square-wave 레벨)
phi_deg = 94.1649;          % [deg]
phi = deg2rad(phi_deg);

% 그래프용 홀수차
m_plot = 1:2:101;

% reference(준-무한급수) 계산용 최대 홀수차
% 너무 작으면 "이론값"처럼 안 보이고, 너무 크면 계산만 오래 걸림
m_ref_max = 20001;
m_ref = 1:2:m_ref_max;

%% ===================== 유효성 체크 =====================
if abs(k) < 1e-12
    error('k가 0이면 식에 1/k가 들어가므로 계산 불가.');
end

if abs(1-delta) < 1e-12
    error('delta = 1 이면 (1-delta)=0 이므로 계산 불가.');
end

%% ===================== 함수 정의 =====================
% 공통 상수
A = 2*sqrt(2)/pi;

% 각 m차 Ip,m, Is,m 복소 phasor (새 모델 수식 적용 - final)
Ip_term = @(m) ...
    (A .* UAB ./ (omega*L .* ( m.^2 - 1 - (m.^4 .* k^2 .* (1-delta)) ./ (m.^2 .* (1-delta) - 1) ))) .* exp(-1j*pi/2) ...
    + ...
    (A .* uab ./ (omega*L .* ( ((m.^2 - 1) ./ (m .* k)) .* (m - 1 ./ (m .* (1-delta))) - m.^2 .* k ))) .* exp(1j*(m*phi + pi/2));

Is_term = @(m) ...
    sqrt(Lp/Ls) .* ( ...
    (A .* UAB ./ (omega*L .* ( (m.^2 - 1) .* (m.^2 .* (1-delta) - 1) ./ (m.^2 .* k .* (1-delta)) - m.^2 .* k ))) .* exp(-1j*(pi/2)) ...
    + ...
    (A .* uab .* (m.^2 - 1) ./ (omega*L .* ( (m.^2 - 1).*(m.^2 - 1./(1-delta)) - m.^4 .* k^2 ))) .* exp(1j*(m*phi + pi/2)) ...
    );

%% ===================== m=1:2:101 항 계산 =====================
Ip_vals = Ip_term(m_plot);
Is_vals = Is_term(m_plot);

Is_1st_term = @(m)sqrt(Lp/Ls) .* (A .* UAB ./ (omega*L .* ( (m.^2 - 1) .* (m.^2 .* (1-delta) - 1) ./ (m.^2 .* k .* (1-delta)) - m.^2 .* k ))) .* exp(-1j*pi/2);
Is_2nd_term = @(m)sqrt(Lp/Ls) .* ((A .* uab .* (m.^2 - 1) ./ (omega*L .* ( (m.^2 - 1).*(m.^2 - 1./(1-delta)) - m.^4 .* k^2 ))) .* exp(1j*(m*phi + pi/2)));

Is_1st_vals = Is_1st_term(m_plot);
Is_2nd_vals = Is_2nd_term(m_plot);

fprintf('\n================ [Is 각 m차 고조파 페이저 상세 분석] ================\n');
fprintf('  m |       1st term (mag / ang)       |       2nd term (mag / ang)       |        Total Is (real+j*imag / ang) \n');
fprintf('------------------------------------------------------------------------------------------------------------------\n');
% 배열 기반의 벡터를 fprintf로 열 단위로 올바르게 출력하기 위해 데이터를 행렬로 묶습니다.
print_data = [m_plot; 
              abs(Is_1st_vals); angle(Is_1st_vals)*180/pi; 
              abs(Is_2nd_vals); angle(Is_2nd_vals)*180/pi; 
              real(Is_vals); imag(Is_vals); angle(Is_vals)*180/pi];
fprintf('%3d | %10.4g / %8.4g deg | %10.4g / %8.4g deg | %12.6g + j(%12.6g) / %8.4g deg\n', print_data);
fprintf('==================================================================================\n\n');


%% ===================== 누적 RMS (True RMS) 계산 =====================
% **중요 수학적 수정**: 주파수가 다른 고조파 페이저들을 단순히 복소수로 더하는 것(sum)이나, 
% 단순히 크기만 더하는 것(sum(|I|))은 전체 실효값(RMS)을 물리적으로 올바르게 나타내지 못합니다. (Parseval's Theorem 위배)
% 전체 파형의 실효 전류(True RMS)는 각 고조파 성분 RMS의 제곱합의 제곱근으로 계산되어야 합니다.

Ip_cum_rms = sqrt(cumsum(abs(Ip_vals).^2));
Is_cum_rms = sqrt(cumsum(abs(Is_vals).^2));

%% ===================== reference(준-무한급수) True RMS 계산 =====================
Ip_ref_terms = Ip_term(m_ref);
Is_ref_terms = Is_term(m_ref);

Ip_ref_rms = sqrt(sum(abs(Ip_ref_terms).^2));
Is_ref_rms = sqrt(sum(abs(Is_ref_terms).^2));

%% ===================== 각 항 자체도 보고 싶으면 =====================
Ip_term_mag = abs(Ip_vals);
Is_term_mag = abs(Is_vals);

%% ===================== 결과 출력 =====================
fprintf('=== Parameters ===\n');
fprintf('UAB = %.6g V, uab = %.6g V, f0 = %.6g Hz, L = %.6g H\n', UAB, uab, f0, L);
fprintf('k = %.6g, delta = %.6g, phi = %.6g deg\n\n', k, delta, phi_deg);

fprintf('=== Reference True RMS (odd m = 1:2:%d) ===\n', m_ref_max);
fprintf('Ip_ref_rms      = %.12g A\n', Ip_ref_rms);
fprintf('Is_ref_rms      = %.12g A\n\n', Is_ref_rms);

fprintf('=== Partial sums (RMS) up to m=%d ===\n', m_plot(end));
fprintf('Ip_partial_rms  = %.12g A\n', Ip_cum_rms(end));
fprintf('Is_partial_rms  = %.12g A\n', Is_cum_rms(end));

%% ===================== 그래프 =====================
%% ===================== 그래프 =====================
figure('Name','Odd Harmonic RMS Convergence', 'Color','w','Position',[100 100 1200 900]);

% --------- (1) Ip True RMS 누적합 vs reference ----------
subplot(2,2,1);
plot(m_plot, Ip_cum_rms, 'o-', 'LineWidth', 1.8, 'MarkerSize', 6); hold on;
yline(Ip_ref_rms, '--', 'LineWidth', 1.8);
grid on;
xlabel('Harmonic order m');
ylabel('Cumulative True RMS I_{p} [A]');
title('I_p Convergence (True RMS)');
legend('Partial RMS (\surd\Sigma|I_m|^2)', 'Reference RMS', 'Location', 'best');

% --------- (2) Is True RMS 누적합 vs reference ----------
subplot(2,2,2);
plot(m_plot, Is_cum_rms, 'o-', 'LineWidth', 1.8, 'MarkerSize', 6); hold on;
yline(Is_ref_rms, '--', 'LineWidth', 1.8);
grid on;
xlabel('Harmonic order m');
ylabel('Cumulative True RMS I_{s} [A]');
title('I_s Convergence (True RMS)');
legend('Partial RMS (\surd\Sigma|I_m|^2)', 'Reference RMS', 'Location', 'best');

% --------- (3) Ip 각 항 magnitude ----------
subplot(2,2,3);
stem(m_plot, Ip_term_mag, 'filled', 'LineWidth', 1.4); grid on;
xlabel('Harmonic order m');
ylabel('|I_{p,m}| [A]');
title('Magnitude of each odd harmonic term for I_p');

% --------- (4) Is 각 항 magnitude ----------
subplot(2,2,4);
stem(m_plot, Is_term_mag, 'filled', 'LineWidth', 1.4); grid on;
xlabel('Harmonic order m');
ylabel('|I_{s,m}| [A]');
title('Magnitude of each odd harmonic term for I_s');

sgtitle('Harmonic RMS Convergence & Magnitude Check for I_p and I_s');

% (참고) 이전 버전에서 실수부(real)와 허수부(imag)를 개별적으로 누적합하여 
% 살펴보는 그래프가 존재했으나, 서로 다른 주파수 성분의 페이저를 복소수 형태로 
% 그대로 더하는 것은 물리적 실효값이나 순시적인 최대값을 대변하지 못하므로 삭제되었습니다.


%% ===================== 시간 파형(Time Domain) 재구성 플롯 =====================
% 계산된 m_plot (1, 3, 5, ... 101) 범위의 페이저를 이용하여 시간 파형을 재구성합니다.
theta_vec_1st = linspace(-pi, 2*pi, 1000); % -180도 ~ 180도 (1주기)
t_vec = theta_vec_1st / omega;    % 시간축 계산
t_us = t_vec * 1e6;                  % 마이크로초(us) 단위 변환

Ip_mag = abs(Ip_vals);
Ip_ang = angle(Ip_vals);

Is_mag = abs(Is_vals);
Is_ang = angle(Is_vals);

ip_fund = zeros(size(t_vec));
ip_3rd = zeros(size(t_vec)); 
ip_5th = zeros(size(t_vec));
ip_higher = zeros(size(t_vec));
ip_total = zeros(size(t_vec));

is_fund = zeros(size(t_vec));
is_3rd = zeros(size(t_vec));
is_5th = zeros(size(t_vec));
is_higher = zeros(size(t_vec));
is_total = zeros(size(t_vec));

for m_i = 1:length(m_plot)
    m = m_plot(m_i);
    % Ip_ang(m_i)
    % 수식 기반 위상 재구성 공식 (sin 기준) 적용
    % 주의: 위상각(Ip_ang, Is_ang)은 각 고조파(m)에 대해 이미 도출된 절대적인 위상 편이값입니다.
    % 따라서 m * (theta + phi) 가 아니라, m * theta + phi 로 들어가야 수학적으로 올바릅니다. (파세발/푸리에 원리)
    
    i_pm = sqrt(2) * Ip_mag(m_i) * sin(m * omega * t_vec + Ip_ang(m_i));
    i_sm = sqrt(2) * Is_mag(m_i) * sin(m * omega * t_vec + Is_ang(m_i));

    % i_pm = sqrt(2) * Ip_mag(m_i) * cos(m * (omega * t_vec + Ip_ang(m_i)));
    % i_sm = sqrt(2) * Is_mag(m_i) * cos(m * (omega * t_vec + Is_ang(m_i)));

    % i_pm = sqrt(2) * Ip_mag(m_i) * sin(m * (omega * t_vec + Ip_ang(m_i)+pi/2));
    % i_sm = sqrt(2) * Is_mag(m_i) * sin(m * (omega * t_vec + Is_ang(m_i)+pi/2));

    %i_pm = sqrt(2) * Ip_mag(m_i) * sin(m * (omega * t_vec + Ip_ang(m_i)));
    %i_sm = sqrt(2) * Is_mag(m_i) * sin(m * omega * t_vec + Is_ang(m_i) - (m-1)*pi/2);
    
    ip_total = ip_total + i_pm;
    is_total = is_total + i_sm;
    
    if m == 1
        ip_fund = i_pm;
        is_fund = i_sm;
    else
        ip_higher = ip_higher + i_pm;
        is_higher = is_higher + i_sm;
        
        if m == 3
            ip_3rd = i_pm;
            is_3rd = i_sm;
        elseif m == 5
            ip_5th = i_pm;
            is_5th = i_sm;
        end
    end
end

% --------- 1차측(Ip) 시간 파형 플롯 ---------
% 고조파 위상에 해당하는 시간 (us) 계산
t_p_fund_us = -Ip_ang(1) / (1 * omega) * 1e6; 
t_p_3rd_us  = -Ip_ang(2) / (3 * omega) * 1e6;
t_p_5th_us  = -Ip_ang(3) / (5 * omega) * 1e6;

figure('Name','I_p Time Domain Waveform','Color','w','Position',[200 200 800 500]);
plot(t_us, ip_total, 'k', 'LineWidth', 2); hold on;
plot(t_us, ip_fund, 'b--', 'LineWidth', 1.5);
plot(t_us, ip_higher, 'r-.', 'LineWidth', 1.5);
plot(t_us, ip_3rd, 'g:', 'LineWidth', 1.5);
plot(t_us, ip_5th, 'm:', 'LineWidth', 1.5);

xline(0, 'k:', 'LineWidth', 1.2); % t=0 (0도) 기준선
yline(0, 'k-', 'LineWidth', 1.2);  % y=0 기준선
xline(t_p_fund_us, 'b:', 'LineWidth', 1.5); % 기본파 위상 지점
xline(t_p_3rd_us, 'g:', 'LineWidth', 1.2);  % 3고조파 위상 지점
xline(t_p_5th_us, 'm:', 'LineWidth', 1.2);  % 5고조파 위상 지점

% 특정 위상 기준선 (-90, 90, 180, 270도) 추가 (0도는 이미 t=0에 위치)
ref_angles_deg = [-90, 90, 180, 270];
ref_times_us = (ref_angles_deg * pi / 180) / omega * 1e6;
for j = 1:length(ref_times_us)
    xline(ref_times_us(j), 'k:', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    % 축 하단 근처에 위상을 Text로 표시
    text(ref_times_us(j), min(ip_total)*1.1, sprintf('%d\\circ ', ref_angles_deg(j)), ...
         'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'Color', [0.4 0.4 0.4]);
end
text(0, min(ip_total)*1.1, '0\circ ', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'Color', [0.4 0.4 0.4]);

grid on;
legend('Total Current I_p', 'Fundamental (m=1)', 'All Higher Harmonics', '3rd Harmonic', '5th Harmonic', ...
       't = 0', 'y = 0', '1st Phase Time', '3rd Phase Time', '5th Phase Time', 'Location', 'best');
xlabel('Time [\mus]');
ylabel('Primary Current I_p [A]');
title(sprintf('Primary Current I_p Reconstructed Waveform (\\phi = %.1f\\circ)', phi_deg));

% --------- 2차측(Is) 시간 파형 플롯 ---------
% 2차측 고조파 위상에 해당하는 시간 (us)
t_s_fund_us = -Is_ang(1) / (1 * omega) * 1e6; 
t_s_3rd_us  = -Is_ang(2) / (3 * omega) * 1e6;
t_s_5th_us  = -Is_ang(3) / (5 * omega) * 1e6;

figure('Name','I_s Time Domain Waveform','Color','w','Position',[250 250 800 500]);
plot(t_us, is_total, 'k', 'LineWidth', 2); hold on;
plot(t_us, is_fund, 'b--', 'LineWidth', 1.5);
plot(t_us, is_higher, 'r-.', 'LineWidth', 1.5);
plot(t_us, is_3rd, 'g:', 'LineWidth', 1.5);
plot(t_us, is_5th, 'm:', 'LineWidth', 1.5);

xline(0, 'k:', 'LineWidth', 1.2); % t=0 (0도) 기준선
yline(0, 'k-', 'LineWidth', 1.2);  % y=0 기준선
xline(t_s_fund_us, 'b:', 'LineWidth', 1.5); % 2차측 기본파 위상 지점
xline(t_s_3rd_us, 'g:', 'LineWidth', 1.2);  % 2차측 3고조파 위상 지점
xline(t_s_5th_us, 'm:', 'LineWidth', 1.2);  % 2차측 5고조파 위상 지점

% 특정 위상 기준선 (-90, 90, 180, 270도) 추가
for j = 1:length(ref_times_us)
    xline(ref_times_us(j), 'k:', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    text(ref_times_us(j), min(is_total)*1.1, sprintf('%d\\circ ', ref_angles_deg(j)), ...
         'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'Color', [0.4 0.4 0.4]);
end
text(0, min(is_total)*1.1, '0\circ ', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'Color', [0.4 0.4 0.4]);

grid on;
legend('Total Current I_s', 'Fundamental (m=1)', 'All Higher Harmonics', '3rd Harmonic', '5th Harmonic', ...
       't = 0', 'y = 0', '1st Phase Time', '3rd Phase Time', '5th Phase Time', 'Location', 'best');
xlabel('Time [\mus]');
ylabel('Secondary Current I_s [A]');
title(sprintf('Secondary Current I_s Reconstructed Waveform (\\phi = %.1f\\circ)', phi_deg));

fprintf('\n시간 도메인(Time Domain) 파형 재구성 및 출력이 완료되었습니다.\n');


%% ===================== 주파수(고조파 차수)에 따른 위상 스펙트럼 플롯 =====================
% 각 고조파의 위상 정보 (rad -> deg 변환)
Ip_ang_deg = rad2deg(Ip_ang);
Is_ang_deg = rad2deg(Is_ang);

figure('Name','Phase Spectrum (Ip and Is)', 'Color','w', 'Position',[300 300 1000 450]);

% --------- 1차측(Ip) 위상 스펙트럼 ---------
subplot(1,2,1);
stem(m_plot, Ip_ang_deg, 'filled', 'MarkerFaceColor', 'b', 'LineWidth', 1.2);
grid on;
xlabel('Harmonic order m');
ylabel('Phase \angle I_{p,m} [deg]');
title('Phase Spectrum of Primary Current (I_p)');
ylim([-180 180]);
yticks(-180:45:180);

% --------- 2차측(Is) 위상 스펙트럼 ---------
subplot(1,2,2);
stem(m_plot, Is_ang_deg, 'filled', 'MarkerFaceColor', 'r', 'LineWidth', 1.2);
grid on;
xlabel('Harmonic order m');
ylabel('Phase \angle I_{s,m} [deg]');
title('Phase Spectrum of Secondary Current (I_s)');
ylim([-180 180]);
yticks(-180:45:180);

sgtitle(sprintf('Harmonic Phase Spectra for I_p and I_s (\\phi = %.1f\\circ)', phi_deg));
fprintf('주파수(고조파 차수)에 따른 위상 스펙트럼 플롯 생성이 완료되었습니다.\n');
