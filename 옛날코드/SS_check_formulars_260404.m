%% harmonic_series_check.m
% 네가 유도한 Ip,m / Is,m 식의 홀수차 급수 검증용
% m = 3,5,7,...,35 누적합과, 큰 차수까지 더한 reference(준-무한급수)를 비교
%
% 주의:
% 1) UAB, uab 는 "구형파 레벨값" 기준으로 넣는다고 가정
% 2) 각 m차의 rms phasor 계수 2*sqrt(2)/(m*pi)를 반영한 식 사용
% 3) 복소수 위상까지 반영해서 누적합 후 magnitude를 그림
%
% 작성 기준 식:
% Ip,m = (2*sqrt(2)/pi) * UAB / [wL * (m^2 - 1 - m^4*k^2*(1-d)/(m^2*(1-d)-1))] ∠(-90deg)
%      + (2*sqrt(2)/pi) * uab / (m^2*w*k*L) ∠(phi + 90deg)
%
% Is,m = (2*sqrt(2)/pi) * UAB / [wL * ( (m^2-1)*(m^2*(1-d)-1)/(m^2*k*(1-d)) - m^2*k )] ∠(-90deg)

% clear; clc; close all;

%% ===================== 사용자 파라미터 =====================
UAB = 650;          % [V] 1차측 구형파 레벨값
uab = 850;           % [V] 2차측 구형파 레벨값(정류기 앞단 square-wave 레벨)
f0  = 85e3;         % [Hz] 기준 주파수
omega = 2*pi*f0;    % [rad/s]
L = 212e-6;         % [H]
Lp = 212e-6;        % [H]
Ls = 155e-6;        % [H]
k = 0.2;            % 결합계수
delta = 0.01;       % delta = DeltaC / C
phi_deg = 94;      % [deg]
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

% 각 m차 Ip,m, Is,m 복소 phasor
Ip_term = @(m) ...
    (A .* UAB ./ (omega*L .* ( m.^2 - 1 - (m.^4 .* k.^2 .* (1-delta)) ./ (m.^2 .* (1-delta) - 1) ))) .* exp(-1j*pi/2) ...
    + ...
    (A .* uab ./ (omega*L .* ( ((m.^2 - 1) ./ (m .* k)) .* (m - 1 ./ (m .* (1-delta))) - m.^2 .* k ))) .* exp(1j*(phi + pi/2));

Is_term = @(m) ...
    sqrt(Lp/Ls) .* ( ...
    (A .* UAB ./ (omega*L .* ( (m.^2 - 1) .* (m.^2 .* (1-delta) - 1) ./ (m.^2 .* k .* (1-delta)) - m.^2 .* k ))) .* exp(-1j*pi/2) ...
    + ...
    (A .* uab .* (m.^2 - 1) ./ (omega*L .* ( (m.^2 - 1).*(m.^2 - 1./(1-delta)) - m.^4 .* k^2 ))) .* exp(1j*(m*phi + pi/2)) ...
    );

%% ===================== m=3:2:35 항 계산 =====================
Ip_vals = Ip_term(m_plot);
Is_vals = Is_term(m_plot);

% 누적합(복소수 누적합 후 magnitude)
Ip_cum = cumsum(Ip_vals);
Is_cum = cumsum(Is_vals);

Ip_cum_mag = abs(Ip_cum);
Is_cum_mag = abs(Is_cum);

%% ===================== reference(준-무한급수) 계산 =====================
Ip_ref_terms = Ip_term(m_ref);
Is_ref_terms = Is_term(m_ref);

Ip_ref_sum = sum(Ip_ref_terms);
Is_ref_sum = sum(Is_ref_terms);

Ip_ref_mag = abs(Ip_ref_sum);
Is_ref_mag = abs(Is_ref_sum);

%% ===================== 각 항 자체도 보고 싶으면 =====================
Ip_term_mag = abs(Ip_vals);
Is_term_mag = abs(Is_vals);

%% ===================== 결과 출력 =====================
fprintf('=== Parameters ===\n');
fprintf('UAB = %.6g V, uab = %.6g V, f0 = %.6g Hz, L = %.6g H\n', UAB, uab, f0, L);
fprintf('k = %.6g, delta = %.6g, phi = %.6g deg\n\n', k, delta, phi_deg);

fprintf('=== Reference sums (odd m = 3:2:%d) ===\n', m_ref_max);
fprintf('Ip_ref_sum      = %.12g %+.12gj\n', real(Ip_ref_sum), imag(Ip_ref_sum));
fprintf('|Ip_ref_sum|    = %.12g\n', Ip_ref_mag);
fprintf('Is_ref_sum      = %.12g %+.12gj\n', real(Is_ref_sum), imag(Is_ref_sum));
fprintf('|Is_ref_sum|    = %.12g\n\n', Is_ref_mag);

fprintf('=== Partial sums up to m=35 ===\n');
fprintf('Ip_partial_sum  = %.12g %+.12gj\n', real(Ip_cum(end)), imag(Ip_cum(end)));
fprintf('|Ip_partial_sum|= %.12g\n', Ip_cum_mag(end));
fprintf('Is_partial_sum  = %.12g %+.12gj\n', real(Is_cum(end)), imag(Is_cum(end)));
fprintf('|Is_partial_sum|= %.12g\n', Is_cum_mag(end));

%% ===================== 그래프 =====================
figure('Color','w','Position',[100 100 1200 900]);

% --------- (1) Ip 누적합 vs reference ----------
subplot(2,2,1);
plot(m_plot, Ip_cum_mag, 'o-', 'LineWidth', 1.8, 'MarkerSize', 6); hold on;
yline(Ip_ref_mag, '--', 'LineWidth', 1.8);
grid on;
xlabel('Harmonic order m');
ylabel('| cumulative \Sigma I_{p,m} |');
title('I_p odd-harmonic cumulative sum');
legend('|partial sum|', '|reference sum|', 'Location', 'best');

% --------- (2) Is 누적합 vs reference ----------
subplot(2,2,2);
plot(m_plot, Is_cum_mag, 'o-', 'LineWidth', 1.8, 'MarkerSize', 6); hold on;
yline(Is_ref_mag, '--', 'LineWidth', 1.8);
grid on;
xlabel('Harmonic order m');
ylabel('| cumulative \Sigma I_{s,m} |');
title('I_s odd-harmonic cumulative sum');
legend('|partial sum|', '|reference sum|', 'Location', 'best');

% --------- (3) Ip 각 항 magnitude ----------
subplot(2,2,3);
stem(m_plot, Ip_term_mag, 'filled', 'LineWidth', 1.4); grid on;
xlabel('Harmonic order m');
ylabel('|I_{p,m}|');
title('Magnitude of each odd harmonic term for I_p');

% --------- (4) Is 각 항 magnitude ----------
subplot(2,2,4);
stem(m_plot, Is_term_mag, 'filled', 'LineWidth', 1.4); grid on;
xlabel('Harmonic order m');
ylabel('|I_{s,m}|');
title('Magnitude of each odd harmonic term for I_s');

sgtitle('Odd-harmonic series check for I_p and I_s');

%% ===================== 선택: 실수부/허수부 누적도 확인 =====================
figure('Color','w','Position',[150 150 1200 500]);

subplot(1,2,1);
plot(m_plot, real(Ip_cum), 'o-', 'LineWidth', 1.6); hold on;
plot(m_plot, imag(Ip_cum), 's-', 'LineWidth', 1.6);
yline(real(Ip_ref_sum), '--', 'LineWidth', 1.5);
yline(imag(Ip_ref_sum), '--', 'LineWidth', 1.5);
grid on;
xlabel('Harmonic order m');
ylabel('Partial sum components');
title('I_p cumulative sum: real / imag');
legend('Real(partial)', 'Imag(partial)', 'Real(reference)', 'Imag(reference)', 'Location', 'best');

subplot(1,2,2);
plot(m_plot, real(Is_cum), 'o-', 'LineWidth', 1.6); hold on;
plot(m_plot, imag(Is_cum), 's-', 'LineWidth', 1.6);
yline(real(Is_ref_sum), '--', 'LineWidth', 1.5);
yline(imag(Is_ref_sum), '--', 'LineWidth', 1.5);
grid on;
xlabel('Harmonic order m');
ylabel('Partial sum components');
title('I_s cumulative sum: real / imag');
legend('Real(partial)', 'Imag(partial)', 'Real(reference)', 'Imag(reference)', 'Location', 'best');