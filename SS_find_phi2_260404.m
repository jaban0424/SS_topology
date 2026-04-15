%% Is_harmonic_spectrum_and_phi2.m
% 2차측 전류 Is,m 기반 분석
% 기준 위상: 입력 구형파(U_AB)가 0도
%
% 사용 식:
% Is,m = (2*sqrt(2)/pi) * UAB / { wL * [ ((m^2-1)*(m^2(1-d)-1))/(m^2*k*(1-d)) - m^2*k ] } ∠(-90deg)
%
% 연구용 정의:
%   phi1 = -90 deg
%   sin(phi2) = (sum of harmonic RMS magnitudes) / (fundamental RMS magnitude)
%   phi = phi1 + phi2
%
% 주의:
% - 여기서 harmonic sum은 m=3,5,7,...,m_max 까지의 truncation 근사임
% - sin(phi2)가 1을 넘을 수 있어 clamp 처리함
% - UAB는 구형파 레벨값 기준

clear; clc; close all;

%% ===================== 사용자 파라미터 =====================
UAB   = 650;        % [V] 입력 구형파 레벨
f0    = 85e3;       % [Hz] 기준 주파수
omega = 2*pi*f0;    % [rad/s]
L     = 212e-6;     % [H]
Lp    = 212e-6;     % [H]
Ls    = 155e-6;     % [H]
k     = 0.2;        % coupling factor
delta = 0.00001;       % delta = DeltaC / C

m_max = 35;
% 마지막 홀수차
m_vec = 1:2:m_max;  % 1,3,5,...,35
m_h   = 3:2:m_max;  % 고조파만

%% ===================== 유효성 체크 =====================
if abs(k) < 1e-12
    error('k = 0이면 식이 정의되지 않음.');
end
if abs(1-delta) < 1e-12
    error('delta = 1이면 (1-delta)=0 이므로 계산 불가.');
end

%% ===================== 공통 함수 =====================
A = 2*sqrt(2)/pi;   % square-wave level -> m차 rms phasor 계수의 공통 인자 일부

% RMS phasor Is,m 의 첫번째 항 (위상은 입력 구형파 기준 -90도)
Is_1st_term = @(m) sqrt(Lp/Ls) .* (A .* UAB ./ (omega*L .* ( (m.^2 - 1) .* (m.^2 .* (1-delta) - 1) ./ (m.^2 .* k .* (1-delta)) - m.^2 .* k ))) .* exp(-1j*pi/2);

%% ===================== 기본파 및 고조파 계산 =====================
Is_all = Is_1st_term(m_vec);
Is_1   = Is_1st_term(1);
Is_h   = Is_1st_term(m_h);

Is1_rms = abs(Is_1);
Ih_rms_sum = sum(abs(Is_h));   % 네가 정의한 "고조파 rms 크기의 무한급수"의 truncation 근사

%% ===================== phi2, phi 계산 =====================
phi1_deg = 90;

ratio_raw = Ih_rms_sum / Is1_rms;
ratio_clamped = min(max(ratio_raw, -1), 1);

phi2_rad = asin(ratio_clamped);
phi2_deg = rad2deg(phi2_rad);

phi_deg  = phi1_deg + phi2_deg;
phi_rad  = deg2rad(phi_deg);

%% ===================== 결과 출력 =====================
fprintf('=== Parameters ===\n');
fprintf('UAB   = %.6g V\n', UAB);
fprintf('f0    = %.6g Hz\n', f0);
fprintf('omega = %.6g rad/s\n', omega);
fprintf('L     = %.6g H\n', L);
fprintf('k     = %.6g\n', k);
fprintf('delta = %.6g\n', delta);
fprintf('m_max = %d\n\n', m_max);

fprintf('=== Fundamental / Harmonics ===\n');
fprintf('|Is,1| (RMS)                = %.12g A\n', Is1_rms);
fprintf('sum_{m=3,5,...,%d} |Is,m|   = %.12g A\n', m_max, Ih_rms_sum);
fprintf('ratio = harmonic_sum / fundamental = %.12g\n', ratio_raw);
if abs(ratio_raw) > 1
    fprintf('주의: ratio가 1을 넘어 asin 불가 -> %.12g 로 clamp 후 phi2 계산\n', ratio_clamped);
end
fprintf('phi1 = %.6f deg\n', phi1_deg);
fprintf('phi2 = %.12f deg\n', phi2_deg);
fprintf('phi  = phi1 + phi2 = %.12f deg\n\n', phi_deg);

%% ===================== 스펙트럼용 데이터 =====================
freq_vec = m_vec * f0;      % 각 고조파의 주파수 [Hz]
freq_h   = m_h * f0;

% 부호 있는 스펙트럼: 네 식에서는 D(m) 부호에 따라 실효 phasor 크기 부호가 바뀜
% 위상은 전부 -90deg라서, 부호 있는 "계수"를 보려면 실수 스칼라 계수를 따로 빼는 게 낫다.
Is_signed_coeff_all = A .* UAB ./ (omega*L .* D(m_vec));   % exp(-j*pi/2) 제외한 signed scalar
Is_signed_coeff_h   = A .* UAB ./ (omega*L .* D(m_h));

Is_mag_all = abs(Is_all);
Is_mag_h   = abs(Is_h);

%% ===================== 전체 스펙트럼 플롯 =====================
figure('Color','w','Position',[100 100 1300 850]);

% (1) 고조파 스펙트럼: 부호 있는 계수
subplot(2,2,1);
stem(freq_h/1e3, Is_signed_coeff_h, 'filled', 'LineWidth', 1.5);
grid on;
xlabel('Frequency (kHz)');
ylabel('Signed harmonic coefficient');
title('Secondary current harmonic spectrum (signed)');
xlim([min(freq_h) max(freq_h)]/1e3);

% (2) 전체 스펙트럼: 기본파 + 고조파, 부호 포함
subplot(2,2,2);
stem(freq_vec/1e3, Is_signed_coeff_all, 'filled', 'LineWidth', 1.5);
grid on;
xlabel('Frequency (kHz)');
ylabel('Signed coefficient');
title('Secondary current spectrum: fundamental + harmonics');
xlim([min(freq_vec) max(freq_vec)]/1e3);

% (3) 고조파 magnitude
subplot(2,2,3);
stem(freq_h/1e3, Is_mag_h, 'filled', 'LineWidth', 1.5);
grid on;
xlabel('Frequency (kHz)');
ylabel('|I_{s,m}| (RMS)');
title('Secondary current harmonic magnitude');
xlim([min(freq_h) max(freq_h)]/1e3);

% (4) 전체 magnitude
subplot(2,2,4);
stem(freq_vec/1e3, Is_mag_all, 'filled', 'LineWidth', 1.5);
grid on;
xlabel('Frequency (kHz)');
ylabel('|I_{s,m}| (RMS)');
title('Secondary current spectrum magnitude: all odd components');
xlim([min(freq_vec) max(freq_vec)]/1e3);

sgtitle(sprintf('I_s spectrum / phi_2 estimation   (phi_1 = %.1f deg, phi_2 = %.4f deg, phi = %.4f deg)', ...
    phi1_deg, phi2_deg, phi_deg));

%% ===================== 시간영역 합성파형(선택) =====================
% 입력 구형파 기준 위상 0도
% 기본파 및 고조파 모두 -90도 위상으로 놓고 truncation 합성
t = linspace(0, 3/f0, 4000);   % 기본주기 3개
is_t = zeros(size(t));

for idx = 1:length(m_vec)
    m = m_vec(idx);
    Ism = Is_all(idx); % RMS phasor
    is_t = is_t + sqrt(2)*abs(Ism)*sin(m*omega*t + angle(Ism));
end

figure('Color','w','Position',[150 150 1200 420]);
plot(t*1e6, is_t, 'LineWidth', 1.6);
grid on;
xlabel('Time (\mus)');
ylabel('i_s(t) [A]');
title(sprintf('Reconstructed secondary current from odd harmonics up to m=%d', m_max));

%% ===================== 누적 harmonic sum 확인 =====================
cum_h_sum = cumsum(abs(Is_h));

figure('Color','w','Position',[180 180 1000 420]);
plot(m_h, cum_h_sum, 'o-', 'LineWidth', 1.7, 'MarkerSize', 6); hold on;
yline(Is1_rms, '--', 'LineWidth', 1.5);
grid on;
xlabel('Last included odd harmonic order');
ylabel('Cumulative harmonic RMS sum');
title('Cumulative sum of |I_{s,m}| for odd harmonics');
legend('\Sigma |I_{s,m}|', '|I_{s,1}|', 'Location', 'best');