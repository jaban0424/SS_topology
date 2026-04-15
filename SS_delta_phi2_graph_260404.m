%% Is_delta_sweep_phi2_and_rms.m
% delta 변화에 따른
% 1) phi2 변화
% 2) 기본파 RMS 변화
% 3) 고조파 RMS 합 변화
%
% 기준 식:
% Is,m = (2*sqrt(2)/pi) * UAB / { wL * [ ((m^2-1)*(m^2(1-d)-1))/(m^2*k*(1-d)) - m^2*k ] } ∠(-90deg)
%
% 정의:
% phi1 = 90 deg
% sin(phi2) = (sum_{m=3,5,...,m_max} |Is,m|) / |Is,1|
%
% 주의:
% ratio > 1 이면 asin 불가이므로 clamp 처리
% UAB는 구형파 레벨값 기준

clear; clc; close all;

%% ===================== 사용자 파라미터 =====================
UAB   = 650;        % [V] 입력 구형파 레벨
f0    = 85e3;       % [Hz]
omega = 2*pi*f0;    % [rad/s]
L     = 212e-6;     % [H]
Lp    = 212e-6;     % [H]
Ls    = 155e-6;     % [H]
k     = 0.2;        % coupling factor

phi1_deg = 90;      % 네가 현재 쓰는 정의
m_max = 2001;
m_vec = 1:2:m_max;  % 1,3,5,...,35
m_h   = 3:2:m_max;  % 고조파만

% delta sweep
delta_vec = linspace(0, 0.7, 1001);

%% ===================== 유효성 체크 =====================
if abs(k) < 1e-12
    error('k = 0이면 식이 정의되지 않음.');
end

%% ===================== 공통 상수 =====================
A = 2*sqrt(2)/pi;

%% ===================== 결과 저장 변수 =====================
Is1_rms_vec   = zeros(size(delta_vec));   % 기본파 RMS
Ih_sum_vec    = zeros(size(delta_vec));   % 고조파 RMS 합
ratio_raw_vec = zeros(size(delta_vec));   % raw ratio
ratio_clp_vec = zeros(size(delta_vec));   % clamp ratio
phi2_rad_vec  = zeros(size(delta_vec));
phi2_deg_vec  = zeros(size(delta_vec));
phi_deg_vec   = zeros(size(delta_vec));

%% ===================== delta sweep =====================
for idx = 1:length(delta_vec)

    delta = delta_vec(idx);

    % 특이점 방지
    if abs(1-delta) < 1e-12
        Is1_rms_vec(idx)   = NaN;
        Ih_sum_vec(idx)    = NaN;
        ratio_raw_vec(idx) = NaN;
        ratio_clp_vec(idx) = NaN;
        phi2_rad_vec(idx)  = NaN;
        phi2_deg_vec(idx)  = NaN;
        phi_deg_vec(idx)   = NaN;
        continue;
    end

    % RMS phasor Is,m 의 첫번째 항 (위상차 도출 용도)
    Is_1st_term = @(m) sqrt(Lp/Ls) .* (A .* UAB ./ (omega*L .* ( (m.^2 - 1) .* (m.^2 .* (1-delta) - 1) ./ (m.^2 .* k .* (1-delta)) - m.^2 .* k ))) .* exp(-1j*pi/2);

    % 기본파 및 고조파
    Is_1 = Is_1st_term(1);
    Is_h = Is_1st_term(m_h);

    Is1_rms = abs(Is_1);
    Ih_sum  = sum(abs(Is_h));

    ratio_raw = Ih_sum / Is1_rms;
    ratio_clamped = min(max(ratio_raw, -1), 1);

    phi2_rad = asin(ratio_clamped);
    phi2_deg = rad2deg(phi2_rad);
    phi_deg  = phi1_deg + phi2_deg;

    % 저장
    Is1_rms_vec(idx)   = Is1_rms;
    Ih_sum_vec(idx)    = Ih_sum;
    ratio_raw_vec(idx) = ratio_raw;
    ratio_clp_vec(idx) = ratio_clamped;
    phi2_rad_vec(idx)  = phi2_rad;
    phi2_deg_vec(idx)  = phi2_deg;
    phi_deg_vec(idx)   = phi_deg;
end

%% ===================== 그래프 =====================
figure('Color','w','Position',[100 100 900 950]);

% (1) phi2 vs delta
subplot(3,1,1);
plot(delta_vec, phi2_deg_vec, 'LineWidth', 1.8);
grid on;
xlabel('\delta = \DeltaC / C');
ylabel('\phi_2 (deg)');
title('\phi_2 trend versus \delta');

% (2) fundamental RMS vs delta
subplot(3,1,2);
plot(delta_vec, Is1_rms_vec, 'LineWidth', 1.8);
grid on;
xlabel('\delta = \DeltaC / C');
ylabel('|I_{s,1}|_{RMS} (A)');
title('Fundamental RMS versus \delta');

% (3) harmonic RMS sum vs delta
subplot(3,1,3);
plot(delta_vec, Ih_sum_vec, 'LineWidth', 1.8);
grid on;
xlabel('\delta = \DeltaC / C');
ylabel('\Sigma |I_{s,m}|_{RMS} (A), m=3,5,...');
title(sprintf('Harmonic RMS sum versus \\delta   (up to m=%d)', m_max));

sgtitle('Delta sweep analysis for \phi_2 and secondary-current RMS');

%% ===================== ratio 확인용 추가 그래프 =====================
figure('Color','w','Position',[120 120 900 420]);
plot(delta_vec, ratio_raw_vec, 'LineWidth', 1.8); hold on;
plot(delta_vec, ratio_clp_vec, '--', 'LineWidth', 1.5);
yline(1, ':', 'LineWidth', 1.2);
grid on;
xlabel('\delta = \DeltaC / C');
ylabel('ratio');
title('ratio = (\Sigma |I_{s,h}|) / |I_{s,1}|');
legend('raw ratio', 'clamped ratio', 'ratio = 1', 'Location', 'best');

%% ===================== 특정 delta에서 값 출력 예시 =====================
delta_pick = 0.05;
[~, pick_idx] = min(abs(delta_vec - delta_pick));

fprintf('=== delta sweep summary ===\n');
fprintf('Picked delta = %.6f\n', delta_vec(pick_idx));
fprintf('|Is,1|_RMS = %.12g A\n', Is1_rms_vec(pick_idx));
fprintf('Sum of harmonic RMS = %.12g A\n', Ih_sum_vec(pick_idx));
fprintf('raw ratio = %.12g\n', ratio_raw_vec(pick_idx));
fprintf('phi2 = %.12f deg\n', phi2_deg_vec(pick_idx));
fprintf('phi  = %.12f deg\n', phi_deg_vec(pick_idx));