%% Ip_turnoff_vs_delta_using_phi_from_Is.m
% 목적:
% 1) Is,m 식으로부터 phi2(delta) 계산
% 2) phi(delta)=phi1+phi2(delta)를 Ip,m 식에 대입
% 3) Ip의 m=1 기본파, m=3,5,... 고조파 합, 전체 합의 허수성분 계산
% 4) 이를 delta에 따라 플롯
%
% 정의:
%   phi1 = 90 deg  (네 현재 정의에 맞춤)
%   sin(phi2) = (sum_{m=3,5,...} |Is,m|) / |Is,1|
%   phi = phi1 + phi2
%
% Ip,m 식:
% Ip,m = (2*sqrt(2)/pi) * UAB / [ wL * (m^2-1 - m^4*k^2*(1-d)/(m^2*(1-d)-1)) ] ∠(-90deg)
%      + (2*sqrt(2)/pi) * uab / (m^2*w*k*L) ∠(phi+90deg)
%
% turn-off current:
%   imag(sum of Ip,m) 를 사용
%
% 주의:
% - "무한급수"는 m=1,3,5,...,m_max_ref 까지 truncation 근사
% - UAB, uab는 구형파 레벨값 기준
% - phi2가 매우 작아도 그대로 사용

clear; clc; close all;

%% ===================== 사용자 파라미터 =====================
UAB   = 650;        % [V] 입력 구형파 레벨
uab   = 850;        % [V] 출력측 구형파 레벨(정류기 입력 square-wave 레벨)
f0    = 85e3;       % [Hz]
omega = 2*pi*f0;    % [rad/s]
L     = 212e-6;     % [H]
Lp    = 212e-6;     % [H]
Ls    = 155e-6;     % [H]
k     = 0.2;        % coupling factor

phi1_deg = 90;      % 네 정의대로 사용
phi1_rad = deg2rad(phi1_deg);

% delta sweep
delta_vec = linspace(0, 0.01, 801);

% 급수 truncation
m_max_ref = 1001;       % 충분히 크게
m_all = 1:2:m_max_ref;  % 1,3,5,...,m_max_ref
m_h   = 3:2:m_max_ref;  % 고조파만

%% ===================== 유효성 체크 =====================
if abs(k) < 1e-12
    error('k = 0이면 식이 정의되지 않음.');
end

%% ===================== 공통 상수 =====================
A = 2*sqrt(2)/pi;

%% ===================== 결과 저장 =====================
% Is 기반
Is1_rms_vec   = zeros(size(delta_vec));
Ish_sum_vec   = zeros(size(delta_vec));
phi2_rad_vec  = zeros(size(delta_vec));
phi2_deg_vec  = zeros(size(delta_vec));
phi_rad_vec   = zeros(size(delta_vec));
phi_deg_vec   = zeros(size(delta_vec));

% Ip 기반
Ip1_vec       = zeros(size(delta_vec));   % 기본파 phasor
Iph_sum_vec   = zeros(size(delta_vec));   % 고조파 phasor sum
Iptot_vec     = zeros(size(delta_vec));   % 전체 phasor sum

% 허수성분(= turn-off current로 사용)
Ip1_im_vec    = zeros(size(delta_vec));
Iph_im_vec    = zeros(size(delta_vec));
Iptot_im_vec  = zeros(size(delta_vec));

% 참고용 magnitude
Ip1_mag_vec   = zeros(size(delta_vec));
Iph_mag_vec   = zeros(size(delta_vec));
Iptot_mag_vec = zeros(size(delta_vec));

%% ===================== delta sweep =====================
for idx = 1:length(delta_vec)

    delta = delta_vec(idx);

    if abs(1-delta) < 1e-12
        Is1_rms_vec(idx)   = NaN;
        Ish_sum_vec(idx)   = NaN;
        phi2_rad_vec(idx)  = NaN;
        phi2_deg_vec(idx)  = NaN;
        phi_rad_vec(idx)   = NaN;
        phi_deg_vec(idx)   = NaN;

        Ip1_vec(idx)       = NaN;
        Iph_sum_vec(idx)   = NaN;
        Iptot_vec(idx)     = NaN;
        Ip1_im_vec(idx)    = NaN;
        Iph_im_vec(idx)    = NaN;
        Iptot_im_vec(idx)  = NaN;
        Ip1_mag_vec(idx)   = NaN;
        Iph_mag_vec(idx)   = NaN;
        Iptot_mag_vec(idx) = NaN;
        continue;
    end

    %% ---------- 1) Is,m 로부터 phi2(delta) 계산 ----------
    Is_1st_term = @(m) sqrt(Lp/Ls) .* (A .* UAB ./ (omega*L .* ( (m.^2 - 1) .* (m.^2 .* (1-delta) - 1) ./ (m.^2 .* k .* (1-delta)) - m.^2 .* k ))) .* exp(-1j*pi/2);

    Is_1 = Is_1st_term(1);
    Is_h = Is_1st_term(m_h);

    Is1_rms = abs(Is_1);
    Ish_sum = sum(abs(Is_h));

    % 네 정의 그대로 사용 (clamp 추가)
    ratio = Ish_sum / Is1_rms;
    phi2_rad = asin(min(max(ratio, -1), 1));
    phi2_deg = rad2deg(phi2_rad);

    phi_rad = phi1_rad + phi2_rad;
    phi_deg = phi1_deg + phi2_deg;

    % 저장
    Is1_rms_vec(idx)  = Is1_rms;
    Ish_sum_vec(idx)  = Ish_sum;
    phi2_rad_vec(idx) = phi2_rad;
    phi2_deg_vec(idx) = phi2_deg;
    phi_rad_vec(idx)  = phi_rad;
    phi_deg_vec(idx)  = phi_deg;

    %% ---------- 2) phi(delta)를 Ip,m에 대입 ----------
    % Ip,m phasor (새 공식 적용)
    Ip_phasor = @(m) ...
        (A .* UAB ./ (omega*L .* ( m.^2 - 1 - (m.^4 .* k.^2 .* (1-delta)) ./ (m.^2 .* (1-delta) - 1) ))) .* exp(-1j*pi/2) ...
        + ...
        (A .* uab ./ (omega*L .* ( ((m.^2 - 1) ./ (m .* k)) .* (m - 1 ./ (m .* (1-delta))) - m.^2 .* k ))) .* exp(1j*(phi_rad + pi/2));

    Ip_all = Ip_phasor(m_all);
    Ip_1   = Ip_phasor(1);
    Ip_h   = Ip_phasor(m_h);

    Ip_h_sum = sum(Ip_h);
    Ip_tot   = sum(Ip_all);

    % 저장
    Ip1_vec(idx)     = Ip_1;
    Iph_sum_vec(idx) = Ip_h_sum;
    Iptot_vec(idx)   = Ip_tot;

    Ip1_im_vec(idx)   = imag(Ip_1);
    Iph_im_vec(idx)   = imag(Ip_h_sum);
    Iptot_im_vec(idx) = imag(Ip_tot);

    Ip1_mag_vec(idx)   = abs(Ip_1);
    Iph_mag_vec(idx)   = abs(Ip_h_sum);
    Iptot_mag_vec(idx) = abs(Ip_tot);
end

%% ===================== 결과 출력 예시 =====================
delta_pick = 0.1;
[~, pick_idx] = min(abs(delta_vec - delta_pick));

fprintf('=== Example at delta = %.6f ===\n', delta_vec(pick_idx));
fprintf('|Is,1|_RMS                = %.12g A\n', Is1_rms_vec(pick_idx));
fprintf('sum |Is,h|               = %.12g A\n', Ish_sum_vec(pick_idx));
fprintf('phi2                     = %.12f deg\n', phi2_deg_vec(pick_idx));
fprintf('phi                      = %.12f deg\n', phi_deg_vec(pick_idx));
fprintf('Imag(Ip,1)               = %.12g A\n', Ip1_im_vec(pick_idx));
fprintf('Imag(sum Ip,h)           = %.12g A\n', Iph_im_vec(pick_idx));
fprintf('Imag(sum Ip,total)       = %.12g A\n', Iptot_im_vec(pick_idx));
fprintf('|Ip,1|                    = %.12g A\n', Ip1_mag_vec(pick_idx));
fprintf('|sum Ip,h|                = %.12g A\n', Iph_mag_vec(pick_idx));
fprintf('|sum Ip,total|            = %.12g A\n', Iptot_mag_vec(pick_idx));

%% ===================== 그래프 1: phi2(delta) =====================
figure('Color','w','Position',[100 100 1000 420]);
plot(delta_vec, phi2_deg_vec, 'LineWidth', 1.8);
grid on;
xlabel('\delta = \DeltaC / C');
ylabel('\phi_2 (deg)');
title('\phi_2 versus \delta');

%% ===================== 그래프 2: Ip 허수성분(= turn-off current) =====================
figure('Color','w','Position',[120 120 1100 700]);

subplot(3,1,1);
plot(delta_vec, -Ip1_im_vec, 'LineWidth', 1.8);
grid on;
xlabel('\delta = \DeltaC / C');
ylabel('Imag(I_{p,1}) [A]');
title('Fundamental component of turn-off current');

subplot(3,1,2);
plot(delta_vec, -Iph_im_vec, 'LineWidth', 1.8);
grid on;
xlabel('\delta = \DeltaC / C');
ylabel('Imag(\Sigma I_{p,h}) [A]');
title(sprintf('Harmonic-sum component of turn-off current (odd m=3:2:%d)', m_max_ref));

subplot(3,1,3);
plot(delta_vec, -Iptot_im_vec, 'LineWidth', 1.8);
grid on;
xlabel('\delta = \DeltaC / C');
ylabel('Imag(\Sigma I_{p,total}) [A]');
title(sprintf('Total turn-off current (fundamental + harmonics), odd m=1:2:%d', m_max_ref));

sgtitle('Turn-off current versus \delta using \phi(\delta) from I_s');

%% ===================== 그래프 3: 참고용 magnitude =====================
figure('Color','w','Position',[140 140 1100 700]);

subplot(3,1,1);
plot(delta_vec, Ip1_mag_vec, 'LineWidth', 1.8);
grid on;
xlabel('\delta = \DeltaC / C');
ylabel('|I_{p,1}| [A]');
title('Fundamental magnitude');

subplot(3,1,2);
plot(delta_vec, Iph_mag_vec, 'LineWidth', 1.8);
grid on;
xlabel('\delta = \DeltaC / C');
ylabel('| \Sigma I_{p,h} | [A]');
title(sprintf('Harmonic-sum magnitude, odd m=3:2:%d', m_max_ref));

subplot(3,1,3);
plot(delta_vec, Iptot_mag_vec, 'LineWidth', 1.8);
grid on;
xlabel('\delta = \DeltaC / C');
ylabel('| \Sigma I_{p,total} | [A]');
title(sprintf('Total magnitude, odd m=1:2:%d', m_max_ref));

sgtitle('Reference magnitude plots versus \delta');

%% ===================== 그래프 4: phi와 turn-off current 한 화면 =====================
figure('Color','w','Position',[160 160 1000 450]);

yyaxis left
plot(delta_vec, phi_deg_vec, 'LineWidth', 1.8);
ylabel('\phi (deg)');

yyaxis right
plot(delta_vec, -Iptot_im_vec, 'LineWidth', 1.8);
ylabel('Imag(\Sigma I_p) [A]');

grid on;
xlabel('\delta = \DeltaC / C');
title('\phi(\delta) and total turn-off current');
legend('\phi', 'Imag(\Sigma I_p)', 'Location', 'best');