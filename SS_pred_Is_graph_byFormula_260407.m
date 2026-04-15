% % Is waveform reconstruction from harmonic phasors % == == == == == == == ==
    == == == == == == == == == == == == == == == == == == == == == == == == ==
    == == ==
    = % [버전 기록] %
      v1.0 : (이전 버전)수식에 기반한 고조파 페이저 도출 및 파형 재구성.(
                 주파수 스펙트럼 x축 : kHz) %
      v1.1 : (현재 버전 - 2026 - 04 - 07 00 : 50 : 23)
                 주파수 스펙트럼의 x축을 고조파
                 차수(m)
로 변경, stem을 꺾은선 그래프(plot)로 변경.
% =========================================================================
% Based on:
% I_{s,m} = sqrt(Lp/Ls) * (2*sqrt(2)/pi) * [ A_m ∠(-90°) + B_m ∠(phi+90°) ]
%
% User must set: UAB, uab, f0(or omega), L, k, Lp, Ls
% delta = 0.05, phi = 94 deg
%
% This script:
% 1) calculates fundamental, 3rd, 5th harmonic phasors
% 2) reconstructs time waveform Is(t)
% 3) plots frequency spectrum
% 4) plots final overlay:
%    - sum of 1~11 odd harmonics
%    - fundamental only
%    - sum of 3~11 odd harmonics

clear;
clc;
close all;

% % == == == == == == == == == == == == = % User parameters % == == == == == ==
                                          == == == == == == == delta = 0.05;
phi_deg = 94;
phi = deg2rad(phi_deg);

% == == == Fill these with your actual values == == == UAB = 650;
% [V_rms] first - side related voltage magnitude uab = 850;
% [V_rms] second - side related voltage magnitude f0 = 85e3;
% [Hz] fundamental frequency omega = 2 * pi * f0;
% [rad / s]

    L = 212e-6;
% [H] k = 0.2;
% coupling coefficient Lp = 212e-6;
% [H] Ls = 155e-6;
% [H] % == == == == == == == == == == == == == == == == == == == == == == ==
    =

        % % Harmonic orders to use m_list = 1 : 2 : 71;
% odd harmonics : 1, 3, 5, 7, 9,
    11

        % % Time axis T0 = 1 / f0;
t = linspace(0, 3 * T0, 4000);
% show 3 cycles

    % % Preallocate Ism = zeros(size(m_list));       % complex RMS phasor for each harmonic
Ism_mag = zeros(size(m_list));
% RMS magnitude Ism_ang = zeros(size(m_list));   % phase [rad]

%% Calculate each harmonic phasor
A = 2*sqrt(2)/pi;
Is_term = @(m) ...
    sqrt(Lp/Ls) .* ( ...
    (A .* UAB ./ (omega*L .* ( (m.^2 - 1) .* (m.^2 .* (1-delta) - 1) ./ (m.^2 .* k .* (1-delta)) - m.^2 .* k ))) .* exp(-1j*pi/2) ...
    + ...
    (A .* uab .* (m.^2 - 1) ./ (omega*L .* ( (m.^2 - 1).*(m.^2 - 1./(1-delta)) - m.^4 .* k^2 ))) .* exp(1j*(m*phi + pi/2)) ...
    );

for idx = 1:length(m_list)
    m = m_list(idx);

    Ism(idx) = Is_term(m);

    Ism_mag(idx) = abs(Ism(idx));
    Ism_ang(idx) = angle(Ism(idx));
end

fprintf('=== Harmonic phasors of Is (RMS phasor) ===\n');
for target = [1 3 5]
    idx = find(m_list == target);
    fprintf('m = %d : |Is_m| = %.6f [A_rms], angle = %.3f [deg]\n', ... 
        target, Ism_mag(idx), rad2deg(Ism_ang(idx)));
end

% Time - domain reconstruction
is_total = zeros(size(t));
is_fund = zeros(size(t));
is_higher = zeros(size(t));   % 3~11 odd harmonics

for idx = 1:length(m_list)
    m = m_list(idx);
    i_m_t = sqrt(2) * Ism_mag(idx) * sin(m * omega * t + Ism_ang(idx));

    is_total = is_total + i_m_t;

    if m == 1
        is_fund = i_m_t;
    else
        is_higher = is_higher + i_m_t;
    end
end

% Individual harmonic waveforms : 1st, 3rd, 5th
idx1 = find(m_list == 1);
idx3 = find(m_list == 3);
idx5 = find(m_list == 5);

i1_t = sqrt(2) * Ism_mag(idx1) * sin(1 * omega * t + Ism_ang(idx1));
i3_t = sqrt(2) * Ism_mag(idx3) * sin(3 * omega * t + Ism_ang(idx3));
i5_t = sqrt(2) * Ism_mag(idx5) * sin(5 * omega * t + Ism_ang(idx5));

% % == == == == == == == == == == == ==
    = % Figure 1 : Fundamental % == == == == == == == == == == == == ==
      figure('Name', 'Is Fundamental', 'Color', 'w');
plot(t * 1e6, i1_t, 'LineWidth', 1.6);
grid on;
xlabel('Time [\mus]');
ylabel('Current [A]');
title('Is Fundamental (m = 1)');

% % == == == == == == == == == == == ==
    = % Figure 2 : 3rd harmonic % == == == == == == == == == == == == ==
      figure('Name', 'Is 3rd Harmonic', 'Color', 'w');
plot(t * 1e6, i3_t, 'LineWidth', 1.6);
grid on;
xlabel('Time [\mus]');
ylabel('Current [A]');
title('Is 3rd Harmonic (m = 3)');

% % == == == == == == == == == == == ==
    = % Figure 3 : 5th harmonic % == == == == == == == == == == == == ==
      figure('Name', 'Is 5th Harmonic', 'Color', 'w');
plot(t * 1e6, i5_t, 'LineWidth', 1.6);
grid on;
xlabel('Time [\mus]');
ylabel('Current [A]');
title('Is 5th Harmonic (m = 5)');

% % == == == == == == == == == == == ==
    = % Figure 4 : Frequency Spectrum % == == == == == == == == == == == == ==
      figure('Name', 'Frequency Spectrum of Is', 'Color', 'w');
plot(m_list, Ism_mag, '-o', 'LineWidth', 1.6, 'MarkerSize', 6,
     'MarkerFaceColor', 'auto');
grid on;
xticks(1 : 10 : max(m_list));
% 고조파 개수가 많으므로 적당한 간격으로 눈금 설정 xlabel('Harmonic Order (m)');
ylabel('|I_{s,m}| [A_{rms}]');
title('Frequency Spectrum of Is (Odd Harmonics)');

% % == == == == == == == == == == == ==
    = % Figure 5 : Overlay plot requested % -1 ~11 odd harmonic sum %
                   -fundamental only % -3 ~11 odd harmonic sum %
      == == == == == == == == == == == == ==
      figure('Name', 'Overlay of Is Waveforms', 'Color', 'w');
plot(t * 1e6, is_total, 'LineWidth', 1.8);
hold on;
plot(t * 1e6, is_fund, '--', 'LineWidth', 1.6);
plot(t * 1e6, is_higher, ':', 'LineWidth', 2.0);
grid on;
xlabel('Time [\mus]');
ylabel('Current [A]');
title('Is Waveform Overlay');
legend('1~11 odd harmonics sum', 'fundamental only', '3~11 odd harmonics sum',
       ... 'Location', 'best');

% % == == == == == == == == == == == ==
    = % Optional : Show all individual odd harmonics in one figure % == == == ==
      == == == == == == == == ==
      figure('Name', 'All odd harmonic components', 'Color', 'w');
hold on;
grid on;
for idx = 1 : length(m_list)
    m = m_list(idx);
    i_m_t = sqrt(2) * Ism_mag(idx) * sin(m * omega * t + Ism_ang(idx));
    plot(t * 1e6, i_m_t, 'LineWidth', 1.2, 'DisplayName', sprintf('m = %d', m));
end xlabel('Time [\mus]');
ylabel('Current [A]');
title('Individual Harmonic Components of Is');
legend show;