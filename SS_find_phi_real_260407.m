function result = find_phi_debug_SS_v2()
% find_phi_debug_SS_v2
%
% 네 기준 반영:
% - 구형파 위상 기준: 상승엣지 = 0도
% - 기본파는 sin 기준
% - 위상이 phi이면 relevant zero crossing은 보통 (180 - phi) 근방
% - 따라서 error = |zc_deg - (180 - phi_deg)|
%
% 디버그 목적:
% - step마다 대표 9개 case를 subplot으로 표시
% - 각 case마다 phi, target zc, actual zc를 수직선으로 그림
% - 한 주기 전체(0~360deg)를 항상 그림

clear; clc; close all;

%% =========================
% User parameters
% ==========================
delta = 0.05;

% ====== 네 실제 값으로 바꿔라 ======
UAB = 650;        % [V_rms]
uab = 850;        % [V_rms]
f0  = 85e3;       % [Hz]
omega = 2*pi*f0;  % [rad/s]

L  = 212e-6;      % [H]
k  = 0.2;
Lp = 212e-6;      % [H]
Ls = 155e-6;      % [H]
% ================================

% 홀수 고조파
m_list = 1:2:101;   % 디버그 먼저 21로 보자. 나중에 101

% theta grid
Ntheta = 40000;
theta = linspace(0, 2*pi, Ntheta);   % full cycle

% 보기용 u_ab square
% 네 기준상 rising edge = 0 deg 이므로 sign(sin(.))가 맞음
% phi가 클수록 앞서간다고 보면 sign(sin(theta + phi))
uab_mode = 1;   % 아래 build_uab_wave() 참고

% phi 탐색 step
phi_search_ranges = [ ...
    88, 100, 721; ...
    0,  1,   721; ...
    0,  0.1, 721; ...
    0,  0.01, 721 ...
];

% zc는 phi 자체 근처가 아니라 (180 - phi) 근처에서 찾는다
zc_half_window_deg = 8;   % target 주변 ±8deg 에서 탐색

% 네 설명 기준으로 falling crossing이 맞다
crossing_type = 'falling';

use_parallel = true;
n_show = 9;

%% =========================
% iterative sweep
% ==========================
history = struct('phi_grid_deg', {}, 'err', {}, 'phi_best_deg', {}, ...
                 'zc_best_deg', {}, 'target_best_deg', {}, ...
                 'valid_mask', {}, 'range_info', {});

phi_center = nan;

for step = 1:size(phi_search_ranges,1)

    if step == 1
        phi_min = phi_search_ranges(step,1);
        phi_max = phi_search_ranges(step,2);
        nphi    = phi_search_ranges(step,3);
        phi_grid_deg = linspace(phi_min, phi_max, nphi);
    else
        half_range = phi_search_ranges(step,2);
        nphi       = phi_search_ranges(step,3);
        phi_grid_deg = linspace(phi_center - half_range, phi_center + half_range, nphi);
    end

    err_vec = inf(size(phi_grid_deg));
    zc_vec_deg = nan(size(phi_grid_deg));
    target_vec_deg = nan(size(phi_grid_deg));
    valid_mask = false(size(phi_grid_deg));

    if use_parallel
        parfor i = 1:length(phi_grid_deg)
            phi_deg = phi_grid_deg(i);
            phi = deg2rad(phi_deg);

            [is_total, ~, ~, ~, ~, ~] = ...
                reconstruct_is_waveform(phi, delta, UAB, uab, omega, L, k, Lp, Ls, m_list, theta);

            target_deg = 180 - phi_deg;
            zc_window_deg = [target_deg - zc_half_window_deg, target_deg + zc_half_window_deg];

            [zc_deg, ok] = find_relevant_zero_crossing(theta, is_total, zc_window_deg, crossing_type);

            target_vec_deg(i) = target_deg;

            if ok
                err_vec(i) = abs(zc_deg - target_deg);
                zc_vec_deg(i) = zc_deg;
                valid_mask(i) = true;
            else
                err_vec(i) = 1e6;
                zc_vec_deg(i) = nan;
                valid_mask(i) = false;
            end
        end
    else
        for i = 1:length(phi_grid_deg)
            phi_deg = phi_grid_deg(i);
            phi = deg2rad(phi_deg);

            [is_total, ~, ~, ~, ~, ~] = ...
                reconstruct_is_waveform(phi, delta, UAB, uab, omega, L, k, Lp, Ls, m_list, theta);

            target_deg = 180 - phi_deg;
            zc_window_deg = [target_deg - zc_half_window_deg, target_deg + zc_half_window_deg];

            [zc_deg, ok] = find_relevant_zero_crossing(theta, is_total, zc_window_deg, crossing_type);

            target_vec_deg(i) = target_deg;

            if ok
                err_vec(i) = abs(zc_deg - target_deg);
                zc_vec_deg(i) = zc_deg;
                valid_mask(i) = true;
            else
                err_vec(i) = 1e6;
                zc_vec_deg(i) = nan;
                valid_mask(i) = false;
            end
        end
    end

    [err_best, idx_best] = min(err_vec);
    phi_best_deg = phi_grid_deg(idx_best);
    zc_best_deg = zc_vec_deg(idx_best);
    target_best_deg = target_vec_deg(idx_best);

    phi_center = phi_best_deg;

    history(step).phi_grid_deg = phi_grid_deg;
    history(step).err = err_vec;
    history(step).phi_best_deg = phi_best_deg;
    history(step).zc_best_deg = zc_best_deg;
    history(step).target_best_deg = target_best_deg;
    history(step).valid_mask = valid_mask;
    history(step).range_info = phi_search_ranges(step,:);

    fprintf('[Step %d] best phi = %.10f deg, target = %.10f deg, zc = %.10f deg, err = %.12e deg\n', ...
        step, phi_best_deg, target_best_deg, zc_best_deg, err_best);

    plot_step_debug_9cases_v2(step, phi_grid_deg, err_vec, zc_vec_deg, target_vec_deg, valid_mask, ...
        delta, UAB, uab, omega, L, k, Lp, Ls, m_list, theta, ...
        uab_mode, crossing_type, n_show, phi_best_deg, zc_half_window_deg);
end

phi_best_deg = phi_center;
phi_best = deg2rad(phi_best_deg);

%% =========================
% final reconstruction
% ==========================
[is_total, is_fund, is_higher, Ism, Ism_mag, Ism_ang] = ...
    reconstruct_is_waveform(phi_best, delta, UAB, uab, omega, L, k, Lp, Ls, m_list, theta);

target_best_deg = 180 - phi_best_deg;
zc_window_deg = [target_best_deg - zc_half_window_deg, target_best_deg + zc_half_window_deg];
[zc_best_deg, ok] = find_relevant_zero_crossing(theta, is_total, zc_window_deg, crossing_type);

uab_wave = build_uab_wave(theta, phi_best, uab_mode);

%% =========================
% final plot
% ==========================
figure('Name','Final Best Case Full Cycle','Color','w');
plot(rad2deg(theta), is_total, 'LineWidth', 1.8); hold on;
plot(rad2deg(theta), is_fund, '--', 'LineWidth', 1.4);
plot(rad2deg(theta), is_higher, ':', 'LineWidth', 1.8);

scale = max(abs(is_total));
if scale == 0, scale = 1; end
plot(rad2deg(theta), 0.65*scale*uab_wave, 'LineWidth', 1.1);

xline(phi_best_deg, '--k', 'LineWidth', 1.2);
xline(target_best_deg, '--m', 'LineWidth', 1.2);
if ok
    xline(zc_best_deg, '--r', 'LineWidth', 1.2);
end

xlim([0 360]);
grid on;
xlabel('\theta [deg]');
ylabel('Current / scaled sign');
title(sprintf('Final best phi = %.8f deg | target zc = %.8f deg | actual zc = %.8f deg', ...
    phi_best_deg, target_best_deg, zc_best_deg));
legend('Is total', 'Is fundamental', 'Is higher', 'scaled u_{ab}', ...
       'phi', 'target zc = 180-\phi', 'actual zc', 'Location', 'best');

%% =========================
% zoom around target region
% ==========================
figure('Name','Final Zoom Around Target ZC','Color','w');
plot(rad2deg(theta), is_total, 'LineWidth', 1.8); hold on;
plot(rad2deg(theta), is_fund, '--', 'LineWidth', 1.4);
plot(rad2deg(theta), is_higher, ':', 'LineWidth', 1.8);

xline(phi_best_deg, '--k', 'LineWidth', 1.2);
xline(target_best_deg, '--m', 'LineWidth', 1.2);
if ok
    xline(zc_best_deg, '--r', 'LineWidth', 1.2);
end

xlim([target_best_deg - 20, target_best_deg + 20]);
grid on;
xlabel('\theta [deg]');
ylabel('Current [A]');
title('Zoom near target zero crossing');

%% =========================
% error curve
% ==========================
figure('Name','Final refinement error curve','Color','w');
plot(history(end).phi_grid_deg, history(end).err, 'LineWidth', 1.5);
grid on;
xlabel('\phi [deg]');
ylabel('|zc - (180-\phi)| [deg]');
title('Final refinement error curve');

fprintf('\n=== Final Result ===\n');
fprintf('delta       = %.8f\n', delta);
fprintf('phi_best    = %.10f deg\n', phi_best_deg);
fprintf('target zc   = %.10f deg\n', target_best_deg);
if ok
    fprintf('actual zc   = %.10f deg\n', zc_best_deg);
    fprintf('final error = %.12e deg\n', abs(zc_best_deg - target_best_deg));
else
    fprintf('zero crossing not found near target window.\n');
end

fprintf('\n=== Harmonic phasors of Is at best phi ===\n');
for target = [1 3 5]
    idx = find(m_list == target, 1);
    if ~isempty(idx)
        fprintf('m = %d : |Is_m| = %.6f [A_rms], angle = %.6f [deg]\n', ...
            target, Ism_mag(idx), rad2deg(Ism_ang(idx)));
    end
end

result = struct();
result.delta = delta;
result.phi_best_deg = phi_best_deg;
result.phi_best_rad = phi_best;
result.target_best_deg = target_best_deg;
result.zc_best_deg = zc_best_deg;
result.theta_deg = rad2deg(theta);
result.is_total = is_total;
result.is_fund = is_fund;
result.is_higher = is_higher;
result.history = history;
result.Ism = Ism;
result.Ism_mag = Ism_mag;
result.Ism_ang = Ism_ang;

end

%% ========================================================================
function plot_step_debug_9cases_v2(step, phi_grid_deg, err_vec, zc_vec_deg, target_vec_deg, valid_mask, ...
    delta, UAB, uab, omega, L, k, Lp, Ls, m_list, theta, ...
    uab_mode, crossing_type, n_show, phi_best_deg, zc_half_window_deg)

N = numel(phi_grid_deg);
if N <= n_show
    pick_idx = 1:N;
else
    pick_idx = round(linspace(1, N, n_show));
end

figure('Name', sprintf('Step %d representative 9 cases', step), 'Color', 'w');
tiledlayout(3,3, 'Padding','compact', 'TileSpacing','compact');

for kk = 1:numel(pick_idx)
    i = pick_idx(kk);
    phi_deg = phi_grid_deg(i);
    phi = deg2rad(phi_deg);

    [is_total, is_fund, is_higher, ~, ~, ~] = ...
        reconstruct_is_waveform(phi, delta, UAB, uab, omega, L, k, Lp, Ls, m_list, theta);

    uab_wave = build_uab_wave(theta, phi, uab_mode);

    target_deg = target_vec_deg(i);
    zc_window_deg = [target_deg - zc_half_window_deg, target_deg + zc_half_window_deg];
    [zc_deg, ok] = find_relevant_zero_crossing(theta, is_total, zc_window_deg, crossing_type);

    nexttile;
    plot(rad2deg(theta), is_total, 'LineWidth', 1.1); hold on;
    plot(rad2deg(theta), is_fund, '--', 'LineWidth', 0.9);
    plot(rad2deg(theta), is_higher, ':', 'LineWidth', 1.0);

    scale = max(abs(is_total));
    if scale == 0, scale = 1; end
    plot(rad2deg(theta), 0.55*scale*uab_wave, 'LineWidth', 0.9);

    xline(phi_deg, '--k', 'LineWidth', 1.0);
    xline(target_deg, '--m', 'LineWidth', 1.0);
    if ok
        xline(zc_deg, '--r', 'LineWidth', 1.0);
    end
    xline(zc_window_deg(1), ':', 'LineWidth', 0.8);
    xline(zc_window_deg(2), ':', 'LineWidth', 0.8);

    xlim([0 360]);
    grid on;

    if ok
        ttl = sprintf('\\phi=%.3f | tgt=%.3f | zc=%.3f | err=%.4g', ...
            phi_deg, target_deg, zc_deg, abs(zc_deg-target_deg));
    else
        ttl = sprintf('\\phi=%.3f | tgt=%.3f | zc=NaN | err=%.4g', ...
            phi_deg, target_deg, err_vec(i));
    end

    if abs(phi_deg - phi_best_deg) < 1e-12
        ttl = ['[BEST] ' ttl];
    end

    title(ttl, 'FontSize', 8);

    if kk == 1
        legend('Is total', 'Is fund', 'Is higher', 'scaled u_{ab}', ...
               'phi', 'target zc', 'actual zc', 'Location', 'best');
    end
end

sgtitle(sprintf('Step %d representative 9 cases | delta=%.5f | crossing=%s', ...
    step, delta, crossing_type));

end

%% ========================================================================
function [is_total, is_fund, is_higher, Ism, Ism_mag, Ism_ang, harm_waves] = ...
    reconstruct_is_waveform(phi, delta, UAB, uab, omega, L, k, Lp, Ls, m_list, theta)
% 모든 기준을 sin으로 통일한 시간영역 복원

A = 2*sqrt(2)/pi;
Is_term = @(m) ...
    sqrt(Lp/Ls) .* ( ...
    (A .* UAB ./ (omega*L .* ( (m.^2 - 1) .* (m.^2 .* (1-delta) - 1) ./ (m.^2 .* k .* (1-delta)) - m.^2 .* k ))) .* exp(-1j*pi/2) ...
    + ...
    (A .* uab .* (m.^2 - 1) ./ (omega*L .* ( (m.^2 - 1).*(m.^2 - 1./(1-delta)) - m.^4 .* k^2 ))) .* exp(1j*(m*phi + pi/2)) ...
    );

Ism = zeros(size(m_list));
Ism_mag = zeros(size(m_list));
Ism_ang = zeros(size(m_list));

for idx = 1:length(m_list)
    m = m_list(idx);
    
    Ism(idx) = Is_term(m);
    Ism_mag(idx) = abs(Ism(idx));
    Ism_ang(idx) = angle(Ism(idx));
end

is_total   = zeros(size(theta));
is_fund    = zeros(size(theta));
is_higher  = zeros(size(theta));
harm_waves = zeros(length(m_list), length(theta));

for idx = 1:length(m_list)
    m = m_list(idx);

    i_m = sqrt(2) * Ism_mag(idx) .* sin(m * theta + Ism_ang(idx));

    harm_waves(idx, :) = i_m;
    is_total = is_total + i_m;

    if m == 1
        is_fund = i_m;
    else
        is_higher = is_higher + i_m;
    end
end

end

%% ========================================================================
function [zc_deg, ok] = find_relevant_zero_crossing(theta, signal, zc_window_deg, crossing_type)

theta_deg = rad2deg(theta);

mask = (theta_deg >= zc_window_deg(1)) & (theta_deg <= zc_window_deg(2));
th = theta_deg(mask);
sg = signal(mask);

zc_list = [];

for i = 1:length(sg)-1
    y1 = sg(i);
    y2 = sg(i+1);

    if y1 == 0
        zc = th(i);
        slope = y2 - y1;
    elseif y1 * y2 < 0
        zc = th(i) - y1 * (th(i+1)-th(i)) / (y2-y1);
        slope = y2 - y1;
    else
        continue;
    end

    switch lower(crossing_type)
        case 'rising'
            if slope > 0
                zc_list(end+1) = zc; %#ok<AGROW>
            end
        case 'falling'
            if slope < 0
                zc_list(end+1) = zc; %#ok<AGROW>
            end
        case 'any'
            zc_list(end+1) = zc; %#ok<AGROW>
        otherwise
            error('Invalid crossing_type');
    end
end

if isempty(zc_list)
    zc_deg = nan;
    ok = false;
    return;
end

target_deg = mean(zc_window_deg);
[~, idx] = min(abs(zc_list - target_deg));
zc_deg = zc_list(idx);
ok = true;

end

%% ========================================================================
function u = build_uab_wave(theta, phi, mode)
% 네 기준:
% rising edge = 0도
% phi가 커질수록 앞서간다고 보고 sign(sin(theta + phi)) 계열을 사용

switch mode
    case 1
        u = sign(sin(theta + phi));
    case 2
        u = -sign(sin(theta + phi));
    case 3
        u = sign(cos(theta + phi));
    case 4
        u = -sign(cos(theta + phi));
    otherwise
        error('Invalid uab_mode');
end
end