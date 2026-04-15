% SS_sweep_phi_grid_260407.m
% 정류기 입력 전압 위상(\phi) 해를 delta에 대해 sweep하며 추적 (Grid Search 방식 차용)
%
% =========================================================================
% [버전 기록]
% v1.2 : (2026-04-07 21:30:00)
%        - 통합 비교 스펙트럼(3D)을 위해 각 차수별 위상 데이터(Ism_ang_all) 추출 및 저장 로직 추가
% v1.1 : (2026-04-07 20:47:08) 
%        - 3D 주파수 스펙트럼 표출 추가 (surf 면 그래프) 및 표시 최대 고조파 한계 설정 변수 추가
%        - 병렬 워커 개수 명시적 할당 기능 추가 등 
% =========================================================================
% 
% 특징:
% - 기존 SS_find_phi_real_260407.m 의 세밀한 범위를 좁혀가는 4-step Grid Search 방식을 
%   모든 delta(0 ~ 0.1)에 대해 반복하여 해를 찾습니다.
% - 찾은 해의 변화(\phi), 2차 전류 크기(RMS), 그리고 오차를 최종 그래프로 출력합니다.
% - (최적화) 시간축 해상도(Ntheta)를 기존 40000에서 4000으로 조정하여 속도를 약 10배 향상시켰습니다.
%   보간 알고리즘 덕분에 정확도에는 큰 차이가 없습니다.

% function 제거: 외부에서 script 형태로 호출하여 작업공간(Workspace) 공유
% 주의: 변수를 공유하기 위해 clear하지 않습니다.
    
    %% =========================
    % User parameters (SS_topology_design.m 에서 불러오기)
    % ==========================
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
    L   = SS_Lp;           % [H] 기존 수식 기호 호환용
    k   = SS_k;            % 결합계수
    f0  = SS_Freq;         % [Hz] 기준 주파수
    omega = SS_AglFreq;    % [rad/s]

    % 아래는 SS_topology_design.m 에 명시되지 않아 기존 값을 유지
    uab = 850;             % [V_rms] 2차측 구형파 레벨값

    % 해석할 고조파 차수 및 delta sweep 범위
    if ~exist('m_list', 'var'), m_list = 1:2:101; end
    if ~exist('delta_list', 'var'), delta_list = linspace(0, 0.8, 41); end

    % 3D 스펙트럼에서 그래프로 보여줄 최대 고조파 차수 한계
    max_m_plot = 21;

    % 시간 평면 해상도: 속도 최적화를 위해 40000 -> 4000으로 축소 (결과 오차 미미함)
    Ntheta = 4000;
    theta = linspace(0, 2*pi, Ntheta);

    % 기존 파일의 4-step Grid Search 범위
    phi_search_ranges = [ ...
        88, 100, 721; ...
        0,  1,   721; ...
        0,  0.1, 721; ...
        0,  0.01, 721 ...
    ];

    zc_half_window_deg = 8;
    crossing_type = 'falling';
    use_parallel = true;

    % 워커개수(병렬)로 명시적 설정
    parnum=10;
    if use_parallel
        poolobj = gcp('nocreate');
        if isempty(poolobj)
            parpool(parnum);
        elseif poolobj.NumWorkers ~= parnum
            delete(gcp('nocreate'));
            parpool(parnum);
        end
    end

    %% =========================
    % Sweep 시작
    % ==========================
    phi_best_list = nan(size(delta_list));
    err_best_list = nan(size(delta_list));
    Is_rms_list   = nan(size(delta_list));
    Is_fund_rms_list = nan(size(delta_list));
    Ism_mag_all   = zeros(length(m_list), length(delta_list)); % 3D 스펙트럼용 모든 고조파 크기

    fprintf('=== Delta Sweep 시작: 총 %d 포인트 ===\n', length(delta_list));
    tic; % 경과시간 측정

    for d_idx = 1:length(delta_list)
        delta = delta_list(d_idx);
        fprintf('Processing delta = %.4f (%2d/%d)... ', delta, d_idx, length(delta_list));
        
        phi_center = nan;

        % 4단계 Grid 단계적 좁히기
        for step = 1:size(phi_search_ranges,1)
            
            % Step 1은 기존 고정 범위 사용, 이후 Step은 이전 범위 중앙 기준
            if step == 1
                phi_min = phi_search_ranges(step,1);
                phi_max = phi_search_ranges(step,2);
                nphi    = phi_search_ranges(step,3);
                % --- 연속성 최적화 옵션 ---
                % 만약 첫 스텝부터 88~100 사이가 아니라서 해를 놓치는 것을 막으려면,
                % 두 번째 delta부터는 phi_min, max를 이전 최적해 주변으로 묶어줄 수도 있습니다.
                % 하지만 현재 코드에서는 "기존 방법 그대로 차용"하기로 했으므로, 88~100을 유지합니다.
                phi_grid_deg = linspace(phi_min, phi_max, nphi);
            else
                half_range = phi_search_ranges(step,2);
                nphi       = phi_search_ranges(step,3);
                phi_grid_deg = linspace(phi_center - half_range, phi_center + half_range, nphi);
            end

            err_vec = inf(size(phi_grid_deg));
            zc_vec_deg = nan(size(phi_grid_deg));

            % 병렬 연산 (parfor)
            if use_parallel
                parfor i = 1:length(phi_grid_deg)
                    p_deg = phi_grid_deg(i);
                    p_rad = deg2rad(p_deg);

                    % [~, ... ] 형태로 7개 출력을 받도록 설정
                    [is_total, ~, ~, ~, ~, ~, ~] = ...
                        reconstruct_is_waveform(p_rad, delta, UAB, uab, omega, L, k, Lp, Ls, m_list, theta);

                    target_deg = 180 - p_deg;
                    zc_window_deg = [target_deg - zc_half_window_deg, target_deg + zc_half_window_deg];
                    [zc_deg, ok] = find_relevant_zero_crossing(theta, is_total, zc_window_deg, crossing_type);

                    if ok
                        err_vec(i) = abs(zc_deg - target_deg);
                        zc_vec_deg(i) = zc_deg;
                    end
                end
            end
            
            % 현재 스텝에서 가장 오차가 적은 곳을 다음 스텝의 센터로 설정
            [err_best, idx_best] = min(err_vec);
            phi_center = phi_grid_deg(idx_best);
        end % end of step

        % 최종 찾은 최적해 저장
        phi_best_list(d_idx) = phi_center;
        err_best_list(d_idx) = err_best;
        
        % 최적해에 대해 전류 크기 얻기 위한 1회 추가 연산
        [is_total, ~, ~, ~, Ism_mag, Ism_ang, ~] = ...
            reconstruct_is_waveform(deg2rad(phi_center), delta, UAB, uab, omega, L, k, Lp, Ls, m_list, theta);
        
        Is_rms_list(d_idx) = rms(is_total);
        
        % m=1 인 기본파 RMS 크기
        idx1 = find(m_list == 1, 1);
        if ~isempty(idx1)
            Is_fund_rms_list(d_idx) = Ism_mag(idx1);
        end
        
        % 3D 플롯용 전체 고조파 저장
        Ism_mag_all(:, d_idx) = Ism_mag(:);
        Ism_ang_all(:, d_idx) = Ism_ang(:);
        
        fprintf('Best phi = %8.4f deg | Error = %8.2e deg | Is_RMS = %.2f A\n', ...
            phi_center, err_best, Is_rms_list(d_idx));
    end
    
    elapsed = toc;
    fprintf('=== Sweep 종료 (소요 시간: %.1f 초) ===\n', elapsed);

    %% =========================
    % 결과 Plot
    % ==========================
    figure('Name', 'SS Topology Delta Sweep Result', 'Color', 'w', 'Position', [100 100 1000 600]);
    
    % [1] Delta vs Phi
    subplot(3,1,1);
    plot(delta_list, phi_best_list, '-o', 'LineWidth', 1.8, 'MarkerSize', 4, 'MarkerFaceColor', 'b');
    grid on;
    ylabel('Best \phi [deg]');
    title('Relative Phase \phi vs Detuning \delta');
    
    % [2] Delta vs Current Magnitude
    subplot(3,1,2);
    plot(delta_list, Is_rms_list, '-o', 'LineWidth', 1.8, 'MarkerSize', 4, 'DisplayName', 'Total RMS');
    hold on;
    plot(delta_list, Is_fund_rms_list, '-x', 'LineWidth', 1.5, 'MarkerSize', 4, 'DisplayName', 'Fundamental RMS');
    grid on;
    ylabel('Current [A_{rms}]');
    title('Secondary Current Magnitude vs Detuning \delta');
    legend('Location', 'best');
    
    % [3] Delta vs Search Error
    subplot(3,1,3);
    plot(delta_list, err_best_list, '-s', 'LineWidth', 1.5, 'MarkerSize', 4, 'Color', 'r');
    grid on;
    xlabel('Detuning Parameter \delta');
    ylabel('Mismatch Error [deg]');
    title('Zero-Crossing Target Mismatch Error vs Detuning \delta');
    
    % [4] 3D Frequency Spectrum Plot
    figure('Name', '3D Frequency Spectrum', 'Color', 'w', 'Position', [150 150 800 600]);
    
    % 최대 고조파수 한계까지만 데이터 자르기
    plot_idx = m_list <= max_m_plot;
    M_list_plot = m_list(plot_idx);
    Ism_mag_plot = Ism_mag_all(plot_idx, :);
    
    [DeltaGrid, MGrid] = meshgrid(delta_list, M_list_plot);
    surf(DeltaGrid, MGrid, Ism_mag_plot, 'FaceAlpha', 0.85, 'EdgeColor', 'interp');
    grid on;
    xlabel('Detuning \delta');
    ylabel('Harmonic Order m');
    zlabel('Current Magnitude |I_{sm}| [A]');
    title(sprintf('3D Frequency Spectrum vs Detuning \\delta (Up to m=%d)', max_m_plot));
    view(-45, 30);
    
% end % 함수 본문 종결 제거

%% ========================================================================
%  아래 보조 함수들은 SS_find_phi_real_260407.m 에서 온전히 재사용된 함수들입니다.
% ========================================================================

function [is_total, is_fund, is_higher, Ism, Ism_mag, Ism_ang, harm_waves] = ...
    reconstruct_is_waveform(phi, delta, UAB, uab, omega, L, k, Lp, Ls, m_list, theta)

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
