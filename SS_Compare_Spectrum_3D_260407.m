%% SS_Compare_Spectrum_3D_260407.m
% 이 스크립트는 수식(formula)과 시뮬레이션(Simulink) 환경 양쪽에서 
% delta sweep을 수행하고, 2차측 전류의 특정 고조파 크기 및 위상 기반
% 3D 주파수 스펙트럼(surf, scatter3 등)을 중첩하여 비교합니다.
%
% =========================================================================
% [버전 기록]
% v1.2 : (2026-04-08)
%        - 수식(Formula)과 시뮬레이션(Simulink)의 시간 도메인(Time Domain) 재구성 파형 비교 Figure 2개 추가
%        - 잔류 메모리 누수를 방지하기 위한 Parallel Pool 캐시 및 임시 변수 자동 정리 섹션 추가
% v1.1 : (2026-04-07 23:45:00) 삭제된 시뮬링크 호출부(오타) 임시 복구 
% v1.0 : (2026-04-07 21:30:00)
%        - 3D 주파수 스펙트럼(크기) surf 표면과 scatter3 계산점 동시 플롯 적용
%        - 기본파(m=1)를 제외한 스펙트럼 비교 Figure 추가 (더 나은 고조파 관찰)
%        - 각 고조파별 위상(Phase) 스펙트럼 비교 Figure 추가 구현
% =========================================================================

% 충돌 방지를 위해 꼭 필요한 변수만 정리합니다 (명시적 clear 제외)
% 윈도우 초기화
close all;

%% ===================== 1. 공통 파라미터 설정 =====================
% 해석할 고조파 차수 
m_list = 1:2:101; 

% Delta sweep 범위 (공통)
delta_min = 0;
delta_max = 0.5;
num_points = 11; 
delta_list = linspace(delta_min, delta_max, num_points);

% 3D 스펙트럼에서 표시할 최대 고조파 한계
max_m_plot = 21;

%% ===================== 2. 수식 모델 기반 연산 =====================
fprintf('\n\n=======================================================\n');
fprintf(' [1단계] 수학 수식(Formula) 기반 Delta Sweep 실행 중...\n');
fprintf('=======================================================\n');

% 스크립트 실행 (미리 선언한 m_list, delta_list 공유됨)
SS_sweep_delta_for_phi_formula_260407;

% 계산결과 백업
Ism_mag_formula = Ism_mag_all;

%% ===================== 3. Simulink 시뮬레이션 연산 =====================
fprintf('\n\n=======================================================\n');
fprintf(' [2단계] Simulink(parsim) 기반 Delta Sweep 실행 중...\n');
fprintf('=======================================================\n');

% 스크립트 실행 (delta_list 와 m_list 가 존재하므로 재사용됨)
SS_sweep_delta_for_phi_Sumulink_260407;
% 계산결과 백업 (SS_sweep_delta_for_phi_Sumulink_260407 내에서 생성됨)
Ism_mag_simulink = Ism_sim_mag_all;

%% ===================== 4. 통합 3D 스펙트럼 비교 플롯 (크기) =====================
fprintf('\n\n=======================================================\n');
fprintf(' [3단계] 통합 3D 주파수 스펙트럼 시각화\n');
fprintf('=======================================================\n');

figure('Name', 'Unified 3D Frequency Spectrum Comparison', 'Color', 'w', 'Position', [100 200 900 700]);

% x축과 y축 데이터 추출
plot_idx = m_list <= max_m_plot;
M_list_plot = m_list(plot_idx);

Ism_mag_formula_plot = Ism_mag_formula(plot_idx, :);
Ism_mag_simulink_plot = Ism_mag_simulink(plot_idx, :);

[DeltaGrid, MGrid] = meshgrid(delta_list, M_list_plot);

% (1) Formula 결과 (파란색 계열)
surf(DeltaGrid, MGrid, Ism_mag_formula_plot, ...
    'FaceColor', 'b', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
hold on;
h1 = scatter3(DeltaGrid(:), MGrid(:), Ism_mag_formula_plot(:), 30, 'b', 'filled');

% (2) Simulink 결과 (빨간색 계열)
surf(DeltaGrid, MGrid, Ism_mag_simulink_plot, ...
    'FaceColor', 'r', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
h2 = scatter3(DeltaGrid(:), MGrid(:), Ism_mag_simulink_plot(:), 30, 'r', 'filled');

grid on;
xlabel('Detuning \delta');
ylabel('Harmonic Order m');
zlabel('Current Magnitude |I_{sm}| [A]');
title(sprintf('Formula vs Simulink Magnitude Spectrum (Up to m=%d)', max_m_plot));
view(-45, 30);

% 레전드는 산점도(scatter3) 객체 지정
legend([h1, h2], {'Formula Points', 'Simulink Points'}, 'Location', 'best');

%% ===================== 5. 통합 3D 플롯 (m=1 제외) =====================
figure('Name', 'Spectrum Comparison Without m=1', 'Color', 'w', 'Position', [150 150 900 700]);

% m=1 제외한 인덱스
plot_idx_no_fund = (m_list > 1) & (m_list <= max_m_plot);
M_list_no_fund = m_list(plot_idx_no_fund);

Ism_mag_form_no_fund = Ism_mag_formula(plot_idx_no_fund, :);
Ism_mag_sim_no_fund = Ism_mag_simulink(plot_idx_no_fund, :);

[DeltaGrid2, MGrid2] = meshgrid(delta_list, M_list_no_fund);

% Formula 결과
surf(DeltaGrid2, MGrid2, Ism_mag_form_no_fund, ...
    'FaceColor', 'b', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
hold on;
h1_nf = scatter3(DeltaGrid2(:), MGrid2(:), Ism_mag_form_no_fund(:), 30, 'b', 'filled');

% Simulink 결과
surf(DeltaGrid2, MGrid2, Ism_mag_sim_no_fund, ...
    'FaceColor', 'r', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
h2_nf = scatter3(DeltaGrid2(:), MGrid2(:), Ism_mag_sim_no_fund(:), 30, 'r', 'filled');

grid on;
xlabel('Detuning \delta');
ylabel('Harmonic Order m');
zlabel('Current Magnitude |I_{sm}| [A]');
title(sprintf('Formula vs Simulink Magnitude Spectrum (Excluding m=1, Up to m=%d)', max_m_plot));
view(-45, 30);
legend([h1_nf, h2_nf], {'Formula Points', 'Simulink Points'}, 'Location', 'best');

%% ===================== 6. 통합 3D 플롯 (위상) =====================
figure('Name', 'Phase Spectrum Comparison', 'Color', 'w', 'Position', [200 100 900 700]);

% 수식 및 시뮬레이션에서 위상 정보 가져오기 (단위 변환: rad -> deg)
% 스크립트 실행 후 Ism_ang_all 과 Ism_sim_ang_all 이 작업공간에 남아있음
if exist('Ism_ang_all', 'var') && exist('Ism_sim_ang_all', 'var')
    Ism_ang_formula_plot = rad2deg(Ism_ang_all(plot_idx, 1:length(delta_list)));
    Ism_ang_simulink_plot = rad2deg(Ism_sim_ang_all(plot_idx, 1:length(delta_list)));
    
    % 위상은 wrapTo180으로 -180 ~ 180도 범위로 표현하는 것이 일반적
    Ism_ang_formula_plot = wrapTo180(Ism_ang_formula_plot);
    Ism_ang_simulink_plot = wrapTo180(Ism_ang_simulink_plot);

    surf(DeltaGrid, MGrid, Ism_ang_formula_plot, ...
        'FaceColor', 'b', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
    hold on;
    h1_ph = scatter3(DeltaGrid(:), MGrid(:), Ism_ang_formula_plot(:), 30, 'b', 'filled');

    surf(DeltaGrid, MGrid, Ism_ang_simulink_plot, ...
        'FaceColor', 'r', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
    h2_ph = scatter3(DeltaGrid(:), MGrid(:), Ism_ang_simulink_plot(:), 30, 'r', 'filled');

    grid on;
    xlabel('Detuning \delta');
    ylabel('Harmonic Order m');
    zlabel('Phase \angleI_{sm} [deg]');
    title(sprintf('Formula vs Simulink Phase Spectrum (Up to m=%d)', max_m_plot));
    view(-45, 30);
    legend([h1_ph, h2_ph], {'Formula Phase', 'Simulink Phase'}, 'Location', 'best');
else
    disp('위상 데이터(Ism_ang_all, Ism_sim_ang_all)를 찾을 수 없습니다.');
end

%% ===================== 7. 시간 함수(Time Domain) 재구성 플롯 (Formula) =====================
% 특정 delta 값 (중간 값)을 선택하여 시간에 따른 파형 관찰
idx_target = ceil(length(delta_list) / 2);
target_delta = delta_list(idx_target);

% 위상이 0인 지점을 가운데 두기 위해 -180도 ~ 180도로 시간(위상) 축 설정
theta_vec = linspace(-pi, pi, 1000);
theta_deg = rad2deg(theta_vec);

mag_f = Ism_mag_formula(:, idx_target);
ang_f = Ism_ang_all(:, idx_target);

is_fund_f = zeros(size(theta_vec));
is_higher_f = zeros(size(theta_vec));
is_total_f = zeros(size(theta_vec));

for m_i = 1:length(m_list)
    m = m_list(m_i);
    % 수식 기반 위상 재구성 공식 적용 (sin 기준)
    i_m = sqrt(2) * mag_f(m_i) .* sin(m*(theta_vec + pi/2) + ang_f(m_i) - pi/2);
    
    is_total_f = is_total_f + i_m;
    if m == 1
        is_fund_f = i_m;
    else
        is_higher_f = is_higher_f + i_m;
    end
end

figure('Name', 'Formula Time Domain Waveform', 'Color', 'w', 'Position', [250 250 800 500]);
plot(theta_deg, is_total_f, 'k', 'LineWidth', 2); hold on;
plot(theta_deg, is_fund_f, 'b--', 'LineWidth', 1.5);
plot(theta_deg, is_higher_f, 'r-.', 'LineWidth', 1.5);

xline(0, 'k--', 'LineWidth', 1.2); % 위상이 0도가 되는 곳 (수직 점선)
yline(0, 'k-', 'LineWidth', 1.2);  % y=0 수평 실선

grid on;
legend('Total Current (All Harmonics)', 'Fundamental (m=1)', 'Higher Harmonics (m \geq 3)', ...
       'Phase = 0\circ', 'y = 0', 'Location', 'best');
xlabel('Phase \theta [deg]');
ylabel('Current [A]');
title(sprintf('Formula Reconstructed Time Waveform (\\delta = %.4f)', target_delta));


%% ===================== 8. 시간 함수(Time Domain) 재구성 플롯 (Simulink) =====================
mag_s = Ism_mag_simulink(:, idx_target);
ang_s = Ism_sim_ang_all(:, idx_target);

is_fund_s = zeros(size(theta_vec));
is_higher_s = zeros(size(theta_vec));
is_total_s = zeros(size(theta_vec));

for m_i = 1:length(m_list)
    m = m_list(m_i);
    % Simulink는 exp(-j*w*t) 로 페이저를 추출했으므로 수학적 본질은 코사인 기준.
    % 즉 파형은 sqrt(2)*|X|*cos(m*wt + ang) 형태로 복원됨.
    % 사용자의 기호(sin 기준)에 맞추어 cos(x) = sin(x + pi/2) 를 이용해 sin으로 표현.
    i_m = sqrt(2) * mag_s(m_i) .* sin(m*theta_vec + ang_s(m_i) + pi/2);
    
    is_total_s = is_total_s + i_m;
    if m == 1
        is_fund_s = i_m;
    else
        is_higher_s = is_higher_s + i_m;
    end
end

figure('Name', 'Simulink Time Domain Waveform', 'Color', 'w', 'Position', [300 300 800 500]);
plot(theta_deg, is_total_s, 'k', 'LineWidth', 2); hold on;
plot(theta_deg, is_fund_s, 'b--', 'LineWidth', 1.5);
plot(theta_deg, is_higher_s, 'r-.', 'LineWidth', 1.5);

xline(0, 'k--', 'LineWidth', 1.2); % 위상이 0도가 되는 곳 (수직 점선)
yline(0, 'k-', 'LineWidth', 1.2);  % y=0 수평 실선

grid on;
legend('Total Current (All Harmonics)', 'Fundamental (m=1)', 'Higher Harmonics (m \geq 3)', ...
       'Phase = 0\circ', 'y = 0', 'Location', 'best');
xlabel('Phase \theta [deg]');
ylabel('Current [A]');
title(sprintf('Simulink Reconstructed Time Waveform (\\delta = %.4f)', target_delta));

fprintf('모든 통합 분석 절차가 완료되었습니다.\n\n');

%% ===================== 7. 메모리 및 캐시 완전 정리 =====================
fprintf('=======================================================\n');
fprintf(' [마무리] 백그라운드 메모리 및 캐시 정리 중...\n');
fprintf('=======================================================\n');

% 1. Simulink Data Inspector 캐시 비우기 (메모리 가장 많이 차지하는 원인 중 하나)
try
    Simulink.sdi.clear;
    Simulink.sdi.clearPreferences;
catch
    % Simulink를 로드하지 못한 환경일 경우 패스
end

% 2. 사용하지 않는 로드된 모델 정리
try
    bdclose('all');
catch
end

% 3. 병렬 풀(Parallel Pool)에 남아있는 워커 캐시 및 워커 프로세스 강제 종료
% 여러 스크립트 실행 후 풀이 남아있을 경우 한 코어당 메모리(약 1~2GB)가 끝까지 점유될 수 있음.
poolobj = gcp('nocreate');
if ~isempty(poolobj)
    fprintf('활성화된 병렬 풀(Workers: %d)을 발견. 풀을 종료하여 코어 메모리를 반환합니다...\n', poolobj.NumWorkers);
    delete(poolobj);
else
    fprintf('현재 활성화된 병렬 풀이 없어 추가적인 풀 종료는 생략합니다.\n');
end

% 4. 사용하지 않는 큰 임시 변수 정리 (Workspace 정리)
% 플롯에 사용한 핵심 결과 데이터 6가지는 남겨두고, 자잘한 중간 변수는 메모리에서 제거
keep_vars = {'Ism_mag_formula', 'Ism_mag_simulink', 'Ism_ang_all', 'Ism_sim_ang_all', 'm_list', 'delta_list', 'max_m_plot'};
% 남겨야 할 변수 제외하고 모두 지우는 코드 
clearvars -except Ism_mag_formula Ism_mag_simulink Ism_ang_all Ism_sim_ang_all m_list delta_list max_m_plot;

fprintf('잔류 코어 메모리 반환 및 캐시 정리가 완료되었습니다. (메모리 절약 최적화 완료)\n');

