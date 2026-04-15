%% 준비
mdl = 'WPTmodel';          % 너의 Simulink 모델명
fund = 85e3;                 % 기본 주파수 (예시 85 kHz)
settleTime = 0.02;           % 과도 버릴 시간
stopTime   = 0.04;           % 전체 시뮬시간
Fs = 2e6;          % THD 계산용 샘플 주파수(로깅을 균일 샘플로)
stept=5e-8;

freq = 85*10^3;
w = 2*pi*freq;
w2 = w^2;
U_DC = 380;
U_AB = sqrt(8)/pi*U_DC;
U_L = 380;
U_ab = sqrt(8)/pi*U_L;


R =43.75;

Lp0 = 211.443*10^-6;
Ls0 = 155.310*10^-6;
M0 = sqrt((Lp0 - 207.885e-6)*Ls0);
k0 = M0 / sqrt(Lp0*Ls0);
Ls_val=Ls0*1e6; Lp_val=Lp0*1e6; M_val=M0*1e6;

a = 0.043498938;

% L_in = U_AB*M0/U_ab;
% C_p = 1/(w2*L_in);
% C_f = 1/(w2*(Lp0-L_in));
% C_s = 1/(w2*Ls0);

L_in=26.33*1e-6;
C_p=133.20*1e-9;
C_s=35.06*1e-9;
C_f=12.81*1e-9;

Gv = M0/L_in;

%% 3D 배열 로드
S = load('wpt_lut.mat');   % 변수: Lp, Ls, Lp_short, M, k (10x10x3)
Lp = S.Lp; Ls = S.Ls; Lp_short = S.Lp_short; M = S.M; k = S.k;

[nx, ny, nz] = size(Lp);
assert(all(size(Ls) == [nx,ny,nz]) && all(size(Lp_short) == [nx,ny,nz]) ...
    && all(size(M) == [nx,ny,nz]) && all(size(k) == [nx,ny,nz]), '사이즈 불일치');

%% (x,y,z) → 파라미터 리스트 펼치기
param = struct('x',{},'y',{},'z',{},'Ls',{},'Lp',{},'Lp_short',{},'M',{},'k',{});
for zz = 1:nz
    for xx = 1:nx
        for yy = 1:ny
            if any(isnan([Ls(xx,yy,zz), Lp(xx,yy,zz), Lp_short(xx,yy,zz), M(xx,yy,zz), k(xx,yy,zz)]))
                continue;
            end
            param(end+1) = struct( ...
                'x', xx-1, 'y', yy-1, 'z', zz-1, ...
                'Ls', Ls(xx,yy,zz), ...
                'Lp', Lp(xx,yy,zz), ...
                'Lp_short', Lp_short(xx,yy,zz), ...
                'M',  M(xx,yy,zz), ...
                'k',  k(xx,yy,zz));
        end
    end
end  
N = numel(param);
fprintf('총 %d개 시뮬레이션\n', N);

%% 모델 세팅
load_system(mdl);
set_param(mdl, 'StopTime', num2str(stopTime));
set_param(mdl, 'FastRestart', 'off');

%% SimulationInput 만들기
simIn(N,1) = Simulink.SimulationInput(mdl);
for i = 1:N
    p = param(i);
    simIn(i) = Simulink.SimulationInput(mdl) ...
        .setVariable('Ls_val', p.Ls) ...
        .setVariable('Lp_val', p.Lp) ...
        .setVariable('Lp_short_val', p.Lp_short) ...
        .setVariable('M_val',  p.M) ...
        .setVariable('k_val',  p.k);
end

%% 병렬 실행
out = parsim(simIn, ...
    'UseFastRestart', 'off', ...
    'TransferBaseWorkspaceVariables', 'on', ...
    'ShowProgress', 'on');

%% 후처리 (효율, THD, 리플)
eff  = nan(N,1);
thdP = nan(N,1);
rpp  = nan(N,1);
piav = nan(N,1);
poav = nan(N,1);

for i = 1:N
    if out(i).ErrorMessage ~= ""
        warning('실패 %d: %s', i, out(i).ErrorMessage);
        continue;
    end

    logs = out(i).logsout;

    % Pin/Pout 평균으로 효율
    Pin  = logs.get('P_in').Values;
    Pout = logs.get('P_out').Values;
    idx = Pin.Time >= settleTime;
    piav(i) = mean(Pout.Data(idx));
    poav(i) = mean(Pin.Data(idx));
    eff(i) = mean(Pout.Data(idx)) / max(mean(Pin.Data(idx)), eps);

    % THD (i_ac 기준)
    i_ac = logs.get('i_ac').Values;
    idxI = i_ac.Time >= settleTime;
    i_sig = i_ac.Data(idxI);
    t_sig = i_ac.Time(idxI);
    % (교체) 불균일이면 먼저 균일화 → FFT 기반 THD 계산
    if numel(t_sig) ~= numel(unique(t_sig)) || any(diff(t_sig) <= 0) || std(diff(t_sig)) > 1e-12
        t0=t_sig(1); t1=t_sig(end);
        t_u = (t0:1/Fs:t1).';
        i_u = interp1(t_sig, i_sig, t_u, 'linear', 'extrap');
    else
        % 이미 균일이면 그리 써
        i_u = i_sig;
    end
    thdP(i) = thd_fft(i_u, Fs, fund, 15, 1);   % (% 단위)


    % 리플 (v_dc Pk-Pk)
    vdc = logs.get('v_dc').Values;
    idxV = vdc.Time >= settleTime;
    seg = vdc.Data(idxV);
    rpp(i) = max(seg) - min(seg);
end

T = table( ...
    [param.z].', [param.x].', [param.y].', ...
    [param.Ls].', [param.Lp].', [param.Lp_short].', [param.M].', [param.k].', ...
    eff, thdP, rpp, poav, piav,...
    'VariableNames', {'z','x','y','Ls','Lp','Lp_short','M','k','Eff','THD','RippleVpp', 'PowerInputAvg', 'PowerOutputAvg'});

save('batch_results.mat','T')
fprintf('스텝타임 : %e\n ',stept)
disp(head(T))


%% thd 함수 정의


function pct = thd_fft(x, Fs, f0, maxH, rad)
% THD(%)를 FFT로 계산 (툴박스 불필요)
% x  : column vector (균일 샘플 신호; 불균일이면 보간해서 전달)
% Fs : sampling rate [Hz]
% f0 : fundamental [Hz]
% maxH : 계산할 최대 고조파 (기본 15)
% rad : 기본/고조파 근처 ±bin 합산 반경 (기본 1)

    if nargin < 4 || isempty(maxH), maxH = 15; end
    if nargin < 5 || isempty(rad),  rad  = 1;  end

    x = x(:);
    x = x - mean(x,'omitnan');             % DC 제거
    N = numel(x);
    if N < 8 || f0 <= 0 || Fs <= 0
        pct = NaN; return
    end

    % Hann 창
    n = (0:N-1)'; 
    w = 0.5 - 0.5*cos(2*pi*n/(N-1));
    xw = x .* w;

    % FFT 및 one-sided 파워 스펙트럼
    X = fft(xw);
    S = abs(X(1:floor(N/2)+1)).^2;         % power ∝ |FFT|^2
    K = numel(S)-1;                         % 최대 유효 bin index

    % 기본주파수 bin
    k0 = round(f0 * N / Fs);
    if k0 < 1 || k0 > K
        pct = NaN; return
    end

    % 경계 보정 함수
    fbin = @(k) max(min(k, K), 1);

    % 기본파 전력(누설 보정 겸 ±rad bin 합산)
    Pf = sum(S(fbin(k0-rad):fbin(k0+rad)));

    % 고조파 전력 합
    Ph = 0;
    for h = 2:maxH
        kh = round(h * k0);
        if kh > K, break; end
        Ph = Ph + sum(S(fbin(kh-rad):fbin(kh+rad)));
    end

    % THD(%)
    pct = sqrt(Ph / max(Pf, eps)) * 100;
end

%% csv로 저장
%writetable(T, '1028res.csv')

%% plot
templog1=out(1).logsout.get('P_in').Values;
templog2=out(1).logsout.get('P_out').Values;
tempt=templog1.Time;
tempidx=0.03<=tempt & tempt<=0.03005;
tempidx=0.02<=tempt & tempt<=0.04;

plot(tempt(tempidx), templog1.Data(tempidx), tempt(tempidx), templog2.Data(tempidx))