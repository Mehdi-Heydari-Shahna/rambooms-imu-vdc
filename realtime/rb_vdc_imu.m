function [u, q_ref, qd_ref, info, tune, vdc, imu] = rb_vdc_imu(q_meas, enable, acc1, gyr1, acc2, gyr2, acc3, gyr3)
%RB_VDC_IMU Auto-tuned VDC with built-in motion sequence for the Rambooms boom, run either with
%   the joint angle sensors (enable = 1, IMU observer in shadow) or with the IMU observer
%   instead of the joint angle sensors (enable = 2). Simulink MATLAB Function block, 1 ms.
%
%   Inputs
%     q_meas   joint angles [deg] 4x1: slew, lift, tilt, crowd (joint_angles_deg)
%     enable   0 = stop: valve commands 0, the run is paused
%              1 = run with the joint angle sensors; the IMU observer runs in parallel (shadow)
%              2 = run with the IMU observer; q_meas is not used
%              The same value after 0 continues the paused run. The other value, or any
%              value after a completed run (info(1) = 4) and 0, starts a new run with tuning.
%              0 acts at once, 1 and 2 after 20 ms.
%     acc1, acc2, acc3   accelerometers of IMU 1 (lift link), 2 (dipper), 3 (tool link):
%              EthercatIMUx_acc [m/s^2] 3x1
%     gyr1, gyr2, gyr3   gyros of IMU 1-3: EthercatIMUx_gyro 3x1 (unit set by C.ob_gunit)
%   Outputs
%     u        valve commands 4x1 for the valve channels 1-4 (slew, lift, tilt, crowd valve)
%     q_ref    reference joint angles [deg] 4x1 (mode 2: slew relative to the IMU slew angle)
%     qd_ref   reference joint rates [deg/s] 4x1
%     info     10x1: phase (0 wait, 1 tuning, 2 check, 3 motion, 4 done, 5 no valve moved a
%              joint), step, valve/joint/boulder, round, speed factor, segment time [s], wait
%              time [s], max error in segment [deg], angle update interval [ms], warnings
%     tune     40x1, per joint: valve channel, sign, gain + and -, dead zone + and -, loop
%              delay [s], lambda [1/s], gain factor + and - after adaptation
%     vdc      14x1: required joint rates [deg/s] (4), control error [deg] (4), required
%              piston rates [mm/s] (3), required cylinder forces [kN] (3)
%     imu      14x1: IMU observer: slew angle [deg] (from the gyros, 0 at start-up), lift,
%              tilt, crowd angle [deg], slew, lift, tilt, crowd rate [deg/s], weights of the
%              three joint measurements (0.02 rejected .. 1 full), status (0 ok, +1 not
%              initialized, +2 invalid IMU sample: not finite or values frozen, +8 slew gyro
%              bias not estimated yet), mode of the present run (1, 2), time the boom has been
%              still [s]
%
%   Start: pump on, boom near the home pose with the hammer off the ground, room for about
%   +-10 deg on every joint and +-20 deg of slew. Keep the boom still for 5 s before enable:
%   the gyro bias of the slew rate is estimated at rest (mode 2 starts after about 4 s of
%   rest). In mode 2 the slew angle is held while the slew valve is closed and the slew rests.
%   Sequence: auto-tuning one valve at a time, closed-loop check, motion of the rock-breaking
%   task without stones (home, 4 boulders x [move, straight descent, press, hold, retract],
%   2 rounds), back home and hold. Mode 2 uses the same tuning, check and motion.
%   Warnings (bits): 1 valve without response, 2 valve/joint order differs, 4 target not
%   reached in time, 8 invalid joint angle (mode 2: invalid IMU sample), 16 lambda reduced in
%   check, 32 step test skipped, 64 lambda reduced after an oscillation.

persistent C S O
if isempty(C)
    C = rb_cfg();
    S = rb_init(C);
    O = obs_init();
end

% IMU observer: always running (shadow in mode 1, feedback in mode 2)
O = obs_step(O, C, acc1, gyr1, acc2, gyr2, acc3, gyr3, all(S.uout == 0), S.uout(S.map(1)) == 0);

% requested mode: 0 stop (at once), 1 joint angles, 2 IMU observer (1 and 2 are accepted
% when the value has been the same for en_t)
en = double(enable);
r = 0;
if en > 1.5
    r = 2;
elseif en > 0.5
    r = 1;
end
if r ~= S.rq_new
    S.rq_new = r;
    S.rq_t = 0;
else
    S.rq_t = S.rq_t + C.dt;
end
if r == 0
    S.req = 0;
elseif S.rq_t >= C.en_t
    S.req = r;
end
req = S.req;
if req == 0
    S.off_t = S.off_t + C.dt;
else
    S.off_t = 0;
end
% a new run (with tuning) starts when the other mode is requested, or after a completed
% run when enable was 0 in between
if S.phase > 0 && req > 0 && (req ~= S.mode || S.rerun)
    S = rb_new(S, C);
end
if S.phase == 4 && S.off_t >= C.en_off
    S.rerun = true;
end

% joint angles of the controller: joint angle sensors (mode 1) or IMU observer (mode 2)
src = S.src;
if S.phase == 0 && req > 0
    src = req;
end
if src ~= S.src
    % other source of the angles: nothing learned from the previous source is kept
    S = rb_new(S, C);
    S.src = src;
end
qin = [double(q_meas(1)); double(q_meas(2)); double(q_meas(3)); double(q_meas(4))];
okin = true;
rdy = true;
if src == 2
    if C.slew_imu
        qin(1) = O.slew;
    end
    qin(2) = O.q(1);
    qin(3) = O.q(2);
    qin(4) = O.q(3);
    okin = O.valid;
    rdy = O.ready || ~C.slew_imu;
end
S = rb_sense(S, C, qin, okin);
if src == 2
    % joint rates from the bias-corrected gyro rates (no differentiation of the angles), with
    % the same low-pass as the rates from the joint angles
    a = C.dt / (C.vtau + C.dt);
    if C.slew_imu
        S.vi(1) = S.vi(1) + a * (O.slewd - S.vi(1));
        S.v(1) = S.vi(1);
        S.vdel(1) = C.vdel_imu;
    end
    for j = 2:4
        S.vi(j) = S.vi(j) + a * (O.qd(j - 1) - S.vi(j));
        S.v(j) = S.vi(j);
        S.vdel(j) = C.vdel_imu;
    end
end
for j = 1:4
    S.vs(j) = S.vs(j) + C.dt / (C.vs_tau + C.dt) * (S.v(j) - S.vs(j));
end

% run: joint angles valid for 0.5 s at the start (mode 2: also after a sensor fault)
go = S.vt >= 0.5 || (S.phase > 0 && (src == 1 || ~S.paused));
if req > 0 && rdy && S.valid && S.phase < 5 && go
    if S.paused
        S = rb_resume(S, C);
    end
    if S.phase == 0
        S.mode = req;
        S = rb_start(S, C);
    end
    if S.phase == 1
        S = rb_tune(S, C);
    else
        S = rb_plan(S, C);
        S = rb_control(S, C);
    end
    S = rb_out(S, C, true);
else
    if S.phase == 5 && req == 0
        S.phase = 0;
        S.warn = 0;
    elseif S.phase > 0
        S.paused = true;
    end
    if S.phase == 0
        S.qd = S.q;
    end
    S = rb_out(S, C, false);
end
S = rb_vdc(S, C);

u = S.uout;
q_ref = S.qd;
qd_ref = S.vd;
info = [S.phase; S.stepcode; S.unit; S.tk_r; S.speed; S.sg_t; S.gate_t; S.emax; ...
    max(S.upd) * C.dt * 1000; S.warn];
tune = zeros(40, 1);
for j = 1:4
    i0 = (j - 1) * 10;
    tune(i0 + 1) = S.map(j);
    tune(i0 + 2) = S.sgn(j);
    tune(i0 + 3) = S.Kp(j);
    tune(i0 + 4) = S.Kn(j);
    tune(i0 + 5) = S.dbp(j);
    tune(i0 + 6) = S.dbn(j);
    tune(i0 + 7) = S.th(j);
    tune(i0 + 8) = S.lam(j);
    tune(i0 + 9) = S.Kp(j) / max(S.Kp0(j), 1e-9);
    tune(i0 + 10) = S.Kn(j) / max(S.Kn0(j), 1e-9);
end
vdc = [S.qr_dot; S.err; S.xr * 1000; S.fr / 1000];
imu = [O.slew; O.q; O.slewd; O.qd; O.wt; O.status; S.mode; O.still_t];
end


function C = rb_cfg()
% Machine-specific values (link geometry and masses, IMU installation and calibration, home
% pose, joint limits of the planner and strike points) are read from rb_params.m, which is not
% part of the repository: create it from rb_params_template.m.
P = rb_params();
% ---------------------------------------------------------------- settings
umax = 1.0;             % valve command range: commands are within +-umax
dt = 0.001;             % sample time [s]
nround = 2;             % rounds of the boulder sequence
s0 = 0.5;               % speed factor of the first moves (1 = full speed)
sstep = 0.15;           % speed factor increase per boulder
hgt = [0.55, 0.10, -0.06, 0.70];    % approach, end of descent, press, retract above top [m]
vmax = [6; 4; 4; 6];                % joint speed at full speed factor [deg/s]
amax = [5; 3; 3; 5];                % joint acceleration at full speed factor [deg/s^2]

C.umax = umax;
C.ulim = 0.8 * umax;                % largest command used
C.du_max = 5 * umax;                % command rate limit [1/s]
C.dt = dt;
C.nround = nround;
C.s0 = s0;
C.sstep = sstep;
C.home = P.home;                    % home pose (rb_params)
C.phi = P.phi;                      % hammer axis angle during the boulder sequence (rb_params)
C.bld = P.bld;                      % strike points (rb_params)
C.hgt = hgt;
C.lim_lo = P.lim_lo;                % joint limits used by the planner (rb_params)
C.lim_hi = P.lim_hi;
C.vmax = vmax;
C.amax = amax;
C.vtip = 0.15;                      % tip speed of the straight descent [m/s]
C.vpress = 0.05;                    % tip speed while pressing [m/s]
C.tmin = 2.0;                       % shortest move [s]
C.thold = 3.0;                      % hold at the press point [s]

% auto-tuning
C.order = [4; 3; 2; 1];             % tuning order of the valves: crowd, tilt, lift, slew
C.id_settle = 1.0;                  % still time before the tests of a valve [s]
C.id_rate1 = 0.05 * umax;           % slow opening until the joint moves [1/s]
C.id_rate2 = 0.15 * umax;           % opening rate after the joint moves [1/s]
C.id_v1 = 2.0;                      % speed at the end of the slow opening [deg/s]
C.id_dq1 = 3.0;                     % travel limit of the slow opening [deg]
C.id_dq2 = 6.0;                     % travel of the steps [deg]
C.id_trise = 0.5;                   % rise time of the steps [s]
C.id_tstep = 3.0;                   % longest step [s]
C.id_tdown = 0.6;                   % valve closing time after a test [s]
C.id_tstill = 0.5;                  % still time required after a test [s]
C.id_tstill_max = 4.0;
C.id_th0 = 0.10;                    % delay assumed for the slow-opening fit [s]
C.id_tdet = 0.03;                   % motion must be seen this long to assign the joint [s]
C.nrec = max(1, round(0.02 / dt));  % slow-opening record interval [samples]
C.nsrec = max(1, round(0.005 / dt)); % step record interval [samples]
C.ngth = 41;                        % delay grid: 0 ... 0.40 s
C.ngT = 13;                         % lag grid: 0 ... 0.30 s

% controller
C.lam_min = 0.7;                    % limits of the position gain lambda [1/s]
C.lam_max = 2.5;
C.e_in = 0.12;                      % position hold: enter below this error [deg]
C.e_out = 0.35;                     % position hold: leave above this error [deg]
C.vblend = 0.3;                     % dead-zone compensation blend at rest [deg/s]
C.vff = 0.1;                        % planned rate above which the dead zone follows the plan [deg/s]
C.gate_tol = 0.4;                   % target reached: error [deg]
C.gate_time = 0.3;                  % target reached: time [s]
C.gate_t1 = 6.0;                    % after this time the tolerance is relaxed [s]
C.gate_t2 = 15.0;                   % after this time the sequence continues [s]
C.osc_band = 0.04;                  % oscillation detection: error band [deg]
C.osc_win = 5.0;                    % oscillation detection: window [s] (6 sign changes)
C.adapt_rate = 0.3;                 % valve gain adaptation rate [1/s]
C.adapt_lo = 0.6;                   % adaptation range relative to the identified gain
C.adapt_hi = 1.6;
C.db_rate = 0.1 * umax;             % dead-zone increase while a joint is stuck [1/s]
C.db_learn = 0.05 * umax;           % dead-zone learning in slow moves [1/(deg s)]

% signal processing (history lengths: see rb_init)
C.vwin = max(2, round(0.05 / dt));  % velocity window [samples]
C.vtau = 0.02;                      % velocity filter time constant [s]

% link geometry (joint-angle convention of q_meas) and link masses (rb_params)
C.J1 = P.J1;
C.L1 = P.L1;
C.L2 = P.L2;
C.L3 = P.L3;
C.dl = P.dl;
C.cyl = P.cyl;                      % per cylinder (lift, tilt, crowd): c_x, c_y, r_x, r_y [m]
C.mass = P.mass;                    % per link (lift, dipper, tool): m [kg], c_s, c_n [m], I_c [kg m^2]
C.llen = [P.L1, P.L2, 0];
C.g0 = 9.81;
C.kv = 2.0;                         % VDC velocity feedback of the required forces [1/s]

% robustness to sensor noise and progressive valves
C.osc_tau = 0.05;                   % error low-pass for the oscillation detection [s]
C.vs_tau = 0.1;                     % velocity low-pass for the stuck detection [s]
C.id_du = 0.1 * umax;               % step command at most this much above the end of the slow opening
C.db_up = 0.15 * umax;              % dead-zone learning: upper limit at least identified + this

% enable input
C.en_t = 0.02;                      % enable 1 or 2 accepted after this time [s]
C.en_off = 0.1;                     % after a completed run, enable 0 for this time -> new run [s]

% mode 2 (IMU observer)
C.slew_imu = true;                  % slew angle from the IMU gyros (false: slew joint angle sensor)
C.vdel_imu = 0.025;                 % delay of the low-passed gyro rates [s] (0.005 + vtau)

% IMU observer (IMU 1 lift link, IMU 2 dipper, IMU 3 tool link); installation and calibration
% of the IMUs from rb_params
fk = tan(pi * 20 / (1 / dt));       % 2nd-order Butterworth low-pass, 20 Hz
qk = 1 / sqrt(2);
nk = 1 / (1 + fk / qk + fk * fk);
C.ob_lpf = [fk * fk * nk, 2 * fk * fk * nk, fk * fk * nk, 2 * (fk * fk - 1) * nk, (1 - fk / qk + fk * fk) * nk];
C.ob_kP = 0.8;                      % observer gains
C.ob_kI = 0.01;
C.ob_c = 0.5;                       % residual scale of the weight [m/s^2]
C.ob_wmin = 0.02;                   % minimum weight
C.ob_L = [P.L1, P.L2];              % lift and dipper length, pin to pin [m]
C.ob_C1 = P.ob_C1;                  % IMU axes to boom-plane axes
C.ob_C2 = P.ob_C2;
C.ob_C3 = P.ob_C3;
C.ob_gunit = P.ob_gunit;            % gyro inputs x ob_gunit = deg/s
C.ob_s = P.ob_s;                    % IMU position along the link from the proximal pin [m]
C.ob_n = P.ob_n;                    % IMU position normal to the link [m]
C.ob_beta = P.ob_beta;              % mounting angle [rad]
C.ob_bx = P.ob_bx;                  % accelerometer bias x [m/s^2]
C.ob_by = P.ob_by;                  % accelerometer bias y [m/s^2]
C.ob_gs = P.ob_gs;                  % z gyro scale
C.ob_ga = P.ob_ga;                  % z gyro sensitivity to x rate
C.ob_gb = P.ob_gb;                  % z gyro sensitivity to y rate
C.ob_gbias = P.ob_gbias;            % z gyro bias [rad/s]
% slew rate = gyro rate about the gravity direction (IMU 1 and 2), gyro bias estimated at rest
C.ob_yw = [0.5, 0.5, 0];            % weights of the IMUs
C.ob_yc = P.ob_yc;                  % share of the link pitch rate seen in the slew rate
C.ob_tg = 1.0;                      % low-pass of the gravity direction [s]
C.ob_dw = 0.05 * pi / 180;          % still: change of the low-passed rates below this [rad/s]
C.ob_wmax = 0.3 * pi / 180;         % still: low-passed rates below this [rad/s]
C.ob_tstill = 1.0;                  % still for this long before the bias is updated [s]
C.ob_tavg = 10.0;                   % bias: plain average over the first 10 s of rest
C.ob_tb = 20.0;                     % bias: then low-pass with this time constant [s]
C.ob_tready = 3.0;                  % mode 2 starts after this much bias averaging at rest [s]
C.ob_nstale = 20;                   % IMU values unchanged for more samples: invalid (lost device)
C.ob_nsa = 50;                      % ... accelerations alone (stuck sensor)
C.ob_nsg = 100;                     % ... rates alone (at rest unchanged for up to 18 samples)
C.ob_nre = 20;                      % after more invalid samples in a row the observer restarts
end


function S = rb_init(C)
S.phase = 0;
S.paused = false;
S.valid = false;
S.vt = 0;
S.bad = 0;
S.q = zeros(4, 1);
S.qlast = zeros(4, 1);
S.chg = zeros(4, 1);
S.upd = ones(4, 1);
S.qstep = ones(4, 1);
S.qh = zeros(4, 128);               % position history [samples]
S.qi = 0;
S.v = zeros(4, 1);
S.vdel = (0.5 * C.vwin * C.dt + C.vtau) * ones(4, 1);    % delay of the joint rates [s]
S.wrap = 0;
S.slew0 = 0;
S.warn = 0;
S.stepcode = 0;
S.unit = 0;
% auto-tuning
S.id_ch = 1;
S.id_pass = 1;
S.chres = zeros(4, 1);
S.id_test = 1;
S.id_st = 1;
S.id_t = 0;
S.id_u = 0;
S.id_uend = 0;
S.id_det = false;
S.id_q0 = zeros(4, 1);
S.id_sv = zeros(4, 1);
S.id_sv2 = zeros(4, 1);
S.id_s1 = 0;
S.id_s2 = 0;
S.id_sy = zeros(4, 1);
S.id_sty = zeros(4, 1);
S.id_cand = 0;
S.id_cnt = 0;
S.id_n = 0;
S.id_resp = 0;
S.id_still = 0;
S.id_fail = false;
S.id_fail1 = false;
S.id_fail2 = false;
S.id_ub = zeros(2, 1);
S.id_dq = 0;
S.drift = zeros(4, 1);
S.vsig = 0.1 * ones(4, 1);
S.rb = zeros(10, 1000);             % slow-opening record (20 ms samples)
S.rn = zeros(2, 1);
S.sb = zeros(4, 300);               % step record (5 ms samples)
S.sn = zeros(2, 1);
S.sdir = zeros(2, 1);
S.sus = zeros(2, 1);
S.sacc = zeros(2, 2);
S.sok = true(2, 1);
S.gi = 0;
S.gbest = [1e300; 0; 0];
S.Kq = zeros(2, 1);
S.dbq = zeros(2, 1);
% identified model and gains per joint (slew, lift, tilt, crowd)
S.map = [1; 2; 3; 4];
S.sgn = ones(4, 1);
S.ok = false(4, 1);
S.Kp = zeros(4, 1);
S.Kn = zeros(4, 1);
S.Kp0 = zeros(4, 1);
S.Kn0 = zeros(4, 1);
S.dbp = zeros(4, 1);
S.dbn = zeros(4, 1);
S.dbp0 = zeros(4, 1);
S.dbn0 = zeros(4, 1);
S.th = 0.15 * ones(4, 1);
S.lam = ones(4, 1);
S.dly = 150 * ones(4, 1);
S.vlim = ones(4, 1);
% controller
S.rq = zeros(4, 640);               % reference history [samples]
S.rv = zeros(4, 640);
S.ri = 0;
S.hold = false(4, 1);
S.stuck = zeros(4, 1);
S.osc_s = zeros(4, 1);
S.osc_n = zeros(4, 1);
S.osc_t = zeros(4, 1);
S.uj = zeros(4, 1);
S.uch = zeros(4, 1);
S.uout = zeros(4, 1);
S.qr_dot = zeros(4, 1);
S.err = zeros(4, 1);
% planner
S.qd = zeros(4, 1);
S.vd = zeros(4, 1);
S.ad = zeros(4, 1);
S.sg_type = 0;
S.sg_t = 0;
S.sg_T = 0;
S.sg_qa = zeros(4, 1);
S.sg_off = zeros(4, 1);
S.sg_qb = zeros(4, 1);
S.sg_pa = zeros(2, 1);
S.sg_pb = zeros(2, 1);
S.gate_t = 0;
S.gate_ok = 0;
S.emax = 0;
S.emax_b = 0;
S.speed = C.s0;
S.ck_k = 1;
S.ck_st = 1;
S.ck_n = 0;
S.ck_dir = 1;
S.ck_mdir = 1;
S.ck_a = 0;
S.ck_os = 0;
S.ck_zc = 0;
S.ck_last = 0;
S.ck_bad = false;
S.tk_state = 0;
S.tk_r = 1;
S.tk_b = 1;
S.tk_sub = 0;
S.resume = false;
% VDC
S.fr = zeros(3, 1);
S.xr = zeros(3, 1);
% run mode, angle source, noise robustness
S.mode = 0;                         % mode of the present run: 1 joint angles, 2 IMU observer
S.src = 1;                          % angle source of the controller
S.rerun = false;                    % run completed and enable was 0: next enable starts a new run
S.vs = zeros(4, 1);                 % low-passed joint rates [deg/s]
S.ef = zeros(4, 1);                 % low-passed control error [deg]
S.id_ue = zeros(2, 1);              % command at the end of the slow openings
S.id_vpk = 0;                       % peak speed in a step [deg/s]
S.vi = zeros(4, 1);                 % low-passed gyro rates (mode 2) [deg/s]
% enable input
S.req = 0;                          % accepted request: 0 stop, 1, 2
S.rq_new = 0;                       % present request and the time it has been the same [s]
S.rq_t = 0;
S.off_t = 0;                        % time with request 0 [s]
end


function S = rb_new(S, C)
% everything reset for a new run, except the handling of the enable input
rn = S.rq_new;
rt = S.rq_t;
rq = S.req;
ot = S.off_t;
S = rb_init(C);
S.rq_new = rn;
S.rq_t = rt;
S.req = rq;
S.off_t = ot;
end


% =================================================================================== sensing
function S = rb_sense(S, C, q_meas, okin)
nq = size(S.qh, 2);
qm = [double(q_meas(1)); double(q_meas(2)); double(q_meas(3)); double(q_meas(4))];
good = all(isfinite(qm)) && okin;
if good
    if S.valid
        % slew angle kept continuous if the sensor value wraps around
        d = qm(1) + S.wrap - S.q(1);
        if d > 180
            S.wrap = S.wrap - 360;
        elseif d < -180
            S.wrap = S.wrap + 360;
        end
    end
    qm(1) = qm(1) + S.wrap;
    % a jump of more than 2 deg in one sample is treated as an invalid sample
    if S.valid && max(abs(qm - S.q)) > 2
        good = false;
    end
end
if good
    if ~S.valid
        % first valid sample, or valid again after a sensor fault: new history, pause
        for i = 1:nq
            S.qh(:, i) = qm;
        end
        S.v = zeros(4, 1);
        S.qlast = qm;
        if S.phase > 0
            S.paused = true;
        end
    end
    S.bad = 0;
    S.valid = true;
    S.vt = S.vt + C.dt;
    % sensor update interval (samples between value changes while the joint moves) and
    % sensor resolution (smallest change of the value)
    for j = 1:4
        S.chg(j) = S.chg(j) + 1;
        if qm(j) ~= S.qlast(j)
            if abs(S.v(j)) > 1.0
                S.upd(j) = 0.95 * S.upd(j) + 0.05 * S.chg(j);
            end
            S.qstep(j) = min(S.qstep(j), abs(qm(j) - S.qlast(j)));
            S.chg(j) = 0;
            S.qlast(j) = qm(j);
        end
    end
    S.q = qm;
else
    S.bad = S.bad + 1;
    S.warn = bitor_d(S.warn, 8);
    S.vt = 0;
    if S.bad > 20
        S.valid = false;
    end
end
% velocity: difference over a window (longer for slow sensor updates), then low-pass
S.qi = mod(S.qi, nq) + 1;
S.qh(:, S.qi) = S.q;
w = max(C.vwin, min(4 * round(max(S.upd)), nq - 8));
S.vdel = (0.5 * w * C.dt + C.vtau) * ones(4, 1);
k = mod(S.qi - 1 - w, nq) + 1;
a = C.dt / (C.vtau + C.dt);
for j = 1:4
    vr = (S.q(j) - S.qh(j, k)) / (w * C.dt);
    S.v(j) = S.v(j) + a * (vr - S.v(j));
end
end


function S = rb_start(S, C)
S.slew0 = S.q(1);
S.phase = 1;
S.warn = 0;
S.id_ch = 1;
S.id_pass = 1;
S.chres = zeros(4, 1);
S = tune_next(S, 1);
S.qd = S.q;
S.vd = zeros(4, 1);
S.ad = zeros(4, 1);
end


function S = rb_resume(S, C)
S.paused = false;
S.uj = zeros(4, 1);
S.uch = zeros(4, 1);
if S.phase == 1
    % repeat the tests of the current valve
    S = tune_next(S, 1);
elseif S.phase >= 2
    S.resume = true;
end
end


% =================================================================================== auto-tuning
function S = rb_tune(S, C)
dt = C.dt;
ch = C.order(S.id_ch);
S.uch = zeros(4, 1);
S.unit = ch;
S.stepcode = S.id_test;
S.id_t = S.id_t + dt;
if S.id_test == 1
    % ----- settle: drift (least-squares slope) and velocity noise with all valves closed
    if S.id_n == 0
        S.id_q0 = S.q;
        S.id_sv = zeros(4, 1);
        S.id_sv2 = zeros(4, 1);
        S.id_s1 = 0;
        S.id_s2 = 0;
        S.id_sy = zeros(4, 1);
        S.id_sty = zeros(4, 1);
    end
    S.id_n = S.id_n + 1;
    S.id_sv = S.id_sv + S.v;
    S.id_sv2 = S.id_sv2 + S.v .* S.v;
    t = S.id_t;
    S.id_s1 = S.id_s1 + t;
    S.id_s2 = S.id_s2 + t * t;
    S.id_sy = S.id_sy + (S.q - S.id_q0);
    S.id_sty = S.id_sty + t * (S.q - S.id_q0);
    if S.id_t >= C.id_settle
        n = S.id_n;
        den = n * S.id_s2 - S.id_s1 * S.id_s1;
        for j = 1:4
            if den > 0
                S.drift(j) = (n * S.id_sty(j) - S.id_s1 * S.id_sy(j)) / den;
            else
                S.drift(j) = 0;
            end
            m = S.id_sv(j) / n;
            S.vsig(j) = sqrt(max(S.id_sv2(j) / n - m * m, 0)) + 0.02;
        end
        S = tune_next(S, 2);
    end
elseif S.id_test == 2 || S.id_test == 3
    % ----- slow opening, test 2 positive command, test 3 negative command
    S = tune_ramp(S, C, ch);
elseif S.id_test == 4
    % ----- fit of gain and dead zone to the slow openings
    S = tune_fit_ramps(S, C);
    if S.id_fail
        S = tune_finish(S, C, false);
    else
        S = tune_next(S, 5);
    end
elseif S.id_test == 5 || S.id_test == 6
    % ----- smooth steps at normal speed, first towards the larger margin
    S = tune_step(S, C, ch);
elseif S.id_test == 7
    % ----- delay and lag: grid search, one grid point per sample
    S = tune_grid(S, C);
end
end


function S = tune_next(S, test)
S.id_test = test;
S.id_st = 1;
S.id_t = 0;
S.id_n = 0;
S.id_u = 0;
S.id_det = false;
S.id_cand = 0;
S.id_cnt = 0;
S.id_still = 0;
end


function S = tune_ramp(S, C, ch)
dt = C.dt;
k = S.id_test - 1;                  % 1: positive command, 2: negative command
if k == 1
    cdir = 1;
else
    cdir = -1;
end
if S.id_st == 1
    if S.id_n == 0
        S.id_q0 = S.q;
        S.rn(k) = 0;
        if k == 1
            S.id_resp = 0;
            S.id_fail = false;
            S.id_fail1 = false;
            S.id_fail2 = false;
        end
    end
    S.id_n = S.id_n + 1;
    if S.id_det
        rate = C.id_rate2;
    else
        rate = C.id_rate1;
    end
    S.id_u = S.id_u + rate * dt;
    S.uch(ch) = cdir * S.id_u;
    % travel and speed caused by the valve (drift with closed valves removed)
    d = S.q - S.id_q0 - S.drift * S.id_t;
    vc = S.v - S.drift;
    far = false;
    jc = 0;
    dm = 0;
    for j = 1:4
        if S.id_resp == 0
            free = ~S.ok(j);
        else
            free = j == S.id_resp;
        end
        % travel or speed limit of the test reached (speed: joint found, or another joint)
        if abs(d(j)) >= C.id_dq1 || (abs(vc(j)) >= C.id_v1 && (S.id_det || ~free))
            far = true;
        end
        thr = max(0.15, 0.25 * S.vsig(j));
        vthr = max(0.25, 2.5 * S.vsig(j));
        if free && abs(d(j)) > thr && vc(j) * d(j) > vthr * abs(d(j)) && abs(d(j)) / thr > dm
            dm = abs(d(j)) / thr;
            jc = j;
        end
    end
    if ~S.id_det
        % the joint is assigned to the valve after it has moved for id_tdet
        if jc > 0 && jc == S.id_cand
            S.id_cnt = S.id_cnt + 1;
        else
            S.id_cand = jc;
            S.id_cnt = 0;
        end
        if jc > 0 && S.id_cnt * dt >= C.id_tdet
            S.id_det = true;
            S.id_ub(k) = max(S.id_u - C.id_rate1 * (0.15 + C.id_tdet), 0);
            if S.id_resp == 0
                S.id_resp = jc;
                if cdir * d(jc) > 0
                    S.sgn(jc) = 1;
                else
                    S.sgn(jc) = -1;
                end
                if jc ~= ch
                    S.warn = bitor_d(S.warn, 2);
                end
                S.map(jc) = ch;
            end
        elseif S.id_u >= C.ulim || far
            % no motion up to the largest command (or another joint moved): the joint may be at
            % an end stop, the values of the other direction are used; no motion in both
            % directions: valve without response
            if k == 1
                S.id_fail1 = true;
            elseif S.id_fail1
                S.id_fail = true;
                S.warn = bitor_d(S.warn, 1);
            else
                S.id_fail2 = true;
            end
            S.id_uend = S.id_u;
            S.id_ue(k) = S.id_u;
            S.id_st = 2;
            S.id_t = 0;
        end
    else
        j = S.id_resp;
        if far || S.id_u >= C.ulim
            if abs(vc(j)) < 0.5 * C.id_v1 && abs(d(j)) < 1.0
                % the joint stopped soon after it started (end stop) or another joint moved:
                % direction not used
                if k == 1
                    S.id_fail1 = true;
                elseif S.id_fail1
                    S.id_fail = true;
                    S.warn = bitor_d(S.warn, 1);
                else
                    S.id_fail2 = true;
                end
            end
            S.id_uend = S.id_u;
            S.id_ue(k) = S.id_u;
            S.id_st = 2;
            S.id_t = 0;
        end
    end
    % record command and velocities every 20 ms
    if mod(S.id_n, C.nrec) == 0 && S.rn(k) < size(S.rb, 2)
        n = S.rn(k) + 1;
        S.rn(k) = n;
        r0 = (k - 1) * 5;
        S.rb(r0 + 1, n) = S.id_u;
        for j = 1:4
            S.rb(r0 + 1 + j, n) = vc(j);
        end
    end
elseif S.id_st == 2
    % close the valve smoothly
    S.uch(ch) = cdir * S.id_uend * close_shape(S.id_t, C.id_tdown);
    if S.id_t >= C.id_tdown
        S.id_st = 3;
        S.id_t = 0;
        S.id_still = 0;
    end
else
    % wait until the joints are still
    S = tune_wait_still(S, C);
    if S.id_st == 4
        if S.id_fail
            S = tune_finish(S, C, false);
        else
            S = tune_next(S, S.id_test + 1);
        end
    end
end
end


function y = close_shape(t, T)
% smooth valve closing from 1 to 0 in the time T
y = 0.5 + 0.5 * cos(pi * min(t / T, 1));
end


function S = tune_wait_still(S, C)
still = true;
for j = 1:4
    if abs(S.v(j) - S.drift(j)) > max(0.3, 4 * S.vsig(j))
        still = false;
    end
end
if still
    S.id_still = S.id_still + C.dt;
else
    S.id_still = 0;
end
if S.id_still >= C.id_tstill || S.id_t >= C.id_tstill_max
    S.id_st = 4;
end
end


function S = tune_fit_ramps(S, C)
% fit v = K (|u(t - th0)| - db) to the moving part of both slow openings
j = S.id_resp;
if j == 0
    S.id_fail = true;
    return
end
sh = round(C.id_th0 / (C.nrec * C.dt));
for k = 1:2
    r0 = (k - 1) * 5;
    n = S.rn(k);
    sx = 0;
    sy = 0;
    sxx = 0;
    sxy = 0;
    m = 0;
    vmin = max(0.4, 3 * S.vsig(j));
    for i = sh + 1:n
        y = abs(S.rb(r0 + 1 + j, i));
        if y > vmin
            x = S.rb(r0 + 1, i - sh);
            sx = sx + x;
            sy = sy + y;
            sxx = sxx + x * x;
            sxy = sxy + x * y;
            m = m + 1;
        end
    end
    ub = S.id_ub(k);
    K = 0;
    db = ub;
    den = m * sxx - sx * sx;
    if m >= 5 && den > 1e-12
        K = (m * sxy - sx * sy) / den;
        if K > 0
            db = -(sy - K * sx) / m / K;
        end
    end
    if K > 0 && db >= 0 && db < 0.5 * ub
        % dead zone far below the command at which the joint started to move (slow inertial
        % response or stick-slip, e.g. of the slew): dead zone just below that command, gain
        % fitted through it
        db = max(db, ub - 0.08 * C.umax);
        den = sxx - 2 * db * sx + m * db * db;
        if den > 1e-12
            K = (sxy - db * sy) / den;
        end
    end
    if ~(K > 0) || db < 0 || db > ub + 0.1 * C.umax
        % not enough points for a line: gain from the last point above the breakaway command
        if n >= 1
            K = abs(S.rb(r0 + 1 + j, n)) / max(S.rb(r0 + 1, n) - ub, 0.02 * C.umax);
        end
        db = ub;
    end
    S.Kq(k) = max(K, 1e-3);
    S.dbq(k) = max(db, 0);
end
if S.id_fail1
    S.Kq(1) = S.Kq(2);
    S.dbq(1) = S.dbq(2);
    S.id_ue(1) = S.id_ue(2);
end
if S.id_fail2
    S.Kq(2) = S.Kq(1);
    S.dbq(2) = S.dbq(1);
    S.id_ue(2) = S.id_ue(1);
end
end


function S = tune_step(S, C, ch)
j = S.id_resp;
k = S.id_test - 4;                  % 1: first step, 2: second step
% rise and closing time: twice as long for the slew (large inertia)
tr = C.id_trise;
td = C.id_tdown;
if j == 1
    tr = 2 * tr;
    td = 2 * td;
end
if S.id_st == 1
    if S.id_n == 0
        S.id_q0 = S.q;
        S.sn(k) = 0;
        S.sacc(k, 1) = 0;
        S.sacc(k, 2) = 0;
        S.sok(k) = true;
        if k == 1
            [mlo, mhi] = margins(S, C, j);
            if mhi >= mlo
                S.sdir(1) = 1;
            else
                S.sdir(1) = -1;
            end
            S.id_dq = min(C.id_dq2, 0.6 * max(mlo, mhi));
            if S.id_dq < 1.0
                S.warn = bitor_d(S.warn, 32);
                S.id_dq = 0;
            end
        else
            S.sdir(2) = -S.sdir(1);
        end
        % command for the joint speed vmax: dead zone + speed / gain of this direction, at most
        % id_du above the end of the slow opening (valves with progressive flow curves)
        kd = ramp_index(S, S.sdir(k));
        S.sus(k) = min(max(S.dbq(kd) + C.vmax(j) / S.Kq(kd), S.dbq(kd) + 0.02 * C.umax), C.ulim);
        S.sus(k) = min(S.sus(k), max(S.id_ue(kd) + C.id_du, S.dbq(kd) + 0.02 * C.umax));
        S.id_vpk = 0;
    end
    if S.id_dq <= 0
        S.id_st = 3;
        S.id_t = 0;
        S.id_uend = 0;
    else
        S.id_n = S.id_n + 1;
        t = S.id_t;
        if t < tr
            um = S.sus(k) * (0.5 - 0.5 * cos(pi * t / tr));
        else
            um = S.sus(k);
        end
        S.id_u = um;
        S.uch(ch) = S.sgn(j) * S.sdir(k) * um;
        % record 5 ms samples of the command and of the velocity in the step direction
        if mod(S.id_n, C.nsrec) == 0 && S.sn(k) < size(S.sb, 2)
            n = S.sn(k) + 1;
            S.sn(k) = n;
            S.sb(2 * k - 1, n) = um;
            S.sb(2 * k, n) = S.sdir(k) * (S.v(j) - S.drift(j));
        end
        if t > tr + 0.3
            S.sacc(k, 1) = S.sacc(k, 1) + S.sdir(k) * (S.v(j) - S.drift(j));
            S.sacc(k, 2) = S.sacc(k, 2) + 1;
        end
        S.id_vpk = max(S.id_vpk, S.sdir(k) * (S.v(j) - S.drift(j)));
        % end of the step: travel reached, time limit, a joint faster than 1.5 vmax, or
        % another joint moved as far as the step
        far = false;
        for jj = 1:4
            if abs(S.v(jj)) > 1.5 * C.vmax(jj) || (jj ~= j && abs(S.q(jj) - S.id_q0(jj)) >= S.id_dq)
                far = true;
            end
        end
        if abs(S.q(j) - S.id_q0(j)) >= S.id_dq || t >= C.id_tstep || far
            % a step that travelled less than 30 % is used only if it was cut short while the
            % joint was moving fast (not at an end stop)
            S.sok(k) = abs(S.q(j) - S.id_q0(j)) >= 0.3 * S.id_dq || (S.id_vpk >= 0.5 * C.vmax(j) && ...
                S.sdir(k) * (S.v(j) - S.drift(j)) >= 0.25 * C.vmax(j));
            S.id_uend = um;
            S.id_st = 2;
            S.id_t = 0;
        end
    end
elseif S.id_st == 2
    S.uch(ch) = S.sgn(j) * S.sdir(k) * S.id_uend * close_shape(S.id_t, td);
    if S.id_t >= td
        S.id_st = 3;
        S.id_t = 0;
        S.id_still = 0;
    end
else
    S = tune_wait_still(S, C);
    if S.id_st == 4
        if k == 1
            S = tune_next(S, 6);
        else
            S = tune_next(S, 7);
            S.gi = 0;
            S.gbest = [1e300; 0.15; 0];
        end
    end
end
end


function kd = ramp_index(S, jdir)
% slow-opening record of the command sign that moves the joint in direction jdir
if S.sgn(S.id_resp) * jdir > 0
    kd = 1;
else
    kd = 2;
end
end


function [mlo, mhi] = margins(S, C, j)
if j == 1
    q = S.q(1) - S.slew0;
else
    q = S.q(j);
end
mlo = q - C.lim_lo(j);
mhi = C.lim_hi(j) - q;
end


function S = tune_grid(S, C)
% one (delay, lag) pair of the grid per call: least-squares gain and cost for both steps
S.gi = S.gi + 1;
ith = floor((S.gi - 1) / C.ngT);
th = ith * 0.01;
T = (S.gi - 1 - ith * C.ngT) * 0.025;
ts = C.nsrec * C.dt;
sh = round(th / ts);
a = ts / (T + ts);
cost = 0;
for k = 1:2
    n = S.sn(k);
    if n > 20 && S.id_dq > 0 && S.sok(k)
        kd = ramp_index(S, S.sdir(k));
        db = S.dbq(kd);
        m = 0;
        svm = 0;
        smm = 0;
        svv = 0;
        for i = 1:n
            if i > sh
                x = max(S.sb(2 * k - 1, i - sh) - db, 0);
            else
                x = 0;
            end
            m = m + a * (x - m);
            vv = S.sb(2 * k, i);
            svm = svm + vv * m;
            smm = smm + m * m;
            svv = svv + vv * vv;
        end
        if smm > 1e-12
            cost = cost + svv - svm * svm / smm;
        else
            cost = cost + svv;
        end
    end
end
if cost < S.gbest(1)
    S.gbest(1) = cost;
    S.gbest(2) = th;
    S.gbest(3) = T;
end
if S.gi >= C.ngth * C.ngT
    S = tune_finish(S, C, true);
end
end


function S = tune_finish(S, C, good)
j = S.id_resp;
ch = C.order(S.id_ch);
% a very large dead zone in the first pass (e.g. pump started during the test): test again
S.chres(ch) = 2;
if good && j > 0 && S.id_pass == 1 && max(S.dbq(1), S.dbq(2)) > 0.5 * C.ulim
    good = false;
    S.chres(ch) = 3;
end
if good && j > 0
    % gains: steady speed of the steps, otherwise the slow-opening fit
    for k = 1:2
        kd = ramp_index(S, S.sdir(k));
        if S.sacc(k, 2) > 50 && S.id_dq > 0 && S.sok(k)
            vs = S.sacc(k, 1) / S.sacc(k, 2);
            K = vs / max(S.sus(k) - S.dbq(kd), 0.02 * C.umax);
            if K > 0.3 * S.Kq(kd) && K < 3 * S.Kq(kd)
                S.Kq(kd) = K;
            end
        end
    end
    if S.id_dq > 0 && (S.sok(1) || S.sok(2))
        thl = S.gbest(2) + S.gbest(3) - S.vdel(j);
    else
        thl = 0.15;
    end
    thl = min(max(thl, 0.03), 0.5);
    S.th(j) = thl;
    S.dly(j) = round(thl / C.dt);
    S.lam(j) = min(max(1 / (3 * thl), C.lam_min), C.lam_max);
    % gains of joint directions: slew in deg/s, cylinders in m/s per unit command
    if S.sgn(j) > 0
        kp = 1;
        kn = 2;
    else
        kp = 2;
        kn = 1;
    end
    if j == 1
        sc = 1;
    else
        sc = abs(cyl_arm(C, j, S.q(j))) * pi / 180;
    end
    S.Kp(j) = S.Kq(kp) * sc;
    S.Kn(j) = S.Kq(kn) * sc;
    S.dbp(j) = S.dbq(kp);
    S.dbn(j) = S.dbq(kn);
    S.Kp0(j) = S.Kp(j);
    S.Kn0(j) = S.Kn(j);
    % joint speed used by the planner: at most 70 % of the valve capacity in the slower direction
    vcap = min(S.Kq(1) * (C.ulim - S.dbq(1)), S.Kq(2) * (C.ulim - S.dbq(2)));
    S.vlim(j) = min(C.vmax(j), max(0.7 * vcap, 0.5));
    S.dbp0(j) = S.dbp(j);
    S.dbn0(j) = S.dbn(j);
    S.ok(j) = true;
    S.chres(ch) = 1;
end
% next valve; a second pass repeats once the valves with a very large dead zone and, if
% another valve moved a joint, the valves without response
while S.id_ch < 4 || (S.id_pass == 1 && ((any(S.ok) && any(S.chres == 2)) || any(S.chres == 3)))
    if S.id_ch >= 4
        S.id_pass = 2;
        S.id_ch = 0;
    end
    S.id_ch = S.id_ch + 1;
    if S.id_pass == 1 || S.chres(C.order(S.id_ch)) >= 2
        S = tune_next(S, 1);
        return
    end
end
% all valves tested: closed-loop check, or phase 5 if no valve moved a joint (e.g. pump off)
if ~any(S.ok)
    S.phase = 5;
else
    S.phase = 2;
    S.qd = S.q;
    S.vd = zeros(4, 1);
    S.ad = zeros(4, 1);
    S.ck_k = 1;
    S.ck_st = 1;
    S.ck_n = 0;
    S.sg_type = 0;
    S.hold = true(4, 1);
    S = ref_fill(S, C);
end
end


% =================================================================================== planner
function S = rb_plan(S, C)
dt = C.dt;
if S.resume
    % continue after a stop: smooth move from the measured position to the target of the step
    S.resume = false;
    S.gate_t = 0;
    S.gate_ok = 0;
    S.qd = S.q;
    if S.sg_type > 0
        S = plan_joint(S, C, S.sg_qb, S.speed);
    else
        S = ref_fill(S, C);
    end
end
if S.sg_type == 0
    S = next_step(S, C);
end
% reference of the present segment
S.sg_t = S.sg_t + dt;
t = min(S.sg_t, S.sg_T);
[s, sd, sdd] = quintic(t, S.sg_T);
if S.sg_type == 1
    for j = 1:4
        dq = S.sg_qb(j) - S.sg_qa(j) - S.sg_off(j);
        S.qd(j) = S.sg_qa(j) + S.sg_off(j) + s * dq;
        S.vd(j) = sd * dq;
        S.ad(j) = sdd * dq;
    end
elseif S.sg_type == 2
    [q0, ok0] = line_q(S, C, t);
    [q1, ok1] = line_q(S, C, t + 0.002);
    [qm, okm] = line_q(S, C, t - 0.002);
    if ok0 && ok1 && okm
        for j = 2:4
            S.qd(j) = q0(j - 1) + (1 - s) * S.sg_off(j);
            if S.sg_t < S.sg_T
                S.vd(j) = (q1(j - 1) - qm(j - 1)) / 0.004 - sd * S.sg_off(j);
                S.ad(j) = (q1(j - 1) - 2 * q0(j - 1) + qm(j - 1)) / 0.000004 - sdd * S.sg_off(j);
            else
                S.vd(j) = 0;
                S.ad(j) = 0;
            end
        end
    else
        S.vd = zeros(4, 1);
        S.ad = zeros(4, 1);
    end
    S.qd(1) = S.sg_qb(1) + (1 - s) * S.sg_off(1);
    S.vd(1) = -sd * S.sg_off(1);
    S.ad(1) = -sdd * S.sg_off(1);
else
    S.qd = S.sg_qb;
    S.vd = zeros(4, 1);
    S.ad = zeros(4, 1);
end
for j = 1:4
    S.emax = max(S.emax, abs(S.err(j)));
end
% wait until all joints have reached the target
if S.sg_t >= S.sg_T
    S.gate_t = S.gate_t + dt;
    tol = C.gate_tol;
    if S.gate_t > C.gate_t1
        tol = 4 * C.gate_tol;
    end
    reached = true;
    for j = 1:4
        if S.ok(j)
            e = abs(S.q(j) - S.qd(j));
            if ~(S.hold(j) || e < tol) || abs(S.v(j)) > max(0.5, 4 * S.vsig(j))
                reached = false;
            end
        end
    end
    if reached
        S.gate_ok = S.gate_ok + dt;
    else
        S.gate_ok = 0;
    end
    if S.phase == 2
        S = check_eval(S, C);
    end
    if S.gate_ok >= C.gate_time || S.gate_t >= C.gate_t2
        if S.gate_t >= C.gate_t2
            S.warn = bitor_d(S.warn, 4);
        end
        S = next_step(S, C);
    end
end
end


function S = next_step(S, C)
% sequence of moves: check moves (phase 2), boulder sequence (phase 3), hold (phase 4)
S.gate_t = 0;
S.gate_ok = 0;
if S.phase == 2
    S = check_next(S, C);
end
if S.phase == 3
    S = task_next(S, C);
end
if S.phase == 4
    S.stepcode = 0;
    S = plan_hold(S, S.sg_qb, 1e6);
end
end


function S = check_next(S, C)
% small closed-loop move out and back per joint; lambda reduced after overshoot or oscillation
while S.phase == 2
    if S.ck_k > 4
        S.phase = 3;
        S.tk_state = 0;
        S.tk_r = 1;
        S.tk_b = 1;
        S.tk_sub = 0;
        S.speed = C.s0;
        S.emax_b = 0;
        return
    end
    j = C.order(S.ck_k);
    if ~S.ok(j)
        S.ck_k = S.ck_k + 1;
        S.ck_st = 1;
        S.ck_n = 0;
        continue
    end
    S.unit = j;
    S.stepcode = S.ck_st;
    if S.ck_st == 1
        [mlo, mhi] = margins(S, C, j);
        if mhi >= mlo
            S.ck_dir = 1;
        else
            S.ck_dir = -1;
        end
        S.ck_a = min(4.0, 0.6 * max(mlo, mhi));
        S.ck_bad = false;
        qb = S.qd;
        qb(j) = S.qd(j) + S.ck_dir * S.ck_a;
        S = check_move(S, C, qb, S.ck_dir, 2);
        return
    elseif S.ck_st == 2
        S.ck_bad = S.ck_bad || check_bad(S);
        qb = S.qd;
        qb(j) = S.qd(j) - S.ck_dir * S.ck_a;
        S = check_move(S, C, qb, -S.ck_dir, 3);
        return
    else
        S.ck_bad = S.ck_bad || check_bad(S);
        if S.ck_bad && S.ck_n < 2
            S.lam(j) = max(0.7 * S.lam(j), C.lam_min);
            S.warn = bitor_d(S.warn, 16);
            S.ck_n = S.ck_n + 1;
            S.ck_st = 1;
        else
            S.ck_k = S.ck_k + 1;
            S.ck_st = 1;
            S.ck_n = 0;
        end
    end
end
end


function S = check_move(S, C, qb, mdir, st)
S.ck_mdir = mdir;
S.ck_os = 0;
S.ck_zc = 0;
S.ck_last = 0;
S.ck_st = st;
S = plan_joint(S, C, qb, 0.5);
end


function bad = check_bad(S)
bad = S.ck_os > max(0.3, 0.1 * S.ck_a) || S.ck_zc >= 3;
end


function S = check_eval(S, C)
% overshoot beyond the target and sign changes of the error after the end of a check move
if S.ck_st == 2 || S.ck_st == 3
    j = C.order(min(S.ck_k, 4));
    e = S.ck_mdir * (S.q(j) - S.qd(j));
    S.ck_os = max(S.ck_os, e);
    if abs(e) > 0.15
        if S.ck_last ~= 0 && e * S.ck_last < 0
            S.ck_zc = S.ck_zc + 1;
        end
        if e > 0
            S.ck_last = 1;
        else
            S.ck_last = -1;
        end
    end
end
end


function S = task_next(S, C)
% home pose, then per boulder: 1 move, 2 descent, 3 press, 4 hold, 5 retract;
% home pose after each round
if S.tk_state == 0
    S.tk_state = 1;
    S.tk_b = 1;
    S.tk_sub = 0;
    S.unit = 0;
    S.stepcode = 10;
    S = plan_joint(S, C, home_pose(S, C), S.speed);
    return
end
if S.tk_state == 2
    S.tk_r = S.tk_r + 1;
    if S.tk_r > C.nround
        S.phase = 4;
        return
    end
    S.tk_state = 1;
    S.tk_b = 1;
    S.tk_sub = 0;
end
S.tk_sub = S.tk_sub + 1;
if S.tk_sub > 5
    % boulder done: faster if the tracking was good, slower if not
    if S.emax_b < 1.0
        S.speed = min(1.0, S.speed + C.sstep);
    elseif S.emax_b > 2.5
        S.speed = max(C.s0, S.speed - C.sstep);
    end
    S.emax_b = 0;
    S.tk_b = S.tk_b + 1;
    S.tk_sub = 1;
    if S.tk_b > 4
        S.tk_state = 2;
        S.tk_b = 4;
        S.unit = 0;
        S.stepcode = 10;
        S = plan_joint(S, C, home_pose(S, C), S.speed);
        return
    end
end
b = S.tk_b;
S.unit = b;
S.stepcode = S.tk_sub;
rho = C.bld(b, 1);
zt = C.bld(b, 2);
qs = S.slew0 + C.bld(b, 3);
if S.tk_sub == 1
    S = plan_joint(S, C, task_pose(C, rho, zt + C.hgt(1), qs, S.sg_qb), S.speed);
elseif S.tk_sub == 2
    S = plan_line(S, C, [rho; zt + C.hgt(1)], [rho; zt + C.hgt(2)], S.speed * C.vtip);
elseif S.tk_sub == 3
    S = plan_line(S, C, [rho; zt + C.hgt(2)], [rho; zt + C.hgt(3)], S.speed * C.vpress);
elseif S.tk_sub == 4
    S = plan_hold(S, S.sg_qb, C.thold);
else
    S = plan_joint(S, C, task_pose(C, rho, zt + C.hgt(4), qs, S.sg_qb), S.speed);
end
end


function q = home_pose(S, C)
q = [S.slew0 + C.home(1); C.home(2); C.home(3); C.home(4)];
end


function q = task_pose(C, rho, z, qs, qprev)
[qa, ok] = ik(C, rho, z);
q = qprev;
q(1) = qs;
if ok
    q(2) = qa(1);
    q(3) = qa(2);
    q(4) = qa(3);
end
end


function S = plan_joint(S, C, qb, speed)
% joint move with a quintic profile; duration from the speed and acceleration limits
qa = S.qd;
qt = qb;
T = C.tmin;
for j = 1:4
    if j == 1
        qt(1) = min(max(qb(1), S.slew0 + C.lim_lo(1)), S.slew0 + C.lim_hi(1));
    else
        qt(j) = min(max(qb(j), C.lim_lo(j)), C.lim_hi(j));
    end
    if ~S.ok(j)
        qt(j) = qa(j);
    end
    dq = abs(qt(j) - qa(j));
    T = max(T, 1.875 * dq / (speed * S.vlim(j)));
    T = max(T, sqrt(5.7735 * dq / (speed * C.amax(j))));
end
S.sg_type = 1;
S.sg_qa = qa;
S.sg_qb = qt;
S.sg_t = 0;
S.sg_T = T;
S = seg_start(S, C);
end


function S = plan_line(S, C, pa, pb, v)
% straight tip motion in the boom plane at the task hammer angle
[qe, ok] = ik(C, pb(1), pb(2));
qb = S.qd;
if ok
    qb(2) = qe(1);
    qb(3) = qe(2);
    qb(4) = qe(3);
    d = sqrt((pb(1) - pa(1)) ^ 2 + (pb(2) - pa(2)) ^ 2);
    S.sg_type = 2;
    S.sg_qa = S.qd;
    S.sg_qb = qb;
    S.sg_pa = pa;
    S.sg_pb = pb;
    S.sg_t = 0;
    S.sg_T = max(C.tmin, 1.875 * d / v);
    for j = 2:4
        S.sg_T = max(S.sg_T, 1.875 * abs(qb(j) - S.qd(j)) / (S.speed * S.vlim(j)));
    end
    S = seg_start(S, C);
else
    S = plan_joint(S, C, qb, S.speed);
end
end


function S = plan_hold(S, q, T)
S.sg_type = 3;
S.sg_qa = q;
S.sg_qb = q;
S.sg_off = zeros(4, 1);
S.sg_t = 0;
S.sg_T = T;
S.emax = 0;
end


function S = seg_start(S, C)
% a move starts at the measured position: the difference to the planned start point is
% removed smoothly during the move, so the move ends exactly at the planned target
for j = 1:4
    if S.ok(j) && abs(S.sg_qb(j) - S.sg_qa(j)) > 1e-6
        S.sg_off(j) = S.q(j) - S.qd(j);
    else
        S.sg_off(j) = 0;
    end
end
S.qd = S.qd + S.sg_off;
S.vd = zeros(4, 1);
S.ad = zeros(4, 1);
S = ref_fill(S, C);
S.emax = 0;
end


function [q, ok] = line_q(S, C, t)
s = quintic(min(max(t, 0), S.sg_T), S.sg_T);
rho = S.sg_pa(1) + s * (S.sg_pb(1) - S.sg_pa(1));
z = S.sg_pa(2) + s * (S.sg_pb(2) - S.sg_pa(2));
[q, ok] = ik(C, rho, z);
end


function [s, sd, sdd] = quintic(t, T)
Tp = max(T, 1e-3);
x = min(max(t / Tp, 0), 1);
s = x * x * x * (10 - 15 * x + 6 * x * x);
sd = 30 * x * x * (1 - x) * (1 - x) / Tp;
sdd = 60 * x * (1 - x) * (1 - 2 * x) / (Tp * Tp);
end


function [q, ok] = ik(C, rho, z)
% lift, tilt, crowd [deg] for the tip at reach rho and height z with the task hammer angle
ph = C.phi * pi / 180;
jx = rho - C.L3 * cos(ph + C.dl) - C.J1(1);
jz = z - C.L3 * sin(ph + C.dl) - C.J1(2);
c2 = (jx * jx + jz * jz - C.L1 * C.L1 - C.L2 * C.L2) / (2 * C.L1 * C.L2);
q = zeros(3, 1);
ok = abs(c2) < 1;
if ok
    q2 = -acos(c2);
    q1 = atan2(jz, jx) - atan2(C.L2 * sin(q2), C.L1 + C.L2 * cos(q2));
    q(1) = q1 * 180 / pi;
    q(2) = q2 * 180 / pi;
    q(3) = C.phi - q(1) - q(2);
end
end


function m = cyl_arm(C, j, qdeg)
% piston rate per joint rate dL/dq [m/rad] of the cylinder of joint j (2 lift, 3 tilt, 4 crowd)
i = j - 1;
q = qdeg * pi / 180;
cq = cos(q);
sq = sin(q);
ux = cq * C.cyl(i, 3) - sq * C.cyl(i, 4);
uy = sq * C.cyl(i, 3) + cq * C.cyl(i, 4);
vx = C.cyl(i, 1) + ux;
vy = C.cyl(i, 2) + uy;
m = (-vx * uy + vy * ux) / sqrt(vx * vx + vy * vy);
end


% =================================================================================== control
function S = ref_fill(S, C)
% reference history set to the present reference (no jump of the delayed reference)
for i = 1:size(S.rq, 2)
    S.rq(:, i) = S.qd;
    S.rv(:, i) = zeros(4, 1);
end
end


function S = rb_control(S, C)
dt = C.dt;
nr = size(S.rq, 2);
S.ri = mod(S.ri, nr) + 1;
S.rq(:, S.ri) = S.qd;
for j = 1:4
    S.uj(j) = 0;
    if ~S.ok(j)
        S.qr_dot(j) = 0;
        S.err(j) = 0;
        continue
    end
    % reference delayed by the loop delay, so that the feedback acts on real errors only
    kd = mod(S.ri - 1 - S.dly(j), nr) + 1;
    e = S.rq(j, kd) - S.q(j);
    S.err(j) = e;
    S.emax_b = max(S.emax_b, abs(e));
    % low-passed error (sensor noise removed) for the oscillation detection
    S.ef(j) = S.ef(j) + C.dt / (C.osc_tau + C.dt) * (e - S.ef(j));
    moving = abs(S.vd(j)) > 0.02 || abs(S.rq(j, kd) - S.qd(j)) > 0.02;
    if S.hold(j)
        if moving || abs(e) > C.e_out
            S.hold(j) = false;
            S.stuck(j) = 0;
        end
    elseif ~moving && abs(e) < C.e_in && abs(S.v(j)) < max(0.5, 4 * S.vsig(j))
        S.hold(j) = true;
    end
    % VDC required joint rate and piston rate
    qr = S.vd(j) + S.lam(j) * e;
    S.qr_dot(j) = qr;
    if j == 1
        sc = 1;
    else
        marm = cyl_arm(C, j, S.q(j)) * pi / 180;
        sc = abs(marm);
        S.xr(j - 1) = marm * qr;
    end
    % valve opening: flow for the required rate plus the dead zone. At rest the dead zone
    % follows the sign of the required rate (smooth transition, full compensation when the
    % joint does not move); during a planned move it is compensated in the planned direction,
    % so the valve closes but does not reverse when the joint is slightly ahead.
    sr = min(abs(qr) / C.vblend, 1);
    if S.stuck(j) > 0.4
        sr = 1;
    end
    c = valve_open(S, j, sc * qr, sr, qr >= 0);
    w = min(abs(S.vd(j)) / C.vff, 1);
    if w > 0
        c = w * valve_open(S, j, sc * qr, 1, S.vd(j) >= 0) + (1 - w) * c;
    end
    S.uj(j) = S.sgn(j) * c;
    if S.hold(j)
        S.uj(j) = 0;
    end
    % oscillation (e.g. an unmodelled hydraulic resonance): the low-passed error changes sign 6
    % times within the window (band at least 1.5 sensor steps) -> lambda reduced by 30 %
    if ~S.hold(j)
        band = max(C.osc_band, 1.5 * S.qstep(j));
        sg = 0;
        if S.ef(j) > band
            sg = 1;
        elseif S.ef(j) < -band
            sg = -1;
        end
        if sg ~= 0
            if S.osc_s(j) ~= 0 && sg ~= S.osc_s(j)
                S.osc_n(j) = S.osc_n(j) + 1;
            end
            S.osc_s(j) = sg;
        end
    end
    S.osc_t(j) = S.osc_t(j) + dt;
    if S.osc_t(j) > C.osc_win
        S.osc_t(j) = 0;
        S.osc_n(j) = 0;
    end
    if S.osc_n(j) >= 6
        S.lam(j) = max(0.7 * S.lam(j), C.lam_min);
        S.warn = bitor_d(S.warn, 64);
        S.osc_n(j) = 0;
        S.osc_t(j) = 0;
    end
    % valve gain adaptation: measured rate against the rate requested one loop delay earlier
    kr = mod(S.ri - 1 - S.dly(j) - round(S.vdel(j) / dt), nr) + 1;
    qro = S.rv(j, kr);
    if ~S.hold(j) && abs(qro) > 0.8 && abs(S.ad(j)) < 0.5 * C.amax(j) && abs(S.uj(j)) < 0.95 * C.ulim
        ratio = min(max(S.v(j) / qro, 0.2), 5);
        if qro > 0
            S.Kp(j) = min(max(S.Kp(j) * (1 + C.adapt_rate * dt * (ratio - 1)), C.adapt_lo * S.Kp0(j)), ...
                C.adapt_hi * S.Kp0(j));
        else
            S.Kn(j) = min(max(S.Kn(j) * (1 + C.adapt_rate * dt * (ratio - 1)), C.adapt_lo * S.Kn0(j)), ...
                C.adapt_hi * S.Kn0(j));
        end
    end
    % joint at rest does not move although a correction is requested: after 1 s the dead
    % zone is increased (error above e_out), after 2 s a small error is accepted (hold)
    if ~S.hold(j) && ~moving && abs(e) > C.e_in && abs(S.vs(j)) < max(0.3 * abs(qr), 2 * S.vsig(j))
        S.stuck(j) = S.stuck(j) + dt;
    else
        S.stuck(j) = 0;
    end
    if S.stuck(j) > 1.0 && abs(e) > C.e_out
        if qr > 0
            S.dbp(j) = S.dbp(j) + C.db_rate * dt;
        else
            S.dbn(j) = S.dbn(j) + C.db_rate * dt;
        end
    end
    if S.stuck(j) > 2.0 && abs(e) <= C.e_out
        S.hold(j) = true;
        S.stuck(j) = 0;
    end
    % dead zone learning in slow moves: joint behind the reference -> larger, ahead -> smaller
    kp = mod(kd - 2, nr) + 1;
    vdl = (S.rq(j, kd) - S.rq(j, kp)) / dt;
    if ~S.hold(j) && abs(vdl) > 0.05 && abs(vdl) < 0.8
        if vdl > 0
            S.dbp(j) = S.dbp(j) + C.db_learn * e * dt;
        else
            S.dbn(j) = S.dbn(j) - C.db_learn * e * dt;
        end
    end
    S.dbp(j) = min(max(S.dbp(j), 0.5 * S.dbp0(j)), max(1.5 * S.dbp0(j) + 0.05 * C.umax, S.dbp0(j) + C.db_up));
    S.dbn(j) = min(max(S.dbn(j), 0.5 * S.dbn0(j)), max(1.5 * S.dbn0(j) + 0.05 * C.umax, S.dbn0(j) + C.db_up));
end
S.rv(:, S.ri) = S.qr_dot;
end


function c = valve_open(S, j, xr, s, pos)
% valve opening in the joint direction for the piston (slew: joint) rate xr, with the dead
% zone of the direction pos (true: joint angle increasing) weighted by s
if pos
    c = S.dbp(j) * s + xr / max(S.Kp(j), 1e-9);
else
    c = -S.dbn(j) * s + xr / max(S.Kn(j), 1e-9);
end
end


function S = rb_out(S, C, run)
% valve channel of each joint, limits and rate limit
uc = zeros(4, 1);
if run
    if S.phase == 1
        uc = S.uch;
    else
        for j = 1:4
            if S.ok(j)
                uc(S.map(j)) = S.uj(j);
            end
        end
    end
    for i = 1:4
        uc(i) = min(max(uc(i), -C.ulim), C.ulim);
        du = min(max(uc(i) - S.uout(i), -C.du_max * C.dt), C.du_max * C.dt);
        S.uout(i) = S.uout(i) + du;
    end
else
    S.uout = uc;
end
end


% =================================================================================== VDC forces
function S = rb_vdc(S, C)
% required link twists and wrenches (planar VDC recursion) and required cylinder forces
d2r = pi / 180;
q = S.q(2:4) * d2r;
qd = S.v(2:4) * d2r;
qr = S.qr_dot(2:4) * d2r;
qrr = zeros(3, 1);
nr = size(S.rq, 2);
for i = 1:3
    j = i + 1;
    kd = mod(S.ri - 1 - S.dly(j), nr) + 1;
    kp = mod(kd - 2, nr) + 1;
    vdl = (S.rq(j, kd) - S.rq(j, kp)) / C.dt;
    qrr(i) = (S.ad(j) + S.lam(j) * (vdl - S.v(j))) * d2r;
end
vrx = 0;
vry = 0;
vdx = 0;
vdy = 0;
vhx = 0;
vhy = 0;
wr = 0;
wd = 0;
wh = 0;
dx = 0;
ph = 0;
F = zeros(3, 3);
for i = 1:3
    c = cos(q(i));
    s = sin(q(i));
    ax = vrx;
    ay = vry + wr * dx;
    bx = vdx;
    by = vdy + wd * dx;
    hx = vhx;
    hy = vhy + wh * dx;
    nvrx = c * ax + s * ay;
    nvry = -s * ax + c * ay;
    nvhx = c * hx + s * hy;
    nvhy = -s * hx + c * hy;
    nvdx = qd(i) * nvry + (c * bx + s * by);
    nvdy = -qd(i) * nvrx + (-s * bx + c * by);
    vrx = nvrx;
    vry = nvry;
    vhx = nvhx;
    vhy = nvhy;
    vdx = nvdx;
    vdy = nvdy;
    wr = wr + qr(i);
    wd = wd + qrr(i);
    wh = wh + qd(i);
    ph = ph + q(i);
    m = C.mass(i, 1);
    cs = C.mass(i, 2);
    cn = C.mass(i, 3);
    ic = C.mass(i, 4);
    acx = vdx - wh * vry - wd * cn - wh * wr * cs;
    acy = vdy + wh * vrx + wd * cs - wh * wr * cn;
    ex = acx + C.g0 * sin(ph);
    ey = acy + C.g0 * cos(ph);
    F(1, i) = m * ex + C.kv * m * (vrx - vhx);
    F(2, i) = m * ey + C.kv * m * (vry - vhy);
    F(3, i) = ic * wd + m * (cs * ey - cn * ex) + C.kv * (ic + m * (cs * cs + cn * cn)) * (wr - wh);
    dx = C.llen(i);
end
for i = 2:-1:1
    c = cos(q(i + 1));
    s = sin(q(i + 1));
    cfx = c * F(1, i + 1) - s * F(2, i + 1);
    cfy = s * F(1, i + 1) + c * F(2, i + 1);
    F(1, i) = F(1, i) + cfx;
    F(2, i) = F(2, i) + cfy;
    F(3, i) = F(3, i) + F(3, i + 1) + C.llen(i) * cfy;
end
for i = 1:3
    m = cyl_arm(C, i + 1, S.q(i + 1));
    if abs(m) < 0.05
        if m < 0
            m = -0.05;
        else
            m = 0.05;
        end
    end
    S.fr(i) = F(3, i) / m;
end
end


% =================================================================================== IMU observer
function O = obs_init()
O.init = false;
O.valid = false;
O.x = zeros(3, 1);                  % lift, tilt, crowd angle [rad]
O.b = zeros(3, 1);                  % gyro bias estimates of the joint rates [rad/s]
O.lp = zeros(15, 4);                % 20 Hz low-pass states
O.wzf = zeros(3, 1);
O.qdr = zeros(3, 1);                % joint rates [rad/s]
O.wt = zeros(3, 1);                 % weights of the accelerometer measurements
O.g = zeros(3, 3);                  % low-passed specific force per IMU (columns) [m/s^2]
O.wf = zeros(3, 3);                 % low-passed rates, 0.2 s [rad/s]
O.ws = zeros(3, 3);                 % low-passed rates, 2 s [rad/s]
O.bg = zeros(3, 3);                 % gyro bias per IMU for the slew rate [rad/s]
O.still_t = 0;                      % time the boom has been still [s]
O.still_n = 0;                      % time used for the gyro bias [s]
O.ready = false;                    % gyro bias of the slew rate estimated
O.yf = 0;                           % slew rate low-passed, 0.2 s and 2 s [rad/s]
O.ys = 0;
O.sstill_t = 0;                     % time the slew has been at rest [s]
O.fl = zeros(3, 3);                 % previous IMU samples (stale data check)
O.wl = zeros(3, 3);
O.nsa = zeros(3, 1);                % samples without change per IMU: accelerations, rates
O.nsg = zeros(3, 1);
O.nbad = 0;                         % invalid samples in a row
O.psi = 0;                          % slew angle from the gyros [rad]
O.psid = 0;                         % slew rate [rad/s]
O.status = 1;
O.q = zeros(3, 1);                  % outputs [deg], [deg/s]
O.qd = zeros(3, 1);
O.slew = 0;
O.slewd = 0;
end


function O = obs_step(O, C, acc1, gyr1, acc2, gyr2, acc3, gyr3, uzero, sclosed)
% Joint angles from the link IMUs (confidence-weighted pairwise observer) and the slew angle
% from the gyro rates about the gravity direction. acc in m/s^2, gyro in deg/s (EtherCAT axes).
% uzero: all valve commands are 0, sclosed: the slew valve command is 0.
dt = C.dt;
f = zeros(3, 3);
om = zeros(3, 3);
for r = 1:3
    for i = 1:3
        f(r, 1) = f(r, 1) + C.ob_C1(r, i) * double(acc1(i));
        f(r, 2) = f(r, 2) + C.ob_C2(r, i) * double(acc2(i));
        f(r, 3) = f(r, 3) + C.ob_C3(r, i) * double(acc3(i));
        om(r, 1) = om(r, 1) + C.ob_C1(r, i) * double(gyr1(i));
        om(r, 2) = om(r, 2) + C.ob_C2(r, i) * double(gyr2(i));
        om(r, 3) = om(r, 3) + C.ob_C3(r, i) * double(gyr3(i));
    end
end
for k = 1:3
    for r = 1:3
        om(r, k) = om(r, k) * C.ob_gunit * (pi / 180);
    end
end
valid = true;
for k = 1:3
    for r = 1:3
        if ~isfinite(f(r, k)) || ~isfinite(om(r, k))
            valid = false;
        end
    end
end
if valid
    % invalid: an IMU whose values have not changed (device lost and last values held: all six
    % for ob_nstale samples; a stuck sensor: three accelerations for ob_nsa or three rates for
    % ob_nsg samples), or before the start a specific force far from 1 g (no data yet)
    for k = 1:3
        fn = sqrt(f(1, k) * f(1, k) + f(2, k) * f(2, k) + f(3, k) * f(3, k));
        sa = true;
        sg = true;
        for r = 1:3
            if f(r, k) ~= O.fl(r, k)
                sa = false;
            end
            if om(r, k) ~= O.wl(r, k)
                sg = false;
            end
            O.fl(r, k) = f(r, k);
            O.wl(r, k) = om(r, k);
        end
        if sa
            O.nsa(k) = O.nsa(k) + 1;
        else
            O.nsa(k) = 0;
        end
        if sg
            O.nsg(k) = O.nsg(k) + 1;
        else
            O.nsg(k) = 0;
        end
        if min(O.nsa(k), O.nsg(k)) > C.ob_nstale || O.nsa(k) > C.ob_nsa || O.nsg(k) > C.ob_nsg
            valid = false;
        end
        if ~O.init && (fn < 0.5 * C.g0 || fn > 3 * C.g0)
            valid = false;
        end
    end
end
if valid
    first = ~O.init;
    % 20 Hz low-pass restarted from the present sample at the start and after a longer loss of
    % valid data (angles, slew angle and gyro biases kept)
    refill = first || O.nbad > C.ob_nre;
    O.nbad = 0;
    % calibrated z rates and 20 Hz low-pass of the signals of the lever-arm compensation
    wz = zeros(3, 1);
    for k = 1:3
        wz(k) = (om(3, k) - C.ob_gbias(k) - C.ob_ga(k) * om(1, k) - C.ob_gb(k) * om(2, k)) / C.ob_gs(k);
    end
    yf = zeros(5, 3);
    for k = 1:3
        for r = 1:5
            if r == 1
                uu = f(1, k);
            elseif r == 2
                uu = f(2, k);
            elseif r == 3
                uu = wz(k);
            elseif r == 4
                uu = om(1, k);
            else
                uu = om(2, k);
            end
            i = (k - 1) * 5 + r;
            if refill
                O.lp(i, 1) = uu;
                O.lp(i, 2) = uu;
                O.lp(i, 3) = uu;
                O.lp(i, 4) = uu;
            end
            y = C.ob_lpf(1) * uu + C.ob_lpf(2) * O.lp(i, 1) + C.ob_lpf(3) * O.lp(i, 2) ...
                - C.ob_lpf(4) * O.lp(i, 3) - C.ob_lpf(5) * O.lp(i, 4);
            O.lp(i, 2) = O.lp(i, 1);
            O.lp(i, 1) = uu;
            O.lp(i, 4) = O.lp(i, 3);
            O.lp(i, 3) = y;
            yf(r, k) = y;
        end
    end
    al = zeros(3, 1);
    for k = 1:3
        if ~refill
            al(k) = (yf(3, k) - O.wzf(k)) / dt;
        end
        O.wzf(k) = yf(3, k);
    end
    % joint angle from the specific forces of the two IMUs next to the joint
    z = zeros(3, 1);
    cx = zeros(3, 1);
    cy = zeros(3, 1);
    px = zeros(3, 1);
    py = zeros(3, 1);
    db = zeros(3, 1);
    for i = 1:3
        [vx, vy] = ob_transport(C, i, yf(1, i), yf(2, i), yf(3, i), yf(4, i), yf(5, i), al(i), false);
        cx(i) = vx;
        cy(i) = vy;
        if i == 1
            px(1) = 0;
            py(1) = C.g0;
            db(1) = C.ob_beta(1);
        else
            [vx, vy] = ob_transport(C, i - 1, yf(1, i - 1), yf(2, i - 1), yf(3, i - 1), yf(4, i - 1), ...
                yf(5, i - 1), al(i - 1), true);
            px(i) = vx;
            py(i) = vy;
            db(i) = C.ob_beta(i) - C.ob_beta(i - 1);
        end
        z(i) = wrapang(atan2(cx(i), cy(i)) - atan2(px(i), py(i)) - db(i));
    end
    if first
        O.x = z;
        O.b = zeros(3, 1);
    end
    % weighted PI update
    for i = 1:3
        e = wrapang(z(i) - O.x(i));
        th = O.x(i) + db(i);
        ct = cos(th);
        st = sin(th);
        ux = ct * cx(i) - st * cy(i) - px(i);
        uy = st * cx(i) + ct * cy(i) - py(i);
        res = sqrt(ux * ux + uy * uy) / C.ob_c;
        O.wt(i) = max(exp(-(res * res)), C.ob_wmin);
        if i == 1
            wrel = wz(1);
        else
            wrel = wz(i) - wz(i - 1);
        end
        d = wrel - O.b(i) + C.ob_kP * O.wt(i) * e;
        O.x(i) = O.x(i) + dt * d;
        O.b(i) = O.b(i) - dt * C.ob_kI * O.wt(i) * e;
        O.qdr(i) = wrel - O.b(i);
    end
    % slew rate: gyro rate about the gravity direction; gyro bias estimated while the boom is
    % still (valves closed, rates small and constant)
    ag = dt / (C.ob_tg + dt);
    af = dt / (0.2 + dt);
    as = dt / (2.0 + dt);
    if first
        O.g = f;
        O.wf = om;
        O.ws = om;
    end
    still = uzero;
    for k = 1:3
        dw = 0;
        wn = 0;
        for r = 1:3
            O.g(r, k) = O.g(r, k) + ag * (f(r, k) - O.g(r, k));
            O.wf(r, k) = O.wf(r, k) + af * (om(r, k) - O.wf(r, k));
            O.ws(r, k) = O.ws(r, k) + as * (om(r, k) - O.ws(r, k));
            dw = dw + (O.wf(r, k) - O.ws(r, k)) * (O.wf(r, k) - O.ws(r, k));
            wn = wn + O.wf(r, k) * O.wf(r, k);
        end
        if sqrt(dw) > C.ob_dw || sqrt(wn) > C.ob_wmax
            still = false;
        end
    end
    if still
        O.still_t = O.still_t + dt;
    else
        O.still_t = 0;
    end
    if O.still_t >= C.ob_tstill
        if O.still_n < C.ob_tavg
            kb = 1 / (O.still_n / dt + 1);
        else
            kb = dt / (C.ob_tb + dt);
        end
        for k = 1:3
            for r = 1:3
                O.bg(r, k) = O.bg(r, k) + kb * (om(r, k) - O.bg(r, k));
            end
        end
        O.still_n = O.still_n + dt;
    end
    wy = 0;
    ws = 0;
    for k = 1:3
        if C.ob_yw(k) > 0
            gn = sqrt(O.g(1, k) * O.g(1, k) + O.g(2, k) * O.g(2, k) + O.g(3, k) * O.g(3, k));
            if gn > 0.5 * C.g0
                for r = 1:3
                    wy = wy + C.ob_yw(k) * (om(r, k) - O.bg(r, k)) * O.g(r, k) / gn;
                end
                wy = wy - C.ob_yw(k) * C.ob_yc(k) * (om(3, k) - O.bg(3, k));
                ws = ws + C.ob_yw(k);
            end
        end
    end
    if ws > 0
        wy = wy / ws;
    end
    % slew at rest (slew valve closed, slew rate small and constant): slew angle held
    if first
        O.yf = wy;
        O.ys = wy;
    end
    O.yf = O.yf + af * (wy - O.yf);
    O.ys = O.ys + as * (wy - O.ys);
    if sclosed && abs(O.yf) < C.ob_wmax && abs(O.yf - O.ys) < C.ob_dw
        O.sstill_t = O.sstill_t + dt;
    else
        O.sstill_t = 0;
    end
    if O.still_t >= C.ob_tstill || O.sstill_t >= C.ob_tstill
        wy = 0;
    end
    O.psid = wy;
    O.psi = O.psi + dt * wy;
    O.init = true;
else
    % short gap of valid data: angles continued with the last rates
    O.nbad = O.nbad + 1;
    if O.init && O.nbad <= C.ob_nre
        for i = 1:3
            O.x(i) = O.x(i) + dt * O.qdr(i);
        end
        O.psi = O.psi + dt * O.psid;
    end
end
O.valid = valid;
O.ready = O.init && O.still_n >= C.ob_tready;
O.status = 0;
if ~O.init
    O.status = O.status + 1;
end
if ~valid
    O.status = O.status + 2;
end
if ~O.ready
    O.status = O.status + 8;
end
for i = 1:3
    O.q(i) = O.x(i) * (180 / pi);
    O.qd(i) = O.qdr(i) * (180 / pi);
end
O.slew = O.psi * (180 / pi);
O.slewd = O.psid * (180 / pi);
end


function [vx, vy] = ob_transport(C, k, fx, fy, wk, wx, wy, ak, distal)
% specific force of IMU k at its proximal joint (distal = false) or distal joint (distal = true)
vx = fx - C.ob_bx(k);
vy = fy - C.ob_by(k);
rs = C.ob_s(k);
rn = C.ob_n(k);
if distal
    rs = rs - C.ob_L(k);
end
a_s = -ak * rn - wk * wk * rs;
a_n = ak * rs - wk * wk * rn;
cb = cos(C.ob_beta(k));
sb = sin(C.ob_beta(k));
rx = cb * rs + sb * rn;
ry = -sb * rs + cb * rn;
dp = wx * rx + wy * ry;
n2 = wx * wx + wy * wy;
vx = vx - (cb * a_s + sb * a_n + wx * dp - n2 * rx);
vy = vy - (-sb * a_s + cb * a_n + wy * dp - n2 * ry);
end


function a = wrapang(a)
a = mod(a + pi, 2 * pi) - pi;
end


function y = bitor_d(a, b)
% bitwise or of two non-negative integer-valued doubles
y = a;
if mod(floor(a / b), 2) == 0
    y = a + b;
end
end
