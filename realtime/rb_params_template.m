function P = rb_params()
%RB_PARAMS Machine-specific parameters of rb_vdc_imu (template).
%   Copy this file to rb_params.m in the same folder (the function name above matches the name of
%   the copy), enter the values of your machine and delete the error line below. rb_params.m is
%   listed in .gitignore.
%
%   Conventions (as in rb_vdc_imu): boom plane with x along the reach from the slew axis and y up;
%   lift angle = angle of the lift link (lift joint to tilt joint) from x, tilt and crowd angles
%   relative to the preceding link, all positive counter-clockwise (x to y) [deg]; the angle of the
%   tool link (lift + tilt + crowd) is the hammer axis angle.

error('rb_params:template', 'Create rb_params.m from rb_params_template.m and enter the machine parameters.');

% ---------------------------------------------------------------- link geometry and masses
P.J1 = [NaN; NaN];          % lift joint in the boom plane: reach from the slew axis, height [m]
P.L1 = NaN;                 % lift link: lift joint to tilt joint [m]
P.L2 = NaN;                 % dipper: tilt joint to crowd joint [m]
P.L3 = NaN;                 % tool: crowd joint to the chisel tip [m]
P.dl = NaN;                 % angle of the line from the crowd joint to the tip, relative to the hammer axis [rad]
% cylinders, rows lift, tilt, crowd: [c_x, c_y, r_x, r_y] [m]
%   c: joint relative to the barrel-side pin, in the frame of the link that carries the barrel
%   r: rod-side pin relative to the joint, in the frame of the link moved by the cylinder
P.cyl = NaN(3, 4);
% links, rows lift, dipper, tool (with the breaker), cylinder masses lumped to the links:
%   [m, c_s, c_n, I_c]: mass [kg]; centre of mass from the proximal joint along and normal to the
%   link [m]; moment of inertia about the centre of mass, axis normal to the boom plane [kg m^2]
P.mass = NaN(3, 4);

% ---------------------------------------------------------------- IMU installation and calibration
% IMU 1 lift link, IMU 2 dipper, IMU 3 tool link. Identify for each installation from a log with
% slewing; verify the axis mappings and the gyro unit in mode 1 against the encoders before mode 2.
% Observer axes of IMU k: x in the boom plane, rotated by ob_beta(k) from the link; y in the boom
% plane, 90 deg counter-clockwise from x; z normal to the boom plane.
P.ob_C1 = NaN(3, 3);        % IMU 1 axes to observer axes
P.ob_C2 = NaN(3, 3);        % IMU 2 axes to observer axes
P.ob_C3 = NaN(3, 3);        % IMU 3 axes to observer axes
P.ob_gunit = NaN;           % gyro inputs x ob_gunit = deg/s (1 for gyro signals in deg/s)
P.ob_s = NaN(1, 3);         % IMU position along the link from the proximal joint [m]
P.ob_n = NaN(1, 3);         % IMU position normal to the link [m]
P.ob_beta = NaN(1, 3);      % mounting angle [rad]
P.ob_bx = NaN(1, 3);        % accelerometer bias x [m/s^2]
P.ob_by = NaN(1, 3);        % accelerometer bias y [m/s^2]
P.ob_gs = NaN(1, 3);        % z gyro scale
P.ob_ga = NaN(1, 3);        % z gyro sensitivity to x rate
P.ob_gb = NaN(1, 3);        % z gyro sensitivity to y rate
P.ob_gbias = NaN(1, 3);     % z gyro bias [rad/s]
P.ob_yc = NaN(1, 3);        % share of the link pitch rate seen in the slew rate (IMUs with ob_yw > 0)

% ---------------------------------------------------------------- home pose, planner limits, strike points
P.home = NaN(4, 1);         % home: slew from start [deg], lift, tilt, crowd [deg]
P.phi = NaN;                % hammer axis angle during the boulder sequence [deg]
P.bld = NaN(4, 3);          % 4 boulders: reach [m], top height [m], slew from start [deg]
P.lim_lo = NaN(4, 1);       % joint limits used by the planner [deg] (slew from start)
P.lim_hi = NaN(4, 1);
end
