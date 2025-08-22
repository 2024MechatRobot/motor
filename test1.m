%% Delta Robot Simulation – Upper‑Link 1 Rotated −60° about Y‑axis
% -------------------------------------------------------------------------
% 2025‑08‑06 │ 상부 링크 1개를 **Y축** 기준 시계방향 60° 회전한 정적 테스트.
% -------------------------------------------------------------------------
clc; clear; close all;

%% Load STL Models --------------------------------------------------------
base  = stlread('base.stl');
upper = stlread('upper_link.stl');

%% Parameters & hinge / tip positions ------------------------------------
f = 519.6;          % [mm]
L = 175;            % upper‑arm length [mm]
J1 = [0; -f/(2*sqrt(3)); 0];   % hinge (column)

% 시계방향 −60° 회전 행렬 (Y축)
theta = pi/3;                       % −60°
Ry60  = [ cos(theta) 0 sin(theta);
          0          1 0;
         -sin(theta) 0 cos(theta)];  % Y‑axis rotation matrix
L1c   = Ry60 * [0; -L; 0];           % rotated direction vector (column)
A1    = (J1 + L1c).';                % tip (row)

%% Figure ----------------------------------------------------------------
figure; axis equal; view(3); grid on; hold on;
patch('Faces', base.ConnectivityList, 'Vertices', base.Points, ...
      'FaceColor', [0.9 0.9 0.9], 'EdgeColor', 'none');
plot3(J1(1), J1(2), J1(3), 'ro', 'MarkerSize', 8, 'LineWidth', 2);

drawSTL(upper, J1.', A1);
plot3([J1(1) A1(1)], [J1(2) A1(2)], [J1(3) A1(3)], 'k-', 'LineWidth', 2);

title('Upper link 1 rotated −60° clockwise around Y');

%% ─────────────────────────── Functions ──────────────────────────────────
function drawSTL(model, fromPt, toPt, modelAxis)
    if nargin < 4, modelAxis = [0 0 1]; end
    dir = toPt - fromPt; linkLength = norm(dir);
    if linkLength < 1e-6
        R = eye(3);
    else
        rotAxis  = cross(modelAxis, dir);
        rotAngle = acos(dot(modelAxis, dir) / linkLength);
        if norm(rotAxis) < 1e-6
            R = eye(3);
        else
            rotAxis = rotAxis / norm(rotAxis);
            R = axisAngleToRotationMatrix(rotAxis, rotAngle);
        end
    end
    V = (R * model.Points.').' + fromPt;
    patch('Faces', model.ConnectivityList, 'Vertices', V, ...
          'FaceColor', [0.2 0.6 1.0], 'EdgeColor', 'none');
end

function R = axisAngleToRotationMatrix(axis, angle)
    x = axis(1); y = axis(2); z = axis(3);
    c = cos(angle); s = sin(angle); C = 1 - c;
    R = [x*x*C + c,   x*y*C - z*s, x*z*C + y*s; ...
         y*x*C + z*s, y*y*C + c,   y*z*C - x*s; ...
         z*x*C - y*s, z*y*C + x*s, z*z*C + c];
end
