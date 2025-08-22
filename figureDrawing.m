function [] = figureDrawing(len, position, angles, con, rot, w_b)
%% STL 파일 불러오기 (한 번만 실행)
persistent base_faces base_points upper_faces upper_points
if isempty(base_faces)
    % base.stl
    base_model = stlread('base.stl');
    base_faces = base_model.ConnectivityList;
    base_points = base_model.Points;

    % upper_link.stl
    upper_model = stlread('upper_link.stl');
    upper_faces = upper_model.ConnectivityList;
    upper_points = upper_model.Points;
end

%% 기본 파라미터
f = len(3);
e = len(4);
L = len(1);
pose = position;
angle = angles;
R = rot;
J1 = con(1,:); J2 = con(2,:); J3 = con(3,:);
E1 = con(4,:); E2 = con(5,:); E3 = con(6,:);
wb = w_b;

%% Base STL 배치
theta_base = deg2rad(60);
Rz_base = [cos(theta_base), -sin(theta_base), 0;
           sin(theta_base),  cos(theta_base), 0;
           0,                0,               1];
base_transformed = (Rz_base * base_points')';
base_transformed(:,3) = base_transformed(:,3) + 95;
trisurf(base_faces, base_transformed(:,1), base_transformed(:,2), base_transformed(:,3), ...
    'FaceColor', [0.8 0.8 0.8], 'EdgeColor', 'none');
hold on;

%% Upper Arm STL 배치
% A점 계산
L1 = [0 -L*cos(angle(1)) -L*sin(angle(1))];
A1 = J1 + L1;
link_vector = A1 - J1;
link_dir_original = [1 0 0];

% 회전 행렬 (Rodrigues)
v = cross(link_dir_original, link_vector);
s = norm(v);
c = dot(link_dir_original, link_vector);
vx = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
if s ~= 0
    R_align = eye(3) + vx + vx^2 * ((1 - c)/(s^2));
else
    R_align = eye(3);
end

% 회전 및 이동
upper_rotated = (R_align * upper_points')';
center = mean(upper_rotated, 1);
upper_translated = upper_rotated + (J1 - center);
trisurf(upper_faces, upper_translated(:,1), upper_translated(:,2), upper_translated(:,3), ...
    'FaceColor', [0.2 0.6 0.8], 'EdgeColor', 'none');

%% Delta 로봇 기본 프레임
B1 = [-sqrt(3)*wb,-wb,0];
B2 = [sqrt(3)*wb,-wb,0];
B3 = [0,2*wb,0];
patch([B1(1) B2(1) B3(1)], [B1(2) B2(2) B3(2)], [B1(3) B2(3) B3(3)], 'white');

JL1 = [-sqrt(3)*wb/10,-wb,0];
JR1 = [sqrt(3)*wb/10,-wb,0];
JL2 = JL1*R; JR2 = JR1*R; JL3 = JL2*R; JR3 = JR2*R;
line([JL1(1) JR1(1)], [JL1(2) JR1(2)], [JL1(3) JR1(3)], 'LineWidth', 3);
line([JL2(1) JR2(1)], [JL2(2) JR2(2)], [JL2(3) JR2(3)], 'LineWidth', 3);
line([JL3(1) JR3(1)], [JL3(2) JR3(2)], [JL3(3) JR3(3)], 'LineWidth', 3);

%% 엔드이펙터 및 링크 시각화
p_e1 = pose+E1; p_e2 = pose+E2; p_e3 = pose+E3;
plot3([p_e1(1) p_e2(1) p_e3(1) p_e1(1)], [p_e1(2) p_e2(2) p_e3(2) p_e1(2)], [p_e1(3) p_e2(3) p_e3(3) p_e1(3)]);
plot3(p_e1(1),p_e1(2),p_e1(3),'.b');
plot3(p_e2(1),p_e2(2),p_e2(3),'.b');
plot3(p_e3(1),p_e3(2),p_e3(3),'.b');

x = pose(1); y = pose(2); z = pose(3);
plot3(x,y,z,'*g'); plot3(x,y,z,'og');

L2 = [0 -L*cos(angle(2)) -L*sin(angle(2))]*R;
L3 = [0 -L*cos(angle(3)) -L*sin(angle(3))]*R*R;
A2 = J2 + L2; A3 = J3 + L3;
line([J2(1) A2(1)], [J2(2) A2(2)], [J2(3) A2(3)], 'LineWidth', 2);
line([J3(1) A3(1)], [J3(2) A3(2)], [J3(3) A3(3)], 'LineWidth', 2);
plot3(A1(1),A1(2),A1(3),'og');
plot3(A2(1),A2(2),A2(3),'og');
plot3(A3(1),A3(2),A3(3),'og');

EL1=pose+[-sqrt(3)*wb/10 -e/sqrt(3) 0];
ER1=pose+[ sqrt(3)*wb/10 -e/sqrt(3) 0];
EL2=pose+[-sqrt(3)*wb/10 -e/sqrt(3) 0]*R;
ER2=pose+[ sqrt(3)*wb/10 -e/sqrt(3) 0]*R;
EL3=pose+[-sqrt(3)*wb/10 -e/sqrt(3) 0]*R*R;
ER3=pose+[ sqrt(3)*wb/10 -e/sqrt(3) 0]*R*R;
AL1=JL1+L1; AR1=JR1+L1;
AL2=JL2+L2; AR2=JR2+L2;
AL3=JL3+L3; AR3=JR3+L3;

plot3([AL1(1) AR1(1) ER1(1) EL1(1) AL1(1)], [AL1(2) AR1(2) ER1(2) EL1(2) AL1(2)], [AL1(3) AR1(3) ER1(3) EL1(3) AL1(3)]);
plot3([AL2(1) AR2(1) ER2(1) EL2(1) AL2(1)], [AL2(2) AR2(2) ER2(2) EL2(2) AL2(2)], [AL2(3) AR2(3) ER2(3) EL2(3) AL2(3)]);
plot3([AL3(1) AR3(1) ER3(1) EL3(1) AL3(1)], [AL3(2) AR3(2) ER3(2) EL3(2) AL3(2)], [AL3(3) AR3(3) ER3(3) EL3(3) AL3(3)]);

text(-420,250,-380,'Angles ( rad )');
text(-420,250,-430,num2str(angle));
text(-420,-100,-400,'position(mm)');
text(-420,-100,-450,sprintf('x=%.1f, y=%.1f, z=%.1f', pose));

drawing = 0;
end
