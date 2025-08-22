% 하부링크 STL 불러오기
model = stlread('lower_link2.stl');
points = model.Points;
faces  = model.ConnectivityList;

% Z축 방향 → X축 방향 (Y축 -90° 회전)
theta = -pi/2;
Ry = [cos(theta),  0, sin(theta);
      0,           1, 0;
     -sin(theta),  0, cos(theta)];
points_rot = (Ry * points')';

% 회전된 모델 표시
trisurf(faces, points_rot(:,1), points_rot(:,2), points_rot(:,3), ...
    'FaceColor', [0.7 0.85 1.0], 'EdgeColor', 'k');
axis equal; grid on; view(3);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Lower Link (rotated to +X axis)');

% 중심점 찍기
ctr = mean(points_rot, 1);
hold on;
plot3(ctr(1), ctr(2), ctr(3), 'r*', 'MarkerSize', 10);

% 변환된 모델을 struct로 저장해서 run.m에서 사용
lowerLinkModel = struct('ConnectivityList', faces, 'Points', points_rot);
