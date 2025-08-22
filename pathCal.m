function pose = pathCal(n, side)
% 사각형 경로 생성
% n     : 전체 분할 수 (모서리당 n/4개씩)
% side  : 한 변의 길이 (정사각형)

% 중심에서 시작해서 시계방향으로 사각형 이동
half = side / 2;
z = -400;  % 고정된 높이

% 꼭짓점
P1 = [-half,  half, z];
P2 = [ half,  half, z];
P3 = [ half, -half, z];
P4 = [-half, -half, z];

% 모서리당 포인트 수
m = round(n / 4);

% 선형 보간: 각 모서리를 m개 포인트로
L1 = [linspace(P1(1), P2(1), m)', linspace(P1(2), P2(2), m)', linspace(P1(3), P2(3), m)'];
L2 = [linspace(P2(1), P3(1), m)', linspace(P2(2), P3(2), m)', linspace(P2(3), P3(3), m)'];
L3 = [linspace(P3(1), P4(1), m)', linspace(P3(2), P4(2), m)', linspace(P3(3), P4(3), m)'];
L4 = [linspace(P4(1), P1(1), m)', linspace(P4(2), P1(2), m)', linspace(P4(3), P1(3), m)'];

% 전체 경로 연결
pose = [L1; L2; L3; L4];
end
