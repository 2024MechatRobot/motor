import numpy as np

def delta_inverse_kinematics(x, y, z, r_base=100, r_platform=50, l_upper=200, l_lower=400):
    """
    3축 델타 로봇의 역기구학 계산
    x, y, z: 엔드 이펙터의 목표 위치 (mm)
    r_base: 상단 플레이트 반지름 (mm)
    r_platform: 하단 플레이트 반지름 (mm)
    l_upper: 상단 링크 길이 (mm)
    l_lower: 하단 링크 길이 (mm)
    """
    # 상단 플레이트에서의 모터 위치 (각도 120도 간격)
    angles = [0, 120, 240]
    motor_positions = [(r_base * np.cos(np.radians(a)), r_base * np.sin(np.radians(a))) for a in angles]
    
    # 하단 플레이트 중심 좌표 보정
    x -= r_platform * np.cos(np.radians(30))
    y -= r_platform * np.sin(np.radians(30))
    
    theta_solutions = []
    
    for (xm, ym) in motor_positions:
        dx = x - xm
        dy = y - ym
        dz = z
        
        # 링크 길이 관계식에서 각도 계산
        d = np.sqrt(dx**2 + dy**2 + dz**2)
        
        if d > (l_upper + l_lower) or d < abs(l_upper - l_lower):
            raise ValueError("도달할 수 없는 위치입니다.")
        
        cos_theta = (l_upper**2 + d**2 - l_lower**2) / (2 * l_upper * d)
        theta = np.arccos(np.clip(cos_theta, -1, 1))
        
        # x, y 평면에서의 회전 각도
        base_angle = np.arctan2(dz, np.sqrt(dx**2 + dy**2))
        final_angle = base_angle - theta
        
        theta_solutions.append(np.degrees(final_angle))
    
    return theta_solutions

# 예제 실행
goal_position = (50, 50, -300)
theta_values = delta_inverse_kinematics(*goal_position)
print("각 모터의 역기구학 각도:", theta_values)
