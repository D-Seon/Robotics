# 🤖 Planar 2-Link & 3-Link Manipulator Kinematics Simulation

로봇공학(Robotics) 교과목 프로젝트로, 평면형 2-Link 및 3-Link 로봇 매니퓰레이터의 정기구학(Forward Kinematics)과 역기구학(Inverse Kinematics)을 수식적으로 유도하고 MATLAB으로 구현·시뮬레이션한 저장소입니다.

## 📌 Project Overview

* **개발 환경:** `MATLAB R2026a`
* **주요 내용:**
  * 다관절 평면 매니퓰레이터의 기구학적 모델링 및 좌표 변환 행렬 도출
  * 관절 각도 입력에 따른 End-Effector 위치 계산 (**Forward Kinematics**)
  * 목표 위치 및 지정된 경로(타원형 궤적 등) 추종을 위한 관절 각도 역산 (**Inverse Kinematics**)
  * MATLAB 기반 실시간 링크 거동 및 궤적 시각화
