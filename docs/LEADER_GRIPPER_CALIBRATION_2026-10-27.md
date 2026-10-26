# 리더 그리퍼 보정 (2026-10-27)

- RX-28 ID: 2, 포트: /dev/ttyUSB0, baudrate: 115200.
- 최대 열림: raw 1000 (사용자 지정).
- 집기: raw 477 (사용자가 잡은 자세에서 현재 위치를 3회 읽어 모두 477 확인).
- leader_apriltag_drive의 gripper_open_raw=1000, gripper_close_raw=477.
- cooperative_manual 사용 시 open_raw:=1000 close_raw:=477을 지정한다.
- 공용 gripper_sequence를 직접 실행할 때 open_raw:=1000 close_raw:=477을 지정한다.

이번 보정은 읽기만 수행했다. 모터 목표 위치와 토크는 변경하지 않았다.
