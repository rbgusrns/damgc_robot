# GitHub 등록 및 운영 기록

## 현재 등록 상태

2026-07-27 현재 로컬 Git 설정을 확인한 결과입니다.

| 항목 | 확인 결과 |
| --- | --- |
| 원격 저장소 | `https://github.com/rbgusrns/damgc_robot.git` |
| 원격 이름 | `origin` |
| 기본 작업 브랜치 | `main` |
| 로컬 HEAD | `7b0f850` (`follower`) |
| 원격 동기화 | `origin/main`과 일치 |

최근 커밋은 기본 Orin ROS 2 패키지 추가, 리더 패키지 추가, 팔로워 기능 추가 순서로 구성되어 있습니다.

## 최초 등록 절차

```bash
cd /home/maze/damgc_robot
git init
git branch -M main
git add README.md src docs
git commit -m "Initial ROS 2 workspace"
git remote add origin https://github.com/rbgusrns/damgc_robot.git
git push -u origin main
```

이미 `origin`이 등록된 경우에는 `git remote add`를 다시 실행하지 않습니다.

## 변경사항 등록 절차

```bash
source /opt/ros/humble/setup.bash
cd /home/maze/damgc_robot
colcon build --symlink-install
git status
git add docs README.md
git diff --cached
git commit -m "Document project architecture and GitHub workflow"
git push origin main
```

소스 코드가 함께 변경된 경우에는 `git add docs README.md src`를 사용하되, `build/`, `install/`, `log/` 생성물은 커밋하지 않습니다.

## 인증 및 주의사항

- HTTPS push에는 GitHub 계정 인증 또는 Personal Access Token이 필요합니다.
- 토큰, SSH 개인키, 민감한 카메라 설정은 저장소에 올리지 않습니다.
- push 전 `git status`와 `git diff --cached`로 대상 파일을 확인합니다.
- ROS 빌드 생성물은 `.gitignore`에 두고 소스·설정·문서만 버전 관리합니다.

## 등록 확인 체크리스트

- [x] `origin` 원격 URL 등록
- [x] `main` 브랜치 사용
- [x] 리더·팔로워 소스가 `src/` 아래에 정리됨
- [x] ROS 2 Humble `colcon build --symlink-install` 성공
- [x] 프로젝트 공통 문서가 `docs/`에 추가됨
- [ ] 실제 장비에서 리더/팔로워 통합 시험
- [ ] 주행 제어·STM32·그리퍼 연동
