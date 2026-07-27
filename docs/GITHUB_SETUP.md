# GitHub 등록 및 운영 기록

## 현재 등록 상태

2026-07-27 문서 정비 전에 로컬 Git 설정을 확인한 결과입니다. 커밋 해시는 문서가
변경될 때마다 달라지므로 최신 값은 `git log -1 --oneline`으로 확인합니다.

| 항목 | 확인 결과 |
| --- | --- |
| 원격 저장소 | `https://github.com/rbgusrns/damgc_robot.git` |
| 원격 이름 | `origin` |
| 기본 작업 브랜치 | `main` |
| 확인 당시 기준 커밋 | `09990f3` (`plan`) |
| 원격 동기화 | `origin/main`과 일치 |

이후 문서 정비나 소스 변경이 있으면 로컬 HEAD가 위 값과 달라지는 것이 정상입니다.

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
git diff --check
git add docs README.md src
git diff --cached
git commit -m "Align project docs with development plan"
git push origin main
```

실제로 변경한 경로만 `git add`에 넣습니다. `build/`, `install/`, `log/`, `data/`
생성물은 커밋하지 않습니다.

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
- [x] 기준 계획과 현재 상태 문서를 분리함
- [ ] 실제 장비에서 리더/팔로워 통합 시험
- [ ] 주행 제어·STM32·그리퍼 연동
