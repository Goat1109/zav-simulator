# ARGO 웹 제어

노트북/휴대폰의 브라우저에서 기존 argo 작업을 실행하고 상태·로그를 확인한다. 서버는 Radxa의 argo 컨테이너 안에서 동작한다. 접속한 노트북/휴대폰에서 ROS를 설치할 필요는 없다.

## 전원 인가 후 휴대폰 접속

현재 Radxa에는 부팅 자동 실행을 등록했다. **Radxa에도 전원을 공급**하고, Radxa와 휴대폰을 같은 Wi-Fi/핫스팟에 연결한 뒤 아래 주소를 연다. SSH 접속이나 `argo`, `zweb` 입력은 필요 없다.

**고정 주소: `http://argo.local:8088`**

접속 코드는 기존 `zweb` 출력의 값을 사용하며 재부팅해도 유지된다. 코드 입력까지 생략하려면 `zweb`이 출력하는 **개인 북마크** 전체 주소를 휴대폰 즐겨찾기에 직접 저장한다. 형식은 `http://argo.local:8088/#code=접속코드`이다. 이 링크는 로그인 권한을 포함하므로 본인 기기에서 사용한다. 페이지는 로그인 후 주소창에서 코드 부분을 지우므로, 로그인 후 주소창을 그대로 북마크하면 코드가 빠진다.

부팅 시 **argo 컨테이너·웹 서버·접속 이름 등록만** 시작한다. 센서·브리지·Bag·zcomm·시험은 자동으로 시작하지 않는다. RC 시동/모드 전환/착륙 절차도 바뀌지 않는다.

`argo.local`은 Avahi로 각 유선/Wi-Fi 인터페이스의 현재 IPv4를 알린다. DHCP 주소가 바뀌면 약 5초 내 갱신한다. 시스템 호스트 이름, Wi-Fi SSID, IP 설정은 변경하지 않는다. 휴대폰/핫스팟에서 `.local` 이름 검색이나 기기 간 통신을 막으면 이 주소로 접속할 수 없다. 이 경우 현재 IP로 먼저 접속하고 해당 네트워크 설정을 확인한다. 현재 Wi-Fi 주소 예시는 `http://192.168.0.13:8088`, 유선 주소는 `http://192.168.1.50:8088`이다. 휴대폰 실제 접속과 전원 재인가 시험은 별도로 확인해야 한다.

## 수동 실행·자동 실행 관리

현재 URL과 접속 코드·개인 북마크 주소를 다시 확인하려면:

```bash
argo
zweb
```

`zweb`은 기존 별칭 `/root/argo_ws/scripts/run_web_remote.sh`를 사용한다. 서버를 백그라운드로 시작하고 URL과 접속 코드를 출력한다. SSH 창을 닫아도 서버는 유지된다. 다시 `zweb`을 입력하면 중복 서버를 띄우지 않고 현재 URL/코드를 보여준다. 새 패키지 빌드는 필요 없다.

접속 세션은 12시간이며 서버를 다시 시작하면 기존 세션은 끝난다. 개인 북마크 링크를 열면 유지된 코드로 다시 로그인한다.

자동 실행은 **Radxa 호스트의 radxa 사용자 서비스**이며 로그아웃 상태에서도 시작하도록 linger를 설정했다. Docker 서비스는 기존대로 부팅 시 실행된다. 웹 서비스가 Docker 준비를 기다린 뒤 `argo_dev`를 시작하고 웹 서버를 감독한다. 컨테이너/웹 프로세스가 종료되면 다시 시도한다. ROS 작업을 자동 재실행하지 않는다.

```bash
# 아래 명령은 argo 컨테이너 밖, Radxa 호스트에서 실행
systemctl --user status argo-web.service argo-web-name.service
journalctl --user -u argo-web.service -u argo-web-name.service -n 40

# 웹 서버만 재시작 (착륙·비무장 상태에서 수행)
systemctl --user restart argo-web.service

# 자동 실행 해제 및 웹/이름 등록 중단. 이미 실행한 ROS 작업은 유지됨
systemctl --user disable --now argo-web.service argo-web-name.service

# 다시 설치/활성화
bash ~/argo_ws/scripts/install_web_autostart.sh
```

설치 원본은 `scripts/web_control/argo-web.service`, `argo-web-name.service`, `boot_web.sh`, `publish_name.py`다. 사용자 서비스 사본은 `~/.config/systemd/user/`에 있다. 설치 스크립트는 `~/argo_ws`와 기존 `argo_dev` 컨테이너를 사용한다.

현재 컨테이너는 host 네트워크를 사용하므로 포트 매핑은 추가하지 않는다. 접속되지 않으면 같은 네트워크인지, 선택한 Radxa IP가 현재 주소인지, 핫스팟의 기기 간 통신 차단/방화벽이 있는지 확인한다. 수동 서버는 `zweb --port 8089`로 포트를 지정할 수 있다. 자동 실행 서비스의 포트를 바꾸려면 `boot_web.sh`의 실행 인자와 공개 URL도 함께 바꾼다. 사설 LAN용 서버이며 인터넷 공개/포트 포워딩을 설정하지 않았다.

서버 시작 자체는 `/zav_real/status` 구독과 노드/발행자 조회만 시작한다. 센서·브리지·이동 요청을 자동 실행하지 않는다. 카메라/PointCloud/odometry 원본을 추가 구독하지 않는다.

## 화면과 지원 작업

화면은 SUPER 임무 → RTH → Offboard 시험 순서로 구분한다. SUPER 임무에는 전방 거리 이동과 `Patrol · 7m 정사각형`이 있다. Patrol은 각 코너에서 정지하고 바깥쪽을 바라본다. RTH는 이륙 전에 저장한 Home을 사용하며 진행 중 SUPER 임무를 교체할 수도 있다. Home XY와 `Home Z+0.4m`를 단일 3D 목표로 보내고 SUPER가 회피 경로를 계산한다. 목표에서 정지한 뒤 PX4 착륙으로 넘긴다. 다층 복귀는 지도에 층 사이의 연결 경로가 있고 현재 지도 범위·가상 고도 제한 안에서 경로가 성립해야 한다. `COM_DISARM_LAND`가 0초 초과 30초 이하로 읽혀야 한다. 센서 좌표계가 리셋되거나 Home이 없으면 거절한다.

`자동 이륙 · 40cm 호버링`은 기존 센서/브리지 작업과 중복 실행할 수 없다. 초기 호버 기준은 `src/zav_super/config/automatic_takeoff.json`의 0.31을 자동 사용하며 매번 입력하지 않는다. 지상·비무장 상태에서 버튼 확인을 하면 SUPER와 브리지를 함께 실행한다. 시작 시 FC에서 읽은 `MPC_THR_HOVER`와 비교하는 검사는 유지한다. RC8 HIGH·Kill OFF·스로틀 최저·정상 EV/FC 조건에서 실제 Offboard 전환과 정상 시동, OMMPC 수직 상승을 요청한다. 이륙 후 `HOVER`에서 전방 이동·Patrol을 시작한다. Position으로 회수하려면 swB를 Position에 두고, Offboard 상태에서 스로틀을 중앙으로 맞춘 뒤 RC8 LOW로 전환하고 실제 FC Position을 확인한다. 자동 이륙 때의 최저 스로틀을 그대로 두면 Position 전환 후 하강 입력이 된다. 이 버튼은 모터가 실제로 돌 수 있는 동작이며 부팅·페이지 접속·새로고침으로 자동 실행되지 않는다. 자세한 기준점·단계·실패 처리는 [SUPER 운용 문서](super_hardware.md)의 2026-11-25 절을 참조한다.

상단 Radxa 사용량에서 전체 CPU와 RAM을 확인한다. CPU는 전체 코어를 합쳐 100% 기준이며, RAM은 `MemTotal - MemAvailable` 사용량과 전체 용량을 GiB로 표시한다. 브리지가 없어도 조회할 수 있다. 기존 상태 요청에 포함하며 시스템 수치는 여러 접속자가 공유해 최대 초당 한 번 읽는다. 페이지가 닫히면 추가 수집을 하지 않고, 별도 ROS 구독·프로세스·기록 파일을 만들지 않는다. 초당 요약값이므로 짧은 부하 급증이나 EV 지연 원인을 확정하는 기록은 아니다.

| 화면 | 기능 |
|---|---|
| 기체 운용 | 실제 FC 모드/시동, 배터리, LIO Hz/지연, EV/failsafe, 기존 state 텍스트 |
| 센서·추정기 | `super`, `livo`, `lio`, `super_livo`, `super_lio` |
| 카메라 단독 | 카메라 없는 SUPER/LIO 구성에 영상 녹화·zcomm용 드라이버 추가 |
| FC 브리지 | 기존 `bridge` 별칭의 SUPER 브리지, 추가 `bridge_legacy` 기존 시험 브리지 |
| SUPER 임무 | `super_example`, `super_patrol` |
| RTH | `super_rth` · 복귀·착륙 |
| Offboard 시험 | `go_front/right/back/left`, `waypoint_square/circle` |
| 기록·전송 | `bag_livo/lio/superlivo/superlio`, `zcomm_fast_livo`, `zcomm_super_lio` |
| 저장된 Bag | 폴더·크기 확인, 저장된 metadata.yaml 및 db3/mcap 다운로드/이어받기 |
| 실행 로그 | 실행 상태, 터미널 출력, 복사, 개별 종료, 활동 기록 |
| PX4 진단 | 기존 `px4` USB 콘솔, 읽기 전용 `ver all`, `ver hw`, `mavlink/ekf2/commander status` |

`state`는 웹 안에 통합되어 별도 터미널 프로세스를 만들지 않는다. `zweb`은 이 화면을 여는 서버 명령이다. 임의 쉘 명령 입력, PX4 무장·펌웨어 업로드·파라미터 쓰기는 제공하지 않는다.

서버 시작 시 `/root/.bashrc`의 알려진 별칭을 읽어서 해당 실행 인자를 반영한다. 허용된 ROS launch/service 경로만 사용하며 브라우저가 실행 파일이나 쉘 문자열을 지정할 수 없다. Bag 스크립트와 PX4 진단 경로는 이 워크스페이스에 고정된다. 별칭을 바꾸면 지상에서 웹 서버를 다시 시작한다.

## SUPER 운용

1. **SUPER 자율주행**을 실행한다. 기존 LIO/LIVO/SUPER-LIO를 따로 중복 실행하지 않는다.
2. 영상 녹화/zcomm을 함께 쓰려면 지상에서 **카메라 단독**도 실행한다. SUPER의 기본 추정은 카메라를 융합하지 않는다.
3. **SUPER 브리지**를 실행하고 state를 확인한다.
4. 기존 RC 절차로 Position에서 이륙·정지 호버링한다.
5. 준비 표시 확인 후 **조종기의 RC8 HIGH**로 전환한다. 실제 FC 모드 Offboard/HOVER를 확인한다.
6. **SUPER 전방 거리 (m)**에 0m 초과~20m를 입력한다. 기본값은 10m다. **SUPER 전방 이동 → 시험 시작**을 누르고 확인창의 거리를 확인한다. 실제 이동 요청은 확인 후 한 번만 보낸다. 목표는 요청을 수락한 시점의 위치·기체 전방·현재 높이를 기준으로 정하며, 우회 경로 총길이는 입력 거리보다 길 수 있다. 입력만 바꾸면 이동하지 않으며, 이미 수락한 목표도 바뀌지 않는다.
7. 중단/착륙은 **RC8 LOW로 회수 → 실제 Position 확인 → 기존 RC 착륙** 순서다.

SUPER의 원본 common.yaml 속도 상한 20m/s와 기존 Position 속도 제한의 차이는 [SUPER 실기 안내](super_hardware.md)를 따른다. 이 웹 화면이 추력/궤적 설정을 바꾸지 않는다.

## 기존 Offboard 시험

1. `livo`, `lio`, `super_livo`, `super_lio` 중 필요한 센서 구성을 선택한다.
2. **기존 시험 브리지**를 실행한다. SUPER 브리지와 동시에 실행할 수 없다.
3. RC로 Position 이륙·정지, RC8 HIGH로 실제 Offboard 호버링에 진입한다.
4. 원하는 5m 직선/정사각형/원 시험을 선택하고 이동 요청을 확인한다.

SUPER 전용 브리지에서 기존 6개 시험을 보내거나, 기존 시험 브리지에서 SUPER 이동 요청을 보내는 것은 차단한다. 브리지 종류 변경은 착륙·비무장 후 진행한다.

웹의 거리 입력은 `/zav/super/go_forward` (`zav_super/srv/GoForward`) 서비스 한 번으로 거리와 목표 요청을 함께 보낸다. 거리를 바꾸려고 제어기를 재시작하거나 비행 중 ROS 파라미터를 바꿀 필요가 없다. 기존 `super_example` 터미널 별칭은 `/zav/super/go_front`를 호출하므로 실행 시 설정한 기본 거리(기본 10m)를 사용한다. 두 서비스는 같은 정지 호버링·피드백 검사와 Offboard 진입당 목표 1개 제한을 공유한다.

## Bag·zcomm

Bag은 기존 `scripts/run_bag.sh`를 그대로 호출한다. **모드 선택은 폴더 이름 지정이며 센서 구성을 자동으로 바꾸지 않는다.** 저장 위치 `argo_ws/bags/YYMMDD_HHMMSS_fastlivo|fastlio|superlivo|superlio`, LiDAR, IMU와 원본 해상도 JPEG 토픽 `/camera/full/image_raw/compressed`를 녹화한다. LIVO 모드는 시각 오도메트리 재생용 `/camera/image_raw`도 녹화한다. 카메라가 발행되지 않으면 이미지 메시지는 기록되지 않는다.

**녹화 종료·저장**은 SIGINT를 보낸다. 종료·저장 대기 상태가 끝난 뒤 다운로드한다. metadata.yaml이 없는 진행 중 bag의 다운로드는 막는다. metadata.yaml과 모든 db3/mcap 파일을 같은 폴더에 받아야 한다. 큰 파일은 HTTP Range 이어받기를 지원한다.

zcomm은 기존 launch와 `drone_fast_livo.yaml`/`drone_super_lio.yaml`을 사용한다. 토픽·보정·수신 서버 설정은 변경하지 않는다. **실행 중은 프로세스 상태**이며 수신 서버 연결/전송 성공은 해당 로그로 확인한다. 카메라를 사용하는 전송 구성이므로 카메라 발행도 확인한다. 종료 시 기존 노드의 flush 동작을 기다린다.

## 종료·연결 단절

- **페이지 닫기/로그아웃/휴대폰 Wi-Fi 단절/SSH 종료**는 실행 중인 ROS 작업을 종료하지 않는다. Offboard 제어 연산·전송은 Radxa 안에서 계속 실행된다. 웹 연결을 비행용 긴급 정지 수단으로 사용하지 않는다.
- **센서·브리지 종료/재시작**은 시동 ON에서 차단한다. FC 상태가 없으면 지상·비무장 상태를 직접 확인해야 종료할 수 있다. Bag 저장 완료 → 브리지 종료 → 센서 종료 순서로 정리한다.
- 웹 밖에서 이미 실행한 센서/브리지/전송 노드를 감지하면 중복 시작을 차단한다. 그 작업은 시작했던 원래 터미널에서 종료한다. 시스템 전체 `pkill`은 사용하지 않는다.
- 종료 버튼은 이 서버가 생성한 작업의 식별 태그와 프로세스 그룹을 확인하고 SIGINT만 보낸다. Bag/전송 저장이 늦어도 강제로 SIGKILL하지 않는다.
- 수동 서버만 종료하려면 `zweb --stop`을 사용한다. 자동 실행이 켜져 있으면 서비스가 웹 서버를 다시 시작하므로, 계속 중단하려면 호스트에서 `systemctl --user stop argo-web.service`를 사용한다. **센서·브리지 작업은 그대로 유지된다.** 재실행하면 저장한 작업 식별 정보를 확인해 관리 화면에 다시 표시한다. PID가 다른 프로세스로 재사용됐으면 신호를 보내지 않는다.
- 이동 요청은 자동 재시도하지 않는다. HTTP/서비스 응답이 없다는 것이 기체가 움직이지 않았다는 뜻은 아니므로 실제 기체/FC 상태를 먼저 확인한다.

현재 서버/작업 로그·프로세스 기록은 `log/web_control/`에 저장한다. 자동 실행 서버의 표준 출력은 위의 사용자 journal에서도 확인한다. 접속 코드는 같은 폴더의 `access.json`에 권한 600으로 보관하고 기존 `server.json` 코드에서 한 번 이관한다. `zweb` 출력에서도 확인할 수 있다. 접속 코드는 웹 API로 노출하지 않으며 브라우저에는 HttpOnly 세션 쿠키를 사용한다. 명령 API는 접속 코드 인증·동일 Origin·CSRF 토큰·요청 중복 방지를 사용한다.

## 오프라인 검증

```bash
# 실제 프로세스를 시작하지 않는 UI 데모 (가상 작업)
bash scripts/run_web_remote.sh --demo --bind 127.0.0.1 --port 8099

# 웹/API/프로세스 관리 테스트: FC·센서 접근 없음
python3 -m unittest discover -s scripts/web_control/tests -v

# 실제 ROS launch 수명 검증: Python sleep 가짜 프로세스만 실행
ROS_DOMAIN_ID=218 ROS_LOCALHOST_ONLY=1 \
  python3 scripts/web_control/tests/test_launch_lifecycle.py
```

브라우저 스모크 테스트는 `tests/browser_smoke.py`이며 DEMO 서버인지 먼저 검사한다. 노트북/휴대폰 화면, 로그인, 이동 요청 확인, 무장 중 종료 차단, Bag 시작/종료, zcomm 중복 차단을 검증한다. Playwright는 테스트 도구이며 실제 웹 서버 실행에는 필요 없다. 테스트 화면은 `log/hardware_checks/web_control/`에 저장했다. 실제 센서·FC·비행 명령으로 웹 버튼을 검증하지 않았다.


## EPIC 자율탐사

운용 화면의 탐사 설정에서 `gym`, `frontier whole`, `frontier single floor`를 선택한 뒤 **EPIC + SUPER 자율탐사**를 실행한다. 설정은 그 실행에 고정되며, 변경은 착륙·비무장 후 종료·재실행으로 적용한다. 센서·SUPER 브리지는 지상·비무장에서 준비해 시작 위치·방향에 경계를 정렬하고, Position 수동 이륙·정지 호버 → RC8 실제 Offboard → RC7 LOW/HIGH 0.3초 → 중간 0.3초 순서로 탐사를 시작한다.

EPIC 영역에는 실제 FSM, 설정, RC7, 구간과 시작 차단/종료 사유를 표시한다. RC7을 중간에서 빼면 탐사 중단·위치 유지 요청, RC8 LOW는 기존 조종권 회수다. 웹 연결 종료는 비행을 중단하지 않는다. 자세한 경계 좌표·고도 제한·failsafe와 검증 범위는 [EPIC 실기 안내](../../zav_super/EPIC.md)를 따른다.
