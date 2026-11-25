# PX4 1.17.0 진단 패치 준비 상태

## 실 FC 첫 업로드 후 발견 및 v2 수정

첫 패치 업로드 후 정상 부팅, 949개 기존 파라미터 유지, `zav_sync`의 약 50초 후
수렴을 확인했다. `zav_baro`는 수신되지 않았다. 콘솔에서 실제 EKF 2개와
primary=1을 확인했으며, baro uORB instance 0의 `estimator_instance`가 1,
uORB instance 1의 값이 0으로 역전돼 있었다. BARO_CTRL=0에서 topic을 생성자에서
미리 예약하지 않아 첫 publish 순서에 따라 번호가 뒤바뀐 것이다.
첫 패치는 이 불일치를 발견하면 전송을 차단하도록 되어 있어 READY로 통과하지 않았다.

v2는 EKF2의 AdvertiseTopics에서 baro aid topic을 status topic과 같은 생성 순서로
예약한다. 제어·융합 알고리즘과 다중 EKF 설정은 변경하지 않는다.
실제 첫 publish 순서를 역전시키는 회귀 테스트를 포함해 41 passed, 1 skipped,
hkust_nxt-dual_default 전체 빌드 성공을 확인했다.

v2 업로드 후 TELEM2에서 65초간 확인한 결과 기존 파라미터 949개가 전부 유지됐고,
`zav_baro=0` 및 약 50초 후 수렴한 `zav_sync=1`이 각각 약 10 Hz로 수신됐다.
검증 기록은 `log/hardware_checks/20261029T074631Z_post_flash_v2_check.json`이다.
이는 baro OFF 진단 및 시각 수렴 경로의 검증이며, baro ON 전환·실제 융합·LAND·
EV 융합까지 검증한 것은 아니다. EV 전송과 파라미터 쓰기는 수행하지 않았다.

**이후 사용할 업로드 후보는 `firmware_build/artifacts/hkust_nxt-dual_v1.17.0_zav_diagnostics_v2.px4`다.**
v2 해시와 크기는 `firmware_build/artifacts/manifest_v2.json`에 기록했다.
아래 첫 빌드 기록의 `zav_diagnostics.px4`는 이 수정으로 대체된 첫 버전이므로 재사용하지 않는다.
실 FC 확인 기록은 `log/hardware_checks/20261029T073824Z_post_flash_check.json`과
`log/hardware_checks/20261029T073932Z_baro_diagnostic_console.json`에 있다.

콘솔의 OS 버전 표시는 0.0.0으로 나타나지만 NuttX 소스 해시는 원본과 동일한
fb2fadf6f599c1406f052db013efd00a2518e72c다. shallow checkout의 태그 메타데이터
차이이며 별도 NuttX 버전으로 교체한 것은 아니다.

이 문서는 소스 검토 및 지상 시험 준비 기록이다. 2026-10-29에 실제 타깃의
NuttX 펌웨어 전체 빌드는 성공했다. 업로드, 부팅 및 실 FC 진단 메시지 검증은 아직 수행하지 않았다.

## 확인된 실행 환경

- 실제 FC의 `ver all` 결과: HKUST_NXT_DUAL, 1.17.0, Git `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`.
  이 해시는 공식 v1.17.0 태그 및 보관한 공식 원본 패키지와 일치한다.
- USB-TTL: `/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0`, 921600 baud, FC TELEM2.
- `MAV_1_CONFIG=102`, `MAV_1_MODE=2`, `MAV_1_RATE=40000`, `MAV_PROTO_VER=2` 읽기 확인.
- 센서 ROS 도메인: 119. source `config/hardware.yaml`의 경로·도메인은 사용자 수정 확인.
- 현재 FC에서는 `zav_sync`, `zav_baro`가 아직 관찰되지 않았다.

## 1.17 변경점과 수정안

`EstimatorAidSource1d.msg`에 더 이상 `fusion_enabled`가 없다.
1.17용 패치는 같은 primary EKF의 `estimator_status_flags.cs_baro_hgt`를 읽어
융합 활성 여부를 판단하고, aid sample의 `fused`, `time_last_fuse`,
`innovation_rejected`, `test_ratio`로 실제 융합 성공을 따로 확인한다.
오래된 데이터와 primary 전환 실패 시 성공 진단을 보내지 않는다.
상태 플래그는 정상 상태에서 약 1 Hz로 발행되므로 2초 신선도 제한을 사용하고,
baro sample 및 마지막 융합 시각에는 0.5초 제한을 사용한다.

`zav_sync`는 FC 내부 Timesync 필터의 `sync_converged()`를 전송한다.
1.17의 timesync 클래스는 MAVLink 참조를 사용하므로 `_mavlink.get_channel()`을 사용한다.
SBC에서 TIMESYNC 응답을 받았다는 사실만으로 FC 필터 수렴을 대신하지 않는다.
ODOMETRY는 기존과 같이 측정 epoch 시각을 전달하며 FC가 offset을 적용한다.

검토한 공식 소스는 `test/fixtures/px4_v1_17_0/provenance.json`의 URL과 SHA-256에
기록돼 있다. 생성된 패치는 `firmware/px4_v1.17.0_diagnostics.patch`다.

## 수행한 소프트웨어 검증

- 공식 태그의 파일·메시지 스키마를 고정한 오프라인 fixture 사용.
- dry-run 무변경, 반복 적용 시 무변경, 잘못된 스키마·앵커 시 쓰기 전 중단.
- 실제 메시지 스키마에서 생성한 C++ 구조체로 patched ESTIMATOR_STATUS 코드를
  컴파일하고, baro OFF/정상 융합/innovation reject/오래된 sample·flags·fusion/
  NaN/primary 전환·전환 실패 동작 검증.
- 이 C++ 검사는 uORB와 MAVLink 전송을 대역으로 대체한다. 전체 펌웨어 빌드나
  실제 FC에서의 센서 동작을 검증하는 검사는 아니다.
- 브리지의 1.17 버전 일치만으로 clock·baro 진단 차단이 해제되지 않는지 검사.
- 최종 테스트 결과: 40 passed, 1 skipped (기존 선택적 1.14 소스 fixture 부재).
- 공식 태그의 `hkust_nxt-dual_default` 전체 빌드 성공. GCC 13.2.1 20231009 사용.
- 실제 FC의 파라미터 949/949개 및 콘솔 결과 백업 완료.
- 수정된 소스는 MAVLink 진단 관련 두 파일뿐이다. 패키지의 압축 해제 이미지와
  빌드 `.bin`의 일치, board ID 1013, 플래시 한도 이내인 것을 확인했다.

## 준비된 파일

워크스페이스 기준 경로:

- 업로드 후보: `firmware_build/artifacts/hkust_nxt-dual_v1.17.0_zav_diagnostics.px4`
- 공식 복구 원본: `firmware_build/artifacts/hkust_nxt-dual_v1.17.0_original.px4`
- 체크섬·빌드 정보: `firmware_build/artifacts/manifest.json`
- 파라미터 백업: `log/firmware_backups/20261029T072057Z.params`
- FC 정보·전체 백업: `log/firmware_backups/20261029T072057Z.json`
- 빌드 로그: `firmware_build/px4_diagnostics_build.log`

패치 이미지 크기는 1,807,736 bytes, 플래시 한도는 1,835,008 bytes다.
공식 원본의 1,807,032 bytes보다 704 bytes 크다. 패키지의 설명에 나오는
MatekH743 문구는 이 공식 보드 소스의 firmware.prototype에서 그대로 온 것으로,
실제 빌드 타깃과 board ID를 기준으로 대조했다.

## 실제 펌웨어 작업 전에 필요한 정보

실제 소스와 보드 타깃 대조는 위와 같이 완료했다. 다른 FC에는 이 결과를 적용하지 않는다.
정확한 보드 및 설정을 확인하지 않은 채 일반 타깃이나 SITL 바이너리를 올리지 않는다.
FC 파라미터와 기존 펌웨어 복구 방법을 확보한다.

공식 v1.17.0 태그에 맞는 실제 펌웨어 소스가 확인된 다음 소스 준비 명령은 다음과 같다.
`--verify-only`는 원본에 패치가 없으면 실패하는 것이 정상이며 어떤 파일도 수정하지 않는다.

```bash
python3 src/zav_real_control/scripts/prepare_mavlink.py \
  --px4-dir /path/to/PX4-Autopilot --baseline v1.17.0 --verify-only

# 확인된 별도 소스 체크아웃에서만 패치 적용
python3 src/zav_real_control/scripts/prepare_mavlink.py \
  --px4-dir /path/to/PX4-Autopilot --baseline v1.17.0

python3 src/zav_real_control/scripts/prepare_mavlink.py \
  --px4-dir /path/to/PX4-Autopilot --baseline v1.17.0 --verify-only
```

펌웨어 전체 빌드와 파일 검토는 완료했다. 업로드는 별도의 단계다.
실 FC 검증 전에는 `firmware_verified=false`, `publish_enabled=false`를 유지한다.
현재 `expected_firmware=[1,14,3]` 차단도 그대로 유지했다.
향후 검증된 1.17 펌웨어를 사용하게 될 때 이 버전 값을 변경하고,
MONITOR_ONLY에서 실제 `zav_sync`와 신선한 `zav_baro`를 확인한 다음에만
후속 propeller-off EV 융합 시험을 준비한다. `firmware_verified`를 켜는 것만으로
미검증 패치를 검증된 것으로 취급하지 않는다.
