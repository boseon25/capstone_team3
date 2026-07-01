# WSLg 렌더 가이드 (개인 개발용) — GPU 가속 Gazebo

> 베이스는 noVNC(소프트웨어 렌더, 학생 배포용)로 그대로 둔다.
> 이 가이드는 **강사 개인 개발/테스트에서 GPU 가속 화면**이 필요할 때만 사용한다.

## ★★ 이 환경 실측 — 검증된 작동 조합 (2026-06 기록)

> 아래는 실제로 RTF를 회복시킨 조합. **렌더 백엔드별 실측 결과**:
>
> | 조합 | RTF | 결과 |
> |------|-----|------|
> | 소프트웨어(llvmpipe) + ogre2 | **0.037** | 안 죽지만 사실상 사용 불가 |
> | d3d12(Intel Xe) + ogre2 | 크래시 | `Ogre::UnimplementedException` (copyTo 미구현) |
> | d3d12(Intel Xe) + ogre1 (GUI/센서 렌더) | 크래시 | `sun` 라이트 중복 생성 |
> | **d3d12(Intel Xe) + ogre1 + 헤드리스(-s) + lite** | **0.9954** | ✅ 정답 |
>
> **정답 시뮬 기동 명령 (그대로 사용):**
> ```bash
> ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py \
>   model:=lite world:=warehouse rviz:=false \
>   gz_args:="-s -r --render-engine ogre --headless-rendering"
> ```
> 핵심 4요소: ① `--render-engine ogre`(ogre2 아님 → copyTo 회피) ② `-s`(서버만, GUI sun 충돌 회피)
> ③ `--headless-rendering` ④ `model:=lite`(카메라 센서 제거). 시각화는 Foxglove가 담당.
> ※ GPU는 `D3D12 (Intel Iris Xe)`로 가속됨(NVIDIA 미사용). 이 정도로 RTF 0.99 달성.

## 개념 (사실)
- 실행 환경 = **여전히 Docker 컨테이너** (코드·colcon·ros2 launch 그대로).
- 바뀌는 것 = **디스플레이 경로**: 컨테이너 자체 Xvnc(:1, 브라우저) → **WSLg(:0, Windows 네이티브 창)**.
- WSLg는 호스트 GPU(d3d12)에 연결돼 있어 NVIDIA/Intel 가속을 받기 쉽다.
- ⚠️ Windows+WSL2에서는 NVIDIA여도 OpenGL은 `/dev/dxg`→Mesa d3d12 경로다(네이티브 GLX 아님).

## 전제 조건
1. Windows 11 + WSL2 (WSLg 포함) — Windows 10은 WSLg 미지원.
2. **Docker Desktop → Settings → Resources → WSL Integration 에서 해당 우분투 배포판 ON.**
   (꺼져 있으면 WSL 셸에서 `docker` 미동작 → PowerShell로 띄우게 되고 → WSLg 소켓 전달 실패.)
3. **`~/.docker/config.json` 의 `credsStore`(desktop.exe) 제거** — WSL에서 빌드 시
   `fork/exec docker-credential-desktop.exe: exec format error` 가 나면 이것.
   `cp ~/.docker/config.json ~/.docker/config.json.bak && sed -i '/credsStore/d;/credStore/d' ~/.docker/config.json`
4. NVIDIA로 가속하려면 **Windows에서 WSL을 NVIDIA(고성능)로 라우팅**:
   - 설정 → 시스템 → 디스플레이 → 그래픽 → WSL 호스트를 "고성능"
   - 또는 NVIDIA 제어판 → 3D 설정 → 기본 GPU = NVIDIA
   (하이브리드 노트북은 기본이 Intel일 수 있음 → 아래 판정에서 Intel로 나오면 이 라우팅 필요)

## 실행 (반드시 WSL 우분투 터미널)
```bash
# PowerShell ❌ — WSL 우분투 셸에서:
cd <키트 경로>/capstone
docker compose -f docker-compose.yml -f docker-compose.wslg.yml up --build -d

# 컨테이너 진입
docker exec -it nd1_capstone_dev bash
```

## ★ 판정 (가속 여부 — 이게 핵심)
```bash
glxinfo | grep -E "OpenGL renderer|Accelerated"
#  D3D12 (NVIDIA ...) / Accelerated: yes  → 성공 (NVIDIA 가속)
#  D3D12 (Intel ...)                      → GPU는 탔으나 Intel → Windows에서 NVIDIA 라우팅
#  llvmpipe (...)                         → 가속 실패 (아래 트러블슈팅)
```
`glxinfo`가 없으면: `apt-get update && apt-get install -y mesa-utils`

## 실행 (컨테이너 내부)
```bash
cd ~/ros2_ws && colcon build && source install/setup.bash
ros2 launch nd1_capstone bringup.launch.py sim_mode:=true
# Gazebo를 띄우면 Windows 바탕화면에 네이티브 창으로 뜬다.
```

## 트러블슈팅
| 증상 | 원인 | 조치 |
|------|------|------|
| renderer=llvmpipe | 이미지 Mesa가 d3d12 약함 | Dockerfile에 kisak-mesa PPA 추가 후 재빌드(아래) |
| D3D12 (Intel)인데 NVIDIA 원함 | Windows GPU 라우팅 Intel | 위 "전제 조건 4"로 NVIDIA 지정 |
| 창이 안 뜸 / DISPLAY 오류 | PowerShell에서 실행함 | **WSL 우분투 터미널**에서 재실행 |
| `cannot open display :0` | WSLg 미동작(Win10) | Win11 필요, 또는 noVNC로 복귀 |
| 컨테이너 안 `/mnt/wslg` 비어있음 | WSL 통합 OFF 또는 PowerShell 기동 | 전제조건 2 켜고 **WSL 셸에서** `down`→`-f` 두 개로 재기동 |
| `docker-credential-desktop.exe: exec format error` | WSL Docker가 Windows 헬퍼 호출 | 전제조건 3 (credsStore 제거) |
| `Ogre::UnimplementedException` (copyTo) | d3d12 + **ogre2** 미구현 | `--render-engine ogre`(ogre1)로 전환 |
| `sun ... already exists` (CreateDirectionalLight) | d3d12 + ogre1 GUI/센서 렌더 조명 중복 | **`-s`(서버만) + 헤드리스 + `model:=lite`** (정답 조합) |
| RTF 매우 낮음(<0.1) | 소프트웨어 렌더(llvmpipe) | 위 "검증된 작동 조합"으로 GPU+ogre+헤드리스 |

### (선택) d3d12 성숙 Mesa — Dockerfile에 추가 시
```dockerfile
RUN apt-get update && apt-get install -y software-properties-common && \
    add-apt-repository -y ppa:kisak/kisak-mesa && \
    apt-get update && apt-get upgrade -y && \
    apt-get install -y mesa-utils && rm -rf /var/lib/apt/lists/*
```

## 되돌리기 (noVNC로 복귀)
override 없이 베이스만 실행하면 끝:
```bash
docker compose up   # noVNC http://localhost:8080 (소프트웨어 렌더)
```

## 주의 (기준)
- **학생 배포본은 noVNC 유지** — 브라우저만 열면 누구나 켜지는 강점을 깨지 말 것.
- WSLg 경로는 강사 개인 개발/테스트용 opt-in.
- d3d12/NVIDIA 가속의 실제 성공·안정성은 **본인 컨테이너의 `glxinfo`로만** 확인됨(환경 편차 큼).
