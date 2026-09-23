#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

ENV_DIR="/goinfre/$USER/envs/mujoco_env"
CONDA_DIR="/goinfre/$USER/miniforge3"
CONDA_BIN="$CONDA_DIR/bin/conda"

echo "🔍 [1/4] NFS 캐시 심볼릭 링크 점검..."
mkdir -p /goinfre/$USER/.cache /goinfre/$USER/.conda
if [ ! -L "$HOME/.cache" ]; then
    rm -rf "$HOME/.cache"
    ln -s /goinfre/$USER/.cache "$HOME/.cache"
fi
if [ ! -L "$HOME/.conda" ]; then
    rm -rf "$HOME/.conda"
    ln -s /goinfre/$USER/.conda "$HOME/.conda"
fi

echo "📦 [2/4] Miniforge 설치 및 쉘 바인딩 점검..."
if [ ! -d "$CONDA_DIR" ]; then
    echo " -> Miniforge 다운로드 및 설치 중..."
    INSTALLER="/goinfre/$USER/miniforge_installer.sh"
    curl -fsSL -o "$INSTALLER" https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh
    bash "$INSTALLER" -b -p "$CONDA_DIR" > /dev/null
    rm -f "$INSTALLER"
    
    # zsh 및 bash 바인딩, base 자동 실행 방지
    "$CONDA_BIN" init zsh > /dev/null 2>&1 || true
    "$CONDA_BIN" init bash > /dev/null 2>&1 || true
    "$CONDA_BIN" config --set auto_activate_base false > /dev/null 2>&1 || true
fi

echo "🐍 [3/4] Python 3.10 가상환경 및 의존성 점검..."
if [ ! -d "$ENV_DIR" ]; then
    echo " -> Python 3.10 환경 생성 중..."
    "$CONDA_BIN" create -y -p "$ENV_DIR" python=3.10 > /dev/null
    "$ENV_DIR/bin/pip" install --upgrade pip > /dev/null
fi

# 의존성 패키지(requirements.txt) 누락/미설치 시 보완 설치
if ! "$ENV_DIR/bin/python" -c "import mujoco" > /dev/null 2>&1; then
    echo " -> requirements.txt 패키지 설치 중..."
    "$ENV_DIR/bin/pip" install --upgrade pip > /dev/null
    "$ENV_DIR/bin/pip" install -r requirements.txt
fi

echo "🔗 [4/4] 심볼릭 링크 및 편의 스크립트 구성..."
# VS Code / Cursor 인터프리터 인식용
ln -sfn "$ENV_DIR" .venv

# zsh 새로고침 없이도 바로 진입할 수 있는 헬퍼 스크립트 생성
cat << 'EOF' > activate.sh
#!/usr/bin/env bash
source /goinfre/$USER/miniforge3/bin/activate /goinfre/$USER/envs/mujoco_env
EOF
chmod +x activate.sh

echo ""
echo "=================================================="
echo "🎉 환경 구축 완료!"
echo "다음 중 편한 방법으로 가상환경을 켜세요:"
echo " 1) source activate.sh"
echo " 2) conda activate /goinfre/\$USER/envs/mujoco_env (새 터미널 열었을 때)"
echo "종료 시: conda deactivate"
echo "=================================================="