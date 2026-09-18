#!/usr/bin/env bash
# 重建视觉/深度学习虚拟环境 (competition vision stack)
#
# 用法:
#   tools/setup_vision_env.sh              # CPU 版 torch (本机, 无显卡)
#   tools/setup_vision_env.sh --cuda 121   # GPU 版 torch (CUDA 12.1, 搬到显卡机时用)
#   tools/setup_vision_env.sh --recreate   # 先删掉已有 .venv 再重建
#
# 环境说明:
#   * 虚拟环境放在仓库根的 .venv/ (已 gitignore), 不写 $HOME, 不需要 sudo
#   * 若检测到 /opt/ros 则带 --system-site-packages, 让 rospy/cv_bridge 可见
#   * pip 本身也是从这里手动引导的 (本机 python3 没有 pip 且 ensurepip 缺失)

set -euo pipefail
cd "$(dirname "$0")/.."
ROOT="$(pwd)"

MIRROR="${MIRROR:-https://pypi.tuna.tsinghua.edu.cn/simple}"
TORCH_VER="${TORCH_VER:-2.4.1}"          # 2.4.1 是支持 Python 3.8 的最后一版
CUDA=""
RECREATE=0

while [ $# -gt 0 ]; do
    case "$1" in
        --cuda)     CUDA="${2:?--cuda 后面要给版本号, 如 121}"; shift 2 ;;
        --recreate) RECREATE=1; shift ;;
        --mirror)   MIRROR="$2"; shift 2 ;;
        -h|--help)  sed -n '2,14p' "$0"; exit 0 ;;
        *) echo "未知参数: $1" >&2; exit 2 ;;
    esac
done

PY=python3
VENV="$ROOT/.venv"

# ---------------------------------------------------------------- 0. 准备
echo "==> Python: $($PY -V 2>&1)"
if [ "$RECREATE" = 1 ] && [ -d "$VENV" ]; then
    echo "==> 删除已有 .venv"
    rm -rf "$VENV"
fi

# pip 缓存和临时目录都放在工作区外, 避免写只读的 $HOME
export PIP_CACHE_DIR="${PIP_CACHE_DIR:-/tmp/pipcache}"
export PIP_CONFIG_FILE=/dev/null
export TMPDIR="${TMPDIR:-/tmp}"
mkdir -p "$PIP_CACHE_DIR"

# ---------------------------------------------------------------- 1. venv
if [ ! -d "$VENV" ]; then
    VENV_ARGS=(--without-pip)
    if [ -d /opt/ros ]; then
        VENV_ARGS+=(--system-site-packages)
        echo "==> 检测到 /opt/ros, 使用 --system-site-packages (rospy/cv_bridge 可见)"
    fi
    echo "==> 创建 $VENV"
    $PY -m venv "${VENV_ARGS[@]}" "$VENV"
else
    echo "==> 复用已有 $VENV"
fi

VPY="$VENV/bin/python"

# ------------------------------------------------- 2. 引导 pip (如缺失)
if ! "$VPY" -m pip --version >/dev/null 2>&1; then
    echo "==> 系统无 pip 且 ensurepip 缺失, 从镜像手动引导 pip"
    "$VPY" - "$MIRROR" <<'PY'
import re, sys, urllib.request, urllib.parse, zipfile, sysconfig, os
mirror = sys.argv[1].rstrip('/') + '/'
index = mirror + '/pip/'
print("   拉取索引:", index)
html = urllib.request.urlopen(index, timeout=60).read().decode('utf-8', 'replace')
cands = re.findall(r'href="([^"#]+\.whl)(?:#sha256=[0-9a-f]+)?"', html)
def key(u):
    m = re.search(r'pip-(\d+)\.(\d+)(?:\.(\d+))?-py3-none-any\.whl$', u)
    return tuple(int(x or 0) for x in m.groups()) if m else None
def pyver_ok(k):
    # pip >= 25 已放弃 Python 3.8
    if sys.version_info < (3, 9) and k >= (25, 0, 0):
        return False
    return True
ok = sorted((key(u), u) for u in cands if key(u) and pyver_ok(key(u)))
if not ok:
    sys.exit("找不到兼容当前 Python 的 pip wheel")
url = urllib.parse.urljoin(index, ok[-1][1])
print("   下载:", url.split('/')[-1])
data = urllib.request.urlopen(url, timeout=300).read()
tmp = os.path.join(os.environ.get('TMPDIR', '/tmp'), 'pip.whl')
open(tmp, 'wb').write(data)
dest = sysconfig.get_paths()['purelib']
zipfile.ZipFile(tmp).extractall(dest)
print("   ✓ 解包到", dest)
PY
    # 解包 wheel 不会生成 bin/pip 入口脚本, 补一个
    "$VPY" -m pip install --progress-bar off --no-deps --force-reinstall \
        --index-url "$MIRROR" pip
fi
echo "==> $("$VPY" -m pip --version)"

# ---------------------------------------------------------------- 3. torch
if [ -n "$CUDA" ]; then
    TORCH_CHANNEL="cu${CUDA}"
    TORCH_SPEC="torch==${TORCH_VER}+${TORCH_CHANNEL}"
    TORCH_INDEX="https://download.pytorch.org/whl/${TORCH_CHANNEL}"
    echo "==> 安装 GPU 版 torch (CUDA ${CUDA})"
else
    TORCH_CHANNEL="cpu"
    TORCH_SPEC="torch==${TORCH_VER}+cpu"
    TORCH_INDEX="https://download.pytorch.org/whl/cpu"
    echo "==> 安装 CPU 版 torch"
fi

# 先把 wheel 下到本地再离线装。两个原因:
#   1) 索引页里的 href 指向 CDN download-r2.pytorch.org, 本机访问该主机返回 403,
#      必须把主机名替换成 download.pytorch.org (同路径可用)
#   2) 下到本地后进度可见, 中断可重来
# 依赖(filelock/sympy/...)仍从镜像取, 所以不加 --no-index。
mkdir -p "$VENV/wheels"
"$VPY" - "$TORCH_INDEX" "$TORCH_VER" "$TORCH_CHANNEL" "$VENV/wheels" <<'PY'
import os, re, sys, time, urllib.request, urllib.parse

index, want_ver, channel, outdir = sys.argv[1:5]
py = "cp%d%d" % sys.version_info[:2]

def vkey(name):
    m = re.search(r'-(\d+)\.(\d+)\.(\d+)', name)
    return tuple(int(x) for x in m.groups()) if m else (0, 0, 0)

for pkg in ("torch", "torchvision"):
    html = urllib.request.urlopen("%s/%s/" % (index, pkg), timeout=60).read().decode("utf-8", "replace")
    hits = set(re.findall(r'href="(https://[^"#]+' + pkg + r'-[^"#]*' + py + r'[^"#]*\.whl)', html))
    if not hits:
        sys.exit("   找不到 %s 的 %s wheel (通道 %s)" % (pkg, py, channel))
    if pkg == "torch":
        pinned = [h for h in hits if re.search(r'-%s\+' % re.escape(want_ver), h)]
        if pinned:
            hits = set(pinned)
    url = sorted(hits, key=lambda h: vkey(urllib.parse.unquote(h.split('/')[-1])))[-1]
    url = url.replace("download-r2.pytorch.org", "download.pytorch.org")
    out = os.path.join(outdir, urllib.parse.unquote(url.split("/")[-1]))
    if os.path.exists(out) and os.path.getsize(out) > 1_000_000:
        print("   [跳过] %s (%.1f MB)" % (os.path.basename(out), os.path.getsize(out) / 1e6))
        continue
    print("   [下载] %s" % os.path.basename(out))
    t0 = time.time()
    with urllib.request.urlopen(url, timeout=180) as r, open(out, "wb") as f:
        while True:
            c = r.read(1 << 20)
            if not c:
                break
            f.write(c)
    print("      %.1f MB / %.0fs" % (os.path.getsize(out) / 1e6, time.time() - t0))
PY

# shellcheck disable=SC2086
"$VPY" -m pip install --progress-bar off \
    --find-links "$VENV/wheels" --index-url "$MIRROR" \
    $TORCH_SPEC torchvision

# ------------------------------------------------------------ 4. 其余依赖
echo "==> 安装 ultralytics / OpenCV (清华镜像)"
"$VPY" -m pip install --progress-bar off --index-url "$MIRROR" \
    "opencv-python==4.10.0.84" ultralytics pillow pyyaml matplotlib pandas tqdm scipy requests

echo "==> 安装 HyperLPR3 (中文车牌识别)"
"$VPY" -m pip install --progress-bar off --index-url "$MIRROR" hyperlpr3 \
    || echo "   ! HyperLPR3 安装失败, 稍后可单独重试 (不影响 YOLO)"

# ------------------------------------------------- 5. 缓存目录与模型预下载
CACHE="$ROOT/.cache"
mkdir -p "$CACHE/ultralytics" "$CACHE/home"

# HyperLPR3 把模型目录硬编码成 $HOME/.hyperlpr3, 没有环境变量可覆盖。
# 若 $HOME 不可写(受限沙箱), 就把它指到仓库内的 .cache/home。
if [ -d "$HOME/.hyperlpr3/20230229" ]; then
    echo "==> HyperLPR3 模型已在 $HOME/.hyperlpr3"
elif [ -d "$CACHE/home/.hyperlpr3/20230229" ]; then
    echo "==> HyperLPR3 模型已在仓库缓存 $CACHE/home/.hyperlpr3"
elif "$VPY" -c "import hyperlpr3" >/dev/null 2>&1; then
    echo "==> 预下载 HyperLPR3 模型 (约 18 MB)"
    HOME="$CACHE/home" "$VPY" -c "import hyperlpr3 as l; l.LicensePlateCatcher()" >/dev/null 2>&1 \
        && echo "   ✓ 已缓存到 $CACHE/home/.hyperlpr3" \
        || echo "   ! 下载失败, 首次使用时再试 (需要联网)"
fi

# ---------------------------------------------------------------- 6. 自检
echo
echo "================ 自检 ================"
HOME="$CACHE/home" YOLO_CONFIG_DIR="$CACHE/ultralytics" "$VPY" - <<'PY'
import importlib
for name, label in [("torch","PyTorch"), ("torchvision","torchvision"),
                    ("cv2","OpenCV"), ("numpy","numpy"),
                    ("ultralytics","ultralytics"), ("hyperlpr3","HyperLPR3")]:
    try:
        m = importlib.import_module(name)
        print("  ok   %-13s %s" % (label, getattr(m, "__version__", "")))
    except Exception as e:
        print("  FAIL %-13s %s" % (label, e))
try:
    import torch
    print("  torch CUDA 可用:", torch.cuda.is_available())
except Exception:
    pass
PY
echo
echo "完成。激活方式:  source .venv/bin/activate"
echo
echo "注意: 本机 \$HOME 可能不可写, 建议在 vision 相关脚本里带上这两个变量:"
echo "  export YOLO_CONFIG_DIR=$CACHE/ultralytics"
echo "  export HOME=$CACHE/home      # 仅为 HyperLPR3 的模型目录"
echo "  车牌识别可直接用: $ROOT/tools/plate_ocr.py --selftest"
