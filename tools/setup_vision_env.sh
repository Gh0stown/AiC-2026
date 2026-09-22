#!/usr/bin/env bash
# 重建视觉/深度学习虚拟环境 (competition vision stack)
#
# 用法:
#   tools/setup_vision_env.sh              # CPU 版 torch (本机, 无显卡)
#   tools/setup_vision_env.sh --cuda 121   # GPU 版 torch (CUDA 12.1, 搬到显卡机时用)
#   tools/setup_vision_env.sh --recreate   # 先删掉已有 .venv 再重建
#
# 可覆盖的环境变量:
#   TORCH_BASE       索引页与 wheel 的根 (默认 https://download.pytorch.org/whl)
#   TORCH_FILE_BASE  只把 **wheel 文件** 换成国内镜像 (索引页仍走 TORCH_BASE)
#                    例: TORCH_FILE_BASE=https://mirrors.aliyun.com/pytorch-wheels
#   MIRROR           pip 镜像 (默认清华)
#   TORCH_VER        torch 版本 (默认 2.4.1, Python 3.8 的最后一版)
#
# 环境说明:
#   * 虚拟环境放在仓库根的 .venv/ (已 gitignore), 不写 $HOME, 不需要 sudo
#   * 若检测到 /opt/ros 则带 --system-site-packages, 让 rospy/cv_bridge 可见
#   * pip 本身也是从这里手动引导的 (本机 python3 没有 pip 且 ensurepip 缺失)

set -euo pipefail
cd "$(dirname "$0")/.."
ROOT="$(pwd)"

MIRROR="${MIRROR:-https://pypi.tuna.tsinghua.edu.cn/simple}"
TORCH_BASE="${TORCH_BASE:-https://download.pytorch.org/whl}"
TORCH_FILE_BASE="${TORCH_FILE_BASE:-}"
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
    TORCH_INDEX="${TORCH_BASE}/${TORCH_CHANNEL}"
    echo "==> 安装 GPU 版 torch (CUDA ${CUDA})"
else
    TORCH_CHANNEL="cpu"
    TORCH_SPEC="torch==${TORCH_VER}+cpu"
    TORCH_INDEX="${TORCH_BASE}/cpu"
    echo "==> 安装 CPU 版 torch"
fi

# 先把 wheel 下到本地再离线装。两个原因:
#   1) 索引页里的 href 指向 CDN download-r2.pytorch.org, 本机访问该主机返回 403,
#      必须把主机名替换成 download.pytorch.org (同路径可用)
#   2) 下到本地后进度可见, 中断可重来
# 依赖(filelock/sympy/...)仍从镜像取, 所以不加 --no-index。
mkdir -p "$VENV/wheels"
"$VPY" - "$TORCH_INDEX" "$TORCH_VER" "$TORCH_CHANNEL" "$VENV/wheels" "$TORCH_FILE_BASE" <<'PY'
import os, re, sys, time, zipfile, urllib.request, urllib.parse

index, want_ver, channel, outdir, filebase = sys.argv[1:6]
py = "cp%d%d" % sys.version_info[:2]

def vkey(name):
    m = re.search(r'-(\d+)\.(\d+)\.(\d+)', name)
    return tuple(int(x) for x in m.groups()) if m else (0, 0, 0)

for pkg in ("torch", "torchvision"):
    html = urllib.request.urlopen("%s/%s/" % (index, pkg), timeout=60).read().decode("utf-8", "replace")
    # ★ 必须一起过滤**平台标签** —— 索引页里同一版本同时有 linux_x86_64 和
    #   win_amd64, 而 hits 是 set, 迭代顺序不确定, 只按版本排序会**随机选中平台**
    #   (踩过: 下到过 torch-2.4.1+cu121-cp38-cp38-win_amd64.whl)。
    plat = "win_amd64" if sys.platform.startswith("win") else (
           "macosx" if sys.platform == "darwin" else "linux_x86_64")
    hits = set(re.findall(r'href="(https://[^"#]+' + pkg + r'-[^"#]*' + py
                          + r'[^"#]*' + plat + r'[^"#]*\.whl)', html))
    if not hits:
        sys.exit("   找不到 %s 的 %s wheel (通道 %s)" % (pkg, py, channel))
    if pkg == "torch":
        pinned = [h for h in hits if re.search(r'-%s\+' % re.escape(want_ver), h)]
        if pinned:
            hits = set(pinned)
    url = sorted(hits, key=lambda h: vkey(urllib.parse.unquote(h.split('/')[-1])))[-1]
    url = url.replace("download-r2.pytorch.org", "download.pytorch.org")
    if filebase:                      # 可选: wheel 文件走国内镜像 (索引页不变)
        url = re.sub(r'^https://[^/]+/whl/', filebase.rstrip('/') + '/', url)
    out = os.path.join(outdir, urllib.parse.unquote(url.split("/")[-1]))
    # ★ 只判"文件 > 1MB"是不够的 —— 下载中断留下的残file同样满足, 会被当成
    #   完整的 wheel 交给 pip, 然后报一堆看不懂的解压错误。wheel 本质是 zip,
    #   直接校验它是不是合法 zip。
    if os.path.exists(out) and os.path.getsize(out) > 1_000_000 \
            and zipfile.is_zipfile(out):
        print("   [跳过] %s (%.1f MB)" % (os.path.basename(out), os.path.getsize(out) / 1e6))
        continue
    print("   [下载] %s" % os.path.basename(out))
    t0 = time.time()
    with urllib.request.urlopen(url, timeout=180) as r, open(out, "wb") as f:
        expect = int(r.headers.get("Content-Length") or 0)
        while True:
            c = r.read(1 << 20)
            if not c:
                break
            f.write(c)
    got = os.path.getsize(out)
    if (expect and got != expect) or not zipfile.is_zipfile(out):
        try:
            os.remove(out)
        except OSError:
            pass
        sys.exit("     下载不完整 (%d/%d 字节), 已删除, 请重跑" % (got, expect))
    print("      %.1f MB / %.0fs" % (got / 1e6, time.time() - t0))
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

# ★ numpy 必须钉版本装进 venv。
#   本脚本用 --system-site-packages (为了让 rospy/cv_bridge 可见), 于是系统自带的
#   numpy 1.17.4 会遮蔽 venv —— pip 看到 "Requirement already satisfied" 就不装了。
#   后果 (实测): cv2 报 "module compiled against API version 0xe but this version
#   of numpy is 0xd", pandas 报 "numpy.random has no attribute BitGenerator",
#   ultralytics 要求的 numpy>=1.23 也不满足。
#   1.24.4 是 Python 3.8 上最后一个 numpy, 同时满足 cv2/ultralytics/scipy/onnxruntime。
echo "==> 把 numpy 钉到 1.24.4 (避免被系统版遮蔽)"
"$VPY" -m pip install --progress-bar off --only-binary=:all: \
    --index-url "$MIRROR" "numpy==1.24.4" 2>&1 | tail -3

# ★ 同样的遮蔽问题也发生在下面这三个包上 (都是 2026-09 实测):
#     matplotlib 3.1.2 (系统)  < ultralytics 要求的 >=3.3
#         -> 后果最严重: ultralytics 画 PR 曲线时调 FontManager.addfont, 3.1.2 没有这个方法,
#            直接 AttributeError 崩掉。整段训练能跑完、best.pt 也存下来了, 但收尾的
#            model.val() / yolo val 会崩, 拿不到任何指标图。混淆矩阵那步只是 warning,
#            PR 曲线那步是硬崩, 所以很容易被当成"训练完了但 val 报错"。
#     Pillow 7.0.0 (系统)      < ultralytics 要求的 >=7.1.0
#     requests 2.22.0 (系统)   < ultralytics 要求的 >=2.23.0
#   Pillow 选 9.5.0 而不是 10.x: Pillow 10 删掉了 Image.ANTIALIAS 等老 API,
#   hyperlpr3 有回归风险 (它现在还靠这些老 API)。
#   PyYAML 5.3.1 (系统) 不用动, 它满足 ultralytics 的 >=5.3.1。
echo "==> 把 matplotlib / Pillow / requests 钉进 venv (避免被系统版遮蔽)"
for spec in "matplotlib==3.7.5" "Pillow==9.5.0" "requests==2.32.3"; do
    "$VPY" -m pip install --progress-bar off --only-binary=:all: \
        --index-url "$MIRROR" "$spec" 2>&1 | tail -2
done

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
# ★ 中文字体: plate_ocr.py --selftest 要用 PIL 合成一张蓝牌, 它按顺序找
#   NotoSansCJK-Bold.ttc / DroidSansFallbackFull.ttf / uming.ttc。
#   注意 fonts-droid-fallback 提供的是 **CJK 回退字体, 没有拉丁字形** ——
#   装它的话数字/字母会渲染成空心方框, OCR 自检会变成 0/3 (踩过)。
#   要装 fonts-noto-cjk (提供首个候选)。
FONT_OK=0
for f in /usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc \
         /usr/share/fonts/truetype/arphic/uming.ttc; do
    [ -f "$f" ] && FONT_OK=1 && break
done
if [ "$FONT_OK" = 1 ]; then
    echo "==> 中文字体: OK"
else
    echo "==> 中文字体: !! 缺失 —— plate_ocr.py --selftest 会失败"
    echo "    需要 root 装一次:  sudo apt install -y fonts-noto-cjk"
    echo "    注意别只装 fonts-droid-fallback: 它没有拉丁字形, 数字/字母会变方框"
fi

echo "================ 自检 ================"
HOME="$CACHE/home" YOLO_CONFIG_DIR="$CACHE/ultralytics" "$VPY" - <<'PY'
import importlib

# (import 名, 显示名, 最低版本) —— 最低版本来自 ultralytics 8.4.159 的 requirements。
# 之所以要连"来自哪个目录"一起打印: 本 venv 是 --system-site-packages, 系统包会遮蔽
# venv 包, 而 pip 看到系统版"已满足"就不装了。只看版本号看不出这个问题, 看路径才能。
CHECKS = [
    ("torch",        "PyTorch",       None),
    ("torchvision",  "torchvision",   None),
    ("cv2",          "OpenCV",        None),
    ("numpy",        "numpy",         "1.23.0"),
    ("matplotlib",   "matplotlib",    "3.3.0"),
    ("PIL",          "Pillow",        "7.1.0"),
    ("requests",     "requests",      "2.23.0"),
    ("yaml",         "PyYAML",        "5.3.1"),
    ("ultralytics",  "ultralytics",   None),
    ("hyperlpr3",    "HyperLPR3",     None),
]


def ver(s):
    out = []
    for part in s.split(".")[:3]:
        num = ""
        for ch in part:
            if ch.isdigit():
                num += ch
            else:
                break
        out.append(int(num or 0))
    while len(out) < 3:
        out.append(0)
    return tuple(out)


bad = 0
for name, label, need in CHECKS:
    try:
        m = importlib.import_module(name)
    except Exception as e:
        print("  FAIL %-13s 导入失败: %s" % (label, e)); bad += 1; continue
    v = getattr(m, "__version__", "")
    f = str(getattr(m, "__file__", ""))
    where = "venv" if "/.venv/" in f else ("SYSTEM" if "dist-packages" in f else "?")
    flag = "ok  "
    if need and ver(v) < ver(need):
        flag = "FAIL"; bad += 1
    elif where == "SYSTEM" and need:
        # 版本够但来自系统: 能用, 但换台机器可能就变旧版, 提醒一句
        flag = "warn"
    print("  %s %-13s %-12s (%s)%s" % (
        flag, label, v, where, "" if not need else "  需要 >=%s" % need))

try:
    import torch
    print("  torch CUDA 可用:", torch.cuda.is_available())
except Exception:
    pass
print()
if bad:
    print("!! 有 %d 项不达标 —— 重跑本脚本, 或按上面提示单独 pip install 对应版本" % bad)
else:
    print("全部达标。")
PY
echo
echo "完成。激活方式:  source .venv/bin/activate"
echo
echo "注意: 本机 \$HOME 可能不可写, 建议在 vision 相关脚本里带上这两个变量:"
echo "  export YOLO_CONFIG_DIR=$CACHE/ultralytics"
echo "  export HOME=$CACHE/home      # 仅为 HyperLPR3 的模型目录"
echo "  车牌识别可直接用: $ROOT/tools/plate_ocr.py --selftest"
