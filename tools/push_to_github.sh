#!/usr/bin/env bash
# 把本仓库推送到 GitHub。
#
# 用法:
#   tools/push_to_github.sh git@github.com:用户名/仓库名.git      # SSH
#   tools/push_to_github.sh https://github.com/用户名/仓库名.git  # HTTPS
#
# 脚本会: 配置远程 origin -> 检查能否访问 -> 必要时合并远程已有提交 -> push。
# 只在首次需要, 之后直接 git push 即可。

set -euo pipefail
cd "$(dirname "$0")/.."

URL="${1:-}"
if [ -z "$URL" ]; then
    echo "用法: $0 <仓库地址>" >&2
    echo "例:   $0 git@github.com:yourname/ai-competition-arena.git" >&2
    exit 2
fi

# ------------------------------------------------------------------ 1. 提交身份
NAME=$(git config user.name || true)
MAIL=$(git config user.email || true)
if [ -z "$NAME" ] || [ -z "$MAIL" ]; then
    echo "还没配置 git 提交身份, 先执行 (换成你自己的):"
    echo '  git config --global user.name  "你的名字"'
    echo '  git config --global user.email "你的GitHub邮箱"'
    echo "配好后重跑本脚本。"
    exit 3
fi

# ---------------------------------------------------------------- 2. 远程 origin
if git remote get-url origin >/dev/null 2>&1; then
    git remote set-url origin "$URL"
else
    git remote add origin "$URL"
fi
git branch -M main
echo "origin = $(git remote get-url origin)"

# ------------------------------------------------------------- 3. 连通性与权限
echo "检查远程仓库可否访问 ..."
if ! git ls-remote --exit-code origin >/dev/null 2>&1; then
    cat >&2 <<'MSG'
访问失败。常见原因:
  * 地址写错了(尤其用户名/仓库名大小写), 或仓库还没在 GitHub 上创建
  * HTTPS: GitHub 已停用密码, 需要在 Settings -> Developer settings ->
    Personal access tokens 生成 token, 然后
      git config --global credential.helper store
    下一次 push 时用户名填 GitHub 用户名、密码填 token
  * SSH: 需要先把公钥加到 GitHub
      ssh-keygen -t ed25519 -C "你的邮箱"    # 一路回车
      cat ~/.ssh/id_ed25519.pub              # 内容贴到 GitHub -> SSH keys
      ssh -T git@github.com                  # 验证
MSG
    exit 4
fi

# ------------------------------------------- 4. 远程非空时先合并(建仓库时勾了 README)
if [ -n "$(git ls-remote origin HEAD 2>/dev/null)" ]; then
    echo "远程已有提交, 先 rebase 合并 ..."
    git pull --rebase origin main || {
        echo "有冲突, 手动处理后重跑本脚本。" >&2
        exit 5
    }
fi

# ------------------------------------------------------------------- 5. 推送
git push -u origin main
echo
echo "推送完成 ✓"
echo "仓库文件数: $(git ls-files | wc -l)"

# ------------------------------------- 6. 仓库里首个提交作者是占位身份, 可选改成你
if [ "$(git log -1 --format=%ae)" != "$MAIL" ]; then
    echo
    echo "注意: 当前提交作者是 $(git log -1 --format='%an <%ae>')"
    read -r -p "要改成 $NAME <$MAIL> 并强推吗? [y/N] " ans
    if [ "${ans,,}" = "y" ]; then
        git -c user.name="$NAME" -c user.email="$MAIL" \
            commit --amend --reset-author --no-edit
        git push -f -u origin main
        echo "已更新作者并重推 ✓"
    fi
fi
