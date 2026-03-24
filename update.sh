#!/bin/bash
# T32 VIO 更新脚本
# 用法: ./update.sh "修改说明"

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查参数
if [ $# -eq 0 ]; then
    echo -e "${RED}错误: 请提供修改说明${NC}"
    echo "用法: ./update.sh \"修改说明\""
    echo "示例: ./update.sh \"修复FAST角点检测阈值问题\""
    exit 1
fi

COMMIT_MSG="$1"

# 获取当前时间
DATE=$(date '+%Y-%m-%d %H:%M:%S')

echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}  T32 VIO 更新脚本${NC}"
echo -e "${YELLOW}========================================${NC}"
echo ""

# 检查是否在git仓库中
if [ ! -d .git ]; then
    echo -e "${RED}错误: 当前目录不是git仓库${NC}"
    exit 1
fi

# 检查远程仓库
echo -e "${YELLOW}[1/6] 检查远程仓库...${NC}"
REMOTE=$(git remote -v)
if [ -z "$REMOTE" ]; then
    echo -e "${RED}错误: 未配置远程仓库${NC}"
    exit 1
fi
echo "$REMOTE"
echo ""

# 显示当前状态
echo -e "${YELLOW}[2/6] 当前Git状态:${NC}"
git status -s
echo ""

# 添加所有修改
echo -e "${YELLOW}[3/6] 添加修改到暂存区...${NC}"
git add -A
echo -e "${GREEN}✓ 已添加所有修改${NC}"
echo ""

# 显示即将提交的变更
echo -e "${YELLOW}[4/6] 即将提交的变更:${NC}"
git diff --cached --stat
echo ""

# 提交
echo -e "${YELLOW}[5/6] 提交修改...${NC}"
git commit -m "[$DATE] $COMMIT_MSG"
echo -e "${GREEN}✓ 提交成功${NC}"
echo ""

# 推送到远程
echo -e "${YELLOW}[6/6] 推送到GitHub...${NC}"
git push origin main
echo -e "${GREEN}✓ 推送成功${NC}"
echo ""

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  更新完成!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "提交信息: [$DATE] $COMMIT_MSG"
echo "仓库地址: https://github.com/wg520235/t32_vio"
