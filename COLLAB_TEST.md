# gongke 协作迭代测试

测试编号：`GONGKE-COLLAB-20261001-01`

这是一次用于验证「本地编辑 → Git 提交 → GitHub 推送 → 队友查看/拉取」的测试迭代。

## 当前工作约定

- 共同仓库：<https://github.com/zjj20040618-sudo/gongke>，本次提交到 `main`。
- 当前电脑的工作目录：`C:/Users/15119/gongkesai/gongke`。
- 旧的 `C:/Users/15119/gongkesai/fw/jiejie` 演示工程和 GitHub `zjj20040618-sudo/jiejie` 不再作为工作版本；本次没有删除它们。
- `MDK-ARM/jiejie.uvprojx` 是当前固件工程文件名，仍然正常使用，不要因为旧仓库也叫 jiejie 而混淆。

## 队友如何确认

1. 在 GitHub 打开 `gongke` 仓库的 `main` 分支，查看本文件。
2. 能看到测试编号 `GONGKE-COLLAB-20261001-01`，说明可以查看这次迭代。
3. 如果本地已有同一仓库，在自己的 VS Code 中拉取后查看本文件；有未提交的修改时先保留自己的改动，不要强制覆盖。

本次仅添加这个文档，不修改电控固件、视觉代码、硬件参数或 Keil 工程。不需要因此重新编译或烧录。
