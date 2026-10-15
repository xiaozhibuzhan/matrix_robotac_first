> 历史记录，归档于 2026-10-15。下文状态和命令只代表当时版本；当前操作以 [项目入口](../../README.md) 和 [统一手册](../../navigation/OPERATIONS_UBUNTU.md) 为准。

# Codex 配置文件权限错误修复（含 CC-switch 场景）

## 错误信息

```text
Error loading config.toml: Failed to read project config file
/home/xiaozhibuzhan/.codex/config.toml: Permission denied (os error 13)
```

## 可能原因

即使当前没有使用 `sudo`，CC-switch 在切换配置时也可能出现以下情况：

- `~/.codex` 或 `config.toml` 的所有者是 `root`；
- 文件权限被设置为当前用户不可读；
- `config.toml` 是指向其他目录的符号链接，而目标文件没有读取权限；
- 用户目录或 `.codex` 目录缺少执行（进入目录）权限。

更换终端不会改变 Linux 文件权限，因此通常无法解决此问题。

## 推荐修复方法

在 Ubuntu 终端执行以下命令：

```bash
sudo mkdir -p "$HOME/.codex"
sudo chown -R "$USER":"$(id -gn)" "$HOME/.codex"
chmod 700 "$HOME/.codex"
[ -f "$HOME/.codex/config.toml" ] && chmod 600 "$HOME/.codex/config.toml"
```

然后确认所有者和权限：

```bash
ls -ld "$HOME/.codex"
ls -l "$HOME/.codex/config.toml"
```

正常情况下，所有者应显示为当前登录用户 `xiaozhibuzhan`，而不是 `root`。

## 检查是否为符号链接

执行：

```bash
ls -l "$HOME/.codex/config.toml"
readlink -f "$HOME/.codex/config.toml"
```

如果 `config.toml` 是符号链接，继续检查真实目标文件：

```bash
TARGET=$(readlink -f "$HOME/.codex/config.toml")
ls -l "$TARGET"
sudo chown "$USER":"$(id -gn)" "$TARGET"
chmod 600 "$TARGET"
```

如果目标位于 CC-switch 的目录中，确保该目录及其上级目录允许当前用户进入：

```bash
namei -l "$TARGET"
```

## 检查路径中每一级目录的权限

```bash
namei -l "$HOME/.codex/config.toml"
```

当前用户需要对路径中的目录拥有 `x`（进入）权限，并对配置文件拥有 `r`（读取）权限。

## 仍然无法读取时：重新创建配置文件

先备份旧文件：

```bash
mv "$HOME/.codex/config.toml" "$HOME/.codex/config.toml.bak"
touch "$HOME/.codex/config.toml"
chown "$USER":"$(id -gn)" "$HOME/.codex/config.toml"
chmod 600 "$HOME/.codex/config.toml"
```

然后运行：

```bash
codex
```

如果 CC-switch 需要管理这份配置，再在 CC-switch 中重新选择或导入配置。完成切换后，再次检查：

```bash
ls -l "$HOME/.codex/config.toml"
```

## 检查 ACL 权限（少数情况）

如果普通权限看起来正确，但仍然报错，检查 ACL：

```bash
getfacl "$HOME/.codex/config.toml"
```

如发现拒绝当前用户访问，可移除额外 ACL：

```bash
setfacl -b "$HOME/.codex/config.toml"
chmod 600 "$HOME/.codex/config.toml"
```

## 重要注意事项

- 不要使用 `sudo codex` 启动 Codex；
- 不要让 CC-switch 以 root 身份运行并写入用户配置；
- 修复后应始终使用普通用户运行 `codex`；
- 如果 CC-switch 每次切换后又恢复错误权限，需要检查其启动方式和配置文件保存目录，并确保它以当前用户身份运行。
