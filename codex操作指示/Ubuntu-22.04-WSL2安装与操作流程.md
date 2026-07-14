# Ubuntu-22.04 WSL2 安装与操作流程

本文档记录本次在 Windows 上安装并打开 `Ubuntu-22.04`、配置默认用户、迁移 Docker Desktop WSL 集成、删除 `Ubuntu-24.04`、并验证 Docker 可用的操作流程。

## 1. 检查当前 WSL 状态

先查看已有 WSL 发行版：

```powershell
wsl.exe --list --verbose
```

当时检查到的状态为：

```text
* Ubuntu-24.04   Running   2
  docker-desktop Running   2
```

再查看 WSL 默认版本和可安装发行版：

```powershell
wsl.exe --status
wsl.exe --list --online
```

确认在线列表中存在：

```text
Ubuntu-22.04    Ubuntu 22.04 LTS
```

## 2. 安装 Ubuntu 22.04

最初尝试使用 WSL 自带安装命令：

```powershell
wsl.exe --install Ubuntu-22.04 --no-launch
```

该命令长时间未返回。随后尝试 Web 下载模式：

```powershell
wsl.exe --install -d Ubuntu-22.04 --no-launch --web-download
```

该方式因访问 Microsoft WSL 发行版索引失败而中止，错误码为：

```text
Wsl/InstallDistro/0x80072efe
```

最终改用 `winget` 安装标准 Ubuntu 22.04 LTS 包：

```powershell
winget search --id Canonical.Ubuntu.2204 --exact
winget install --id Canonical.Ubuntu.2204 --exact --source winget --accept-package-agreements --accept-source-agreements --disable-interactivity
```

安装完成后确认 Appx 包存在：

```powershell
Get-AppxPackage CanonicalGroupLimited.Ubuntu22.04LTS |
  Select-Object Name,PackageFullName,Version,InstallLocation |
  Format-List
```

## 3. 注册并打开 Ubuntu-22.04

安装包完成后，WSL 发行版尚未注册。通过启动器触发初始化：

```powershell
ubuntu2204.exe --help
```

该启动器触发了 `Ubuntu-22.04` 的 WSL 注册。随后检查：

```powershell
wsl.exe --list --verbose
```

确认出现：

```text
Ubuntu-22.04    Running    2
```

之后用 root 用户直接进入并确认系统版本：

```powershell
wsl.exe -d Ubuntu-22.04 -u root -- cat /etc/os-release
```

确认结果：

```text
PRETTY_NAME="Ubuntu 22.04.2 LTS"
VERSION_ID="22.04"
VERSION_CODENAME=jammy
```

## 4. 创建默认 Linux 用户

新注册的发行版中没有普通用户，因此创建 Windows 同名用户 `ztwx4`，加入 `sudo` 组，并设置为 WSL 默认用户：

```powershell
wsl.exe -d Ubuntu-22.04 -u root -- sh -lc "id -u ztwx4 >/dev/null 2>&1 || useradd -m -s /bin/bash -G sudo ztwx4; printf '[user]\ndefault=ztwx4\n' > /etc/wsl.conf; printf 'ztwx4 ALL=(ALL) NOPASSWD:ALL\n' > /etc/sudoers.d/ztwx4; chmod 0440 /etc/sudoers.d/ztwx4; chown -R ztwx4:ztwx4 /home/ztwx4"
```

说明：

- `/etc/wsl.conf` 用来指定 WSL 默认登录用户。
- `/etc/sudoers.d/ztwx4` 让 `ztwx4` 可以无密码执行 `sudo`，避免非交互环境中无法提权。

使配置生效：

```powershell
wsl.exe --terminate Ubuntu-22.04
```

验证默认用户：

```powershell
wsl.exe -d Ubuntu-22.04 -- whoami
```

结果：

```text
ztwx4
```

## 5. 设置 Ubuntu-22.04 为默认 WSL 发行版

将默认 WSL 发行版切换为 `Ubuntu-22.04`：

```powershell
wsl.exe --set-default Ubuntu-22.04
```

验证：

```powershell
wsl.exe --list --verbose
```

预期结果：

```text
* Ubuntu-22.04   Running   2
  docker-desktop Running   2
  Ubuntu-24.04   Running   2
```

此时星号表示 `Ubuntu-22.04` 已成为默认发行版。

## 6. 修改 Docker Desktop WSL 集成配置

Docker Desktop 原配置中只集成了旧发行版：

```json
"IntegratedWslDistros": [
  "Ubuntu-24.04"
]
```

配置文件位置：

```powershell
$env:APPDATA\Docker\settings-store.json
```

将其修改为：

```json
"IntegratedWslDistros": [
  "Ubuntu-22.04"
]
```

修改后完整核心配置类似：

```json
{
  "AutoStart": false,
  "DisplayedOnboarding": true,
  "EnableDockerAI": true,
  "IntegratedWslDistros": [
    "Ubuntu-22.04"
  ],
  "LastContainerdSnapshotterEnable": 1775466305,
  "LicenseTermsVersion": 2,
  "SettingsVersion": 43,
  "UseContainerdSnapshotter": true
}
```

## 7. 删除 Ubuntu-24.04

先终止旧发行版：

```powershell
wsl.exe --terminate Ubuntu-24.04
```

再注销旧发行版：

```powershell
wsl.exe --unregister Ubuntu-24.04
```

注意：`wsl.exe --unregister Ubuntu-24.04` 会删除 `Ubuntu-24.04` 发行版内的 Linux 文件系统数据。

删除后验证：

```powershell
wsl.exe --list --verbose
```

最终结果：

```text
* Ubuntu-22.04   Running   2
  docker-desktop Running   2
```

## 8. 验证 Docker Desktop 在 Ubuntu-22.04 中可用

先验证 Windows 侧 Docker Engine：

```powershell
docker version --format '{{.Client.Version}} / {{.Server.Version}}'
```

结果：

```text
29.3.1 / 29.3.1
```

再进入 `Ubuntu-22.04` 验证 Docker CLI 和 Docker Desktop daemon 连接：

```powershell
wsl.exe -d Ubuntu-22.04 -- sh -lc 'whoami; command -v docker || true; docker version --format "{{.Client.Version}} / {{.Server.Version}}" 2>&1 || true'
```

结果显示：

```text
ztwx4
/usr/bin/docker
29.3.1
```

最后运行：

```powershell
wsl.exe -d Ubuntu-22.04 -- docker ps --format "table {{.ID}}\t{{.Image}}\t{{.Status}}"
```

确认可以看到 Docker Desktop 中正在运行的容器，例如：

```text
CONTAINER ID   IMAGE       STATUS
5596ab21258d   mysql:8.0   Up 2 hours
11981a0bfd16   mysql:5.6   Up 2 hours
```

## 9. 常用打开方式

以后打开默认 Ubuntu 22.04：

```powershell
wsl
```

或明确打开 `Ubuntu-22.04`：

```powershell
wsl.exe -d Ubuntu-22.04
```

以 root 身份打开：

```powershell
wsl.exe -d Ubuntu-22.04 -u root
```

在 Windows PowerShell 中直接让 Ubuntu 执行单条命令：

```powershell
wsl.exe -d Ubuntu-22.04 -- <linux-command>
```

示例：

```powershell
wsl.exe -d Ubuntu-22.04 -- lsb_release -ds
wsl.exe -d Ubuntu-22.04 -- docker ps
```

## 10. 当前最终状态

最终验证命令：

```powershell
wsl.exe --list --verbose
wsl.exe -d Ubuntu-22.04 -- sh -lc 'whoami && lsb_release -ds && docker ps --format "{{.Image}} {{.Status}}" | head -n 3'
```

最终状态：

```text
* Ubuntu-22.04   Running   2
  docker-desktop Running   2

ztwx4
Ubuntu 22.04.2 LTS
mysql:8.0
mysql:5.6
```

