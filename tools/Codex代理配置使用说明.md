# Codex / VS Code 代理一键配置脚本使用说明

本文档说明如何在 Ubuntu 主机、Ubuntu 虚拟机、远程桌面和 SSH 场景中使用
`tools/setup_codex_proxy.sh`，让 Codex、VS Code 终端和桌面图标启动的 VS Code
都能继承代理环境变量。

## 适用场景

- Ubuntu 24.04 虚拟机需要通过宿主机 Windows 代理访问 ChatGPT / Codex。
- 通过 ToDesk、Windows 远程桌面、SSH 连接到 Ubuntu 后配置 Codex 网络。
- 桌面图标打开 VS Code 时，希望 VS Code 扩展也自动吃到代理。
- 多台机器需要重复配置相同逻辑。

## 前置条件

宿主机代理软件需要开启局域网访问，例如：

- Allow LAN
- 允许局域网连接
- Listen LAN
- 局域网共享

常见代理端口：

- `7890`
- `7897`
- `1080`
- `10808`

在 VMware 虚拟机里，宿主机代理地址常见是 `192.168.x.1`，例如：

```bash
192.168.50.1:7890
```

## 脚本位置

```bash
tools/setup_codex_proxy.sh
```

查看帮助：

```bash
./tools/setup_codex_proxy.sh --help
```

## 推荐用法

如果你已经知道宿主机代理 IP 和端口：

```bash
./tools/setup_codex_proxy.sh --host 192.168.50.1 --port 7890 --test
```

如果宿主机 IP 就是 Ubuntu 默认网关，可以让脚本自动推断：

```bash
./tools/setup_codex_proxy.sh --test
```

如果代理端口是 SOCKS：

```bash
./tools/setup_codex_proxy.sh --host 192.168.50.1 --port 7890 --socks --test
```

## 脚本会修改什么

脚本只修改当前用户目录下的配置：

```text
~/.config/environment.d/proxy.conf
~/.config/codex-proxy.env
~/.local/share/applications/code.desktop
~/.bashrc
```

作用分别是：

- `environment.d/proxy.conf`：让图形登录会话继承代理变量。
- `codex-proxy.env`：保存 shell 可复用的代理变量。
- `code.desktop`：让桌面图标启动 VS Code 时直接带代理变量。
- `.bashrc`：让 SSH / 终端登录后自动加载代理变量。

## 运行后必须做什么

运行脚本后建议执行：

```text
1. 完全退出 VS Code。
2. 注销 Ubuntu 用户并重新登录。
3. 从桌面图标重新打开 VS Code。
```

然后在 VS Code 终端验证：

```bash
env | grep -i proxy
```

应该看到类似：

```text
HTTP_PROXY=http://192.168.50.1:7890
HTTPS_PROXY=http://192.168.50.1:7890
ALL_PROXY=http://192.168.50.1:7890
NO_PROXY=localhost,127.0.0.1,::1
```

## 验证 Codex

建议使用 VS Code 扩展自带的 Codex 路径验证，避免 `/snap/bin/codex`
抢先被执行：

```bash
/home/ubuntu/.vscode/extensions/openai.chatgpt-26.602.30954-linux-x64/bin/linux-x86_64/codex doctor
```

重点看：

```text
proxy env vars present
websocket connected
reachability
```

如果 `websocket connected` 通过，说明代理变量已经传给 Codex。

如果 `reachability` 仍然超时，通常是代理节点或规则问题，不是 VS Code
没有吃到代理。可以尝试：

```text
1. 宿主机代理切到 Global / 全局模式。
2. 换一个节点。
3. 确认代理软件没有把 chatgpt.com 或 backend-api 直连。
4. 确认宿主机防火墙允许虚拟机访问代理端口。
```

## 常见问题

### curl 返回 HTTP/2 403 是否失败？

不一定。裸 `curl` 访问 `chatgpt.com` 经常会被 Cloudflare challenge 拦截，
返回 `HTTP/2 403` 是常见现象。重点是不要出现：

```text
Could not resolve host
Couldn't connect to server
request timed out
```

### 桌面图标打开 VS Code 还是没有代理？

先确认脚本是否生成了用户级桌面文件：

```bash
grep '^Exec=' ~/.local/share/applications/code.desktop
```

如果看到 `Exec=env HTTP_PROXY=... /usr/share/code/code`，说明桌面图标覆盖已写入。
然后完全退出 VS Code，注销重登，再从图标打开。

### SSH 场景需要桌面图标配置吗？

不需要也可以。SSH 场景主要依赖：

```text
~/.bashrc
~/.config/codex-proxy.env
```

如果只想配置 SSH / shell，不配置桌面图标：

```bash
./tools/setup_codex_proxy.sh --host 192.168.50.1 --port 7890 --no-desktop --test
```

### 如何跳过 .bashrc 修改？

```bash
./tools/setup_codex_proxy.sh --host 192.168.50.1 --port 7890 --no-shell
```

## 回滚方法

删除脚本生成的用户级配置：

```bash
rm -f ~/.config/environment.d/proxy.conf
rm -f ~/.config/codex-proxy.env
rm -f ~/.local/share/applications/code.desktop
```

然后手动从 `~/.bashrc` 删除以下片段：

```bash
# Codex / VS Code proxy environment
if [ -f "$HOME/.config/codex-proxy.env" ]; then
  source "$HOME/.config/codex-proxy.env"
fi
```

最后注销并重新登录。
