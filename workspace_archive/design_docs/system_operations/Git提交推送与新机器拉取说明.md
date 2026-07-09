# Git 提交推送与新机器拉取说明

最后更新：2026-05-22

本文面向第一次使用 Git 的人员，说明如何把当前 `humanoid_ws` 代码提交并推送到远程仓库，以及如何在新机器上把已推送版本拉取下来使用。

## 1. 当前仓库情况

当前工作空间：

```bash
/home/ubuntu/humanoid_ws
```

当前分支：

```bash
main
```

当前仓库已经配置 `origin`，并且 `origin` 有两个 push 地址。也就是说，正常情况下执行：

```bash
git push origin main
```

会把同一个提交推送到两个远程仓库。

查看远程仓库配置：

```bash
git remote -v
```

注意：如果远程地址里带账号、密码或 token，不要截图发给别人，也不要写进说明文档。

## 2. 哪些文件应该提交

建议提交：

- 源码：`src/`
- 配置：`config/`、`launch/`、`data/` 中确实需要保留的文件
- 启动脚本：`start_navigation.sh`
- 网页模拟 app：`debug_monitor/navigation_app_simulator.html`
- 使用说明文档：`*.md`
- 忽略规则：`.gitignore`

不建议提交：

- `build/`
- `install/`
- `log/`
- `debug_logs/`
- `.start_navigation.pid`
- `debug_monitor/code_backups/`
- `debug_output*.txt`
- 临时图片、TF 调试图、运行截图

这些运行时文件会让仓库变得很大，而且换机器后也不需要。

## 3. 提交前检查

进入工作空间：

```bash
cd /home/ubuntu/humanoid_ws
```

查看当前改动：

```bash
git status
```

简洁查看：

```bash
git status --short
```

常见状态含义：

```text
M  文件名    已跟踪文件被修改
D  文件名    已跟踪文件被删除
?? 文件名    新文件，还没有被 Git 跟踪
```

不要直接使用：

```bash
git add .
```

原因是它会把日志、临时文件、误删文件一起加入提交。

## 4. 第一次配置 Git 用户信息

如果新机器第一次提交代码，需要先配置用户名和邮箱：

```bash
git config --global user.name "你的名字"
git config --global user.email "你的邮箱"
```

检查：

```bash
git config user.name
git config user.email
```

## 5. 添加需要提交的文件

推荐明确添加文件。

本次一键启动脚本、网页模拟 app 和文档相关文件可以这样添加：

```bash
git add \
  .gitignore \
  start_navigation.sh \
  debug_monitor/navigation_app_simulator.html \
  一键启动脚本使用说明.md \
  系统功能使用说明书.md \
  源码迁移部署说明.md \
  Git提交推送与新机器拉取说明.md
```

如果本次确实修改了导航参数或行为树，也可以继续添加：

```bash
git add \
  data/dynamic_waypoints.json \
  src/humanoid_navigation2/config/nav2_params_xy_yaw.yaml \
  src/humanoid_navigation2/config/behavior_tree/navigate_xy_then_yaw.xml \
  src/humanoid_navigation2/config/behavior_tree/navigate_reverse_xy_then_yaw.xml
```

添加后检查暂存区：

```bash
git status --short
```

如果某一行左侧第一列出现 `A`、`M`，说明这个文件已经被加入本次提交。

## 6. 取消误添加文件

如果不小心把日志加进去了，例如：

```text
A  debug_logs/start_navigation_20260522_175240.log
```

可以取消暂存：

```bash
git restore --staged debug_logs/start_navigation_20260522_175240.log
```

如果误添加了整个日志目录：

```bash
git restore --staged debug_logs
```

这只会取消本次提交，不会删除本地文件。

## 7. 提交代码

确认暂存区没问题后提交：

```bash
git commit -m "feat: update startup script and app simulator"
```

提交成功后，会看到类似：

```text
[main xxxxxxx] feat: update startup script and app simulator
```

查看最近一次提交：

```bash
git log -1 --oneline
```

## 8. 推送到两个远程仓库

当前仓库已经配置一个 `origin` 对应两个 push 地址，所以执行：

```bash
git push origin main
```

正常情况下会依次推送到两个仓库。

推送成功时通常会看到类似：

```text
To https://github.com/xxx/xxx.git
   old..new  main -> main
To https://git.xxx.com/xxx/xxx.git
   old..new  main -> main
```

如果只成功一个、另一个失败，需要看失败原因。

常见失败原因：

- 网络不通
- 仓库地址写错
- 用户名、密码或 token 失效
- 当前账号没有仓库写入权限
- 远程仓库比本地更新，需要先拉取再推送

## 9. 如何配置两个 push 仓库

如果新机器没有配置两个 push 地址，可以按下面方式配置。

先查看：

```bash
git remote -v
```

如果没有 `origin`：

```bash
git remote add origin https://github.com/你的账号/你的仓库.git
```

设置第一个 push 地址：

```bash
git remote set-url --push origin https://github.com/你的账号/你的仓库.git
```

追加第二个 push 地址：

```bash
git remote set-url --add --push origin https://第二个Git服务器/组织名/仓库名.git
```

再次确认：

```bash
git remote -v
```

应该能看到一个 fetch 地址，以及两个 push 地址。

不要把密码直接写在远程地址里。推荐使用 GitHub token、Git 凭据管理器或 SSH key。

## 10. 新机器拉取代码

在新机器安装 Git：

```bash
sudo apt update
sudo apt install -y git
```

选择一个目录，例如家目录：

```bash
cd ~
```

从仓库克隆：

```bash
git clone https://github.com/你的账号/你的仓库.git humanoid_ws
```

进入工作空间：

```bash
cd ~/humanoid_ws
```

确认当前分支和版本：

```bash
git branch --show-current
git log -1 --oneline
```

如果需要切到 `main`：

```bash
git checkout main
```

如果以后要更新到远程最新版：

```bash
cd ~/humanoid_ws
git pull origin main
```

## 11. 新机器编译运行

新机器需要先安装 ROS 2 Jazzy 和系统依赖，详细依赖安装流程见：

```text
源码迁移部署说明.md
```

基础编译流程：

```bash
cd ~/humanoid_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

启动实机导航：

```bash
./start_navigation.sh
```

启动后日志会生成在：

```text
debug_logs/
```

这个目录是运行时日志，不需要提交到 Git。

## 12. Windows 使用网页模拟 app

Windows 只需要从仓库里取出这个文件：

```text
debug_monitor/navigation_app_simulator.html
```

用 Edge 或 Chrome 打开。

默认 WebSocket 地址已经配置为：

```text
ws://10.192.1.150:8765
```

小主机运行一键启动脚本后，Windows 点击页面里的“连接”，再点击“发送订阅”。

## 13. 推荐日常流程

每次修改代码后按这个顺序操作：

```bash
cd /home/ubuntu/humanoid_ws
git status --short
git add 需要提交的文件
git status --short
git commit -m "简短说明本次修改"
git push origin main
```

推送前必须看一眼 `git status --short`，确认没有把日志、临时文件、误删文件放进提交。

## 14. 本次提交建议

本次建议提交内容：

- `.gitignore`
- `start_navigation.sh`
- `debug_monitor/navigation_app_simulator.html`
- `一键启动脚本使用说明.md`
- `系统功能使用说明书.md`
- `源码迁移部署说明.md`
- `Git提交推送与新机器拉取说明.md`
- 已确认需要保留的导航参数和行为树配置

本次不建议提交：

- `debug_logs/`
- `.start_navigation.pid`
- `start_navigation_bak.sh`
- `debug_monitor/code_backups/`
- 当前工作区里显示为删除的旧调试输出、截图、PDF、临时文件
