# 人形机器人导航 Docker 离线部署包

完整教程请看：

```text
docs/人形机器人导航Docker离线部署使用说明书.md
```

如果 U 盘中只复制了 `docker_offline` 目录，没有复制 `docs` 目录，可以回到开发机查看：

```text
/home/ubuntu/humanoid_ws/docs/人形机器人导航Docker离线部署使用说明书.md
```

## 构建镜像

在开发机或构建机的 `/home/ubuntu/humanoid_ws` 下执行：

```bash
./deploy/docker_offline/scripts/build_image.sh
./deploy/docker_offline/scripts/save_image.sh
./deploy/docker_offline/scripts/fetch_docker_debs.sh
```

生成的镜像包默认位于：

```text
deploy/docker_offline/images/humanoid_nav_1.0.0.tar
```

Docker 离线安装包默认位于：

```text
deploy/docker_offline/docker_debs/
```

如果目标机器是纯净 Ubuntu 24.04 且不能联网，这个目录必须一起复制到 U 盘。

## U 盘部署

把 `deploy/docker_offline` 目录复制到 U 盘。目标机器插入 U 盘后执行：

```bash
cd /media/ubuntu/<U盘名>/docker_offline
sudo ./scripts/install_offline.sh
```

安装完成后，导航服务已创建但保持 disabled。

启动导航：

```bash
sudo systemctl start humanoid-navigation.service
```

停止导航：

```bash
sudo systemctl stop humanoid-navigation.service
```

启动建图：

```bash
cd /home/ubuntu/humanoid_deploy
./scripts/run_mapping.sh hall
```

保存建图：

```bash
./scripts/finish_mapping.sh
```

作废建图：

```bash
./scripts/abort_mapping.sh
```
