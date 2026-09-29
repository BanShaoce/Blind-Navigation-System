# 地图目录

- `rtabmap3.pgm` / `rtabmap3.yaml`：当前二维地图；
- `legacy/`：历史地图，仅用于追溯，不随 ROS 2 包安装。

PGM 与 YAML 必须成对版本化，YAML 的 `image` 应使用相对路径。场地数据库默认不提交仓库，部署时单独放置并通过 `database_path` 指定。地图可能暴露室内布局，发布前请确认数据授权。
