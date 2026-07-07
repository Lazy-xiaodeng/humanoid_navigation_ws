#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scan_bag_inventory.py

文件作用：
  1. 这是离线验证辅助脚本，不属于功能源码。
  2. 递归扫描一个或多个目录下的 rosbag2 metadata.yaml。
  3. 统计每个 bag 是否包含全局重定位验证需要的关键话题。
  4. 输出 CSV 和 Markdown，方便后续新增 bag 后快速判断能做哪类验证。

使用示例：
  python3 src/humanoid_global_relocalization_runtime/test/scan_bag_inventory.py \
    --root /home/ubuntu/nav_drift_test \
    --root /home/ubuntu/fast-lio-bags \
    --exclude /path/to/excluded_ws \
    --csv .codex_tmp/bag_inventory.csv \
    --md .codex_tmp/bag_inventory.md
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import yaml


KEY_TOPICS = [
    "/fast_lio/cloud_registered",
    "/cloud_registered_body",
    "/odom",
    "/robot_realpose",
]


@dataclass
class TopicInfo:
    """保存 metadata 中一个 topic 的类型和消息数量。"""

    topic_type: str = ""
    count: int = 0


@dataclass
class BagInventory:
    """保存一个 bag 的关键验证能力摘要。"""

    bag_path: Path
    storage_identifier: str
    duration_sec: float
    message_count: int
    topics: dict[str, TopicInfo]

    def has(self, topic: str) -> bool:
        return topic in self.topics and self.topics[topic].count > 0

    def count(self, topic: str) -> int:
        return self.topics.get(topic, TopicInfo()).count

    @property
    def registered_world_ready(self) -> bool:
        return self.has("/fast_lio/cloud_registered") and self.has("/odom")

    @property
    def registered_world_with_reference(self) -> bool:
        return self.registered_world_ready and self.has("/robot_realpose")

    @property
    def body_ready(self) -> bool:
        return self.has("/cloud_registered_body")

    @property
    def body_with_reference(self) -> bool:
        return self.body_ready and self.has("/odom") and self.has("/robot_realpose")


def is_under(path: Path, parent: Path) -> bool:
    """判断 path 是否位于 parent 目录下，用于排除禁止扫描的工作空间或生成产物。"""
    try:
        path.resolve().relative_to(parent.resolve())
        return True
    except ValueError:
        return False


def find_metadata_files(roots: Iterable[Path], excludes: Iterable[Path]) -> list[Path]:
    """递归查找 metadata.yaml；不存在的 root 会被跳过，排除目录用于避免误扫禁止工作空间。"""
    metadata_files: list[Path] = []
    exclude_dirs = [item for item in excludes if item.exists()]
    for root in roots:
        if not root.exists():
            continue
        if any(is_under(root, excluded) or is_under(excluded, root) and root == excluded for excluded in exclude_dirs):
            continue
        if root.is_file() and root.name == "metadata.yaml":
            if not any(is_under(root, excluded) for excluded in exclude_dirs):
                metadata_files.append(root)
            continue
        for metadata in root.rglob("metadata.yaml"):
            if any(is_under(metadata, excluded) for excluded in exclude_dirs):
                continue
            metadata_files.append(metadata)
    return sorted(set(metadata_files))


def parse_metadata(path: Path) -> BagInventory:
    """解析 rosbag2 metadata.yaml，抽取 topic 名称、类型、消息数量和整体时长。"""
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    info = data.get("rosbag2_bagfile_information", {})
    topics: dict[str, TopicInfo] = {}
    for item in info.get("topics_with_message_count", []):
        metadata = item.get("topic_metadata", {})
        name = metadata.get("name", "")
        if not name:
            continue
        topics[name] = TopicInfo(
            topic_type=metadata.get("type", ""),
            count=int(item.get("message_count", 0)),
        )

    duration_ns = int(info.get("duration", {}).get("nanoseconds", 0))
    return BagInventory(
        bag_path=path.parent,
        storage_identifier=str(info.get("storage_identifier", "")),
        duration_sec=duration_ns * 1e-9,
        message_count=int(info.get("message_count", 0)),
        topics=topics,
    )


def yes_no(value: bool) -> str:
    return "yes" if value else "no"


def write_csv(path: Path, inventories: list[BagInventory]) -> None:
    """写出机器可读 CSV，后续可继续用 Python/pandas 做统计。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "bag_path",
        "storage",
        "duration_sec",
        "message_count",
        "registered_world_ready",
        "registered_world_with_reference",
        "body_ready",
        "body_with_reference",
        "fast_lio_cloud_registered_count",
        "cloud_registered_body_count",
        "odom_count",
        "robot_realpose_count",
        "pointcloud_topics",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for inv in inventories:
            pointcloud_topics = [
                f"{name}:{topic.count}"
                for name, topic in sorted(inv.topics.items())
                if topic.topic_type == "sensor_msgs/msg/PointCloud2"
            ]
            writer.writerow(
                {
                    "bag_path": str(inv.bag_path),
                    "storage": inv.storage_identifier,
                    "duration_sec": f"{inv.duration_sec:.3f}",
                    "message_count": inv.message_count,
                    "registered_world_ready": yes_no(inv.registered_world_ready),
                    "registered_world_with_reference": yes_no(inv.registered_world_with_reference),
                    "body_ready": yes_no(inv.body_ready),
                    "body_with_reference": yes_no(inv.body_with_reference),
                    "fast_lio_cloud_registered_count": inv.count("/fast_lio/cloud_registered"),
                    "cloud_registered_body_count": inv.count("/cloud_registered_body"),
                    "odom_count": inv.count("/odom"),
                    "robot_realpose_count": inv.count("/robot_realpose"),
                    "pointcloud_topics": ";".join(pointcloud_topics),
                }
            )


def write_markdown(path: Path, inventories: list[BagInventory]) -> None:
    """写出人工可读 Markdown，方便粘贴到验证报告。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# rosbag 话题清单扫描",
        "",
        "| bag | reg_world | reg_world+ref | body | body+ref | /fast_lio/cloud_registered | /cloud_registered_body | /odom | /robot_realpose |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for inv in inventories:
        lines.append(
            "| "
            + " | ".join(
                [
                    str(inv.bag_path),
                    yes_no(inv.registered_world_ready),
                    yes_no(inv.registered_world_with_reference),
                    yes_no(inv.body_ready),
                    yes_no(inv.body_with_reference),
                    str(inv.count("/fast_lio/cloud_registered")),
                    str(inv.count("/cloud_registered_body")),
                    str(inv.count("/odom")),
                    str(inv.count("/robot_realpose")),
                ]
            )
            + " |"
        )
    lines.extend(
        [
            "",
            "说明：",
            "",
            "- `reg_world` 表示具备 `/fast_lio/cloud_registered + /odom`，可跑 registered_world 输入链路。",
            "- `reg_world+ref` 表示额外具备 `/robot_realpose`，可计算准确率。",
            "- `body` 表示具备真实 `/cloud_registered_body`，可验证 body 输入链路。",
            "- `body+ref` 表示同时具备 body、odom、reference，可做真实 body 准确率评估。",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Scan rosbag2 metadata and summarize global relocalization readiness.")
    parser.add_argument("--root", action="append", required=True, help="要递归扫描的目录，可重复指定")
    parser.add_argument("--exclude", action="append", default=[], help="要排除的目录，可重复指定；用于跳过禁止工作空间或生成产物")
    parser.add_argument("--csv", required=True, help="输出 CSV 路径")
    parser.add_argument("--md", required=True, help="输出 Markdown 路径")
    args = parser.parse_args()

    roots = [Path(item).expanduser() for item in args.root]
    excludes = [Path(item).expanduser() for item in args.exclude]
    metadata_files = find_metadata_files(roots, excludes)
    inventories = [parse_metadata(path) for path in metadata_files]
    write_csv(Path(args.csv), inventories)
    write_markdown(Path(args.md), inventories)

    print(f"metadata_files={len(metadata_files)}")
    print(f"registered_world_with_reference={sum(1 for inv in inventories if inv.registered_world_with_reference)}")
    print(f"body_ready={sum(1 for inv in inventories if inv.body_ready)}")
    print(f"body_with_reference={sum(1 for inv in inventories if inv.body_with_reference)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
