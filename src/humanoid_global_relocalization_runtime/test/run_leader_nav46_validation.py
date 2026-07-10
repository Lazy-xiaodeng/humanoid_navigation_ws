#!/usr/bin/env python3
"""Train/test LEADER on the custom nav46 dataset.

文件作用：
  1. 这是离线验证辅助脚本，不属于运行态节点。
  2. 在自定义 nav46 数据集上训练或测试 LEADER 研究链路。
  3. 该脚本用于评估深度方法可行性，不直接影响正式全局重定位功能包。

This script intentionally imports the official LEADER code lazily, so it can be
syntax-checked on CPU machines while real execution happens on a CUDA +
MinkowskiEngine environment.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np


def yaw_to_matrix(x: float, y: float, z: float, yaw_deg: float) -> np.ndarray:
    yaw = math.radians(yaw_deg)
    c = math.cos(yaw)
    s = math.sin(yaw)
    t = np.eye(4, dtype=np.float32)
    t[:3, :3] = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float32)
    t[:3, 3] = np.array([x, y, z], dtype=np.float32)
    return t


def yaw_from_matrix(t: np.ndarray) -> float:
    return math.degrees(math.atan2(float(t[1, 0]), float(t[0, 0])))


def wrap_angle_deg(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def load_manifest(dataset_dir: Path) -> list[dict[str, str]]:
    with (dataset_dir / "manifest.csv").open(newline="") as f:
        return list(csv.DictReader(f))


def build_dataset_class(base_dataset_cls, me_module, pose_util):
    class Nav46LeaderDataset(base_dataset_cls):
        def __init__(
            self,
            rows: list[dict[str, str]],
            voxel_size: float,
            horizontal_res: int,
            min_range: float,
            max_range: float,
        ) -> None:
            self.rows = rows
            self.voxel_size = voxel_size
            self.horizontal_res = horizontal_res
            self.min_range = min_range
            self.max_range = max_range
            self.transforms = [
                yaw_to_matrix(float(r["x"]), float(r["y"]), float(r["z"]), float(r["yaw_deg"]))
                for r in rows
            ]
            pose_arr = np.stack(self.transforms)
            self.center_t = np.array(
                [pose_arr[:, 0, 3].mean(), pose_arr[:, 1, 3].mean(), pose_arr[:, 2, 3].min()],
                dtype=np.float32,
            )

        def get_center_t(self) -> np.ndarray:
            return self.center_t

        def __len__(self) -> int:
            return len(self.rows)

        def __getitem__(self, index: int):
            row = self.rows[index]
            scan = np.fromfile(row["bin"], dtype=np.float32).reshape(-1, 4)
            xyz = scan[:, :3]
            label = scan[:, 3:4]
            ranges = np.linalg.norm(xyz, axis=1)
            mask = (ranges > self.min_range) & (ranges < self.max_range)
            xyz = xyz[mask]
            label = label[mask]
            pl_coords = pose_util.cartesian_to_polar_expansion(xyz, self.voxel_size * self.horizontal_res)
            pl_feats = np.concatenate((pl_coords[:, 2:3], pl_coords[:, 1:2], label), axis=1).astype(np.float32)
            coords, feats = me_module.utils.sparse_quantize(
                coordinates=pl_coords,
                features=pl_feats,
                quantization_size=self.voxel_size,
            )
            return coords, feats, xyz.astype(np.float32), self.transforms[index], np.eye(4, dtype=np.float32), row

    return Nav46LeaderDataset


def collate_factory(torch_module, me_module):
    def collate(batch):
        coords, feats, points, transforms, corrections, rows = list(zip(*batch))
        return {
            "coords": me_module.utils.batched_coordinates(coords),
            "feats": torch_module.from_numpy(np.concatenate(feats)).float(),
            "points": points,
            "T": torch_module.from_numpy(np.stack(transforms)).float(),
            "T_corr": torch_module.from_numpy(np.stack(corrections)).float(),
            "rows": rows,
        }

    return collate


class NullWriter:
    def add_scalar(self, *_args, **_kwargs) -> None:
        return None


def check_environment(leader_root: Path) -> tuple[bool, list[str]]:
    messages: list[str] = []
    ok = True
    sys.path.insert(0, str(leader_root))
    try:
        import torch  # type: ignore

        messages.append(f"torch={torch.__version__} cuda_available={torch.cuda.is_available()}")
        if not torch.cuda.is_available():
            ok = False
    except Exception as exc:  # pragma: no cover - depends on external env
        ok = False
        messages.append(f"torch_import_error={exc}")
    for module_name in ["MinkowskiEngine", "pypatchworkpp", "open3d", "accelerate"]:
        try:
            __import__(module_name)
            messages.append(f"{module_name}=ok")
        except Exception as exc:  # pragma: no cover - depends on external env
            ok = False
            messages.append(f"{module_name}_import_error={exc}")
    return ok, messages


def load_external_modules(leader_root: Path):
    sys.path.insert(0, str(leader_root))
    import torch  # type: ignore
    import MinkowskiEngine as ME  # type: ignore
    from accelerate import Accelerator  # type: ignore
    from torch.utils.data import DataLoader, Dataset  # type: ignore
    from models.model_mink import LEADER  # type: ignore
    from models.sc2pcr import Matcher  # type: ignore
    from run_mink import Logger, TRR, load_checkpoint, process_one_epoch, save_checkpoint, setup_seed  # type: ignore
    from utils import pose_util  # type: ignore

    return SimpleNamespace(
        torch=torch,
        ME=ME,
        Accelerator=Accelerator,
        DataLoader=DataLoader,
        Dataset=Dataset,
        LEADER=LEADER,
        Matcher=Matcher,
        Logger=Logger,
        TRR=TRR,
        load_checkpoint=load_checkpoint,
        process_one_epoch=process_one_epoch,
        save_checkpoint=save_checkpoint,
        setup_seed=setup_seed,
        pose_util=pose_util,
    )


def load_state(args, ext, accelerator, process_info):
    if args.resume_model:
        ext.load_checkpoint(args.resume_model, accelerator, process_info)


def evaluate_to_csv(args, ext, model, loader, process_info, output_csv: Path) -> None:
    torch = ext.torch
    ME = ext.ME
    device = torch.device("cuda")
    model.eval()
    ransac = ext.Matcher(inlier_threshold=args.inlier_threshold, d_thre=2, num_iterations=10, ratio=0.15, nms_radius=0.1, max_points=args.max_ransac_points, k1=30)
    center_t = torch.tensor(process_info["center_t"], dtype=torch.float32, device=device)

    rows_out = []
    with torch.no_grad():
        for input_dict in loader:
            start = time.time()
            coords = input_dict["coords"]
            feats = input_dict["feats"]
            gt_t = input_dict["T"].to(device)
            t_corr = input_dict["T_corr"].to(device)
            sparse = ME.SparseTensor(feats, coords)
            enc = model.encoder(sparse)
            pred_f = model.decoder(enc.F)
            batch_idx = enc.C[:, 0].long()
            stride = torch.tensor(enc.tensor_stride, device=device, dtype=torch.float32)
            voxel_centers = (enc.C[:, 1:].float() + stride / 2) * args.voxel_size
            voxel_centers_l = ext.pose_util.polar_expansion_to_cartesian(voxel_centers, args.horizontal_res * args.voxel_size)

            pred_t = []
            for i in range(gt_t.shape[0]):
                mask = batch_idx == i
                c_pred = pred_f[mask, :3].float()
                u_pred = pred_f[mask, 3].float()
                c_local = voxel_centers_l[mask, :3].float()
                k = max(min(args.top_reliable_points, u_pred.shape[0]), int(args.reliable_fraction * u_pred.shape[0]))
                _, indices = u_pred.topk(k=k)
                t = ransac.estimator(c_local[indices][None], c_pred[indices][None])[0]
                t[:3, 3] += center_t
                pred_t.append(t @ t_corr[i])
            elapsed = (time.time() - start) / max(1, len(pred_t))

            pred_np = torch.stack(pred_t).detach().cpu().numpy()
            gt_np = gt_t.detach().cpu().numpy()
            for meta, pred, gt in zip(input_dict["rows"], pred_np, gt_np):
                trans_error = float(np.linalg.norm(pred[:3, 3] - gt[:3, 3]))
                yaw_error = abs(wrap_angle_deg(yaw_from_matrix(pred) - float(meta["yaw_deg"])))
                rows_out.append(
                    {
                        "target_id": meta["id"],
                        "cloud_index": meta["cloud_index"],
                        "pred_x": float(pred[0, 3]),
                        "pred_y": float(pred[1, 3]),
                        "pred_z": float(pred[2, 3]),
                        "pred_yaw_deg": yaw_from_matrix(pred),
                        "gt_x": float(gt[0, 3]),
                        "gt_y": float(gt[1, 3]),
                        "gt_yaw_deg": float(meta["yaw_deg"]),
                        "translation_error_m": trans_error,
                        "yaw_error_deg": yaw_error,
                        "success_0p2m_3deg": int(trans_error <= 0.2 and yaw_error <= 3.0),
                        "success_0p3m_5deg": int(trans_error <= 0.3 and yaw_error <= 5.0),
                        "success_0p5m_10deg": int(trans_error <= 0.5 and yaw_error <= 10.0),
                        "seconds_per_scan": elapsed,
                    }
                )

    output_csv.parent.mkdir(parents=True, exist_ok=True)
    fields = list(rows_out[0].keys()) if rows_out else []
    with output_csv.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows_out)

    n = max(1, len(rows_out))
    print(
        "leader_eval "
        f"targets={len(rows_out)} "
        f"0.2m/3deg={sum(r['success_0p2m_3deg'] for r in rows_out)}/{n} "
        f"0.3m/5deg={sum(r['success_0p3m_5deg'] for r in rows_out)}/{n} "
        f"0.5m/10deg={sum(r['success_0p5m_10deg'] for r in rows_out)}/{n} "
        f"csv={output_csv}"
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--leader-root", default=".codex_tmp/LEADER")
    parser.add_argument("--dataset-dir", default=".codex_tmp/leader_nav46/hard52")
    parser.add_argument("--mode", choices=["check", "train", "test"], default="check")
    parser.add_argument("--resume-model", default="")
    parser.add_argument("--log-dir", default=".codex_tmp/leader_nav46/log_hard52")
    parser.add_argument("--output-csv", default=".codex_tmp/leader_nav46/hard52_predictions.csv")
    parser.add_argument("--batch-size", type=int, default=12)
    parser.add_argument("--val-batch-size", type=int, default=8)
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--num-workers", type=int, default=4)
    parser.add_argument("--voxel-size", type=float, default=0.2)
    parser.add_argument("--horizontal-res", type=int, default=1024)
    parser.add_argument("--min-range", type=float, default=1.0)
    parser.add_argument("--max-range", type=float, default=80.0)
    parser.add_argument("--init-learning-rate", type=float, default=0.001)
    parser.add_argument("--decay-epoch", type=float, default=1.0)
    parser.add_argument("--seed", type=int, default=20)
    parser.add_argument("--inlier-threshold", type=float, default=2.0)
    parser.add_argument("--top-reliable-points", type=int, default=50)
    parser.add_argument("--reliable-fraction", type=float, default=0.5)
    parser.add_argument("--max-ransac-points", type=int, default=3000)
    args = parser.parse_args()

    leader_root = Path(args.leader_root).resolve()
    env_ok, messages = check_environment(leader_root)
    for msg in messages:
        print(msg)
    if args.mode == "check":
        return 0 if env_ok else 2
    if not env_ok:
        print("LEADER runtime dependencies are not ready; run mode=check on the GPU machine first.")
        return 2

    ext = load_external_modules(leader_root)
    if not ext.torch.cuda.is_available():
        print("CUDA GPU is required by the official LEADER runtime.")
        return 2
    ext.setup_seed(args.seed)

    rows = load_manifest(Path(args.dataset_dir).resolve())
    train_rows = [r for r in rows if r["split"] == "train"]
    test_rows = [r for r in rows if r["split"] == "test"]
    Dataset = build_dataset_class(ext.Dataset, ext.ME, ext.pose_util)
    train_set = Dataset(train_rows, args.voxel_size, args.horizontal_res, args.min_range, args.max_range)
    test_set = Dataset(test_rows, args.voxel_size, args.horizontal_res, args.min_range, args.max_range)
    collate = collate_factory(ext.torch, ext.ME)
    train_loader = ext.DataLoader(train_set, batch_size=args.batch_size, shuffle=True, collate_fn=collate, num_workers=args.num_workers, pin_memory=True)
    test_loader = ext.DataLoader(test_set, batch_size=args.val_batch_size, shuffle=False, collate_fn=collate, num_workers=args.num_workers, pin_memory=True)

    model = ext.LEADER(in_channels=3, out_channels=4, feat_channels=512, width=args.horizontal_res)
    optimizer = ext.torch.optim.Adam(model.parameters(), args.init_learning_rate)
    scheduler = ext.torch.optim.lr_scheduler.StepLR(optimizer, args.decay_epoch, gamma=0.9)
    accelerator = ext.Accelerator()
    model, optimizer, train_loader, test_loader, scheduler = accelerator.prepare(model, optimizer, train_loader, test_loader, scheduler)
    process_info = {
        "epoch": -1,
        "train_iter": 0,
        "val_iter": 0,
        "center_t": (train_set.get_center_t() + np.array([0.0, 0.0, -2.0 * args.max_range], dtype=np.float32)).tolist(),
    }
    load_state(args, ext, accelerator, process_info)
    os.makedirs(args.log_dir, exist_ok=True)
    logger = ext.Logger(SimpleNamespace(log_dir=args.log_dir))

    if args.mode == "train":
        loss_fn = ext.TRR(scale=10.0)
        ransac = ext.Matcher(inlier_threshold=args.inlier_threshold, d_thre=2, num_iterations=10, ratio=0.15, nms_radius=0.1, max_points=args.max_ransac_points, k1=30)
        flags = SimpleNamespace(**vars(args))
        flags.max_epoch = args.epochs
        for epoch in range(process_info["epoch"] + 1, args.epochs):
            process_info["epoch"] = epoch
            ext.process_one_epoch(model, train_loader, optimizer, scheduler, accelerator, ransac, process_info, True, NullWriter(), loss_fn, logger, flags, ext.torch.device("cuda"))
            if accelerator.is_main_process:
                ext.save_checkpoint(os.path.join(args.log_dir, f"checkpoint_epoch_{epoch}"), accelerator, process_info)

    if accelerator.is_main_process:
        evaluate_to_csv(args, ext, model, test_loader, process_info, Path(args.output_csv).resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
