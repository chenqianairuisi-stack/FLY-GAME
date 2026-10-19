#!/usr/bin/env python3
"""Randomize the demo world's floor and window textures.

The script works on the unpacked binary_install tree shipped with this demo
workspace. It is intentionally standalone: run it before starting Gazebo.
"""

from __future__ import annotations

import argparse
import json
import random
import re
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]
RUNTIME_SHARE = WORKSPACE / "binary_install/share/ican_delivery_sim"
MATERIAL_FILE = RUNTIME_SHARE / "materials/scripts/ican_delivery.material"
WORLD_FILE = RUNTIME_SHARE / "worlds/delivery_task2.world"
TEXTURE_DIR = RUNTIME_SHARE / "materials/textures"
QR_DIR = TEXTURE_DIR / "aruco_markers"
WINDOW_DIR = TEXTURE_DIR / "window_candidates"
IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}


def image_files(directory: Path) -> list[Path]:
    if not directory.is_dir():
        return []
    return sorted(
        path for path in directory.iterdir()
        if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
    )


def choose(files: list[Path], rng: random.Random, label: str) -> Path:
    if not files:
        raise RuntimeError(f"{label}候选贴图目录为空或不存在")
    return rng.choice(files)


def replace_material_texture(text: str, material_name: str, filename: str) -> str:
    block = re.compile(
        rf"(material\s+{re.escape(material_name)}\s*\{{.*?texture\s+)([^\s}}]+)",
        re.DOTALL,
    )
    updated, count = block.subn(rf"\g<1>{filename}", text, count=1)
    if count != 1:
        raise RuntimeError(f"材质文件中找不到 {material_name} 的 texture")
    return updated


def update_world_resource_paths(text: str) -> str:
    script_uri = "model://ican_delivery_sim/materials/scripts"
    texture_uri = "model://ican_delivery_sim/materials/textures"
    text = re.sub(r"<uri>file://[^<]*/materials/scripts</uri>",
                  f"<uri>{script_uri}</uri>", text)
    text = re.sub(r"<uri>file://[^<]*/materials/textures</uri>",
                  f"<uri>{texture_uri}</uri>", text)
    if script_uri not in text or texture_uri not in text:
        raise RuntimeError("世界文件中没有可更新的材质资源路径")
    return text


def atomic_write(path: Path, content: str) -> None:
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(content, encoding="utf-8")
    temporary.replace(path)


def install_texture(source: Path, role: str) -> str:
    destination_name = f"demo_{role}_{source.name}"
    destination = TEXTURE_DIR / destination_name
    if source.resolve() != destination.resolve():
        shutil.copy2(source, destination)
    return destination_name


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, help="固定随机种子，便于复现实验")
    parser.add_argument("--dry-run", action="store_true", help="只显示选择结果，不修改文件")
    parser.add_argument("--qr-dir", type=Path, default=QR_DIR,
                        help=f"二维码候选目录（默认：{QR_DIR}）")
    parser.add_argument("--window-dir", type=Path, default=WINDOW_DIR,
                        help=f"第二窗户候选目录（默认：{WINDOW_DIR}）")
    parser.add_argument("--ground-dir", type=Path,
                        help="可选的地面环境候选目录；未指定时保留当前地面贴图")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    for required in (MATERIAL_FILE, WORLD_FILE, TEXTURE_DIR):
        if not required.exists():
            raise RuntimeError(f"找不到运行资源：{required}")

    rng = random.Random(args.seed)
    qr = choose(image_files(args.qr_dir), rng, "第一窗户二维码")
    window_pool = image_files(args.window_dir)
    # The bundled demo pool follows the task rule: window 2 uses the
    # eagle/drone-style images. A custom directory remains unrestricted.
    if args.window_dir.resolve() == WINDOW_DIR.resolve():
        preferred = [
            path for path in window_pool
            if path.stem.lower().startswith(("window_bird", "eagle", "drone"))
        ]
        if preferred:
            window_pool = preferred
    window = choose(window_pool, rng, "第二窗户")

    ground = None
    if args.ground_dir is not None:
        ground = choose(image_files(args.ground_dir), rng, "地面环境")

    selected = {
        "qr_window_1": str(qr),
        "window_2": str(window),
        "ground": str(ground) if ground else "keep-current",
    }
    print(json.dumps(selected, ensure_ascii=False, indent=2))
    if args.dry_run:
        return 0

    material_text = MATERIAL_FILE.read_text(encoding="utf-8")
    material_text = replace_material_texture(
        material_text, "DeliveryWindowTarget1", install_texture(qr, "window_1_qr")
    )
    material_text = replace_material_texture(
        material_text, "DeliveryWindowTarget2", install_texture(window, "window_2")
    )
    if ground is not None:
        material_text = replace_material_texture(
            material_text, "DeliveryGround", install_texture(ground, "ground")
        )

    world_text = update_world_resource_paths(WORLD_FILE.read_text(encoding="utf-8"))
    atomic_write(MATERIAL_FILE, material_text)
    atomic_write(WORLD_FILE, world_text)

    report = {
        "updated_at": datetime.now(timezone.utc).isoformat(),
        "seed": args.seed,
        "workspace": str(WORKSPACE),
        "assignments": selected,
    }
    atomic_write(
        RUNTIME_SHARE / "demo_texture_assignment.json",
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
    )
    print(f"贴图已写入：{MATERIAL_FILE}")
    print(f"世界资源路径已更新：{WORLD_FILE}")
    print("请重新启动 Gazebo 以加载新贴图。")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        raise SystemExit(1)
