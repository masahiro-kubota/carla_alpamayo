from __future__ import annotations

from pathlib import Path
import shutil
import subprocess


def render_png_sequence_to_mp4(
    image_dir: Path,
    output_path: Path,
    fps: float,
    crf: int = 23,
    overwrite: bool = True,
) -> Path:
    ffmpeg_path = shutil.which("ffmpeg")
    if ffmpeg_path is None:
        raise RuntimeError("ffmpeg was not found in PATH.")

    if not image_dir.exists():
        raise FileNotFoundError(f"Image directory does not exist: {image_dir}")

    first_frame = image_dir / "000000.png"
    if not first_frame.exists():
        raise FileNotFoundError(
            f"Expected PNG sequence starting at {first_frame}, but it was not found."
        )

    output_path.parent.mkdir(parents=True, exist_ok=True)

    command = [
        ffmpeg_path,
        "-hide_banner",
        "-loglevel",
        "error",
        "-y" if overwrite else "-n",
        "-framerate",
        f"{fps:.6g}",
        "-i",
        str(image_dir / "%06d.png"),
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        "-preset",
        "fast",
        "-crf",
        str(crf),
        "-movflags",
        "+faststart",
        str(output_path),
    ]
    subprocess.run(command, check=True)
    return output_path
