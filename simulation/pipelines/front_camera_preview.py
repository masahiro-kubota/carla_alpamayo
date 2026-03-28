from __future__ import annotations

import os
import tkinter as tk
from typing import TYPE_CHECKING

from PIL import Image, ImageTk

if TYPE_CHECKING:
    import numpy as np


class FrontCameraPreview:
    def __init__(
        self,
        *,
        source_width: int,
        source_height: int,
        display_scale: float,
        title: str = "CARLA Front Camera",
    ) -> None:
        self.root = tk.Tk()
        self.root.title(title)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.closed = False
        self.image_label = tk.Label(self.root)
        self.image_label.pack()
        self.status_label = tk.Label(
            self.root,
            text="",
            anchor="w",
            justify="left",
            font=("TkFixedFont", 11),
        )
        self.status_label.pack(fill="x")
        self._photo_image: ImageTk.PhotoImage | None = None
        self.display_width = max(1, int(round(source_width * display_scale)))
        self.display_height = max(1, int(round(source_height * display_scale)))
        self.root.update()

    def update(self, rgb_array: np.ndarray, status_text: str) -> None:
        if self.closed:
            return
        image = Image.fromarray(rgb_array)
        if image.size != (self.display_width, self.display_height):
            image = image.resize((self.display_width, self.display_height))
        self._photo_image = ImageTk.PhotoImage(image=image)
        self.image_label.configure(image=self._photo_image)
        self.status_label.configure(text=status_text)
        self.root.update_idletasks()
        self.root.update()

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        self.root.destroy()


def has_display() -> bool:
    return bool(os.environ.get("DISPLAY"))
