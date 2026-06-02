import tkinter as tk
from tkinter import filedialog, messagebox
from PIL import Image, ImageTk
import cv2
import numpy as np

# --- CONFIGURATION ---
# Target resolution for the STM32 LED Matrix
TARGET_W = 19
TARGET_H = 20

# Greyscale via Binary Code Modulation: COLOR_DEPTH bit planes -> 2^COLOR_DEPTH levels.
COLOR_DEPTH = 3              # must match animations.h
GREY_LEVELS = 1 << COLOR_DEPTH   # 8 levels (0..7)

# Flash budget cap. Each greyscale frame costs COLOR_DEPTH * Y_RES * 4 bytes = 240 B.
# With ~13 KB of code: 64K flash -> ~200 frames; 128K flash -> ~450 frames.
# Only the FIRST MAX_FRAMES frames of the video are exported (set None for all).
MAX_FRAMES = 400
# BCM on-time is linear in level, but human brightness perception is not. A gamma > 1
# pushes mid-tones darker so the ramp looks more even. Tune to taste (1.0 = linear).
GAMMA = 2.2


def quantize_to_levels(resized_gray, cutoff):
    """Map an 8-bit grayscale image to 0..GREY_LEVELS-1 brightness levels.
    Pixels below `cutoff` are forced to 0 (kills background noise); the rest of the
    range [cutoff..255] is gamma-corrected and quantized."""
    g = resized_gray.astype(np.float32)
    span = max(1, 255 - cutoff)
    norm = np.clip((g - cutoff) / span, 0.0, 1.0)
    norm = norm ** GAMMA
    return np.round(norm * (GREY_LEVELS - 1)).astype(np.uint8)

class LEDConverterApp:
    def __init__(self, root):
        self.root = root
        self.root.title("STM32 LED Matrix Converter (19x20)")
        
        self.cap = None
        self.total_frames = 0
        self.current_frame_idx = 0
        self.video_path = ""
        
        # --- UI LAYOUT ---
        
        # Top Frame for Images
        self.frame_images = tk.Frame(root)
        self.frame_images.pack(pady=10)
        
        # Left: Original Video
        self.lbl_original = tk.Label(self.frame_images, text="Original Video")
        self.lbl_original.grid(row=0, column=0, padx=10)
        self.canvas_orig = tk.Canvas(self.frame_images, width=300, height=300, bg="black")
        self.canvas_orig.grid(row=1, column=0, padx=10)

        # Right: Matrix Preview
        self.lbl_preview = tk.Label(self.frame_images, text=f"Matrix Preview ({TARGET_W}x{TARGET_H})")
        self.lbl_preview.grid(row=0, column=1, padx=10)
        self.canvas_prev = tk.Canvas(self.frame_images, width=300, height=300, bg="black")
        self.canvas_prev.grid(row=1, column=1, padx=10)

        # Controls Frame
        self.frame_controls = tk.Frame(root)
        self.frame_controls.pack(pady=10, fill="x", padx=20)

        # Load Button
        self.btn_load = tk.Button(self.frame_controls, text="Load Video/GIF", command=self.load_video)
        self.btn_load.pack(side="top", pady=5)

        # Frame Scrubber
        self.lbl_frame = tk.Label(self.frame_controls, text="Frame: 0")
        self.lbl_frame.pack()
        self.scale_frame = tk.Scale(self.frame_controls, from_=0, to=100, orient="horizontal", command=self.on_frame_scroll)
        self.scale_frame.pack(fill="x")

        # Black-level cutoff Slider (pixels below this become level 0 / off)
        self.lbl_thresh = tk.Label(self.frame_controls, text="Black-level cutoff (40)")
        self.lbl_thresh.pack(pady=(10,0))
        self.scale_thresh = tk.Scale(self.frame_controls, from_=0, to=255, orient="horizontal", command=self.on_thresh_scroll)
        self.scale_thresh.set(40)
        self.scale_thresh.pack(fill="x")

        # Generate Button
        self.btn_gen = tk.Button(root, text="GENERATE C CODE", bg="#dddddd", height=2, command=self.generate_code)
        self.btn_gen.pack(pady=20, fill="x", padx=20)

    def load_video(self):
        file_path = filedialog.askopenfilename(filetypes=[("Video files", "*.mp4 *.avi *.gif *.mov")])
        if not file_path:
            return
        
        self.video_path = file_path
        self.cap = cv2.VideoCapture(self.video_path)
        
        if not self.cap.isOpened():
            messagebox.showerror("Error", "Could not open video file.")
            return
            
        self.total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        
        # Reset UI
        self.scale_frame.config(to=self.total_frames - 1)
        self.scale_frame.set(0)
        self.current_frame_idx = 0
        self.update_preview()

    def on_frame_scroll(self, val):
        if self.cap:
            self.current_frame_idx = int(val)
            self.lbl_frame.config(text=f"Frame: {self.current_frame_idx}/{self.total_frames}")
            self.update_preview()

    def on_thresh_scroll(self, val):
        if self.cap:
            self.lbl_thresh.config(text=f"Black-level cutoff ({val})")
            self.update_preview()

    def get_processed_frame(self, frame_idx):
        """ Fetches frame, resizes, and quantizes to GREY_LEVELS brightness levels """
        self.cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
        ret, frame = self.cap.read()
        if not ret:
            return None, None

        # 1. Original (for display)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # 2. Resize to Target (19x20) using AREA interpolation (best for downscaling)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        resized = cv2.resize(gray, (TARGET_W, TARGET_H), interpolation=cv2.INTER_AREA)

        # 3. Quantize to 0..GREY_LEVELS-1, then scale back to 0..255 for an honest preview
        cutoff = int(self.scale_thresh.get())
        levels = quantize_to_levels(resized, cutoff)
        preview = (levels.astype(np.float32) * (255.0 / (GREY_LEVELS - 1))).astype(np.uint8)

        return frame_rgb, preview

    def update_preview(self):
        if not self.cap:
            return

        orig_rgb, binary_small = self.get_processed_frame(self.current_frame_idx)
        if orig_rgb is None:
            return

        # --- Display Original ---
        # Resize for GUI display (keep aspect ratio roughly)
        img_orig = Image.fromarray(orig_rgb)
        img_orig.thumbnail((300, 300)) 
        self.tk_orig = ImageTk.PhotoImage(img_orig)
        self.canvas_orig.create_image(150, 150, image=self.tk_orig)

        # --- Display Preview ---
        # We resize the tiny 19x20 image BACK UP to 300x300 using NEAREST NEIGHBOR.
        # This creates the "blocky" pixel effect so you see exactly what the LEDs show.
        img_binary = Image.fromarray(binary_small)
        img_preview = img_binary.resize((300, 300), Image.Resampling.NEAREST)
        self.tk_prev = ImageTk.PhotoImage(img_preview)
        self.canvas_prev.create_image(150, 150, image=self.tk_prev)

    def generate_code(self):
        if not self.cap:
            messagebox.showwarning("Warning", "Please load a video first.")
            return

        output_filename = filedialog.asksaveasfilename(defaultextension=".c", filetypes=[("C Source", "*.c"), ("Text File", "*.txt")])
        if not output_filename:
            return

        cutoff = int(self.scale_thresh.get())

        # Read frames SEQUENTIALLY (one cap.set seek up front, then plain reads). Per-frame
        # random-access seeking is very slow on most codecs and is what made the long export
        # look like a crash. We build the body first, counting frames actually decoded, so the
        # emitted array size always matches its contents even if the stream ends early.
        self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        frame_bodies = []
        i = 0
        try:
            while True:
                if MAX_FRAMES is not None and i >= MAX_FRAMES:
                    break  # flash budget cap: keep only the first MAX_FRAMES frames
                ret, frame = self.cap.read()
                if not ret:
                    break

                # Process -> per-pixel brightness level 0..GREY_LEVELS-1
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                resized = cv2.resize(gray, (TARGET_W, TARGET_H), interpolation=cv2.INTER_AREA)
                levels = quantize_to_levels(resized, cutoff)

                lines = [f"\t{{ /* Frame {i} */\n"]
                # Decompose each pixel's level into bit planes: plane p holds bit p of the level.
                for p in range(COLOR_DEPTH):
                    row_strs = []
                    for y in range(TARGET_H):
                        row_val = 0
                        for x in range(TARGET_W):
                            if (int(levels[y, x]) >> p) & 1:
                                row_val |= (1 << x)
                        row_strs.append(f"0x{row_val:08X}")
                    lines.append("\t\t{ " + ", ".join(row_strs) + " },\n")
                lines.append("\t},\n")
                frame_bodies.append("".join(lines))

                i += 1
                # Keep the GUI responsive and show progress instead of "Not Responding".
                if i % 10 == 0:
                    target = self.total_frames if MAX_FRAMES is None else min(self.total_frames, MAX_FRAMES)
                    self.btn_gen.config(text=f"Exporting... frame {i}/{target}")
                    self.root.update()
        except Exception as e:
            self.btn_gen.config(text="GENERATE C CODE")
            messagebox.showerror("Export failed", f"Stopped at frame {i}:\n{e}")
            return

        self.btn_gen.config(text="GENERATE C CODE")
        frame_count = len(frame_bodies)
        if frame_count == 0:
            messagebox.showerror("Export failed", "No frames could be decoded from the video.")
            return

        with open(output_filename, "w") as f:
            f.write(f"/* Generated from {self.video_path} */\n")
            f.write(f"/* 8-level greyscale, stored as {COLOR_DEPTH} bit planes (Binary Code Modulation). */\n")
            f.write(f"#include <stdint.h>\n\n")

            f.write(f"#define ANIMATION_FRAMES {frame_count}\n")
            f.write(f"#define COLOR_DEPTH {COLOR_DEPTH}\n")
            f.write(f"#define Y_RES {TARGET_H}\n\n")

            # START OF THE 3D ARRAY: [frame][plane][row]
            # We use 'const' to store this in Flash memory, not RAM
            f.write(f"const uint32_t animation_data[{frame_count}][{COLOR_DEPTH}][{TARGET_H}] = {{\n")
            f.write("".join(frame_bodies))
            f.write("};\n")

        messagebox.showinfo(
            "Success",
            f"Saved {frame_count} frames to {output_filename}\n\n"
            f"Set ANIMATION_FRAMES = {frame_count} in animations.h to match.")

if __name__ == "__main__":
    root = tk.Tk()
    app = LEDConverterApp(root)
    root.mainloop()