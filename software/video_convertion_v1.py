import tkinter as tk
from tkinter import filedialog, messagebox
from PIL import Image, ImageTk
import cv2
import numpy as np

# --- CONFIGURATION ---
# Target resolution for the STM32 LED Matrix
TARGET_W = 19
TARGET_H = 20

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

        # Threshold Slider
        self.lbl_thresh = tk.Label(self.frame_controls, text="B/W Threshold (128)")
        self.lbl_thresh.pack(pady=(10,0))
        self.scale_thresh = tk.Scale(self.frame_controls, from_=0, to=255, orient="horizontal", command=self.on_thresh_scroll)
        self.scale_thresh.set(128)
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
            self.lbl_thresh.config(text=f"B/W Threshold ({val})")
            self.update_preview()

    def get_processed_frame(self, frame_idx):
        """ Fetches frame, resizes, and applies threshold """
        self.cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
        ret, frame = self.cap.read()
        if not ret:
            return None, None

        # 1. Original (for display)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # 2. Resize to Target (19x20) using AREA interpolation (best for downscaling)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        resized = cv2.resize(gray, (TARGET_W, TARGET_H), interpolation=cv2.INTER_AREA)
        
        # 3. Threshold
        thresh_val = int(self.scale_thresh.get())
        _, binary = cv2.threshold(resized, thresh_val, 255, cv2.THRESH_BINARY)
        
        return frame_rgb, binary

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

        thresh_val = int(self.scale_thresh.get())
        
        with open(output_filename, "w") as f:
            f.write(f"/* Generated from {self.video_path} */\n")
            f.write(f"#include <stdint.h>\n\n")
            
            f.write(f"#define ANIMATION_FRAMES {self.total_frames}\n")
            f.write(f"#define Y_RES {TARGET_H}\n\n")

            # START OF THE 2D ARRAY
            # We use 'const' to store this in Flash memory, not RAM
            f.write(f"const uint32_t animation_data[{self.total_frames}][{TARGET_H}] = {{\n")
            
            # Loop through ALL frames
            for i in range(self.total_frames):
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, i)
                ret, frame = self.cap.read()
                if not ret: break
                
                # Process
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                resized = cv2.resize(gray, (TARGET_W, TARGET_H), interpolation=cv2.INTER_AREA)
                _, binary = cv2.threshold(resized, thresh_val, 255, cv2.THRESH_BINARY)
                
                f.write(f"\t{{ /* Frame {i} */\n")
                
                for y in range(TARGET_H):
                    row_val = 0
                    for x in range(TARGET_W):
                        pixel = binary[y, x]
                        if pixel > 0:
                            row_val |= (1 << x) 
                    
                    f.write(f"\t\t0x{row_val:08X}, ")
                
                f.write("\n\t},\n")
                
            f.write("};\n")
            
        messagebox.showinfo("Success", f"Saved C code to {output_filename}")

if __name__ == "__main__":
    root = tk.Tk()
    app = LEDConverterApp(root)
    root.mainloop()