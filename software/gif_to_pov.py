import tkinter as tk
from tkinter import filedialog, messagebox
from PIL import Image, ImageSequence, ImageTk
import math
import io
import time

# --- Constants matching the STM32 C Code ---
MATRIX_WIDTH = 20
MATRIX_HEIGHT = 19
TOTAL_LEDS = MATRIX_WIDTH * MATRIX_HEIGHT # 380
FRAME_DURATION_MS = 50 # 20 frames per second
FRAME_DURATION_S = FRAME_DURATION_MS / 1000.0
BYTES_PER_FRAME = math.ceil(TOTAL_LEDS / 8) # 48 bytes

class GifProcessor:
    """
    Handles loading, processing, and converting a GIF into the STM32's 
    `video_frames` array format.
    """
    def __init__(self):
        self.processed_frames = [] # List of 48-byte lists (one for each frame)
        self.num_frames = 0
        self.gif_path = None

    def process_gif(self, file_path, threshold=128):
        """
        Loads a GIF, resizes/dithers frames to 20x19, and converts them
        into the STM32 bitmap array format.
        """
        self.gif_path = file_path
        self.processed_frames = []
        
        try:
            img = Image.open(file_path)
        except Exception as e:
            messagebox.showerror("Error", f"Could not load GIF: {e}")
            return False

        # --- 1. Frame Extraction and Dithering ---
        original_frames = []
        total_gif_duration = 0

        for frame in ImageSequence.Iterator(img):
            # Convert to grayscale, resize/crop, and binarize
            
            # Step 1: Convert to grayscale and resize/crop
            # We use LANCZOS for high-quality downscaling
            resized_frame = frame.convert('L').resize((MATRIX_WIDTH, MATRIX_HEIGHT), Image.Resampling.LANCZOS)
            
            # Step 2: Binarization (Thresholding)
            # Dithering is often better for low-res, but simple thresholding is often faster/cleaner
            binary_frame = resized_frame.point(lambda p: 255 if p > threshold else 0, mode='1')

            # Extract data: 0 (black/off) or 1 (white/on)
            frame_data = list(binary_frame.getdata())
            
            # Get GIF duration, default to 100ms if not present
            duration_ms = frame.info.get('duration', 100)
            
            original_frames.append({
                'data': frame_data,
                'duration': duration_ms
            })
            total_gif_duration += duration_ms

        if not original_frames:
            messagebox.showerror("Error", "No frames found in GIF.")
            return False

        # --- 2. Frame Rate Matching (Temporal Scaling) ---
        
        # Calculate how many target frames (at 50ms) are needed for the GIF's total duration
        total_target_frames = round(total_gif_duration / FRAME_DURATION_MS)
        if total_target_frames == 0:
            total_target_frames = 1 # At least one frame

        frame_index = 0
        current_time_ms = 0
        
        # A running index for frames we have added to the final sequence
        target_frame_count = 0
        
        # Ensure we don't exceed the total number of frames calculated,
        # which prevents endless loops from float precision errors.
        while target_frame_count < total_target_frames:
            if frame_index >= len(original_frames):
                # Loop the original GIF if needed to fill the time.
                frame_index = 0
                
            if frame_index >= len(original_frames):
                break # Should not happen if original_frames is not empty


            
            # Instead of complex time interpolation, let's use a simpler, more robust method:
            # Distribute the GIF's frames over the target time slots.
            
            # Calculate the ratio of the GIF's frame rate to the target frame rate (20 FPS)
            # This is complex due to variable GIF frame rates. Let's use the simple slot method:

            
            # Simple Frame Duplication Strategy:
            # 1. Total time: total_gif_duration
            # 2. Total target frames: total_target_frames
            # 3. Time per target frame: 50ms
            
            # We want to map original_frames[i] to a sequence of target frames.
            
            # Simple resampling logic (resets the frames list)
            target_frames = []
            
            # We use a simple counter to approximate the frame rate matching
            cumulative_duration = 0
            original_frame_idx = 0
            
            # Iterate through the desired number of 50ms slots
            for i in range(total_target_frames):
                # The duration elapsed in the target sequence
                target_elapsed_ms = i * FRAME_DURATION_MS
                
                # Check if we need to advance the original GIF frame
                # This ensures that the current GIF frame is repeated until its duration is covered
                while cumulative_duration <= target_elapsed_ms and original_frame_idx < len(original_frames) - 1:
                    cumulative_duration += original_frames[original_frame_idx]['duration']
                    if cumulative_duration <= target_elapsed_ms:
                        original_frame_idx += 1
                        
                # Add the currently active original frame data to the target frames list
                target_frames.append(original_frames[original_frame_idx]['data'])
                
            
            if not target_frames:
                messagebox.showerror("Error", "Frame resampling failed.")
                return False
                
            # --- 3. Convert Pixels to Bitmap Array ---
            for frame_data in target_frames:
                bitmap_bytes = bytearray(BYTES_PER_FRAME)
                
                for i in range(TOTAL_LEDS):
                    # Pixel index i corresponds to LED index i
                    pixel_value = frame_data[i] 
                    
                    if pixel_value > 0: # Pixel is "on" (white)
                        byte_index = i // 8
                        bit_index = i % 8
                        
                        # Set the corresponding bit
                        bitmap_bytes[byte_index] |= (1 << bit_index)
                
                self.processed_frames.append(list(bitmap_bytes))

            self.num_frames = len(self.processed_frames)
            # Since the resampling logic above handles all frames, we can break out.
            # The structure for `target_frames` replaces the simpler loop.
            break 
            
        if not self.processed_frames:
            messagebox.showerror("Error", "Failed to process any frames.")
            return False
            
        # Update the C code's expected constants based on the actual frame count
        # (This is informative for the user)
        self.video_num_frames = self.num_frames
        
        return True


    def get_frame(self, index):
        """Returns the 380-bit frame data (as a 20x19 list of 0s/1s) for simulation."""
        if not self.processed_frames:
            return None
            
        frame_bytes = self.processed_frames[index % self.num_frames]
        frame_data = []
        for byte_idx, byte in enumerate(frame_bytes):
            for bit_idx in range(8):
                if len(frame_data) < TOTAL_LEDS:
                    # Check if the bit is set
                    is_on = (byte >> bit_idx) & 1
                    frame_data.append(is_on)
        return frame_data


    def generate_c_array(self):
        """Generates the C code representation of the video_frames array."""
        if not self.processed_frames:
            return "// No video data processed.\n"
        
        # Start of array definition
        c_code = (
            f"// C Constants for {self.gif_path.split('/')[-1] if self.gif_path else 'video'}:\n"
            f"#undef VIDEO_NUM_FRAMES\n"
            f"#define VIDEO_NUM_FRAMES {self.video_num_frames}\n"
            f"#undef FRAME_DURATION_MS\n"
            f"#define FRAME_DURATION_MS {FRAME_DURATION_MS}\n\n"
            
            f"// Video data for {self.video_num_frames} frames (20x19 matrix, {BYTES_PER_FRAME} bytes per frame)\n"
            f"const uint8_t video_frames[VIDEO_NUM_FRAMES][{(TOTAL_LEDS + 7) // 8}] = {{\n"
        )
        
        # Add frame data
        for frame_idx, frame_bytes in enumerate(self.processed_frames):
            hex_values = [f"0x{b:02X}" for b in frame_bytes]
            c_code += f"    {{ {', '.join(hex_values)} }}, // Frame {frame_idx}\n"
            
        # End of array definition
        c_code += "};\n"
        
        return c_code

class SimulatorApp(tk.Tk):
    """
    Tkinter application for loading the GIF, simulating the POV display,
    and outputting the C code.
    """
    def __init__(self):
        super().__init__()
        self.title("Charlieplexing POV Simulator (20x19)")
        self.processor = GifProcessor()
        self.current_frame_index = 0
        self.is_running = False
        self.gif_path = None
        self.threshold_var = tk.IntVar(value=128)
        
        self.setup_ui()
        
    def setup_ui(self):
        # Configuration
        self.cell_size = 18
        self.padding = 15
        canvas_width = MATRIX_WIDTH * self.cell_size + 2 * self.padding
        canvas_height = MATRIX_HEIGHT * self.cell_size + 2 * self.padding
        
        # --- Frame 1: Simulation Area ---
        sim_frame = tk.Frame(self, padx=10, pady=10)
        sim_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        tk.Label(sim_frame, text="20x19 LED Matrix Simulation", font=('Helvetica', 14, 'bold')).pack(pady=(0, 10))

        self.canvas = tk.Canvas(sim_frame, 
                                width=canvas_width, 
                                height=canvas_height, 
                                bg='#1a1a1a', 
                                highlightthickness=0)
        self.canvas.pack(pady=10)
        
        # --- Frame 2: Controls and Output ---
        control_frame = tk.Frame(self, padx=10, pady=10, bg='#f0f0f0')
        control_frame.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Load Button
        tk.Button(control_frame, text="Load GIF Video", command=self.load_gif, 
                  bg='#4CAF50', fg='white', relief=tk.RAISED, bd=3, padx=10, pady=5).pack(pady=(10, 5), fill=tk.X)

        # Threshold Slider
        tk.Label(control_frame, text="Binarization Threshold", font=('Helvetica', 10)).pack(pady=(10,0))
        self.threshold_slider = tk.Scale(control_frame, from_=0, to=255, orient=tk.HORIZONTAL, variable=self.threshold_var, command=self.on_threshold_change)
        self.threshold_slider.pack(pady=(0,5), fill=tk.X)
                  
        # Status Label
        self.status_var = tk.StringVar(value="Status: Ready.")
        tk.Label(control_frame, textvariable=self.status_var, wraplength=200).pack(pady=5)

        # Simulation Controls
        tk.Label(control_frame, text="Simulation Control", font=('Helvetica', 12, 'underline')).pack(pady=(10, 5))
        
        self.sim_btn = tk.Button(control_frame, text="Start Simulation", command=self.toggle_simulation, state=tk.DISABLED,
                                 bg='#2196F3', fg='white', relief=tk.RAISED, bd=3, padx=10, pady=5)
        self.sim_btn.pack(pady=5, fill=tk.X)

        # C Code Output
        tk.Label(control_frame, text="Generated C Array (video_frames)", font=('Helvetica', 12, 'underline')).pack(pady=(10, 5))
        
        self.c_output = tk.Text(control_frame, height=15, width=40, wrap=tk.WORD, state=tk.DISABLED, bg='#e0e0e0')
        self.c_output.pack(pady=5)
        
        tk.Button(control_frame, text="Copy C Code", command=self.copy_c_code,
                  bg='#FF9800', fg='white', relief=tk.RAISED, bd=3, padx=10, pady=5).pack(pady=10, fill=tk.X)

        self.draw_empty_matrix()
        
    def draw_empty_matrix(self):
        """Draws the initial dark grid of LEDs."""
        self.canvas.delete("all")
        for y in range(MATRIX_HEIGHT):
            for x in range(MATRIX_WIDTH):
                self.draw_led(x, y, 0)
                
    def draw_led(self, x, y, state):
        """Draws a single LED on the canvas."""
        x0 = self.padding + x * self.cell_size
        y0 = self.padding + y * self.cell_size
        r = self.cell_size / 3
        
        fill_color = "#333333" if state == 0 else "#FFEB3B" # Dark off / Bright yellow on
        outline_color = "#555555" if state == 0 else "#FFC107"

        self.canvas.create_oval(x0 - r, y0 - r, x0 + r, y0 + r, 
                                fill=fill_color, outline=outline_color, tags=f"led_{x}_{y}")
                                
    def load_gif(self):
        """Opens file dialog, processes the GIF, and updates the GUI."""
        file_path = filedialog.askopenfilename(
            defaultextension=".gif",
            filetypes=[("GIF files", "*.gif")]
        )
        
        if not file_path:
            return

        self.gif_path = file_path
        self.process_and_update_ui()

    def on_threshold_change(self, value):
        if self.gif_path:
            self.process_and_update_ui()

    def process_and_update_ui(self):
        """Helper function to run processing and update the GUI."""
        if not self.gif_path:
            return

        self.status_var.set("Status: Processing GIF...")
        self.update()

        threshold = self.threshold_var.get()
        if self.processor.process_gif(self.gif_path, threshold):
            self.current_frame_index = 0
            self.toggle_simulation(run=False) # Stop simulation if running
            
            # Update C Code Output
            c_code = self.processor.generate_c_array()
            self.c_output.config(state=tk.NORMAL)
            self.c_output.delete(1.0, tk.END)
            self.c_output.insert(tk.END, c_code)
            self.c_output.config(state=tk.DISABLED)

            # Update status and enable simulation
            self.status_var.set(f"Status: Loaded '{self.gif_path.split('/')[-1]}'.\n{self.processor.num_frames} frames generated at {1000/FRAME_DURATION_MS} FPS.")
            self.sim_btn.config(state=tk.NORMAL, text="Start Simulation")
            self.draw_frame(self.current_frame_index)
        else:
            self.status_var.set("Status: Failed to load/process GIF.")
            self.sim_btn.config(state=tk.DISABLED, text="Start Simulation")
            self.draw_empty_matrix()

    def draw_frame(self, index):
        """Updates the canvas with the given frame data."""
        frame_data = self.processor.get_frame(index)
        if not frame_data:
            return

        for y in range(MATRIX_HEIGHT):
            for x in range(MATRIX_WIDTH):
                led_index = y * MATRIX_WIDTH + x
                state = frame_data[led_index]
                
                # Redraw the circle (delete old, draw new for color change)
                tag = f"led_{x}_{y}"
                self.canvas.delete(tag)
                self.draw_led(x, y, state)

    def toggle_simulation(self, run=None):
        """Starts or stops the simulation loop."""
        if run is not None:
            self.is_running = run
        else:
            self.is_running = not self.is_running
            
        if self.is_running:
            self.sim_btn.config(text="Stop Simulation", bg='#D32F2F')
            self.run_simulation_loop()
        else:
            self.sim_btn.config(text="Start Simulation", bg='#2196F3')

    def run_simulation_loop(self):
        """The main simulation loop, called on a timer."""
        if not self.is_running or not self.processor.processed_frames:
            return

        self.draw_frame(self.current_frame_index)
        
        self.current_frame_index = (self.current_frame_index + 1) % self.processor.num_frames
        
        # Schedule the next frame update after FRAME_DURATION_MS
        self.after(FRAME_DURATION_MS, self.run_simulation_loop)
        
    def copy_c_code(self):
        """Copies the generated C code to the clipboard."""
        # Enable, select all, copy, disable
        self.c_output.config(state=tk.NORMAL)
        c_code = self.c_output.get(1.0, tk.END)
        self.clipboard_clear()
        self.clipboard_append(c_code)
        self.c_output.config(state=tk.DISABLED)
        messagebox.showinfo("Copied", "C array code copied to clipboard!")


if __name__ == "__main__":
    app = SimulatorApp()
    app.mainloop()
