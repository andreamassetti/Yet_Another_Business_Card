import tkinter as tk
from tkinter import font

# --- Constants ---
ROWS = 20
COLS = 19
# Set a default "off" color
COLOR_OFF = "white"
# Set a default "on" color
COLOR_ON = "black"

class LEDMatrixApp(tk.Tk):
    """
    A GUI application to create a 20x19 LED matrix pattern
    and convert it into an array of 20 uint32_t values.
    """
    
    def __init__(self):
        super().__init__()
        
        self.title(f"LED Matrix Generator ({COLS}x{ROWS})")
        self.resizable(False, False)
        
        # --- Data Structures ---
        
        # 2D list to store the state of each LED (0 = off, 1 = on)
        self.grid_state = [[0 for _ in range(COLS)] for _ in range(ROWS)]
        
        # 2D list to store references to the button widgets
        self.grid_buttons = [[None for _ in range(COLS)] for _ in range(ROWS)]
        
        # --- Drawing State ---
        self.is_drawing = False
        # The state we are drawing with (0 for 'off', 1 for 'on')
        self.draw_state = 0
        
        # --- UI Setup ---
        
        # Main frame to hold the grid and controls
        main_frame = tk.Frame(self, padx=10, pady=10)
        main_frame.pack()

        # Frame for the 20x19 grid of buttons
        # --- MODIFICATION: Store grid_frame as a class attribute ---
        self.grid_frame = tk.Frame(main_frame, relief="solid", borderwidth=1)
        self.grid_frame.pack(pady=(0, 10))

        # --- MODIFICATION: Bind drag event to the entire grid_frame ---
        self.grid_frame.bind("<B1-Motion>", self.on_grid_drag)

        # Create the grid of buttons
        for r in range(ROWS):
            for c in range(COLS):
                # Use a lambda function to pass the specific row and col
                # to the toggle_pixel method on click
                btn = tk.Button(
                    self.grid_frame, # Use self.grid_frame
                    bg=COLOR_OFF,
                    activebackground=COLOR_OFF,
                    width=2,  # Width in text units
                    height=1, # Height in text units
                    relief="solid",
                    borderwidth=1
                    # REMOVED: command=lambda r=r, c=c: self.toggle_pixel(r, c)
                )
                
                # --- New Event Bindings for Drawing ---
                # On click: start drawing and set the draw state
                btn.bind("<ButtonPress-1>", lambda event, r=r, c=c: self.on_mouse_down(r, c))
                
                # --- MODIFICATION: Add B1-Motion binding to buttons as well ---
                # This ensures that even if the button grabs the drag event,
                # it still gets processed.
                btn.bind("<B1-Motion>", self.on_grid_drag)
                
                # --- MODIFICATION: Removed <Enter> binding ---
                # btn.bind("<Enter>", lambda event, r=r, c=c: self.on_mouse_enter(r, c))
                
                btn.grid(row=r, column=c)
                self.grid_buttons[r][c] = btn

        # Bind the mouse release to the entire app window
        # This stops drawing even if the mouse is released outside the grid
        self.bind("<ButtonRelease-1>", self.on_mouse_up)

        # Frame for controls (buttons and output text)
        controls_frame = tk.Frame(main_frame)
        controls_frame.pack(fill="x")

        # Button to generate the array
        self.generate_btn = tk.Button(
            controls_frame, 
            text="Generate Array", 
            command=self.generate_array
        )
        self.generate_btn.pack(side="left", padx=(0, 5))
        
        # Button to clear the grid
        self.clear_btn = tk.Button(
            controls_frame, 
            text="Clear Grid", 
            command=self.clear_grid
        )
        self.clear_btn.pack(side="left")

        # Output text box
        output_frame = tk.Frame(main_frame)
        output_frame.pack(fill="x", pady=(10, 0))
        
        output_label = tk.Label(output_frame, text="Generated Output:")
        output_label.pack(anchor="w")

        # Setup scrollbar for the text widget
        text_scrollbar = tk.Scrollbar(output_frame, orient="vertical")
        
        # Define a monospaced font for the output
        mono_font = font.Font(family="Courier", size=10)
        
        self.output_text = tk.Text(
            output_frame,
            height=13,
            width=70,
            font=mono_font,
            yscrollcommand=text_scrollbar.set,
            wrap="word"
        )
        
        text_scrollbar.config(command=self.output_text.yview)
        
        # Pack the scrollbar and text widget
        text_scrollbar.pack(side="right", fill="y")
        self.output_text.pack(side="left", fill="x", expand=True)

    # --- MODIFICATION: New "Grid Drag" Handler ---
    
    def on_grid_drag(self, event):
        """
        Called when the mouse is dragged over the grid frame.
        Finds the button under the cursor and paints it.
        """
        if not self.is_drawing:
            return
            
        try:
            # --- NEW METHOD to find widget under cursor ---
            # --- MODIFICATION: Use event.x_root and event.y_root ---
            # These are absolute screen coordinates, so we don't
            # need to calculate them manually relative to the grid_frame.
            x_root = event.x_root
            y_root = event.y_root
            
            # Find the widget at these coordinates
            # We ask the *grid_frame* which of its children is at this
            # absolute screen location.
            widget = self.grid_frame.winfo_containing(x_root, y_root)
            
            # Check if the widget is one of our grid buttons
            # We can check its 'master' (parent)
            if widget and widget.master == self.grid_frame and isinstance(widget, tk.Button):
                # Get the grid info for this button
                info = widget.grid_info()
                r = info.get('row')
                c = info.get('column')
                
                # Check if we got valid row/col
                if r is not None and c is not None:
                    # Check if the coordinates are valid (redundant but safe)
                    if 0 <= r < ROWS and 0 <= c < COLS:
                        self.set_pixel_state(r, c, self.draw_state)
            
            # --- OLD METHOD (commented out) ---
            # Get the (col, row) under the mouse coordinates (event.x, event.y)
            # (c, r) = self.grid_frame.grid_location(event.x, event.y)
            # if 0 <= r < ROWS and 0 <= c < COLS:
            #     self.set_pixel_state(r, c, self.draw_state)
            
        except Exception: # Catch any TclError or other exceptions
            # Mouse is outside the grid cells or an error occurred
            pass

    # --- Drawing Event Handlers ---


    def on_mouse_down(self, r, c):
        """
        Called on the initial click. Toggles the first pixel
        and sets the drawing state for subsequent dragging.
        """
        self.is_drawing = True
        # Toggle the state of the clicked pixel
        new_state = 1 - self.grid_state[r][c]
        # This is the state we will "paint" with
        self.draw_state = new_state
        # Set the clicked pixel
        self.set_pixel_state(r, c, self.draw_state)

    # --- MODIFICATION: Removed on_mouse_enter ---
    # def on_mouse_enter(self, r, c):
    #     """
    #     Called when the mouse enters a button's bounds.
    #     If we are in drawing mode, set the pixel to the current draw state.
    #     """
    #     if self.is_drawing:
    #         self.set_pixel_state(r, c, self.draw_state)

    def on_mouse_up(self, event):
        """
        Called when the mouse button is released anywhere in the app.
        Stops the drawing mode.
        """
        self.is_drawing = False

    def set_pixel_state(self, r, c, state):
        """
        Sets a single pixel to a specific state (0 or 1)
        and updates the button's color.
        """
        # Only update if the state is actually changing
        if self.grid_state[r][c] == state:
            return
            
        self.grid_state[r][c] = state
        
        if state == 1:
            color = COLOR_ON
        else:
            color = COLOR_OFF
            
        self.grid_buttons[r][c].config(bg=color, activebackground=color)

    # --- End of New Handlers ---

    # REMOVED the old toggle_pixel method
    # def toggle_pixel(self, r, c): ...

    def clear_grid(self):
        """
        Resets the entire grid to the "off" state.
        """
        for r in range(ROWS):
            for c in range(COLS):
                if self.grid_state[r][c] == 1:
                    # Use set_pixel_state to keep logic consistent
                    self.set_pixel_state(r, c, 0)
        
        # Clear the output text box
        self.output_text.delete("1.0", tk.END)

    def generate_array(self):
        """
        Generates the array of 20 uint32_t values from the grid state
        and displays it in the output text box.
        """
        output_array = []
        
        # Start building the C-style array string
        output_str = f"/* C-style array: {COLS}x{ROWS} Matrix */\n"
        output_str += f"uint32_t led_matrix[{ROWS}] = {{\n"
        
        for r in range(ROWS):
            row_value = 0
            for c in range(COLS):
                state = self.grid_state[r][c]
                
                # --- Bitwise Conversion ---
                # We map column 'c' to bit position 'c'.
                # Column 0 is the LSB (bit 0).
                # Column 18 is bit 18.
                # The 19 bits (0-18) are the 19 LSBs of the 32-bit integer.
                # The top 13 bits (19-31) remain 0, as requested.
                if state == 1:
                    row_value |= (1 << c)
            
            output_array.append(row_value)
            
            # Format as a hex value (0x..._..._...)
            # 08X pads with leading zeros to 8 hex digits
            hex_val = f"0x{row_value:08X}"
            output_str += f"    {hex_val}, /* Row {r:2} */\n"
            
        output_str += "};\n\n"
        
        # Add the Python list representation
        output_str += "/* Python list */\n"
        output_str += "[\n"
        for i, val in enumerate(output_array):
            output_str += f"    {val}, # Row {i}\n"
        output_str += "]\n"
        
        # Update the output text box
        self.output_text.delete("1.0", tk.END)
        self.output_text.insert(tk.END, output_str)


# --- Main execution ---
if __name__ == "__main__":
    app = LEDMatrixApp()
    app.mainloop()