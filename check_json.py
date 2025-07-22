import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from PIL import Image, ImageTk
import json, os, re, functools, difflib

class LabelEditorApp:
    def __init__(self, root, image_folder, json_path):
        self.root = root
        self.root.title("Label Editor")
        self.image_folder = image_folder
        self.json_path = json_path
        self.data = self.load_json(json_path)
        self.index = 0  # current image index in self.data

        # AUTOMATIC SYNC: no button needed
        self.sync_json_filenames(cutoff=0.6, require_equal_counts=True)

        # Predefined labels and fixed colors.
        self.label_options = ["human1", "human2", "human3", "human4", "human5"]
        self.class_colors = {
            "human1": "#FF0000",  # red
            "human2": "#00AA00",  # green
            "human3": "#0000FF",  # blue
            "human4": "#FFA500",  # orange
            "human5": "#800080"   # purple
        }
        
        self.box_widgets = []  # for comboboxes and remove buttons
        self.bbox_items = {}   # mapping: bbox index -> canvas rectangle id
        # We now keep two types of handles:
        # top-left handles and bottom-right handles.
        self.tl_handle_items = {}  # top-left handle ids (dictionary keyed by bbox index)
        self.br_handle_items = {}  # bottom-right handle ids (dictionary keyed by bbox index)
        
        # drag_data stores info when dragging a bbox or a handle.
        # mode can be "move", "resize_topleft", or "resize_bottomright"
        self.drag_data = {"item": None, "x": 0, "y": 0, "index": None, "mode": None, "fixed_x": 0, "fixed_y": 0}

        self.canvas_width = 800
        self.canvas_height = 600
        self.canvas = tk.Canvas(root, width=self.canvas_width, height=self.canvas_height, bg="gray")
        self.canvas.pack()

        # Controls frame
        self.controls_frame = tk.Frame(root)
        self.controls_frame.pack()

        # Navigation/control buttons
        self.prev_btn = tk.Button(self.controls_frame, text="Previous", command=self.prev_image)
        self.prev_btn.grid(row=0, column=0, padx=5)
        self.next_btn = tk.Button(self.controls_frame, text="Next", command=self.next_image)
        self.next_btn.grid(row=0, column=1, padx=5)
        self.add_box_btn = tk.Button(self.controls_frame, text="Add Box", command=self.add_box)
        self.add_box_btn.grid(row=0, column=2, padx=5)
        self.save_btn = tk.Button(self.controls_frame, text="Save As...", command=self.save_json_as)
        self.save_btn.grid(row=0, column=3, padx=5)

        # Search controls
        self.search_label = tk.Label(self.controls_frame, text="Search:")
        self.search_label.grid(row=1, column=0, padx=5)
        self.search_entry = tk.Entry(self.controls_frame, width=30)
        self.search_entry.grid(row=1, column=1, columnspan=2, padx=5)
        self.search_btn = tk.Button(self.controls_frame, text="Go", command=self.search_image)
        self.search_btn.grid(row=1, column=3, padx=5)

        # Display current image name
        self.image_name_label = tk.Label(self.controls_frame, text="", font=("Arial", 12, "bold"))
        self.image_name_label.grid(row=2, column=0, columnspan=4, pady=5)

        # Bind mouse events to the canvas for dragging/moving/resizing.
        self.canvas.bind("<ButtonPress-1>", self.on_canvas_press)
        self.canvas.bind("<B1-Motion>", self.on_canvas_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_canvas_release)

        self.display_image()

    def sync_json_filenames(self, cutoff: float = 0.6, require_equal_counts: bool = True):
        """
        1) If require_equal_counts is True, abort (with a warning) when
           len(self.data) != number of image files in self.image_folder.
        2) Fuzzy‑match each record['File'] basename against the folder.
        3) If all match above cutoff, rewrite record['File'], sort the list
           by the timestamped basename, overwrite self.json_path in place,
           refresh the display, and show a success message.
        """
        # 1) gather files and counts
        data     = self.data
        img_exts = {'.png','.jpg','.jpeg','.tif','.bmp'}  # adjust as needed
        imgs     = [f for f in os.listdir(self.image_folder)
                    if os.path.splitext(f)[1].lower() in img_exts]

        if require_equal_counts and len(data) != len(imgs):
            messagebox.showwarning(
                "Count mismatch",
                f"JSON has {len(data)} entries but found {len(imgs)} image files."
            )
            return False

        basenames = [os.path.splitext(f)[0] for f in imgs]
        matched   = []

        # 2) fuzzy‑match
        for rec in data:
            orig = rec.get("File", "")
            base = os.path.splitext(orig)[0]
            m = difflib.get_close_matches(base, basenames, n=1, cutoff=cutoff)
            if not m:
                if require_equal_counts:
                    messagebox.showwarning(
                        "No fuzzy match",
                        f"Could not match “{orig}” to any file in\n{self.image_folder}"
                    )
                    return False
                else:
                    continue
            new_fn = m[0] + os.path.splitext(orig)[1]
            rec["File"] = new_fn
            matched.append(new_fn)

        if not matched:
            messagebox.showinfo("Nothing changed", "No filenames were updated.")
            return False

        # 3) sort by timestamp‐style basename
        data.sort(key=lambda r: os.path.splitext(r["File"])[0])

        # 4) overwrite JSON on disk
        with open(self.json_path, 'w') as fp:
            json.dump(data, fp, indent=4)

        messagebox.showinfo(
            "Sync complete",
            f"Rewrote {len(matched)} filenames and saved:\n{self.json_path}"
        )

        return True

    def load_json(self, path):
        """Load JSON and normalize each record’s Labels."""
        with open(path, 'r') as f:
            raw_data = json.load(f)
        normalized_data = []
        for item in raw_data:
            labels_raw = item.get("Labels", [])
            if isinstance(labels_raw, dict):
                # Use only keys that are digits.
                digit_keys = [k for k in labels_raw.keys() if str(k).isdigit()]
                labels = [labels_raw[k] for k in sorted(digit_keys, key=lambda x: int(x))]
            else:
                labels = labels_raw
            norm_labels = []
            for label in labels:
                bbox = label.get("BoundingBoxes", [0, 0, 0, 0])
                norm_labels.append({
                    "Class": label.get("Class", "unknown"),
                    "BoundingBoxes": bbox
                })
            item["Labels"] = norm_labels
            normalized_data.append(item)
        return normalized_data

    def get_color(self, label):
        return self.class_colors.get(label, "#888888")

    def find_closest_image(self, json_filename):
        """
        If the filename from JSON (e.g. "1730367118.112957.png")
        is not found, search for a close match (e.g. "1730367118_112956430.png").
        """
        name, ext = os.path.splitext(json_filename.strip())
        if '.' not in name:
            return None
        base, frac = name.split('.', 1)
        try:
            target_val = int(frac.ljust(9, '0')[:9])
        except ValueError:
            return None
        best_match = None
        best_diff = float('inf')
        for file in os.listdir(self.image_folder):
            if not file.endswith(ext):
                continue
            m = re.match(rf"{re.escape(base)}_(\d+){re.escape(ext)}", file)
            if m:
                try:
                    candidate_val = int(m.group(1))
                    diff = abs(candidate_val - target_val)
                    if diff < best_diff:
                        best_diff = diff
                        best_match = file
                except ValueError:
                    continue
        return os.path.join(self.image_folder, best_match) if best_match else None

    def display_image(self):
        """Display the current image, draw bounding boxes with two resize handles, and update controls and image name."""
        self.canvas.delete("all")
        for widget in self.box_widgets:
            widget.destroy()
        self.box_widgets = []
        self.bbox_items.clear()
        self.tl_handle_items.clear()
        self.br_handle_items.clear()

        if not (0 <= self.index < len(self.data)):
            return
        entry = self.data[self.index]
        filename = entry.get("File", "")
        img_path = os.path.join(self.image_folder, filename)
        if not os.path.exists(img_path):
            matched = self.find_closest_image(filename)
            if matched:
                img_path = matched
                entry["File"] = os.path.basename(matched)
        if not os.path.exists(img_path):
            self.canvas.create_text(self.canvas_width//2, self.canvas_height//2,
                                    text="Image not found", font=("Arial", 16))
            return

        # Update current image name label.
        self.image_name_label.config(text=f"Current Image: {entry.get('File', 'Unknown')}")

        img = Image.open(img_path)
        self.original_size = img.size   # (w_orig, h_orig)
        w_orig, h_orig = self.original_size
        img.thumbnail((self.canvas_width, self.canvas_height))
        self.displayed_size = img.size  # (w_disp, h_disp)
        self.tk_img = ImageTk.PhotoImage(img)
        center_x = self.canvas_width // 2
        center_y = self.canvas_height // 2
        self.canvas.create_image(center_x, center_y, image=self.tk_img)
        self.offset_x = center_x - self.displayed_size[0] // 2
        self.offset_y = center_y - self.displayed_size[1] // 2
        self.scale_x = self.displayed_size[0] / w_orig
        self.scale_y = self.displayed_size[1] / h_orig

        # Draw each bounding box and two handles.
        for i, label in enumerate(entry["Labels"]):
            # Coordinates are assumed in Python-style (top-left origin).
            x, y, w, h = label["BoundingBoxes"]
            sx = x * self.scale_x
            sy = y * self.scale_y
            sw = w * self.scale_x
            sh = h * self.scale_y
            x1 = self.offset_x + sx
            y1 = self.offset_y + sy
            x2 = x1 + sw
            y2 = y1 + sh
            color = self.get_color(label["Class"])
            rect_id = self.canvas.create_rectangle(x1, y1, x2, y2, outline=color, width=2,
                                                     tags=(f"bbox_{i}", "bbox"))
            self.bbox_items[i] = rect_id
            # Create two resize handles:
            handle_size = 8
            # Bottom-right handle:
            br_handle = self.canvas.create_rectangle(
                x2 - handle_size/2, y2 - handle_size/2,
                x2 + handle_size/2, y2 + handle_size/2,
                fill=color, outline=color,
                tags=(f"br_handle_{i}", "handle")
            )
            self.br_handle_items[i] = br_handle
            # Top-left handle:
            tl_handle = self.canvas.create_rectangle(
                x1 - handle_size/2, y1 - handle_size/2,
                x1 + handle_size/2, y1 + handle_size/2,
                fill=color, outline=color,
                tags=(f"tl_handle_{i}", "handle")
            )
            self.tl_handle_items[i] = tl_handle
            # Draw label text.
            self.canvas.create_text(x1 + 4, y1 + 12, text=label["Class"],
                                    fill=color, anchor="nw", font=("Arial", 10))
            # Create control widgets.
            var = tk.StringVar(value=label["Class"])
            combo = ttk.Combobox(self.controls_frame, textvariable=var,
                                 values=self.label_options, width=10)
            combo.grid(row=i+3, column=0, padx=5)
            combo.bind("<<ComboboxSelected>>", lambda e, idx=i, var=var: self.update_label(idx, var.get()))
            btn = tk.Button(self.controls_frame, text="Remove", command=functools.partial(self.remove_box, i))
            btn.grid(row=i+3, column=1, padx=5)
            self.box_widgets.extend([combo, btn])

    def on_canvas_press(self, event):
        """Detect if a bbox or one of its handles is clicked and set drag mode accordingly."""
        item = self.canvas.find_withtag("current")
        if not item:
            return
        tags = self.canvas.gettags(item[0])
        # Check if a top-left handle was clicked.
        if any(t.startswith("tl_handle_") for t in tags):
            mode = "resize_topleft"
            for t in tags:
                if t.startswith("tl_handle_"):
                    try:
                        idx = int(t.split("_")[2])
                        self.drag_data["index"] = idx
                        break
                    except Exception:
                        continue
            # For resizing from the top-left, fix the bottom-right corner.
            if self.drag_data["index"] is not None and self.drag_data["index"] in self.bbox_items:
                coords = self.canvas.coords(self.bbox_items[self.drag_data["index"]])
                if len(coords) >= 4:
                    self.drag_data["fixed_x"] = coords[2]  # bottom-right x fixed
                    self.drag_data["fixed_y"] = coords[3]  # bottom-right y fixed
            self.drag_data["item"] = item[0]  # dragging the top-left handle
            self.drag_data["mode"] = mode
        # Check if a bottom-right handle was clicked.
        elif any(t.startswith("br_handle_") for t in tags):
            mode = "resize_bottomright"
            for t in tags:
                if t.startswith("br_handle_"):
                    try:
                        idx = int(t.split("_")[2])
                        self.drag_data["index"] = idx
                        break
                    except Exception:
                        continue
            # For resizing from bottom-right, fix top-left corner.
            if self.drag_data["index"] is not None and self.drag_data["index"] in self.bbox_items:
                coords = self.canvas.coords(self.bbox_items[self.drag_data["index"]])
                if len(coords) >= 4:
                    self.drag_data["fixed_x"] = coords[0]  # top-left x fixed
                    self.drag_data["fixed_y"] = coords[1]  # top-left y fixed
            self.drag_data["item"] = item[0]  # dragging the bottom-right handle
            self.drag_data["mode"] = mode
        # Else, if a bbox (but not a handle) was clicked, go to move mode.
        elif any(t.startswith("bbox_") for t in tags):
            mode = "move"
            for t in tags:
                if t.startswith("bbox_"):
                    try:
                        idx = int(t.split("_")[1])
                        self.drag_data["index"] = idx
                        break
                    except Exception:
                        continue
            if self.drag_data["index"] is not None and self.drag_data["index"] in self.bbox_items:
                self.drag_data["item"] = self.bbox_items[self.drag_data["index"]]
            self.drag_data["mode"] = mode
        else:
            return
        self.drag_data["x"] = event.x
        self.drag_data["y"] = event.y

    def on_canvas_drag(self, event):
        if self.drag_data["item"] is None:
            return
        dx = event.x - self.drag_data["x"]
        dy = event.y - self.drag_data["y"]
        mode = self.drag_data.get("mode")
        idx = self.drag_data.get("index")
        if mode == "move":
            if idx in self.bbox_items:
                self.canvas.move(self.bbox_items[idx], dx, dy)
            if idx in self.tl_handle_items:
                self.canvas.move(self.tl_handle_items[idx], dx, dy)
            if idx in self.br_handle_items:
                self.canvas.move(self.br_handle_items[idx], dx, dy)
        elif mode == "resize_topleft":
            # Update the top-left corner; bottom-right (fixed) remains unchanged.
            fixed_x = self.drag_data["fixed_x"]
            fixed_y = self.drag_data["fixed_y"]
            new_x1 = event.x
            new_y1 = event.y
            if idx is not None and idx in self.bbox_items:
                self.canvas.coords(self.bbox_items[idx], new_x1, new_y1, fixed_x, fixed_y)
                # Update top-left handle position.
                handle_size = 8
                self.canvas.coords(self.tl_handle_items[idx],
                                   new_x1 - handle_size/2, new_y1 - handle_size/2,
                                   new_x1 + handle_size/2, new_y1 + handle_size/2)
                # Also update the bottom-right handle to keep it at fixed position.
            # (No movement for bottom-right handle in this mode.)
        elif mode == "resize_bottomright":
            # Update the bottom-right corner; top-left (fixed) remains unchanged.
            fixed_x = self.drag_data["fixed_x"]
            fixed_y = self.drag_data["fixed_y"]
            new_x2 = event.x
            new_y2 = event.y
            if idx is not None and idx in self.bbox_items:
                self.canvas.coords(self.bbox_items[idx], fixed_x, fixed_y, new_x2, new_y2)
                handle_size = 8
                self.canvas.coords(self.br_handle_items[idx],
                                   new_x2 - handle_size/2, new_y2 - handle_size/2,
                                   new_x2 + handle_size/2, new_y2 + handle_size/2)
        self.drag_data["x"] = event.x
        self.drag_data["y"] = event.y

    def on_canvas_release(self, event):
        if self.drag_data["item"] is None:
            return
        idx = self.drag_data.get("index")
        if idx is not None and idx in self.bbox_items:
            coords = self.canvas.coords(self.bbox_items[idx])
            if len(coords) >= 4:
                x1, y1, x2, y2 = coords
                new_x = (x1 - self.offset_x) / self.scale_x
                new_y = (y1 - self.offset_y) / self.scale_y
                new_w = (x2 - x1) / self.scale_x
                new_h = (y2 - y1) / self.scale_y
                self.data[self.index]["Labels"][idx]["BoundingBoxes"] = [new_x, new_y, new_w, new_h]
        self.drag_data = {"item": None, "x": 0, "y": 0, "index": None, "mode": None, "fixed_x": 0, "fixed_y": 0}
        self.display_image()

    def update_label(self, idx, new_class):
        if idx < len(self.data[self.index]["Labels"]):
            self.data[self.index]["Labels"][idx]["Class"] = new_class
        self.display_image()

    def remove_box(self, idx):
        if idx < len(self.data[self.index]["Labels"]):
            del self.data[self.index]["Labels"][idx]
        self.display_image()

    def add_box(self):
        self.data[self.index]["Labels"].append({
            "Class": "human1",
            "BoundingBoxes": [100, 100, 50, 50]
        })
        self.display_image()

    def prev_image(self):
        if self.index > 0:
            self.index -= 1
            self.display_image()

    def next_image(self):
        if self.index < len(self.data) - 1:
            self.index += 1
            self.display_image()

    def save_json_as(self):
        default_name = os.path.splitext(os.path.basename(self.json_path))[0] + "_edited.json"
        save_path = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json")],
            initialfile=default_name
        )
        if save_path:
            with open(save_path, 'w') as f:
                json.dump(self.data, f, indent=4)
            messagebox.showinfo("Saved", f"Saved to {save_path}")

    def search_image(self):
        query = self.search_entry.get().strip()
        if not query:
            return
        file_list = [record.get("File", "") for record in self.data]
        matches = difflib.get_close_matches(query, file_list, n=1, cutoff=0.1)
        if matches:
            best_match = matches[0]
            for idx, record in enumerate(self.data):
                if record.get("File", "") == best_match:
                    self.index = idx
                    break
            self.display_image()
            messagebox.showinfo("Found", f"Showing image: {best_match}")
        else:
            messagebox.showinfo("Not Found", "No close matches found.")

if __name__ == "__main__":
    root = tk.Tk()
    img_folder = filedialog.askdirectory(title="Select Image Folder")
    json_file = filedialog.askopenfilename(title="Select JSON File", filetypes=[("JSON files", "*.json")])
    if img_folder and json_file:
        app = LabelEditorApp(root, img_folder, json_file)
        root.mainloop()
    else:
        print("Image folder or JSON file not selected.")

