import sys
from pathlib import Path
import tkinter as tk
from tkinter import ttk, messagebox

# We'll import specialized modules later as we build them out
import cv2
from PIL import Image, ImageTk
# Compatibility for Pillow >= 10.0 where Image.LANCZOS was moved to Image.Resampling.LANCZOS
PIL_RESAMPLING = getattr(Image, "Resampling", Image)
import time
import os
import shutil

try:
    import yaml
    YAML_AVAILABLE = True
except ImportError:
    yaml = None
    YAML_AVAILABLE = False

# Use ultralytics for YOLO11 (YOLO26 is not a real version yet, YOLO11 is the latest stable YOLO from Ultralytics)
try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except ImportError:
    YOLO = None
    ULTRALYTICS_AVAILABLE = False

class TextRedirector(object):
    def __init__(self, widget):
        self.widget = widget
        # Regex to strip ANSI escape sequences (colors, cursor movements, etc.)
        import re
        self.ansi_escape = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')

    def write(self, str_data):
        # Remove ANSI codes before inserting into tkinter text widget
        clean_str = self.ansi_escape.sub('', str_data)
        if not clean_str:
            return
            
        parts = clean_str.split('\r')
        for i, part in enumerate(parts):
            if i > 0:
                self.widget.delete("end-1c linestart", "end-1c")
            if part:
                self.widget.insert(tk.END, part)
                
        self.widget.see(tk.END)
        self.widget.update_idletasks()

    def flush(self):
        pass

class YoloEndToEndApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("YOLO End-to-End: Data Collection, Annotation, Training, and Inference")
        self.geometry("1200x800")
        
        # Ensure base directory exists first!
        self.base_dir = Path(__file__).resolve().parent / "yolo_workspace"
        self.base_dir.mkdir(parents=True, exist_ok=True)
        
        self.dataset_dir = self.base_dir / "dataset"
        self.images_dir = self.dataset_dir / "images"
        self.labels_dir = self.dataset_dir / "labels"
        
        self.images_dir.mkdir(parents=True, exist_ok=True)
        self.labels_dir.mkdir(parents=True, exist_ok=True)
        
        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill="both", expand=True, padx=10, pady=10)
        
        self.cap = None
        self.camera_is_running = False
        
        self._build_collection_tab()
        self._build_annotation_tab()
        self._build_training_tab()
        self._build_inference_tab()

        self.notebook.bind("<<NotebookTabChanged>>", self.on_tab_changed)
        self.protocol("WM_DELETE_WINDOW", self.on_closing)

    def on_tab_changed(self, event):
        # Auto-release cameras when switching away from their respective tabs
        current_tab = self.notebook.index(self.notebook.select())
        if current_tab != 0: # Not on collection tab
            self.stop_camera()
        if current_tab != 3: # Not on inference tab
            self.stop_inference()

    def _build_collection_tab(self):
        self.tab_collect = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_collect, text="1. Data Collection")
        
        controls_frame = ttk.Frame(self.tab_collect)
        controls_frame.pack(side="top", fill="x", pady=10)
        
        self.btn_start_cam = ttk.Button(controls_frame, text="Start Camera", command=self.start_camera_collection)
        self.btn_start_cam.pack(side="left", padx=10)
        
        self.btn_stop_cam = ttk.Button(controls_frame, text="Stop Camera", command=self.stop_camera, state="disabled")
        self.btn_stop_cam.pack(side="left", padx=10)
        
        self.btn_capture = ttk.Button(controls_frame, text="Capture Image", command=self.capture_image, state="disabled")
        self.btn_capture.pack(side="left", padx=10)
        
        self.lbl_collect_status = ttk.Label(controls_frame, text="Ready")
        self.lbl_collect_status.pack(side="right", padx=10)
        
        self.video_label = ttk.Label(self.tab_collect)
        self.video_label.pack(fill="both", expand=True)
        
    def start_camera_collection(self):
        if self.cap is None:
            self.cap = cv2.VideoCapture(0)
        self.camera_is_running = True
        self.btn_start_cam.configure(state="disabled")
        self.btn_stop_cam.configure(state="normal")
        self.btn_capture.configure(state="normal")
        self.update_camera_frame_collection()

    def stop_camera(self):
        self.camera_is_running = False
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        self.video_label.configure(image="")
        self.btn_start_cam.configure(state="normal")
        self.btn_stop_cam.configure(state="disabled")
        self.btn_capture.configure(state="disabled")

    def update_camera_frame_collection(self):
        if self.camera_is_running and self.cap is not None and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                self.current_frame = frame.copy()
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(frame_rgb)
                
                target_w = self.video_label.winfo_width()
                target_h = self.video_label.winfo_height()
                if target_w > 10 and target_h > 10:
                    img = img.resize((target_w, target_h), PIL_RESAMPLING.LANCZOS)

                imgtk = ImageTk.PhotoImage(image=img)
                self.video_label.imgtk = imgtk
                self.video_label.configure(image=imgtk)
            self.after(30, self.update_camera_frame_collection)

    def capture_image(self):
        if hasattr(self, 'current_frame'):
            timestamp = int(time.time() * 1000)
            filename = self.images_dir / f"img_{timestamp}.jpg"
            cv2.imwrite(str(filename), self.current_frame)
            self.lbl_collect_status.configure(text=f"Saved: {filename.name}")
            # Refresh annotation list
            if hasattr(self, 'refresh_image_list'):
                self.refresh_image_list()

    def _build_annotation_tab(self):
        self.tab_annotate = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_annotate, text="2. Data Annotation")
        
        # Left side: Image list
        left_frame = ttk.Frame(self.tab_annotate, width=200)
        left_frame.pack(side="left", fill="y", padx=5, pady=5)
        
        ttk.Label(left_frame, text="Images").pack()
        
        list_frame = ttk.Frame(left_frame)
        list_frame.pack(fill="both", expand=True)
        
        scrollbar = ttk.Scrollbar(list_frame)
        scrollbar.pack(side="right", fill="y")
        
        self.listbox_images = tk.Listbox(list_frame, yscrollcommand=scrollbar.set)
        self.listbox_images.pack(side="left", fill="both", expand=True)
        scrollbar.config(command=self.listbox_images.yview)
        
        self.listbox_images.bind('<<ListboxSelect>>', self.on_image_select)
        
        nav_frame = ttk.Frame(left_frame)
        nav_frame.pack(fill="x", pady=5)
        
        self.btn_prev = ttk.Button(nav_frame, text="Prev", command=self.prev_image)
        self.btn_prev.pack(side="left", fill="x", expand=True, padx=(0, 2))
        
        self.btn_next = ttk.Button(nav_frame, text="Next", command=self.next_image)
        self.btn_next.pack(side="left", fill="x", expand=True, padx=(2, 0))
        
        self.btn_refresh = ttk.Button(left_frame, text="Refresh List", command=self.refresh_image_list)
        self.btn_refresh.pack(fill="x", pady=5)
        
        # Right side: Canvas and tools
        right_frame = ttk.Frame(self.tab_annotate)
        right_frame.pack(side="left", fill="both", expand=True, padx=5, pady=5)
        
        tools_frame = ttk.Frame(right_frame)
        tools_frame.pack(fill="x", pady=5)
        
        ttk.Label(tools_frame, text="Class Name:").pack(side="left")
        self.class_name_var = tk.StringVar(value="object")
        self.entry_class_name = ttk.Entry(tools_frame, textvariable=self.class_name_var, width=15)
        self.entry_class_name.pack(side="left", padx=5)
        
        ttk.Label(tools_frame, text="(Auto-saves when you draw the box)").pack(side="left", padx=15)
        
        self.btn_clear_anno = ttk.Button(tools_frame, text="Clear All", command=self.clear_all_annotations)
        self.btn_clear_anno.pack(side="left", padx=5)
        
        self.btn_undo_anno = ttk.Button(tools_frame, text="Undo Last", command=self.undo_last_annotation)
        self.btn_undo_anno.pack(side="left", padx=5)
        
        self.canvas_anno = tk.Canvas(right_frame, bg="gray")
        self.canvas_anno.pack(fill="both", expand=True)
        
        # Annotation state
        self.anno_image_path = None
        self.anno_image_tk = None
        self.current_bbox = None # (xmin, ymin, xmax, ymax) used for drawing
        self.rect_id = None      # current drawing rect id
        self.drawn_rects = []    # list of dicts: {'id': rect_id, 'bbox': (xmin,ymin,xmax,ymax), 'cls_id': cls_id, 'cls_name': cls_name}
        self.disp_w = 0
        self.disp_h = 0
        self.img_w = 0
        self.img_h = 0
        self.image_files = [] # Store actual file paths matching the listbox
        
        self.canvas_anno.bind("<ButtonPress-1>", self.on_canvas_press)
        self.canvas_anno.bind("<B1-Motion>", self.on_canvas_drag)
        self.canvas_anno.bind("<ButtonRelease-1>", self.on_canvas_release)
        
        self.classes_set = set()

    def refresh_image_list(self):
        self.listbox_images.delete(0, tk.END)
        self.image_files = sorted(list(self.images_dir.glob("*.jpg")))
        for img_path in self.image_files:
            label_path = self.labels_dir / (img_path.stem + ".txt")
            prefix = "[√] " if label_path.exists() else "[ ] "
            self.listbox_images.insert(tk.END, prefix + img_path.name)

    def prev_image(self):
        selection = self.listbox_images.curselection()
        if not selection:
            if self.image_files:
                self.listbox_images.selection_set(0)
                self.listbox_images.see(0)
                self.on_image_select(None)
            return
            
        idx = selection[0]
        if idx > 0:
            self.listbox_images.selection_clear(0, tk.END)
            self.listbox_images.selection_set(idx - 1)
            self.listbox_images.see(idx - 1)
            self.on_image_select(None)

    def next_image(self):
        selection = self.listbox_images.curselection()
        if not selection:
            if self.image_files:
                self.listbox_images.selection_set(0)
                self.listbox_images.see(0)
                self.on_image_select(None)
            return
            
        idx = selection[0]
        if idx < len(self.image_files) - 1:
            self.listbox_images.selection_clear(0, tk.END)
            self.listbox_images.selection_set(idx + 1)
            self.listbox_images.see(idx + 1)
            self.on_image_select(None)

    def on_image_select(self, event):
        selection = self.listbox_images.curselection()
        if not selection: return
        idx = selection[0]
        self.anno_image_path = self.image_files[idx]
        self.load_image_for_annotation()

    def load_image_for_annotation(self):
        if not self.anno_image_path: return
        img = Image.open(self.anno_image_path)
        self.img_w, self.img_h = img.size
        
        target_w = self.canvas_anno.winfo_width()
        target_h = self.canvas_anno.winfo_height()
        if target_w > 10 and target_h > 10:
            img = img.resize((target_w, target_h), PIL_RESAMPLING.LANCZOS)
            self.disp_w = target_w
            self.disp_h = target_h
        else:
            self.disp_w = self.img_w
            self.disp_h = self.img_h
        
        self.anno_image_tk = ImageTk.PhotoImage(img)
        self.canvas_anno.delete("all")
        self.canvas_anno.create_image(0, 0, anchor="nw", image=self.anno_image_tk)
        
        self.current_bbox = None
        self.rect_id = None
        self.drawn_rects.clear()
        
        # Check if annotation exists and load it
        label_path = self.labels_dir / (self.anno_image_path.stem + ".txt")
        if label_path.exists():
            with open(label_path, 'r') as f:
                lines = f.readlines()
                for line in lines:
                    parts = line.strip().split()
                    if len(parts) == 5:
                        cls_id, x_c, y_c, w, h = map(float, parts)
                        cls_id = int(cls_id)
                        # convert yolo to bbox
                        xmin = (x_c - w/2) * self.disp_w
                        xmax = (x_c + w/2) * self.disp_w
                        ymin = (y_c - h/2) * self.disp_h
                        ymax = (y_c + h/2) * self.disp_h
                        
                        bbox = (xmin, ymin, xmax, ymax)
                        rect_id = self.canvas_anno.create_rectangle(xmin, ymin, xmax, ymax, outline="red", width=2)
                        
                        # Add class name label
                        cls_name = "object"
                        if hasattr(self, 'classes_set'):
                            classes_list = sorted(list(self.classes_set))
                            if 0 <= cls_id < len(classes_list):
                                cls_name = classes_list[cls_id]
                                
                        self.canvas_anno.create_text(xmin, ymin-10, text=cls_name, fill="red", anchor="nw")
                        self.drawn_rects.append({'id': rect_id, 'bbox': bbox, 'cls_id': cls_id, 'cls_name': cls_name})

    def on_canvas_press(self, event):
        self.start_x = event.x
        self.start_y = event.y
        if self.rect_id:
            self.canvas_anno.delete(self.rect_id)
        self.rect_id = self.canvas_anno.create_rectangle(self.start_x, self.start_y, self.start_x, self.start_y, outline="red", width=2)

    def on_canvas_drag(self, event):
        cur_x, cur_y = event.x, event.y
        self.canvas_anno.coords(self.rect_id, self.start_x, self.start_y, cur_x, cur_y)

    def on_canvas_release(self, event):
        end_x, end_y = event.x, event.y
        xmin = min(self.start_x, end_x)
        xmax = max(self.start_x, end_x)
        ymin = min(self.start_y, end_y)
        ymax = max(self.start_y, end_y)
        
        # clamp to image boundaries
        xmin = max(0, min(xmin, self.disp_w))
        xmax = max(0, min(xmax, self.disp_w))
        ymin = max(0, min(ymin, self.disp_h))
        ymax = max(0, min(ymax, self.disp_h))
        
        # Ignore tiny boxes (accidental clicks)
        if (xmax - xmin) > 5 and (ymax - ymin) > 5:
            self.current_bbox = (xmin, ymin, xmax, ymax)
            self.canvas_anno.coords(self.rect_id, xmin, ymin, xmax, ymax)
            
            cls_name = self.class_name_var.get().strip()
            if not cls_name:
                cls_name = "object"
            self.canvas_anno.create_text(xmin, ymin-10, text=cls_name, fill="red", anchor="nw")
            
            # Save to drawn rects
            self.drawn_rects.append({
                'id': self.rect_id,
                'bbox': self.current_bbox,
                'cls_name': cls_name
            })
            
            self.rect_id = None # Reset for next draw
            self.save_annotation(show_msg=False)
        else:
            self.clear_current_drawing()

    def clear_current_drawing(self):
        if self.rect_id:
            self.canvas_anno.delete(self.rect_id)
        self.rect_id = None
        self.current_bbox = None

    def undo_last_annotation(self):
        if not self.drawn_rects:
            return
            
        # We need to reload the image entirely to clear text easily, 
        # or we could keep track of text IDs. Let's just pop and redraw all by reloading.
        self.drawn_rects.pop()
        self._rewrite_label_file_from_rects()
        self.load_image_for_annotation()

    def clear_all_annotations(self):
        self.drawn_rects.clear()
        self._rewrite_label_file_from_rects()
        self.load_image_for_annotation()
        
    def _rewrite_label_file_from_rects(self):
        if not self.anno_image_path:
            return
            
        label_path = self.labels_dir / (self.anno_image_path.stem + ".txt")
        
        if not self.drawn_rects:
            if label_path.exists():
                label_path.unlink() # Delete file if no boxes
            self.refresh_image_list()
            return
            
        # Ensure all classes are in the set
        for r in self.drawn_rects:
            self.classes_set.add(r['cls_name'])
            
        classes_list = sorted(list(self.classes_set))
        
        with open(label_path, "w") as f:
            for r in self.drawn_rects:
                cls_id = classes_list.index(r['cls_name'])
                xmin, ymin, xmax, ymax = r['bbox']
                x_center = ((xmin + xmax) / 2.0) / self.disp_w
                y_center = ((ymin + ymax) / 2.0) / self.disp_h
                box_w = (xmax - xmin) / self.disp_w
                box_h = (ymax - ymin) / self.disp_h
                f.write(f"{cls_id} {x_center:.6f} {y_center:.6f} {box_w:.6f} {box_h:.6f}\n")
                
        self._update_yaml(classes_list)
        self.refresh_image_list()

    def save_annotation(self, show_msg=True):
        if not self.anno_image_path or not self.drawn_rects:
            if show_msg: messagebox.showwarning("Warning", "Please select an image and draw a bounding box.")
            return

        label_path = self.labels_dir / (self.anno_image_path.stem + ".txt")
        label_existed_before = label_path.exists()
        self._rewrite_label_file_from_rects()
        if show_msg:
            messagebox.showinfo("Saved", f"Annotation saved for {self.anno_image_path.name}")
        else:
            # Silently refresh the list to show the [√] mark
            idx = self.listbox_images.curselection()
            self.refresh_image_list()
            if idx:
                self.listbox_images.selection_set(idx[0])
                self.listbox_images.see(idx[0])
                
        # Check if all images are annotated (only prompt if this was a new annotation)
        if not label_existed_before:
            unannotated_count = 0
            for img_path in self.image_files:
                if not (self.labels_dir / (img_path.stem + ".txt")).exists():
                    unannotated_count += 1
            
            if unannotated_count == 0 and len(self.image_files) > 0:
                self.after(100, lambda: messagebox.showinfo("All Done", "Great job! All images have been annotated."))
        
    def _update_yaml(self, classes_list):
        yaml_path = self.dataset_dir / "data.yaml"
        # We define train and val as the same folder for this quick demo
        content = f"path: {self.dataset_dir.absolute().as_posix()}\n"
        content += f"train: images\n"
        content += f"val: images\n\n"
        content += f"nc: {len(classes_list)}\n"
        content += f"names: {classes_list}\n"
        with open(yaml_path, "w") as f:
            f.write(content)

    def _build_training_tab(self):
        self.tab_train = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_train, text="3. Model Training")
        
        form_frame = ttk.Frame(self.tab_train)
        form_frame.pack(fill="x", padx=20, pady=20)
        
        ttk.Label(form_frame, text="Pre-trained Model:").grid(row=0, column=0, sticky="w", pady=5)
        self.model_name_var = tk.StringVar(value="yolo26n.pt")
        ttk.Combobox(form_frame, textvariable=self.model_name_var, values=["yolo26n.pt", "yolo26s.pt", "yolo26m.pt"]).grid(row=0, column=1, sticky="w", pady=5)
        
        ttk.Label(form_frame, text="Epochs:").grid(row=1, column=0, sticky="w", pady=5)
        self.epochs_var = tk.StringVar(value="5")
        ttk.Entry(form_frame, textvariable=self.epochs_var, width=10).grid(row=1, column=1, sticky="w", pady=5)
        
        ttk.Label(form_frame, text="Batch Size:").grid(row=2, column=0, sticky="w", pady=5)
        self.batch_var = tk.StringVar(value="8")
        ttk.Entry(form_frame, textvariable=self.batch_var, width=10).grid(row=2, column=1, sticky="w", pady=5)
        
        ttk.Label(form_frame, text="Dataset Split (Train:Val):").grid(row=3, column=0, sticky="w", pady=5)
        split_frame = ttk.Frame(form_frame)
        split_frame.grid(row=3, column=1, sticky="w", pady=5)
        
        self.train_ratio_var = tk.StringVar(value="8")
        self.val_ratio_var = tk.StringVar(value="2")
        
        ttk.Entry(split_frame, textvariable=self.train_ratio_var, width=3).pack(side="left")
        ttk.Label(split_frame, text=":").pack(side="left")
        ttk.Entry(split_frame, textvariable=self.val_ratio_var, width=3).pack(side="left")
        
        self.btn_train = ttk.Button(form_frame, text="Start Training", command=self.start_training)
        self.btn_train.grid(row=4, column=0, columnspan=2, pady=15)
        
        self.train_log = tk.Text(self.tab_train, height=20, bg="black", fg="white")
        self.train_log.pack(fill="both", expand=True, padx=20, pady=10)
        self.log_msg("Ready to train. Ensure you have annotated images and a data.yaml file.")

    def log_msg(self, msg):
        self.train_log.insert(tk.END, msg + "\n")
        self.train_log.see(tk.END)
        self.update_idletasks()

    def start_training(self):
        if not ULTRALYTICS_AVAILABLE:
            messagebox.showerror("Error", "Ultralytics YOLO is not installed. Please install it first.")
            return
            
        yaml_path = self.dataset_dir / "data.yaml"
        if not yaml_path.exists():
            messagebox.showerror("Error", "data.yaml not found! Please annotate at least one image.")
            return
            
        try:
            tr = float(self.train_ratio_var.get())
            va = float(self.val_ratio_var.get())
        except ValueError:
            tr, va = 8, 2
            
        total = tr + va
        if total == 0:
            tr, va = 8, 2
            total = 10
            
        tr_ratio = tr / total
        va_ratio = va / total
        
        images = []
        for img_path in self.images_dir.glob("*.jpg"):
            label_path = self.labels_dir / (img_path.stem + ".txt")
            if label_path.exists():
                images.append(img_path)
                
        if not images:
            messagebox.showerror("Error", "No annotated images found.")
            return
            
        import random
        random.shuffle(images)
        
        n_total = len(images)
        n_train = int(n_total * tr_ratio)
        
        train_imgs = images[:n_train]
        val_imgs = images[n_train:]
        
        split_dir = self.base_dir / "dataset_split"
        if split_dir.exists():
            shutil.rmtree(split_dir)
            
        for split in ["train", "val"]:
            (split_dir / split / "images").mkdir(parents=True, exist_ok=True)
            (split_dir / split / "labels").mkdir(parents=True, exist_ok=True)
            
        def copy_files(img_list, split_name):
            for img_path in img_list:
                label_path = self.labels_dir / (img_path.stem + ".txt")
                shutil.copy(img_path, split_dir / split_name / "images" / img_path.name)
                shutil.copy(label_path, split_dir / split_name / "labels" / label_path.name)
                
        copy_files(train_imgs, "train")
        copy_files(val_imgs, "val")

        if not YAML_AVAILABLE:
            raise ImportError("PyYAML is required for dataset splitting. Install it with: pip install pyyaml")
        with open(yaml_path, 'r') as f:
            orig_data = yaml.safe_load(f)
            
        new_yaml_path = split_dir / "data.yaml"
        content = f"path: {split_dir.absolute().as_posix()}\n"
        content += f"train: train/images\n"
        content += f"val: val/images\n\n"
        content += f"nc: {orig_data['nc']}\n"
        content += f"names: {orig_data['names']}\n"
        with open(new_yaml_path, "w") as f:
            f.write(content)
            
        model_name = self.model_name_var.get()
        epochs = int(self.epochs_var.get())
        batch = int(self.batch_var.get())
        
        self.btn_train.configure(state="disabled")
        self.log_msg(f"Starting training with {model_name} for {epochs} epochs...")
        self.log_msg(f"Dataset split: Train={len(train_imgs)}, Val={len(val_imgs)}")
        if len(val_imgs) == 0:
            self.log_msg("⚠️ Warning: Validation set is empty! 'best.pt' may not be saved correctly. Please use 'last.pt' for inference or add more data.")
        
        # We run training in a separate thread to keep UI responsive
        import threading
        threading.Thread(target=self._run_yolo_train, args=(model_name, new_yaml_path, epochs, batch), daemon=True).start()

    def _run_yolo_train(self, model_name, yaml_path, epochs, batch):
        original_stdout = sys.stdout
        original_stderr = sys.stderr
        try:
            # Redirect stdout and stderr to the text widget
            sys.stdout = TextRedirector(self.train_log)
            sys.stderr = TextRedirector(self.train_log)
            
            model = YOLO(model_name)
            
            # Determine device: check if torch has CUDA, otherwise use cpu
            import torch
            device_str = '0' if torch.cuda.is_available() else 'cpu'
            print(f"Using device: {device_str}")
            
            # Start training
            # Note: ultralytics training blocks the thread.
            results = model.train(
                data=str(yaml_path),
                epochs=epochs,
                batch=batch,
                project=str(self.base_dir / "runs"),
                name="train",
                exist_ok=True, # overwrite existing run for simplicity
                device=device_str
            )
            
            self.trained_weights_path = self.base_dir / "runs" / "train" / "weights" / "best.pt"
            
            # Parse results.csv for metrics
            metrics_msg = ""
            results_csv = self.base_dir / "runs" / "train" / "results.csv"
            if results_csv.exists():
                try:
                    import csv
                    with open(results_csv, 'r') as f:
                        reader = csv.reader(f)
                        headers = [h.strip() for h in next(reader)]
                        last_row = None
                        for row in reader:
                            if len(row) == len(headers):
                                last_row = row
                        if last_row:
                            metrics_msg += "\n" + "="*40 + "\n"
                            metrics_msg += "🎯 Final Training Metrics (Validation):\n"
                            for h, v in zip(headers, last_row):
                                if 'metrics/' in h:
                                    name = h.replace('metrics/', '').strip()
                                    metrics_msg += f"   - {name}: {float(v):.4f}\n"
                            metrics_msg += "="*40
                except Exception as ex:
                    print(f"Failed to parse metrics: {ex}")
            
            # Switch back to main thread for UI updates
            self.after(0, lambda: self.log_msg("\nTraining Complete!"))
            if metrics_msg:
                self.after(0, lambda m=metrics_msg: self.log_msg(m))
            self.after(0, lambda: self.log_msg(f"Weights saved to: {self.trained_weights_path}"))
            self.after(0, lambda: self.weights_var.set(str(self.trained_weights_path)))
            self.after(0, lambda: self.btn_train.configure(state="normal"))
            self.after(0, lambda: messagebox.showinfo("Success", "Training finished! The trained weights have been automatically loaded into the Inference tab."))
            
        except Exception as e:
            self.after(0, lambda: self.log_msg(f"\nError during training: {str(e)}"))
            self.after(0, lambda: self.btn_train.configure(state="normal"))
        finally:
            sys.stdout = original_stdout
            sys.stderr = original_stderr

    def _build_inference_tab(self):
        self.tab_inference = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_inference, text="4. Real-time Inference")
        
        controls_frame = ttk.Frame(self.tab_inference)
        controls_frame.pack(side="top", fill="x", pady=10)
        
        ttk.Label(controls_frame, text="Weights Path:").pack(side="left", padx=5)
        self.weights_var = tk.StringVar(value="")
        self.entry_weights = ttk.Entry(controls_frame, textvariable=self.weights_var, width=50)
        self.entry_weights.pack(side="left", padx=5)
        
        self.btn_browse = ttk.Button(controls_frame, text="Browse", command=self.browse_weights)
        self.btn_browse.pack(side="left", padx=5)
        
        self.btn_start_infer = ttk.Button(controls_frame, text="Start Inference", command=self.start_inference)
        self.btn_start_infer.pack(side="left", padx=10)
        
        self.btn_stop_infer = ttk.Button(controls_frame, text="Stop Inference", command=self.stop_inference, state="disabled")
        self.btn_stop_infer.pack(side="left", padx=10)
        
        ttk.Label(controls_frame, text="Conf:").pack(side="left", padx=5)
        self.conf_var = tk.DoubleVar(value=0.25)
        self.scale_conf = ttk.Scale(controls_frame, from_=0.01, to=1.0, variable=self.conf_var, orient="horizontal", length=100)
        self.scale_conf.pack(side="left", padx=5)
        
        self.lbl_infer_status = ttk.Label(controls_frame, text="Ready")
        self.lbl_infer_status.pack(side="left", padx=10)
        
        self.infer_video_label = ttk.Label(self.tab_inference)
        self.infer_video_label.pack(fill="both", expand=True)
        
        self.infer_cap = None
        self.infer_is_running = False
        self.infer_model = None

    def browse_weights(self):
        from tkinter import filedialog
        path = filedialog.askopenfilename(initialdir=self.base_dir, title="Select Weights", filetypes=(("PyTorch Model", "*.pt"), ("All Files", "*.*")))
        if path:
            self.weights_var.set(path)

    def start_inference(self):
        weights_path = self.weights_var.get()
        if not weights_path or not os.path.exists(weights_path):
            messagebox.showerror("Error", "Valid weights path is required.")
            return
            
        if not ULTRALYTICS_AVAILABLE:
            messagebox.showerror("Error", "Ultralytics YOLO is not installed.")
            return
            
        self.btn_start_infer.configure(state="disabled")
        self.lbl_infer_status.configure(text="Loading model... Please wait.")
        self.update_idletasks()
        
        try:
            self.infer_model = YOLO(weights_path)
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load model: {str(e)}")
            self.btn_start_infer.configure(state="normal")
            self.lbl_infer_status.configure(text="Ready")
            return
            
        self.lbl_infer_status.configure(text="Opening camera...")
        self.update_idletasks()
        
        if self.infer_cap is None:
            self.infer_cap = cv2.VideoCapture(0)
            
        self.infer_is_running = True
        self.btn_stop_infer.configure(state="normal")
        self.lbl_infer_status.configure(text="Running Inference")
        self.update_inference_frame()

    def stop_inference(self):
        self.infer_is_running = False
        if self.infer_cap is not None:
            self.infer_cap.release()
            self.infer_cap = None
        self.infer_video_label.configure(image="")
        self.btn_start_infer.configure(state="normal")
        self.btn_stop_infer.configure(state="disabled")
        self.lbl_infer_status.configure(text="Ready")

    def update_inference_frame(self):
        if self.infer_is_running and self.infer_cap is not None and self.infer_cap.isOpened():
            ret, frame = self.infer_cap.read()
            if ret:
                # Run inference
                conf = self.conf_var.get()
                results = self.infer_model(frame, conf=conf, verbose=False)
                annotated_frame = results[0].plot()
                
                # Convert to PIL
                frame_rgb = cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(frame_rgb)
                
                target_w = self.infer_video_label.winfo_width()
                target_h = self.infer_video_label.winfo_height()
                if target_w > 10 and target_h > 10:
                    img = img.resize((target_w, target_h), PIL_RESAMPLING.LANCZOS)

                imgtk = ImageTk.PhotoImage(image=img)

                self.infer_video_label.imgtk = imgtk
                self.infer_video_label.configure(image=imgtk)
                
            self.after(30, self.update_inference_frame)

    def on_closing(self):
        self.stop_camera()
        self.stop_inference()
        self.destroy()

if __name__ == "__main__":
    app = YoloEndToEndApp()
    app.mainloop()
