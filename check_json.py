#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simple 2-D label-editor with unique timestamp-to-PNG pairing.
Dependencies:  Pillow, tkinter (built-in on Windows/macOS, package tk on Linux).
"""
import os, re, json, functools, difflib, tkinter as tk
from collections import defaultdict
from tkinter import filedialog, messagebox, ttk
from PIL import Image, ImageTk

# ---------------------------------------------------------------------------

_NS   = 1_000_000_000                               # scale frac → nanoseconds
_PNG_RE = re.compile(r"(\d+)_(\d+)\.png$", re.I)    # 1732099924_825932304.png

def _ts_parts(ts: float):
    sec  = int(ts)
    frac = int(round((ts - sec) * _NS))             # round to nearest ns
    return sec, frac

def _png_parts(name: str):
    m = _PNG_RE.match(name)
    if not m:
        return None
    sec  = int(m.group(1))
    frac = int(m.group(2).ljust(9, "0"))            # right-pad → 9 digits
    return sec, frac

# ---------------------------------------------------------------------------

class LabelEditorApp:
    # .....................................................................
    def __init__(self, root, image_folder, json_path):
        self.root = root
        self.root.title("Label Editor")
        self.image_folder = image_folder
        self.json_path    = json_path

        # -------- load + ensure unique filenames -------------------------
        self.data  = self.load_json(json_path)
        self.sync_json_filenames_one2one()           # <<< unique mapping

        self.index = 0                               # current image index

        # ----------------------------------------------------------------
        self.label_options = ["human1","human2","human3","human4","human5"]
        self.class_colors  = {
            "human1":"#FF0000","human2":"#00AA00","human3":"#0000FF",
            "human4":"#FFA500","human5":"#800080"
        }

        # storage for UI items
        self.box_widgets, self.bbox_items = [], {}
        self.tl_handle_items, self.br_handle_items = {}, {}
        self.drag_data = {"item":None,"x":0,"y":0,
                          "index":None,"mode":None,"fixed_x":0,"fixed_y":0}

        # -------- canvas & controls -------------------------------------
        self.canvas_width, self.canvas_height = 800, 600
        self.canvas = tk.Canvas(root, width=self.canvas_width,
                                height=self.canvas_height, bg="gray")
        self.canvas.pack()

        self.controls_frame = tk.Frame(root); self.controls_frame.pack()

        tk.Button(self.controls_frame,text="Previous",
                  command=self.prev_image).grid(row=0,column=0,padx=5)
        tk.Button(self.controls_frame,text="Next",
                  command=self.next_image).grid(row=0,column=1,padx=5)
        tk.Button(self.controls_frame,text="Add Box",
                  command=self.add_box).grid(row=0,column=2,padx=5)
        tk.Button(self.controls_frame,text="Save As...",
                  command=self.save_json_as).grid(row=0,column=3,padx=5)

        tk.Label(self.controls_frame,text="Search:").grid(row=1,column=0,padx=5)
        self.search_entry = tk.Entry(self.controls_frame,width=30)
        self.search_entry.grid(row=1,column=1,columnspan=2,padx=5)
        tk.Button(self.controls_frame,text="Go",
                  command=self.search_image).grid(row=1,column=3,padx=5)

        self.image_name_label = tk.Label(self.controls_frame, text="",
                                         font=("Arial",12,"bold"))
        self.image_name_label.grid(row=2,column=0,columnspan=4,pady=5)

        # mouse bindings
        self.canvas.bind("<ButtonPress-1>", self.on_canvas_press)
        self.canvas.bind("<B1-Motion>",      self.on_canvas_drag)
        self.canvas.bind("<ButtonRelease-1>",self.on_canvas_release)

        self.display_image()

    # ------------------------------------------------------------------
    # ---------- FILENAME SYNC: one-to-one per whole second -------------
    # ------------------------------------------------------------------
    def sync_json_filenames_one2one(self):
        data = self.data

        # bucket pngs by second
        png_sec = defaultdict(list)          # sec -> [(frac,fname),…]
        for f in os.listdir(self.image_folder):
            parts = _png_parts(f)
            if parts:
                sec, frac = parts
                png_sec[sec].append((frac, f))

        # bucket records by second
        rec_sec = defaultdict(list)          # sec -> [(frac,idx),…]
        for i, rec in enumerate(data):
            sec, frac = _ts_parts(rec.get("Timestamp",0.0))
            rec_sec[sec].append((frac, i))

        changed, skipped = 0, 0
        for sec, rec_list in rec_sec.items():
            rec_list.sort()
            png_list = sorted(png_sec.get(sec, []))
            if len(rec_list) != len(png_list):
                skipped += len(rec_list)
                continue          # leave this second unchanged
            for (rfrac, idx), (pfrac, fname) in zip(rec_list, png_list):
                if data[idx].get("File") != fname:
                    data[idx]["File"] = fname
                    changed += 1

        with open(self.json_path,"w",encoding="utf-8") as fp:
            json.dump(data, fp, indent=4)

        messagebox.showinfo("Sync complete",
                            f"Updated {changed} rows; skipped {skipped} "
                            f"(count mismatch).")

    # ------------------------------------------------------------------
    def load_json(self, path):
        """Load JSON and normalise 'Labels' field."""
        with open(path,'r',encoding='utf-8') as f:
            raw = json.load(f)
        out = []
        for item in raw:
            labels = item.get("Labels", [])
            if isinstance(labels, dict):
                item["Labels"] = [labels]
            elif not isinstance(labels, list):
                item["Labels"] = []
            out.append(item)
        return out

    # --------------------- UI helpers ---------------------------------
    def get_color(self,label): return self.class_colors.get(label,"#888888")

    def find_closest_image(self,json_fname):
        """Fallback if exact file missing."""
        base,ext = os.path.splitext(json_fname.strip())
        if '.' not in base: return None
        sec, frac = base.split('.',1)
        try: target = int(frac.ljust(9,'0')[:9])
        except ValueError: return None
        best, diff = None, float('inf')
        for f in os.listdir(self.image_folder):
            if not f.endswith(ext): continue
            m = re.match(rf"{re.escape(sec)}_(\d+){re.escape(ext)}",f)
            if m:
                val = int(m.group(1).ljust(9,'0')[:9])
                d = abs(val - target)
                if d < diff: best,diff = f,d
        return os.path.join(self.image_folder,best) if best else None

    # --------------------- core drawing --------------------------------
    def display_image(self):
        self.canvas.delete("all")
        for w in self.box_widgets: w.destroy()
        self.box_widgets.clear()
        self.bbox_items.clear()
        self.tl_handle_items.clear()
        self.br_handle_items.clear()

        if not (0<=self.index<len(self.data)):
            return
        rec = self.data[self.index]
        fname = rec.get("File","")
        img_path = os.path.join(self.image_folder,fname)
        if not os.path.exists(img_path):
            alt = self.find_closest_image(fname)
            if alt:
                img_path = alt
                rec["File"] = os.path.basename(alt)
        if not os.path.exists(img_path):
            self.canvas.create_text(self.canvas_width//2,
                                    self.canvas_height//2,
                                    text="Image not found",
                                    font=("Arial",16))
            return

        # label
        self.image_name_label.config(text=f"Current Image: {rec['File']}")

        # load and scale
        img = Image.open(img_path)
        w0,h0 = img.size
        img.thumbnail((self.canvas_width,self.canvas_height))
        w1,h1 = img.size
        self.offset_x = (self.canvas_width - w1)//2
        self.offset_y = (self.canvas_height - h1)//2
        self.scale_x  = w1 / w0
        self.scale_y  = h1 / h0
        self.tk_img = ImageTk.PhotoImage(img)
        self.canvas.create_image(self.canvas_width//2,
                                 self.canvas_height//2,
                                 image=self.tk_img)

        # draw boxes
        for i,lbl in enumerate(rec["Labels"]):
            x,y,w,h = lbl["BoundingBoxes"][:4]
            x1 = self.offset_x + x*self.scale_x
            y1 = self.offset_y + y*self.scale_y
            x2 = x1 + w*self.scale_x
            y2 = y1 + h*self.scale_y
            col = self.get_color(lbl["Class"])
            rect = self.canvas.create_rectangle(x1,y1,x2,y2,
                                                outline=col,width=2,
                                                tags=(f"bbox_{i}","bbox"))
            self.bbox_items[i] = rect
            # handles
            size=8
            tl = self.canvas.create_rectangle(x1-size/2,y1-size/2,
                                              x1+size/2,y1+size/2,
                                              fill=col,outline=col,
                                              tags=(f"tl_handle_{i}","handle"))
            br = self.canvas.create_rectangle(x2-size/2,y2-size/2,
                                              x2+size/2,y2+size/2,
                                              fill=col,outline=col,
                                              tags=(f"br_handle_{i}","handle"))
            self.tl_handle_items[i]=tl; self.br_handle_items[i]=br
            # label text
            self.canvas.create_text(x1+4,y1+12,text=lbl["Class"],
                                    fill=col,anchor="nw",font=("Arial",10))
            # combobox + remove btn
            var=tk.StringVar(value=lbl["Class"])
            c = ttk.Combobox(self.controls_frame,textvariable=var,
                             values=self.label_options,width=10)
            c.grid(row=i+3,column=0,padx=5)
            c.bind("<<ComboboxSelected>>",
                   lambda e,idx=i,v=var: self.update_label(idx,v.get()))
            b = tk.Button(self.controls_frame,text="Remove",
                          command=functools.partial(self.remove_box,i))
            b.grid(row=i+3,column=1,padx=5)
            self.box_widgets.extend([c,b])

    # -------- mouse handlers (unchanged) ------------------------------
    def on_canvas_press(self,event):
        item=self.canvas.find_withtag("current")
        if not item: return
        tags=self.canvas.gettags(item[0])
        if any(t.startswith("tl_handle_") for t in tags):
            mode="resize_topleft"
            idx=int([t for t in tags if t.startswith("tl_handle_")][0].split("_")[2])
            coords=self.canvas.coords(self.bbox_items[idx]); fx,fy=coords[2],coords[3]
            self.drag_data.update(item=item[0],mode=mode,index=idx,
                                  fixed_x=fx,fixed_y=fy,x=event.x,y=event.y)
        elif any(t.startswith("br_handle_") for t in tags):
            mode="resize_bottomright"
            idx=int([t for t in tags if t.startswith("br_handle_")][0].split("_")[2])
            coords=self.canvas.coords(self.bbox_items[idx]); fx,fy=coords[0],coords[1]
            self.drag_data.update(item=item[0],mode=mode,index=idx,
                                  fixed_x=fx,fixed_y=fy,x=event.x,y=event.y)
        elif any(t.startswith("bbox_") for t in tags):
            mode="move"
            idx=int([t for t in tags if t.startswith("bbox_")][0].split("_")[1])
            self.drag_data.update(item=self.bbox_items[idx],mode=mode,index=idx,
                                  x=event.x,y=event.y)
    def on_canvas_drag(self,event):
        if self.drag_data["item"] is None: return
        dx,dy=event.x-self.drag_data["x"],event.y-self.drag_data["y"]
        idx=self.drag_data["index"]; mode=self.drag_data["mode"]
        if mode=="move":
            for d in (self.bbox_items,self.tl_handle_items,self.br_handle_items):
                if idx in d: self.canvas.move(d[idx],dx,dy)
        elif mode=="resize_topleft":
            fx,fy=self.drag_data["fixed_x"],self.drag_data["fixed_y"]
            self._resize_bbox(idx,event.x,event.y,fx,fy,handle="tl")
        elif mode=="resize_bottomright":
            fx,fy=self.drag_data["fixed_x"],self.drag_data["fixed_y"]
            self._resize_bbox(idx,fx,fy,event.x,event.y,handle="br")
        self.drag_data["x"],self.drag_data["y"]=event.x,event.y
    def _resize_bbox(self,idx,x1,y1,x2,y2,handle):
        self.canvas.coords(self.bbox_items[idx],x1,y1,x2,y2)
        size=8
        if handle=="tl":
            self.canvas.coords(self.tl_handle_items[idx],
                               x1-size/2,y1-size/2,x1+size/2,y1+size/2)
        else:
            self.canvas.coords(self.br_handle_items[idx],
                               x2-size/2,y2-size/2,x2+size/2,y2+size/2)
    def on_canvas_release(self,event):
        if self.drag_data["item"] is None: return
        idx=self.drag_data["index"]
        coords=self.canvas.coords(self.bbox_items[idx])
        if len(coords)>=4:
            x1,y1,x2,y2=coords
            nx=(x1-self.offset_x)/self.scale_x
            ny=(y1-self.offset_y)/self.scale_y
            nw=(x2-x1)/self.scale_x
            nh=(y2-y1)/self.scale_y
            self.data[self.index]["Labels"][idx]["BoundingBoxes"]=[nx,ny,nw,nh]
        self.drag_data={"item":None,"x":0,"y":0,"index":None,
                        "mode":None,"fixed_x":0,"fixed_y":0}
        self.display_image()

    # -------- misc UI actions ----------------------------------------
    def update_label(self,idx,new_class):
        if idx<len(self.data[self.index]["Labels"]):
            self.data[self.index]["Labels"][idx]["Class"]=new_class
        self.display_image()
    def remove_box(self,idx):
        if idx<len(self.data[self.index]["Labels"]):
            del self.data[self.index]["Labels"][idx]
        self.display_image()
    def add_box(self):
        self.data[self.index]["Labels"].append(
            {"Class":"human1","BoundingBoxes":[100,100,50,50]})
        self.display_image()
    def prev_image(self):
        if self.index>0:
            self.index-=1; self.display_image()
    def next_image(self):
        if self.index<len(self.data)-1:
            self.index+=1; self.display_image()
    def save_json_as(self):
        default=os.path.splitext(os.path.basename(self.json_path))[0]+"_edited.json"
        path=filedialog.asksaveasfilename(defaultextension=".json",
                                          filetypes=[("JSON files","*.json")],
                                          initialfile=default)
        if path:
            with open(path,'w',encoding='utf-8') as f:
                json.dump(self.data,f,indent=4)
            messagebox.showinfo("Saved",f"Saved to {path}")
    def search_image(self):
        q=self.search_entry.get().strip()
        if not q: return
        names=[r.get("File","") for r in self.data]
        m=difflib.get_close_matches(q,names,1,0.1)
        if m:
            self.index=names.index(m[0]); self.display_image()
            messagebox.showinfo("Found",f"Showing image: {m[0]}")
        else:
            messagebox.showinfo("Not Found","No close matches found.")

# ---------------------------------------------------------------------------
if __name__ == "__main__":
    root = tk.Tk()
    img_folder = filedialog.askdirectory(title="Select Image Folder")
    json_file  = filedialog.askopenfilename(title="Select JSON File",
                                            filetypes=[("JSON files","*.json")])
    if img_folder and json_file:
        app = LabelEditorApp(root, img_folder, json_file)
        root.mainloop()
    else:
        print("Image folder or JSON file not selected.")
