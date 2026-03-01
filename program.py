import tkinter as tk
import heapq, math, random, time, threading

empty = 0
wall  = 1


clr = {
    "empty": "#1a2035",
    "wall": "#513749",
    "start": "#b93510", 
    "goal": "#17f50b",
    "visited": "#1e3a5f", 
    "frontier": "#fbbf24",
    "path": "#6366f1",
    "agent": "#f43f5e",
}

def manhattan(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def euclidean(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

def neighbours(node, grid, rows, cols):
    r, c = node
    return [(r+dr, c+dc) for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]
            if 0 <= r+dr < rows and 0 <= c+dc < cols
            and grid[r+dr][c+dc] != wall]

def build_path(came_from, end):
    path, node = [], end
    while node:
        path.append(node); node = came_from.get(node)
    return path[::-1]

def astar(grid, start, goal, h, rows, cols):
    t0 = time.perf_counter()
    pq, came, g = [(0, start)], {start: None}, {start: 0}
    visited, frontier, closed = [], [], set()
    while pq:
        _, cur = heapq.heappop(pq)
        if cur in closed: 
            continue
        closed.add(cur); visited.append(cur)
        if cur == goal:
            return {"found": True, "path": build_path(came, cur),
                    "visited": visited, "frontier": frontier,
                    "cost": g[cur], "time": round((time.perf_counter()-t0)*1000, 2)}
        for nb in neighbours(cur, grid, rows, cols):
            new_g = g[cur] + 1
            if new_g < g.get(nb, float("inf")):
                came[nb] = cur; g[nb] = new_g
                heapq.heappush(pq, (new_g + h(nb, goal), nb))
                frontier.append(nb)
    return {"found": False, "path": [], "visited": visited, "frontier": frontier,
            "cost": 0, "time": round((time.perf_counter()-t0)*1000, 2)}

def gbfs(grid, start, goal, h, rows, cols):
    t0 = time.perf_counter()
    pq, came, seen = [(h(start, goal), start)], {start: None}, {start}
    visited, frontier = [], []
    while pq:
        _, cur = heapq.heappop(pq); visited.append(cur)
        if cur == goal:
            path = build_path(came, cur)
            return {"found": True, "path": path, "visited": visited,
                    "frontier": frontier, "cost": len(path)-1,
                    "time": round((time.perf_counter()-t0)*1000, 2)}
        for nb in neighbours(cur, grid, rows, cols):
            if nb not in seen:
                seen.add(nb); came[nb] = cur
                heapq.heappush(pq, (h(nb, goal), nb)); frontier.append(nb)
    return {"found": False, "path": [], "visited": visited, "frontier": frontier,
            "cost": 0, "time": round((time.perf_counter()-t0)*1000, 2)}


class app:
    def __init__(self, root):
        self.root = root
        self.root.title("pathfinding agent")
        self.root.configure(bg="#0a0e1a")
        self.root.geometry("1050x700")

        self.rows     = tk.IntVar(value=18);  self.cols    = tk.IntVar(value=28)
        self.density  = tk.DoubleVar(value=0.28)
        self.algo     = tk.StringVar(value="astar")
        self.heur     = tk.StringVar(value="manhattan")
        self.dyn_mode = tk.BooleanVar(value=False)
        self.paint    = tk.StringVar(value="wall")

        self.start_c = (2, 2);   self.goal_c = (15, 25)
        self.cs      = 26;       self.running = False
        self.v_vis   = set();    self.v_fron = set();   self.v_path = set()
        self.agent   = None;     self.a_path = [];      self.a_idx  = 0
        self.anim_id = None;     self.dyn_id = None

        
        self._search_start  = None  
        self._live_timer_id = None  

        self.build_ui()
        self.reset_grid()
        self.draw()

    # live timer helpers 
    def _start_live_timer(self):
        self._search_start = time.perf_counter()
        self._tick_live_timer()

    def _tick_live_timer(self):
        if self._search_start is None:
            return
        elapsed = (time.perf_counter() - self._search_start) * 1000
        self.mt.config(text=f"{elapsed:.1f} ms")
        self._live_timer_id = self.root.after(50, self._tick_live_timer)

    def _stop_live_timer(self, final_ms=None):
        if self._live_timer_id:
            self.root.after_cancel(self._live_timer_id)
            self._live_timer_id = None
        self._search_start = None
        if final_ms is not None:
            # Show both algorithm time and total wall time
            self.mt.config(text=f"{final_ms} ms")

    def build_ui(self):
        left = tk.Frame(self.root, bg="#111827", width=230)
        left.pack(side="left", fill="y"); left.pack_propagate(False)
        self.build_panel(left)

        right = tk.Frame(self.root, bg="#0a0e1a")
        right.pack(side="left", fill="both", expand=True)

        self.run_btn = tk.Button(right, text="▶  run search", bg="#6366f1",
                                 fg="white", bd=0, font=("courier",12,"bold"),
                                 padx=26, pady=8, cursor="hand2", command=self.toggle_run)
        self.run_btn.pack(pady=(12, 4))

        self.status = tk.Label(right, text="draw walls or generate maze, then run.",
                               bg="#0a0e1a", fg="#64748b", font=("courier",9))
        self.status.pack()

        # ── completion time banner
        self.time_banner = tk.Label(
            right,
            text="",
            bg="#0a0e1a",
            fg="#fbbf24",
            font=("courier", 11, "bold"),
        )
        self.time_banner.pack(pady=(2, 0))
        self.canvas = tk.Canvas(right, bg="#0d1117", highlightthickness=2,
                                highlightbackground="#1f2937", cursor="crosshair")
        self.canvas.pack(fill="both", expand=True, padx=10, pady=8)
        self.canvas.bind("<ButtonPress-1>", self.on_click)
        self.canvas.bind("<B1-Motion>",     self.on_drag)

    def build_panel(self, parent):
        p = tk.Frame(parent, bg="#111827")
        p.pack(fill="both", expand=True, padx=10, pady=10)

        def sec(t):
            tk.Label(p, text=t, bg="#111827", fg="#6366f1",
                     font=("courier",8,"bold")).pack(anchor="w", pady=(8,0))
            tk.Frame(p, bg="#1f2937", height=1).pack(fill="x", pady=(2,4))

        def lbl(t):
            tk.Label(p, text=t, bg="#111827", fg="#94a3b8",
                     font=("courier",8)).pack(anchor="w")

        def scale(var, lo, hi, step=1):
            tk.Scale(p, from_=lo, to=hi, resolution=step, orient="horizontal",
                     variable=var, bg="#111827", fg="#e2e8f0",
                     highlightthickness=0, troughcolor="#1f2937",
                     font=("courier",8)).pack(fill="x")

        def radios(opts, var):
            for txt, val in opts:
                tk.Radiobutton(p, text=txt, variable=var, value=val,
                               bg="#111827", fg="#e2e8f0", selectcolor="#6366f1",
                               activebackground="#111827", font=("courier",9),
                               indicatoron=0, padx=6, pady=3, bd=0,
                               cursor="hand2").pack(fill="x", pady=1)

        sec("grid config")
        lbl("rows");    scale(self.rows,    5, 35)
        lbl("columns"); scale(self.cols,    5, 55)
        tk.Button(p, text="apply size", bg="#6366f1", fg="white", bd=0, pady=4,
                  font=("courier",8,"bold"), cursor="hand2",
                  command=self.apply_size).pack(fill="x", pady=1)
        lbl("obstacle density"); scale(self.density, 0.05, 0.60, 0.01)

        bf = tk.Frame(p, bg="#111827"); bf.pack(fill="x", pady=2)
        for txt, bg, cmd in [("generate","#4f46e5",self.generate),("clear","#374151",self.clear)]:
            tk.Button(bf, text=txt, bg=bg, fg="white", bd=0, pady=4,
                      font=("courier",8,"bold"), cursor="hand2",
                      command=cmd).pack(side="left", fill="x", expand=True, padx=1)

        sec("algorithm")
        radios([("a* search","astar"),("greedy bfs","gbfs")], self.algo)

        sec("heuristic")
        radios([("manhattan","manhattan"),("euclidean","euclidean")], self.heur)

        sec("map editor")
        radios([("set start","start"),("set goal","goal"),
                ("draw wall","wall"),("erase","erase")], self.paint)

        sec("dynamic mode")
        tk.Checkbutton(p, text=" dynamic obstacles", variable=self.dyn_mode,
                       bg="#111827", fg="#e2e8f0", selectcolor="#6366f1",
                       activebackground="#111827", font=("courier",9),
                       cursor="hand2").pack(anchor="w")

        sec("metrics")
        self.mn = self.mrow(p, "nodes visited")
        self.mc = self.mrow(p, "path cost")
        self.mt = self.mrow(p, "time (ms)")

        sec("legend")
        for c, t in [(clr["start"],"start"),(clr["goal"],"goal"),(clr["path"],"path"),
                     (clr["frontier"],"frontier"),(clr["visited"],"visited"),
                     (clr["wall"],"wall"),(clr["agent"],"agent")]:
            row = tk.Frame(p, bg="#111827"); row.pack(fill="x", pady=1)
            tk.Label(row, bg=c, width=2).pack(side="left", padx=(0,5))
            tk.Label(row, text=t, bg="#111827", fg="#64748b",
                     font=("courier",8)).pack(side="left")

    def mrow(self, p, label):
        row = tk.Frame(p, bg="#111827"); row.pack(fill="x", pady=1)
        tk.Label(row, text=label, bg="#111827", fg="#64748b",
                 font=("courier",8), width=13, anchor="w").pack(side="left")
        v = tk.Label(row, text="—", bg="#111827", fg="#6366f1", font=("courier",10,"bold"))
        v.pack(side="right"); return v


    def reset_grid(self):
        r, c = self.rows.get(), self.cols.get()
        self.grid = [[empty]*c for _ in range(r)]

    def apply_size(self):
        self.stop(); self.reset_grid(); self.clear_vis(); self.draw()

    def generate(self):
        self.stop()
        r, c, d = self.rows.get(), self.cols.get(), self.density.get()
        self.grid = [[wall if random.random() < d else empty for _ in range(c)]
                     for _ in range(r)]
        self.grid[self.start_c[0]][self.start_c[1]] = empty
        self.grid[self.goal_c[0]][self.goal_c[1]]   = empty
        self.clear_vis(); self.draw()
        self.status.config(text=f"maze generated ({int(d*100)}% walls).")

    def clear(self):
        self.stop(); self.reset_grid(); self.clear_vis()
        self.time_banner.config(text="")
        self.draw()

    def clear_vis(self):
        self.v_vis = set(); self.v_fron = set()
        self.v_path = set(); self.agent = None

    
    def cell_clr(self, r, c):
        if self.agent == (r,c): 
            return clr["agent"]
        if (r,c) == self.start_c: 
            return clr["start"]
        if (r,c) == self.goal_c: 
            return clr["goal"]
        if self.grid[r][c] == wall: 
            return clr["wall"]
        if (r,c) in self.v_path: 
            return clr["path"]
        if (r,c) in self.v_vis: 
            return clr["visited"]
        if (r,c) in self.v_fron: 
            return clr["frontier"]
        return clr["empty"]

    def draw(self):
        self.canvas.delete("all")
        cs, rows, cols = self.cs, self.rows.get(), self.cols.get()
        for r in range(rows):
            for c in range(cols):
                x1 = c*(cs+1)+2;  y1 = r*(cs+1)+2
                self.canvas.create_rectangle(x1, y1, x1+cs, y1+cs,
                                             fill=self.cell_clr(r,c),
                                             outline="#0d1117", width=1)
                if (r,c) == self.start_c:
                    self.canvas.create_text(x1+cs//2, y1+cs//2, text="S",
                                            fill="white", font=("courier",max(7,cs//3),"bold"))
                elif (r,c) == self.goal_c:
                    self.canvas.create_text(x1+cs//2, y1+cs//2, text="G",
                                            fill="white", font=("courier",max(7,cs//3),"bold"))
        self.canvas.configure(scrollregion=(0,0,cols*(cs+1)+4,rows*(cs+1)+4))

    def to_cell(self, e):
        c = int(e.x//(self.cs+1)); r = int(e.y//(self.cs+1))
        return (r,c) if 0<=r<self.rows.get() and 0<=c<self.cols.get() else None

    def paint_cell(self, cell):
        r, c = cell; m = self.paint.get()
        if   m == "start": self.start_c = (r,c); self.grid[r][c] = empty
        elif m == "goal":  self.goal_c  = (r,c); self.grid[r][c] = empty
        elif m == "wall"  and (r,c) not in (self.start_c, self.goal_c): self.grid[r][c] = wall
        elif m == "erase" and (r,c) not in (self.start_c, self.goal_c): self.grid[r][c] = empty
        self.clear_vis()
        self.time_banner.config(text="")
        self.draw()

    def on_click(self, e):
        if not self.running:
            cell = self.to_cell(e)
            if cell: self.paint_cell(cell)

    def on_drag(self, e):
        if not self.running:
            cell = self.to_cell(e)
            if cell: self.paint_cell(cell)

    # search + animation 
    def toggle_run(self):
        if self.running: self.stop(); self.status.config(text="stopped.")
        else: self.run_search()

    def run_search(self):
        self.stop(); self.clear_vis()
        self.time_banner.config(text="")
        self.mn.config(text="—"); self.mc.config(text="—"); self.mt.config(text="—")
        self.draw()
        self.running = True
        self.run_btn.config(text="⏹  stop", bg="#ef4444")
        self.status.config(text="searching…")
        self._start_live_timer()                        # ← start live ticker
        h  = manhattan if self.heur.get() == "manhattan" else euclidean
        fn = astar     if self.algo.get() == "astar"     else gbfs
        g  = [row[:] for row in self.grid]
        threading.Thread(target=lambda: self.root.after(0,
            lambda: self.animate(fn(g, self.start_c, self.goal_c, h,
                                    self.rows.get(), self.cols.get()))),
            daemon=True).start()

    def animate(self, res):
        # Stop the live ticker and lock in the algorithm time
        self._stop_live_timer(final_ms=res["time"])

        if not self.running: 
            return
        if not res["found"]:
            self.mn.config(text=str(len(res["visited"])))
            self.mc.config(text="—")
            self.mt.config(text=f"{res['time']} ms")
            self.time_banner.config(
                text=f"⏱  completed in {res['time']} ms  |  no path found",
                fg="#ef4444"
            )
            self.status.config(text="no path found.")
            self.running = False
            self.run_btn.config(text="run search", bg="#6366f1"); return

        vis, fron, path, idx = res["visited"], res["frontier"], res["path"], [0]
        algo_ms = res["time"]   # pure algorithm time (before animation)

        def step():
            if not self.running: return
            i = idx[0]
            if i < len(vis):
                self.v_vis.add(vis[i])
                if i < len(fron): self.v_fron.add(fron[i])
                self.draw(); idx[0] += 1
                self.anim_id = self.root.after(10, step)
            else:
                self.v_fron.clear(); self.v_path = set(path); self.draw()
                self.mn.config(text=str(len(vis)))
                self.mc.config(text=str(res["cost"]))
                self.mt.config(text=f"{algo_ms} ms")

                # completion banner 
                self.time_banner.config(
                    text=f"search done in {algo_ms} ms  |  path: {len(path)-1} steps  |  visited: {len(vis)} nodes",
                    fg="#fbbf24"
                )
                self.status.config(text=f"path found!  {len(path)-1} steps.")
                if self.dyn_mode.get(): self.start_dynamic(path)
                else:
                    self.running = False
                    self.run_btn.config(text="run search", bg="#6366f1")
        step()

    # dynamic obstacle mode
    def start_dynamic(self, path):
        self.a_path = list(path); self.a_idx = 0
        self.agent = path[0]; self.draw(); self.dyn_step()

    def dyn_step(self):
        if not self.running: return
        self.a_idx += 1
        if self.a_idx >= len(self.a_path):
            self.agent = None; self.draw()
            self.status.config(text="agent reached the goal!")
            self.time_banner.config(text="🏁  agent reached the goal!", fg="#17f50b")
            self.running = False
            self.run_btn.config(text="▶  run search", bg="#6366f1"); return

        self.agent = self.a_path[self.a_idx]; self.draw()

        if random.random() < 0.20:
            r = random.randint(0, self.rows.get()-1)
            c = random.randint(0, self.cols.get()-1)
            if (r,c) not in (self.start_c, self.goal_c, self.agent) and self.grid[r][c] == empty:
                self.grid[r][c] = wall
                if (r,c) in self.a_path[self.a_idx:]:
                    self.status.config(text="obstacle detected — replanning…")
                    self.replan(); return

        self.dyn_id = self.root.after(230, self.dyn_step)

    def replan(self):
        h   = manhattan if self.heur.get() == "manhattan" else euclidean
        fn  = astar     if self.algo.get() == "astar"     else gbfs
        res = fn(self.grid, self.agent, self.goal_c, h, self.rows.get(), self.cols.get())
        if res["found"]:
            self.a_path = res["path"]; self.a_idx = 0
            self.v_path = set(res["path"]); self.draw()
            self.mn.config(text=str(len(res["visited"])))
            self.mc.config(text=str(res["cost"]))
            self.mt.config(text=f"{res['time']} ms")
            self.time_banner.config(
                text=f"🔄  replanned in {res['time']} ms  |  new path: {len(res['path'])-1} steps",
                fg="#fbbf24"
            )
            self.status.config(text="replanned successfully.")
            self.dyn_id = self.root.after(230, self.dyn_step)
        else:
            self.status.config(text="no path after obstacle — agent stuck.")
            self.time_banner.config(text="agent stuck — no path available", fg="#ef4444")
            self.running = False
            self.run_btn.config(text="run search", bg="#6366f1")

    def stop(self):
        self.running = False
        self._stop_live_timer()
        if self.anim_id: self.root.after_cancel(self.anim_id); self.anim_id = None
        if self.dyn_id:  self.root.after_cancel(self.dyn_id);  self.dyn_id  = None
        self.run_btn.config(text="run search", bg="#6366f1")


if __name__ == "__main__":
    root = tk.Tk()
    app(root)
    root.mainloop()