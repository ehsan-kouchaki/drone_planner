"""
results_logger.py  —  drop into Drones/utils/
----------------------------------------------
Automatically saves solver results to an Excel table after every run,
and archives a timestamped copy of problem_definition.yaml.

Behaviour:
  • First write for a (UAS#, PoI#) row  → plain number in each cell.
  • Subsequent writes for the same row   → cell becomes =AVERAGE(v1, v2, ..., vN)
    so every new run appends its value inside the average formula.
  • New (UAS#, PoI#) combination         → new row inserted in sorted order.
  • Each run copies problem_definition.yaml →
      results/{UAS}_{PoI}/data_{YYYYMMDD_HHMMSS}.yaml

Column layout (adjust COL_MAP if your sheet differs):
  A  UAS #       B  PoI #
  C  exec Gurobi  D  exec CPLEX   E  exec PyGAD  F  exec ACO
  G  exec CNP     H  exec OR-Tools
  I  make Gurobi  J  make CPLEX   K  make PyGAD  L  make ACO
  M  make CNP     N  make OR-Tools
"""

import os
import shutil
import threading
from datetime import datetime
import openpyxl
from openpyxl.styles import Font, Alignment, PatternFill, Border, Side

# ── Configuration ──────────────────────────────────────────────────────────────

# Root results folder (one level above utils/)
_RESULTS_DIR = os.path.join(os.path.dirname(__file__), "..", "results")

EXCEL_PATH = os.path.join(_RESULTS_DIR, "Results_Table.xlsx")

# Source problem_definition.yaml that p_generator.py writes to.
# Adjust this path if yours differs.
YAML_SOURCE_PATH = os.path.join(
    os.path.dirname(__file__), "..", "cfg", "pd", "problem_definition.yaml"
)

SHEET_NAME     = None   # None → active sheet
FIRST_DATA_ROW = 3      # first row that contains (UAS#, PoI#) data

COL_MAP = {
    "uas":          1,
    "poi":          2,
    "exec_gurobi":  3,
    "exec_cplex":   4,
    "exec_pygad":   5,
    "exec_aco":     6,
    "exec_cnp":     7,
    "exec_ortools": 8,
    "make_gurobi":  9,
    "make_cplex":  10,
    "make_pygad":  11,
    "make_aco":    12,
    "make_cnp":    13,
    "make_ortools":14,
}

_lock = threading.Lock()


# ── Public API ─────────────────────────────────────────────────────────────────

def save_results(problem_data, timing, makespan):
    """
    Parameters
    ----------
    problem_data : dict   — must contain uas_num, addresses, start_nodes, end_nodes
    timing       : dict   — {solver_key: elapsed_seconds}   e.g. {"ortools": 0.06}
    makespan     : dict   — {solver_key: longest_route_sec} e.g. {"ortools": 984}
    """
    uas_num = int(problem_data["uas_num"])
    poi_num = _count_pois(problem_data)

    # ── Archive the problem definition YAML ───────────────────────────────────
    _archive_yaml(uas_num, poi_num)

    # ── Write to Excel ────────────────────────────────────────────────────────
    _ensure_file_exists()

    with _lock:
        wb = openpyxl.load_workbook(EXCEL_PATH)   # formulas kept as strings
        ws = wb.active if SHEET_NAME is None else wb[SHEET_NAME]

        row = _find_or_create_row(ws, uas_num, poi_num)

        for key, solver in [
            ("exec_gurobi",  "gurobi"),
            ("exec_cplex",   "cplex"),
            ("exec_pygad",   "pygad"),
            ("exec_aco",     "aco"),
            ("exec_cnp",     "cnp"),
            ("exec_ortools", "ortools"),
            ("make_gurobi",  "gurobi"),
            ("make_cplex",   "cplex"),
            ("make_pygad",   "pygad"),
            ("make_aco",     "aco"),
            ("make_cnp",     "cnp"),
            ("make_ortools", "ortools"),
        ]:
            src = timing if key.startswith("exec_") else makespan
            _update_cell(ws, row, COL_MAP[key], src.get(solver))

        wb.save(EXCEL_PATH)

    print(f"[results_logger] Saved: UAS={uas_num}, PoI={poi_num}  →  {EXCEL_PATH}")


# ── YAML archiver ─────────────────────────────────────────────────────────────

def _archive_yaml(uas_num, poi_num):
    """
    Copy problem_definition.yaml to
      results/{uas_num}_{poi_num}/data_{YYYYMMDD_HHMMSS}.yaml
    Silently skips if the source file does not exist.
    """
    src = os.path.normpath(YAML_SOURCE_PATH)
    if not os.path.exists(src):
        print(f"[results_logger] WARNING: YAML source not found: {src}")
        return

    # Build destination folder  results/2_12/
    subfolder = os.path.join(
        os.path.normpath(_RESULTS_DIR),
        f"{uas_num}_{poi_num}"
    )
    os.makedirs(subfolder, exist_ok=True)

    # Timestamped filename
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    dst = os.path.join(subfolder, f"data_{timestamp}.yaml")

    shutil.copy2(src, dst)
    print(f"[results_logger] YAML archived → {dst}")


# ── Cell update logic ──────────────────────────────────────────────────────────

def _update_cell(ws, row, col, new_value):
    """
    Write new_value into ws.cell(row, col) using AVERAGE accumulation:

      empty cell          →  plain number  (first run)
      plain number  n     →  =AVERAGE(n, new_value)
      =AVERAGE(a, b, ...) →  =AVERAGE(a, b, ..., new_value)
    """
    if new_value is None:
        return

    v = round(float(new_value), 4)
    cell = ws.cell(row=row, column=col)
    current = cell.value

    if current is None:
        # First entry — plain number is cleaner than =AVERAGE(x)
        cell.value = v

    elif isinstance(current, str) and current.upper().startswith("=AVERAGE("):
        # Already an AVERAGE formula — append the new value inside it
        inner = current[9:].rstrip(")")          # strip "=AVERAGE(" and trailing ")"
        cell.value = f"=AVERAGE({inner}, {v})"

    elif isinstance(current, (int, float)):
        # Second run — promote the plain number to an AVERAGE formula
        cell.value = f"=AVERAGE({round(float(current), 4)}, {v})"

    else:
        # Unexpected content — overwrite safely
        cell.value = v


# ── Row lookup / creation ──────────────────────────────────────────────────────

def _find_or_create_row(ws, uas_num, poi_num):
    """
    Return the Excel row number for (uas_num, poi_num).
    If not found, insert a new row in sorted order (by UAS then PoI).
    """
    col_uas = COL_MAP["uas"]
    col_poi = COL_MAP["poi"]

    # Build merged-cell value map so we read the right value even in merged rows
    merge_map = {}
    for merge in ws.merged_cells.ranges:
        top_val = ws.cell(merge.min_row, merge.min_col).value
        for r in range(merge.min_row, merge.max_row + 1):
            for c in range(merge.min_col, merge.max_col + 1):
                merge_map[(r, c)] = top_val

    def cell_val(r, c):
        return merge_map.get((r, c), ws.cell(r, c).value)

    last_data_row = FIRST_DATA_ROW - 1

    for row in range(FIRST_DATA_ROW, ws.max_row + 1):
        try:
            u = int(cell_val(row, col_uas))
            p = int(cell_val(row, col_poi))
        except (TypeError, ValueError):
            continue

        last_data_row = row

        if u == uas_num and p == poi_num:
            return row   # ← found existing row

        # Determine insertion point for sorted order
        if (u, p) > (uas_num, poi_num):
            # Insert a new row HERE (before the current row)
            ws.insert_rows(row)
            ws.cell(row=row, column=col_uas).value = uas_num
            ws.cell(row=row, column=col_poi).value = poi_num
            _apply_data_row_style(ws, row)
            return row

    # Append after the last data row
    new_row = last_data_row + 1
    ws.cell(row=new_row, column=col_uas).value = uas_num
    ws.cell(row=new_row, column=col_poi).value = poi_num
    _apply_data_row_style(ws, new_row)
    return new_row


# ── Helpers ────────────────────────────────────────────────────────────────────

def _count_pois(problem_data):
    n_total  = len(problem_data["addresses"])
    n_depots = len(set(
        list(problem_data.get("start_nodes", [])) +
        list(problem_data.get("end_nodes",   []))
    ))
    return n_total - n_depots


def _apply_data_row_style(ws, row):
    """Light styling for a newly inserted data row."""
    thin   = Side(style="thin")
    border = Border(left=thin, right=thin, top=thin, bottom=thin)
    center = Alignment(horizontal="center", vertical="center")
    fill   = PatternFill("solid",
                         start_color="DEEAF1" if row % 2 == 0 else "FFFFFF",
                         end_color  ="DEEAF1" if row % 2 == 0 else "FFFFFF")
    for col in range(1, len(COL_MAP)):
        c = ws.cell(row=row, column=col)
        c.border    = border
        c.alignment = center
        c.fill      = fill
        c.font      = Font(name="Arial", size=10)
    ws.row_dimensions[row].height = 18


# ── File bootstrap ─────────────────────────────────────────────────────────────

def _ensure_file_exists():
    """Create Results_Table.xlsx with headers if it does not exist yet."""
    os.makedirs(os.path.dirname(EXCEL_PATH), exist_ok=True)
    if os.path.exists(EXCEL_PATH):
        return

    wb  = openpyxl.Workbook()
    ws  = wb.active
    ws.title = "Results"

    thin       = Side(style="thin")
    border     = Border(left=thin, right=thin, top=thin, bottom=thin)
    center     = Alignment(horizontal="center", vertical="center", wrap_text=True)
    white_font = Font(bold=True, color="FFFFFF", name="Arial", size=10)
    dark_fill  = PatternFill("solid", start_color="1F4E79", end_color="1F4E79")
    mid_fill   = PatternFill("solid", start_color="2E75B6", end_color="2E75B6")

    # Row 1 – group labels
    ws.merge_cells("A1:B1"); ws["A1"] = ""
    ws.merge_cells("C1:H1"); ws["C1"] = "Execution Time (s)"
    ws.merge_cells("I1:N1"); ws["I1"] = "Time to Complete Longest Route (s)"
    for addr in ("C1", "I1"):
        ws[addr].font = white_font; ws[addr].fill = mid_fill; ws[addr].alignment = center

    # Row 2 – column headers
    headers = ["UAS #", "PoI #",
               "Gurobi", "CPLEX", "PyGAD", "ACO", "CNP", "OR-Tools",
               "Gurobi", "CPLEX", "PyGAD", "ACO", "CNP", "OR-Tools"]
    for col, h in enumerate(headers, 1):
        c = ws.cell(row=2, column=col, value=h)
        c.font = white_font; c.fill = dark_fill
        c.alignment = center; c.border = border

    ws.row_dimensions[1].height = 20
    ws.row_dimensions[2].height = 30
    ws.column_dimensions["A"].width = 8
    ws.column_dimensions["B"].width = 8
    for letter in "CDEFGHIJKLMN":
        ws.column_dimensions[letter].width = 12

    wb.save(EXCEL_PATH)
