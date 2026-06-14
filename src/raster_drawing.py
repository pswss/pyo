import numpy as np


def line_indices(start_row, start_col, end_row, end_col):
    """Return ordered row and column indices for a rasterized line."""
    row = int(start_row)
    col = int(start_col)
    end_row = int(end_row)
    end_col = int(end_col)

    rows = []
    cols = []
    delta_row = abs(end_row - row)
    delta_col = abs(end_col - col)
    step_row = 1 if row < end_row else -1
    step_col = 1 if col < end_col else -1
    error = delta_col - delta_row

    while True:
        rows.append(row)
        cols.append(col)
        if row == end_row and col == end_col:
            break

        doubled_error = error * 2
        if doubled_error > -delta_row:
            error -= delta_row
            col += step_col
        if doubled_error < delta_col:
            error += delta_col
            row += step_row

    return np.asarray(rows, dtype=int), np.asarray(cols, dtype=int)
