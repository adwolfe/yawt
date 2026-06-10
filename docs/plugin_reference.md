# Analysis Plugin Reference

Analysis plugins are YAML files that define custom metrics computed over tracked worm data. Each plugin specifies a formula, optional intermediate bindings, an aggregation mode, and a plot configuration.

---

## Standard Variables

These variables are always available in any formula, filter, or binding expression.

### Position

| Variable | Description |
|---|---|
| `x`, `y` | Centroid position (pixels) |
| `x_um`, `y_um` | Centroid position (micrometers) |
| `xhead`, `yhead` | Head tip position (pixels) |
| `xtail`, `ytail` | Tail tip position (pixels) |
| `xhead_um`, `yhead_um` | Head tip position (micrometers) |
| `xtail_um`, `ytail_um` | Tail tip position (micrometers) |

### Time & Frame

| Variable | Description |
|---|---|
| `t` | Time in seconds (`frame / fps`) |
| `frame` | Frame number |
| `fps` | Frames per second |

### Body Measurements

| Variable | Description |
|---|---|
| `area` | Blob area (px²) |
| `area_um2` | Blob area (µm²) |
| `body_length` | Centerline arc length (px) |
| `body_length_um` | Centerline arc length (µm) |
| `aspect_ratio` | Bounding-box long/short ratio |

### Speed

| Variable | Description |
|---|---|
| `speed` | 2 s smoothed centroid speed (auto-selects px or µm) |
| `speed_px` | 2 s smoothed speed (px/s) |
| `speed_um` | 2 s smoothed speed (µm/s) |

### Tracking Quality

| Variable | Description |
|---|---|
| `quality` | Numeric quality code |
| `Single` | Constant: `0` |
| `Merged` | Constant: `1` |
| `Split` | Constant: `2` |
| `Lost` | Constant: `3` |

---

## ROI Reference Variables

These variables are available only when the corresponding reference points are configured in the project.

| Variable | Description |
|---|---|
| `has_start`, `has_end`, `has_center` | `1` if point is configured, else `0` |
| `start_x`, `start_y` | Start point coordinates |
| `end_x`, `end_y` | End point coordinates |
| `center_x`, `center_y` | Center point coordinates |
| `dist_to_start` | Distance from centroid to start (auto px or µm) |
| `dist_to_start_px` | Distance from centroid to start (px) |
| `dist_to_start_um` | Distance from centroid to start (µm) |
| `dist_to_end` | Distance from centroid to end (auto px or µm) |
| `dist_to_end_px` | Distance from centroid to end (px) |
| `dist_to_end_um` | Distance from centroid to end (µm) |
| `dist_to_center` | Distance from centroid to center (auto px or µm) |
| `dist_to_center_px` | Distance from centroid to center (px) |
| `dist_to_center_um` | Distance from centroid to center (µm) |

---

## Binding Primitives

Stateful functions used in the `bindings:` section to compute intermediate values. Bindings are evaluated in order, and later bindings can reference earlier ones.

### `diff(expr)`

Computes the frame-to-frame difference of an expression: `current - previous`.

Special case: `diff(t)` returns `1/fps` (constant time step).

```yaml
bindings:
  dx: diff(x)
  dy: diff(y)
```

### `smooth(expr, window_s [, filter])`

Applies a rolling mean over `window_s` seconds. The optional `filter` expression restricts which frames are included in the average.

```yaml
bindings:
  raw_speed: sqrt(dx*dx + dy*dy) / diff(t)
  smooth_speed: smooth(raw_speed, 2, quality != Lost && diff(t) > 0)
```

---

## Mathematical Functions

Available in all expressions (formula, filter, bindings).

| Function | Description |
|---|---|
| `sqrt(x)` | Square root |
| `abs(x)` | Absolute value |
| `floor(x)` | Floor |
| `ceil(x)` | Ceiling |
| `sin(x)` | Sine |
| `cos(x)` | Cosine |
| `tan(x)` | Tangent |
| `log(x)` | Natural logarithm (returns `0` if `x ≤ 0`) |
| `exp(x)` | Exponential |
| `pow(x, y)` | Power (`x^y` syntax also supported) |
| `min(a, b)` | Minimum of two values |
| `max(a, b)` | Maximum of two values |

### Operators

| Category | Operators |
|---|---|
| Arithmetic | `+` `-` `*` `/` `^` |
| Comparison | `==` `!=` `<` `<=` `>` `>=` |
| Logical | `&&` `\|\|` |
| Conditional | `condition ? true_expr : false_expr` |

---

## Plugin YAML Schema

```yaml
version: 1                     # required
name: Plugin Name              # display name
description: What it computes  # shown in the UI

aggregate: per_worm            # per_worm | per_frame | spatial

bindings:                      # optional intermediate values (evaluated in order)
  name: expr

formula: expr                  # main computation (required)
filter: expr                   # optional — frames where this is 0 are excluded

reduce: mean                   # per_worm only: mean|median|sum|count|min|max|std|last

plot:
  type: box                    # box | bar | line | scatter
  y_label: "Label (px)"        # y-axis label for pixel units
  y_label_um: "Label (µm)"     # y-axis label for micrometer units
  x_label: "X axis label"      # x-axis label (line/scatter only)
```

### `aggregate` modes

| Mode | Description |
|---|---|
| `per_worm` | Produces one scalar per worm; typically used with box or bar plots |
| `per_frame` | Produces a time series per worm; typically used with line plots |
| `spatial` | Produces a binned 2D grid; used for heatmaps |

### `reduce` functions (per_worm only)

| Value | Description |
|---|---|
| `mean` | Arithmetic mean (default) |
| `median` | Median |
| `sum` | Sum of all values |
| `count` | Number of included frames |
| `min` | Minimum value |
| `max` | Maximum value |
| `std` | Standard deviation |
| `last` | Last value in the series |

---

## Built-in Plugins

| File | What it computes | Aggregate | Plot |
|---|---|---|---|
| `speed.yaml` | Mean centroid speed | per_worm | box |
| `speed_timeline.yaml` | Speed over time | per_frame | line |
| `speed_from_xy.yaml` | Speed from x/y displacement using `diff` + `smooth` | per_worm | box |
| `body_length.yaml` | Mean body length | per_worm | box |
| `body_area.yaml` | Mean blob area | per_worm | box |
| `aspect_ratio.yaml` | Mean bounding-box aspect ratio | per_worm | box |
| `total_distance.yaml` | Total path length (sum of per-frame `diff` steps) | per_worm | box |
| `distance_from_start_timeline.yaml` | Distance from start point over time | per_frame | line |
| `chemotaxis_index.yaml` | `(d_start − d_end) / (d_start + d_end)` directional bias | per_worm | box |
| `reversals.yaml` | Reversal count via velocity dot-product sign changes | per_worm | box |

---

## Example: Speed from XY Displacement

```yaml
version: 1
name: Speed from XY
description: Computes speed from centroid displacement using diff and smooth

aggregate: per_worm
reduce: mean

bindings:
  dt: diff(t)
  dx: diff(x)
  dy: diff(y)
  raw_xy_speed: sqrt(dx*dx + dy*dy) / dt
  xy_speed: smooth(raw_xy_speed, 2, quality != Lost && dt > 0 && raw_xy_speed > 0)

formula: xy_speed
filter: quality != Lost

plot:
  type: box
  y_label: "Speed (px/s)"
  y_label_um: "Speed (µm/s)"
```

## Example: Chemotaxis Index

Requires **Start** and **End** reference points to be configured in the project.

```yaml
version: 1
name: Chemotaxis Index
description: Directional bias — positive means worm is closer to end than start

aggregate: per_worm
reduce: mean

formula: (dist_to_start - dist_to_end) / (dist_to_start + dist_to_end)
filter: has_start && has_end && quality != Lost

plot:
  type: box
  y_label: "Chemotaxis Index"
  y_label_um: "Chemotaxis Index"
```
