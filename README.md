# nav_path_recorder

Lifecycle node that records the path travelled by the robot. It collects `/loc/odom` poses, stores them as a 2D path with per-point speed and tool state, optionally smooths it, and saves it to a JSON file (`work_performed` Agri format) that can later be replayed by `nav_replay`.

**Node:** `path_recorder` · **Executable:** `nav_path_recorder_node`

## Overview

The node is a lifecycle node built around a `Path2D` accumulator:

1. **Configure** — read parameters, create the TF→WGS84 helper, declare the `path_performed` publisher (unused for now), and start a 10 s autosave timer.
2. **Activate** — resolve the WGS84 anchor (`lookup_tf_to_wgs84`); on failure activation is refused. Then subscribe to `/loc/odom` and start collecting points.
3. **Record** — on each `/loc/odom` message, append the `(x, y)` position, store the rounded linear speed (minimum magnitude `0.1` m/s) and the tool state for that point.
4. **Autosave** — every 10 s the current path is written to `path_performed.json`.
5. **Deactivate** — stop recording, smooth the path (`filtering(1)`), save it to `path_performed.json`, then clear the buffer.

The file is written in the working directory under the name `path_performed.json` with file type `work_performed` and the configured `vehicle_id`.

## Path2D

The geometry backend is `nav_util::Path2D`, provided by the `nav_util` package (included via `nav_util/path2d.hpp`). This node only drives it; `Path2D`:

- accumulates points with a minimal inter-point distance (`dist_min`, default `0.09 m`);
- stores per-point speed, tool state, and course;
- computes curvature over a sliding window (`active_window`, default `1.5 m`) using a least-squares fit (`nav_util::LeastSquares`, Eigen-based);
- smooths the path (`filtering`) and serializes it to/from the Agri JSON format.

## Parameters

| Parameter               | Default | Description                                                            |
| ----------------------- | ------- | ---------------------------------------------------------------------- |
| `path.filtering_degree` | `0`     | Declared but **not used yet**; deactivation applies `filtering(1)`     |
| `vehicle_id`            | —       | Vehicle identifier stored in the saved file (configure fails if empty) |

## Topics

| Topic       | Type                    | Direction | Description                                        |
| ----------- | ----------------------- | --------- | -------------------------------------------------- |
| `/loc/odom` | `nav_msgs/msg/Odometry` | In        | Robot pose and linear speed used to build the path |

The `/loc/odom` subscription is created in the **active** lifecycle state only.

A `path_performed` publisher is created at configure time but is not used yet; the recorded path is only written to disk.

## Output file

On deactivation (and every 10 s while active) the path is saved as `path_performed.json`:

- file type: `work_performed`
- WGS84 anchor: resolved from TF at activation
- per point: position, speed, tool state, curvature

This file is directly consumable by `nav_replay` as a mission to replay.
