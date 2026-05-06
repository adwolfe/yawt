# Start/End Reimplementation Guide

This document is a handoff note for reimplementing the dedicated Start/End point workflow in a fresh YAWT checkout or branch.

It covers the changes we previously made at a feature level, the files that were touched, and the expected behavior once the work is re-added.

## Goal

Add a dedicated Start/End selection workflow that is separate from worm selection and ROI drawing.

The intended user flow is:

1. The user clicks a dedicated `SE` button near the top-left interaction controls.
2. The next click in the video places the `Start Point`.
3. The following click places the `End Point`.
4. After the second click, the app exits Start/End mode automatically and returns to normal pan mode.
5. Start/End coordinates are written into a separate sheet in the final tracking workbook.

This design avoids overloading the ROI tool and avoids interfering with the existing worm-click workflow.

## What Already Existed

Before the reimplementation, the codebase already had useful pieces in place:

- `TableItems::ItemType::StartPoint`
- `TableItems::ItemType::EndPoint`
- `TableItems::ItemType::ControlPoint`
- serialization/display support for those item types
- point rendering in `VideoLoader`

That means the reimplementation does not need to invent new item types. It mainly needs:

- a better UI for selecting Start/End explicitly
- controlled two-click logic
- export support

## High-Level Design

The cleanest version of this feature had three parts:

1. A dedicated UI button for Start/End selection
2. A small `MainWindow` state machine for the two-click sequence
3. Export of Start/End coordinates into a separate workbook sheet

## Part 1: Dedicated `SE` Button

### File

- `src/gui/mainwindow.ui`

### Intended change

Reuse the existing point-mode button and turn it into a dedicated Start/End tool.

Recommended button properties:

- visible text: `SE`
- tooltip: `Select Start/End points`
- status tip: `Click once for Start, then once for End`
- `checkable = true`
- `autoRaise = true`

### Why

The original generic point button was ambiguous. The dedicated `SE` label makes it obvious that this mode is for landmark placement, not worm editing.

## Part 2: `MainWindow` State for Two-Click Selection

### Files

- `src/gui/mainwindow.h`
- `src/gui/mainwindow.cpp`

### New state to add in `MainWindow`

Add two members:

```cpp
bool m_startEndSelectionActive = false;
TableItems::ItemType m_nextStartEndPointType = TableItems::ItemType::StartPoint;
```

### Purpose of these fields

- `m_startEndSelectionActive` tells the window that the next clicks should be interpreted as Start/End placement
- `m_nextStartEndPointType` tracks whether the next click should create `StartPoint` or `EndPoint`

### Button click behavior

The dedicated point button handler should no longer behave like a generic “add whatever point comes next” mode.

Instead, `pointModeButtonClicked()` should:

1. set `m_startEndSelectionActive = true`
2. set `m_nextStartEndPointType = TableItems::ItemType::StartPoint`
3. switch the video loader into `VideoLoader::InteractionMode::Point`
4. ensure the blob overlay is visible so the user can see the markers
5. show a status message like:
   `Start/End Mode: click the Start point, then the End point`

### Point placement behavior

`handlePointDefined(const QPointF& point)` should be changed from the old generic point-cycling behavior to a controlled two-step flow.

The intended logic is:

1. Ignore the callback if no video is loaded or the click is invalid.
2. Ignore the callback unless `m_startEndSelectionActive` is true.
3. If the next type is `StartPoint`:
   - remove or replace any previous start point
   - add the new point as `StartPoint`
   - set its color to green
   - change `m_nextStartEndPointType` to `EndPoint`
   - show a message like:
     `Start point set. Click the End point.`
4. If the next type is `EndPoint`:
   - remove or replace any previous end point
   - add the new point as `EndPoint`
   - set its color to red
   - set `m_startEndSelectionActive = false`
   - reset `m_nextStartEndPointType = TableItems::ItemType::StartPoint`
   - switch the video loader back to `PanZoom`
   - show a completion message like:
     `Start/End points updated.`

### Important behavior detail

Do not route Start/End placement through the ROI tool.

The user specifically wanted two separate control paths:

- worm selection remains unchanged
- Start/End placement happens only through the dedicated `SE` button

### Interaction-mode syncing

When the app leaves point mode for any reason, `syncInteractionModeButtons(...)` should also clear the temporary Start/End state:

```cpp
m_startEndSelectionActive = false;
m_nextStartEndPointType = TableItems::ItemType::StartPoint;
```

This prevents stale selection state if the user changes tools halfway through.

## Part 3: Marker Colors

### Files

- `src/gui/mainwindow.cpp`
- optionally `src/gui/widgets/videoloader.cpp` if you want dedicated drawing behavior there

### Intended colors

- Start point: green
- End point: red

The earlier implementation used item colors directly so the existing overlay machinery could render them without adding a new drawing path.

This is the simplest approach and keeps the feature localized.

## Part 4: Output Format Change

To preserve Start/End coordinates cleanly, the export was changed from a single CSV into an Excel workbook.

### Files

- `src/core/trackingmanager.h`
- `src/core/trackingmanager.cpp`
- `src/gui/mainwindow.cpp`
- `src/gui/widgets/videofiletreeview.cpp`

### Why workbook export was used

CSV is fine for one table, but Start/End coordinates were easier to preserve in a separate tab.

The final export structure became:

- `Tracks`
- `start-end coords`
- `Parameters`

## Part 5: `Tracks` Sheet

### Intended behavior

The main tracks export still contains worm track rows, including centerline output if that feature is present in the target branch.

A key export fix was:

- exported `WormID` should start at `1`
- add a separate `SourceItemID` column to preserve the original internal item ID

### Why this matters

Once Start/End points exist as items, internal IDs can be consumed before worm IDs appear. That can make the first tracked worm show up as `3` instead of `1` in export.

The workbook should hide that implementation detail from the end user.

Recommended leading columns:

- `WormID`
- `SourceItemID`
- `Frame`
- `PositionX`
- `PositionY`

Then keep the rest of the existing tracking columns.

## Part 6: `start-end coords` Sheet

### Intended sheet name

`start-end coords`

### Recommended columns

- `PointType`
- `SourceItemID`
- `FrameSelected`
- `PositionX`
- `PositionY`

### Expected rows

One row for the Start point and one row for the End point, if present.

Recommended `PointType` values:

- `Start Point`
- `End Point`

## Part 7: `Parameters` Sheet

### Intended sheet name

`Parameters`

### Recommended columns

- `Parameter`
- `Value`
- `Unit`

### Initial parameters to include

- `Capture Rate` with unit `fps`
- `Pixel Size` with unit `pixels/um`

This was added because those values are needed downstream and are part of the experimental context.

## Part 8: Pixel Size Plumbing

### Files

- `src/gui/mainwindow.cpp`
- `src/core/appcontroller.h`
- `src/core/appcontroller.cpp`
- `src/core/trackingmanager.h`
- `src/core/trackingmanager.cpp`

### Intended flow

The pixel size entered in the UI should be forwarded through the stack:

1. `MainWindow` reads the value from the pixel-size widget
2. `MainWindow` passes it to `AppController`
3. `AppController` passes it to `TrackingManager`
4. `TrackingManager` stores it and writes it into the workbook

Recommended API shape:

```cpp
void AppController::setPixelSizePixelsPerUm(double value);
void TrackingManager::setPixelSizePixelsPerUm(double value);
```

### Why this matters

If the export layer cannot see the UI-entered pixel size, the workbook cannot write a reliable `Parameters` sheet.

## Part 9: Workbook Writer Strategy

The earlier implementation wrote `.xlsx` directly without pulling in a new third-party spreadsheet library.

### Approach used

- create the workbook XML structure in a temporary directory
- write sheet XML files manually
- write workbook metadata files manually
- zip the package into `*_tracks.xlsx`

### Practical note

If a lightweight workbook writer already exists in the target branch, use it instead.

If not, the minimal direct-XML approach is acceptable and keeps dependencies unchanged.

## Part 10: Run-Loading Compatibility

### Files

- `src/gui/mainwindow.cpp`
- `src/gui/widgets/videofiletreeview.cpp`

### Intended behavior

When loading a completed run from disk, the UI should accept either:

- legacy `*_tracks.csv`
- new `*_tracks.xlsx`

This makes the transition safer while older run folders still exist.

## Suggested Reimplementation Order

1. Rework the `pointButton` into the dedicated `SE` button
2. Add the two new `MainWindow` state members
3. Update `pointModeButtonClicked()` to start the Start/End selection flow
4. Replace the generic point-cycling logic in `handlePointDefined(...)`
5. Add green/red coloring for Start and End
6. Test placement manually in the UI
7. Update export from CSV to workbook if this is part of the target branch
8. Add the `start-end coords` sheet
9. Add the `Parameters` sheet
10. Update run-loading to accept workbook output

## Manual Test Checklist

After reimplementation, verify all of the following:

1. Clicking `SE` enters point-placement mode.
2. The first click creates exactly one green Start point.
3. The second click creates exactly one red End point.
4. The app exits Start/End mode automatically after the second click.
5. Worm selection still behaves exactly as before.
6. ROI drawing still behaves exactly as before.
7. Replacing Start/End does not create duplicate landmarks.
8. The final workbook opens in Excel.
9. The workbook contains:
   - `Tracks`
   - `start-end coords`
   - `Parameters`
10. `WormID` starts at `1` in the `Tracks` sheet even if internal item IDs do not.
11. `start-end coords` contains the correct Start and End positions.
12. `Parameters` contains the UI-entered capture rate and pixel size.

## File Map Summary

Use this as the short reimplementation checklist:

- `src/gui/mainwindow.ui`
  - change point button to dedicated `SE` tool
- `src/gui/mainwindow.h`
  - add Start/End selection state members
- `src/gui/mainwindow.cpp`
  - start two-click selection on button press
  - implement Start/End placement logic
  - reset state when leaving point mode
- `src/core/appcontroller.h`
  - add pixel-size setter if exporting parameters
- `src/core/appcontroller.cpp`
  - forward pixel size to tracking manager
- `src/core/trackingmanager.h`
  - add workbook export helper declarations
  - add stored pixel-size field if exporting parameters
- `src/core/trackingmanager.cpp`
  - write workbook instead of CSV
  - add `Tracks` sheet
  - add `start-end coords` sheet
  - add `Parameters` sheet
  - export user-facing `WormID` starting at `1`
- `src/gui/widgets/videofiletreeview.cpp`
  - accept workbook runs during run discovery

## Final Notes

Two design choices were especially important and are worth preserving:

1. Start/End placement must stay separate from worm selection.
2. Start/End coordinates should live in their own workbook sheet rather than being mixed into the track rows.

Those two choices made the feature understandable in the UI and much easier to use downstream.
