# Getting Started

## Verify Toolboxes
In MATLAB:
```matlab
ver
```
Look for **Robotics System Toolbox** and **Deep Learning Toolbox**.

## Training vs. Testing
- `ur5Pathplanning.m` → full pipeline (data gen + training + deploy).
- `testModel.m` → deployment-only; expects `models/myTrainedModel.mat` with `net, inputPs, outputPs`.

## Large Files
- Use **Git LFS** for `*.mat`, `*.mp4`, etc.
- This repo ships with a `.gitignore` that excludes typical binaries by default.