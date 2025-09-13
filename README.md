# UR5 ML Path Planning & Torque Prediction (MATLAB)

A clean, reproducible MATLAB project that:

- Generates synthetic UR5 joint-state datasets
- Trains a feed-forward neural network to predict joint torques via inverse dynamics
- Deploys the trained model to iteratively move the UR5 toward a Cartesian target
- Visualizes the robot, workspace, and end-effector trajectory

---

## ✨ Features

- **End-to-end pipeline**: data generation → normalization → training → deployment
- **Conditioned control**: model takes desired end-effector target `(x, y, z)` as input
- **Clear visualizations**: UR5 model, semisphere workspace, and 3D trajectory
- **Modular scripts**: `ur5Pathplanning.m` (train + deploy) and `testModel.m` (deploy from saved model)
- **Binaries ignored**: `.mat`, figures, and videos are gitignored; use **Git LFS** if you plan to version large files

---

## 🧰 Requirements

- **MATLAB** R2022b or newer (recommended)
- Toolboxes:
  - Robotics System Toolbox (for `loadrobot`, `getTransform`, kinematics)
  - Deep Learning Toolbox (for `trainNetwork`, layers)
  - Optimization Toolbox *(optional, for other experiments)*


---

## 📁 Project Structure

```
ur5-ml-path-planning/
├─ src/
│  ├─ ur5Pathplanning.m   # data gen + training + deployment
│  └─ testModel.m         # load saved model and deploy
├─ models/                # place `myTrainedModel.mat` here (gitignored)
├─ demo/              # videos or demo notebooks (gitignored)
├─ docs/
│  └─ getting-started.md
├─ .gitignore
├─ LICENSE
└─ README.md
```

---

## 🚀 Quick Start

1. **Clone** and open the project in MATLAB.
2. Ensure required toolboxes are installed.
3. Run the end-to-end script:
   ```matlab
   cd src
   ur5Pathplanning
   ```
   This will:
   - Visualize UR5 & workspace
   - Generate `numSamples` synthetic samples
   - Train the network (Adam optimizer)
   - Attempt a deployment toward a demo `targetPos`
   - Print test MSE

4. **(Optional)** Save the trained network to `models/myTrainedModel.mat`:
   ```matlab
   save('../models/myTrainedModel.mat','net','inputPs','outputPs','-v7.3');
   ```

5. **Test-only deployment** (uses saved model):
   ```matlab
   cd src
   testModel
   ```

---

## 🧪 Notes on Stability & Movement

- The deployment loop uses **simple Euler integration** of predicted torques (no full dynamics sim).  
  Large steps may destabilize motion. Tune:
  - `timeStep` (e.g., 0.001–0.02)
  - Torque clamp range (e.g., `[-1, 1]` → widen cautiously)
  - Stop criteria: `finalDist <= 0.1`, `configThreshold`

- Because the training uses **random joint states** and **inverse dynamics**, ensure your
  UR5 frames (e.g., `tool0`, `base`) correspond to the `loadrobot('universalUR5',...)` defaults.

---

## 🧩 Troubleshooting

- **`loadrobot` not found** → Install/enable *Robotics System Toolbox*.
- **No movement** → Reduce `timeStep`, widen clamp, or initialize a closer `targetPos`.
- **Model not learning** → Increase `MaxEpochs`, neurons, datasets (`numSamples`), or learning rate.
- **MATLAB Online** → Make sure the model `.mat` path is correct; use full paths if needed.

---

