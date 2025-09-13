# UR5 Torque Learning with MATLAB (Robotics + Deep Learning)

End‑to‑end MATLAB project that **generates training data via inverse dynamics** for the UR5 manipulator, **trains a neural network** to predict joint torques conditioned on a **desired end‑effector target**, and **deploys** the learned policy to move the robot toward a target in simulation.

> Works with **MATLAB R2022b+**, **Robotics System Toolbox**, and **Deep Learning Toolbox**.

---

## ✨ What’s inside

- **Data generation** using `inverseDynamics` on a full UR5 model
- **Neural net regressor** (feature input → FC(256) → BN → ReLU → FC(128) → Dropout → ReLU → FC(64) → ReLU → FC(6))
- **mapminmax** normalization persisted for deployment
- **Interactive visualization** of UR5, workspace, and trajectory
- **Two deployment modes**: live (from trained model in memory) and from a saved `.mat` model

---

## 📦 Repository structure

```
ur5-torque-learning/
├─ src/
│  ├─ 01_generate_data_and_train.m      % generate data, train NN, save myTrainedModel.mat
│  ├─ 02_deploy_demo.m                  % deploy immediately after training (no load)
│  ├─ 03_deploy_with_saved_model.m      % load myTrainedModel.mat and deploy
│  └─ 00_quick_visualizations.m         % UR5 & workspace quick plots
├─ utils/
│  └─ README.md
├─ figures/                             % put screenshots/renders here (ignored by git LFS rules)
├─ .gitignore
├─ LICENSE
└─ README.md  ← you are here
```

---

## 🧰 Requirements

- MATLAB **R2022b or newer** (older may work if you remove `'OutputNetwork','best-validation-loss'`)
- **Robotics System Toolbox**
- **Deep Learning Toolbox**

> Optional: **Git LFS** if you plan to commit large `.mat` model files (> 50–100MB).

---

## 🚀 Quickstart

1) **Clone or download** this repo.

2) Open MATLAB in the repo root and run the training script:
```matlab
cd src
run('01_generate_data_and_train.m')
```
This will:
- load the UR5, visualize it and the workspace,
- generate **5,000** random samples,
- train the network,
- save `myTrainedModel.mat` with `net`, `inputPs`, `outputPs` in the project root.

3) **Deploy** the trained model (two options):
```matlab
% Option A: deploy immediately after training
run('02_deploy_demo.m')

% Option B: load from the saved .mat file and deploy
run('03_deploy_with_saved_model.m')
```

You should see console logs like *“Target reached successfully!”* and a **3D trajectory plot** toward the target.

---

## 📝 Notes & Tips

- If you’re on an older MATLAB version, remove this option from `trainingOptions`:
  ```matlab
  'OutputNetwork','best-validation-loss'
  ```
- To change the random dataset size, edit `numSamples` in the training script.
- Targets are sampled in `[0,1]^3`. Adjust target ranges as needed.
- Torque clamping is set to `[-1, 1]` during deployment for stability. Tune as needed.
- For larger models or datasets, consider **Git LFS**:
  ```bash
  git lfs install
  git lfs track "*.mat"
  git add .gitattributes
  ```

---

## 📸 Suggested screenshots (put in `figures/`)
- UR5 initial visualization (`00_quick_visualizations.m`)
- Training progress screenshot
- End-effector trajectory toward the target

---

## 📄 License

This project is released under the **MIT License** (see `LICENSE`).

---

## 🙌 Acknowledgments

- MATLAB **Robotics System Toolbox** for the UR5 model (`loadrobot('universalUR5', ...)`).
- MATLAB **Deep Learning Toolbox** for training utilities.
