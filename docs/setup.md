# Setup guide

This guide covers three ways to run the project. The **drone code is identical
in all of them**: only the machine that hosts the container changes.

| Path | When to use it |
|---|---|
| A. Local Ubuntu workstation + Docker (recommended) | You have an RTX GPU at home/university |
| B. AWS EC2 GPU instance + Docker ([budget guide](aws_budget_guide.md)) | No suitable local GPU |
| C. Native Isaac Sim install (no Docker) | Fast GUI iteration on a local workstation |

Host-only development (unit tests, planners, controllers, plots) works on
**any** machine with Python 3.11 — see [Section 4](#4-host-only-development-no-gpu).

> Pinned versions: **Isaac Sim 5.1.0**, **Pegasus Simulator v5.1.0**,
> Python 3.11 (bundled with Isaac Sim 5.x). Rationale: see
> [design_decisions.md](design_decisions.md).

---

## 0. Requirements

| | Minimum | Recommended |
|---|---|---|
| GPU | RTX-class NVIDIA GPU with RT cores, 8–12 GB VRAM for this project (NVIDIA's official minimum for 5.x: RTX 4080 16 GB) | RTX 4090 / L40S, >= 16 GB |
| RAM | 32 GB | 64 GB |
| CPU | 8 cores | >= 8 cores, high clock |
| Disk | 100 GB free SSD (image ~20 GB + shader/asset caches) | 200 GB NVMe |
| OS | Ubuntu 22.04 LTS (the version Pegasus v5.1.0 is tested on) | Ubuntu 22.04 LTS; 24.04 works for the container path |

GPUs **without RT cores** (A100, H100, V100, ...) cannot run Isaac Sim's
RTX renderer, which the LiDAR and cameras need.

Verify the official numbers and driver versions here (they change between releases):
- Isaac Sim requirements: <https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html>
- Isaac Sim container install: <https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_container.html>
- Pegasus installation: <https://pegasussimulator.github.io/PegasusSimulator/source/setup/installation.html>

Run the host diagnostics at any time:

```bash
python3 scripts/check_env.py
```

---

## A. Local Ubuntu workstation + Docker

### A.1 NVIDIA driver
```bash
sudo ubuntu-drivers list            # shows the recommended driver
sudo ubuntu-drivers install         # or: sudo apt install nvidia-driver-<version>
sudo reboot
nvidia-smi                          # must list your GPU
```
Use the driver branch recommended on the Isaac Sim requirements page for 5.1.0.

### A.2 Docker + NVIDIA Container Toolkit
```bash
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker "$USER" && newgrp docker
```
Install the toolkit following
<https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html>, then:
```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
docker run --rm --gpus all ubuntu nvidia-smi      # must show the GPU
```

### A.3 NGC access (pull the Isaac Sim image)
The image `nvcr.io/nvidia/isaac-sim:5.1.0` is public on NGC. If the pull is
refused, create a free NGC account, generate an API key at
<https://ngc.nvidia.com/setup/api-key> and log in:
```bash
docker login nvcr.io   # username: $oauthtoken   password: <your NGC API key>
```

### A.4 Build and run
```bash
git clone https://github.com/msant93/isaac-drone-nav.git
cd isaac-drone-nav
make docker-build          # ~20 GB download the first time
make smoke                 # headless takeoff -> hover -> land
./docker/run_gui.sh scripts/smoke_test_hover.py   # same, with the GUI window
```
The **first start takes several minutes** (shader compilation). Caches are
persisted in `docker/cache/`, so later starts are much faster.

---

## B. AWS EC2

Follow **[aws_budget_guide.md](aws_budget_guide.md)**: a step-by-step, low-cost
setup covering the account, GPU quota, `g4dn.2xlarge` with the free Deep Learning
Base AMI, auto-stop, the daily workflow, livestream and a cost checklist.

---

## C. Native install (no Docker, optional)

Follow the official workstation install for Isaac Sim 5.1.0 and then the
Pegasus guide. In short (check both pages for the exact commands):
```bash
# after unpacking Isaac Sim 5.1.0 into ~/isaacsim
export ISAACSIM_PATH="$HOME/isaacsim"
export ISAACSIM_PYTHON="$ISAACSIM_PATH/python.sh"
git clone --branch v5.1.0 https://github.com/PegasusSimulator/PegasusSimulator.git ~/PegasusSimulator
$ISAACSIM_PYTHON -m pip install -e ~/PegasusSimulator/extensions/pegasus.simulator
$ISAACSIM_PYTHON -m pip install -r requirements-isaac.txt
$ISAACSIM_PYTHON -m pip install --no-deps -e .
$ISAACSIM_PYTHON scripts/smoke_test_hover.py --gui
```
Pegasus v5.1.0 also introduces an `isaac_run` shell helper described in its
installation guide. The commands above use `python.sh` directly, which works too.

---

## 4. Host-only development (no GPU)

```bash
conda env create -f environment.yml
conda activate dronenav
make ci            # ruff + mypy + pytest, exactly what GitHub Actions runs
```

---

## 5. Common errors

| Symptom | Cause / fix |
|---|---|
| `could not select device driver "nvidia"` | NVIDIA Container Toolkit missing or Docker not restarted (A.2). |
| `unauthorized` when pulling `nvcr.io/...` | `docker login nvcr.io` with an NGC API key (A.3). |
| First run stuck for minutes on `Compiling shaders` / `RTX ready` missing | Normal on the first start; the cache in `docker/cache/` makes later starts fast. |
| `Permission denied` on `docker/cache/...` | The container runs as uid 1234: `sudo chown -R 1234:1234 docker/cache`. |
| `ModuleNotFoundError: pegasus` | You are using the host Python. Run with `/isaac-sim/python.sh` (container) or `$ISAACSIM_PYTHON`. |
| `ISAACSIM_PATH is not set!` while installing Pegasus | Export `ISAACSIM_PATH` before `pip install -e` (already done in the Dockerfile). |
| Long hang at start-up, or warnings about the asset root path | Pegasus queries NVIDIA's asset server on import. Check the internet connection or proxy; the smoke test itself does not need remote assets. |
| Extension `omni.isaac.dynamic_control` not found | Pegasus v5.1.0 still uses this deprecated extension. Report the log and we will enable it explicitly. |
| Isaac Sim crashes right after a `pip install` | A package upgraded numpy/scipy. Rebuild the image; always install with `-c /opt/isaac-constraints.txt`. |
| Black viewport in the WebRTC client | Ports blocked: check the security group / SSH tunnel and the port list for 5.1.0. |
| Smoke test `FAIL` with `hover_altitude_ok: false` | Send `results/smoke_test/report.json` and the console log, and we will retune the gains. |
