# Technical Test

A Python controller for the JVRC1 humanoid robot using the mc_rtc framework. The robot sequentially moves its left hand, right hand, then both hands to target positions while looking in the direction of the active hand.

Steps to follow to reproduce the results:

#### 1. Clone and Start the Dev Container

```bash
git clone git@github.com:mc-rtc/mc-rtc-superbuild.git
cd mc-rtc-superbuild
```

Open the folder in VSCode. Select the `jammy` (Ubuntu 22.04) image.

(Build `mc_rtc` using the `BUILD_PARALLEL_JOBS=1` to avoid RAM and CPU usage spikes and avoid crashes) 

```bash
cd /home/vscode/superbuild
cmake --preset relwithdebinfo -D BUILD_PARALLEL_JOBS=1
cmake --build --preset relwithdebinfo
```

(This takes a while)

#### 2. Build mc-rtc-magnum (Visualization)

RViz didn't seem to work without the GPU passthrough. Rebuilding the containers was taking a while. So, I installed `mc-rtc-magnum` instead. 


```bash
vim /home/vscode/superbuild/extensions/mc_rtc-magnum.cmake
```

Paste:

```cmake
AddProject(mc_rtc-magnum
  GITHUB mc-rtc/mc_rtc-magnum
  GIT_TAG main
  CMAKE_ARGS -DCMAKE_BUILD_TYPE=RelWithDebInfo
)
```

(I had to insteall some more X11 dependencies to be able to build):

```bash
sudo apt update && sudo apt install -y libxinerama-dev libxcursor-dev libxi-dev libxrandr-dev
```
```
cd /home/vscode/superbuild
cmake --preset relwithdebinfo
cmake --build --preset relwithdebinfo --target mc_rtc-magnum
```

#### 4. Source the Environment

Run this in every new terminal (or add it to `~/.bashrc`):

```bash
source /home/vscode/workspace/install/setup_mc_rtc.sh
```

#### 5. Clone the Repo

```bash
cd /home/vscode/workspace
git clone https://github.com/shiroe008/JRT_Test.git my_first_controller
```

#### 6. Configure mc_rtc

```bash
mkdir -p ~/.config/mc_rtc
vim ~/.config/mc_rtc/mc_rtc.yaml
```

```yaml
MainRobot: JVRC1
Enabled:
  - Python3#my_first_controller.MyFirstController
```

#### 7. Run the Controller

You need two terminals. Source the environment in both:

```bash
source /home/vscode/workspace/install/setup_mc_rtc.sh
```

**Terminal 1 - Controller:**

```bash
PYTHONPATH=/home/vscode/workspace:$PYTHONPATH mc_rtc_ticker
```

**Terminal 2 - Visualizer:**
(Use software rendering)

```bash
export LIBGL_ALWAYS_SOFTWARE=1
mc-rtc-magnum
```

## Controller Behavior

The controller cycles through 6 phases:

| Phase | Action | Head |
|---|---|---|
| 0 | Left hand moves to (0.5, 0.25, 1.1) | Looks left |
| 1 | Left hand returns to initial position | Looks forward |
| 2 | Right hand moves to (0.5, -0.25, 1.1) | Looks right |
| 3 | Right hand returns to initial position | Looks forward |
| 4 | Both hands move to their targets | Looks forward |
| 5 | Both hands return to initial positions | Looks forward |

Then repeats from phase 0.


