# Ubuntu24 Jazzy Bootstrap Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a repeatable one-command deployment path for a fresh Ubuntu 24.04 machine to install ROS 2 Jazzy, clone this workspace, install dependencies, build, and run self-checks.

**Architecture:** Keep deployment assets in `tools/` and user-facing instructions in `docs/`. The shell script is idempotent where practical: it can install missing apt packages, initialize rosdep if needed, clone or update the repository, configure shell environment, build, and optionally launch navigation.

**Tech Stack:** Bash, apt, ROS 2 Jazzy deb packages, rosdep, colcon, Git, FastDDS XML profile, existing ROS workspace layout.

---

### Task 1: Deployment Assets

**Files:**
- Create: `tools/apt_dependencies_ubuntu24_jazzy.txt`
- Create: `tools/fastdds_shm.xml`
- Create: `tools/bootstrap_ubuntu24_jazzy.sh`

- [x] **Step 1: Add a package-list file**

Create a newline-delimited apt dependency list so the shell script stays readable and the dependency set can be maintained without editing control flow.

- [x] **Step 2: Add the bootstrap script**

Implement option parsing for `--workspace`, `--repo`, `--branch`, `--skip-oneapi`, `--no-build`, and `--run-navigation`. The script must check Ubuntu 24.04, install ROS 2 Jazzy from official deb packages, install project apt/Python dependencies, clone or update the repository, run rosdep with known workspace-local skip keys, configure FastDDS and `.bashrc`, build with colcon, and run package-level self-checks.

- [x] **Step 3: Add a FastDDS XML template**

Create `tools/fastdds_shm.xml` and have the script copy it to `~/.config/fastdds_shm.xml`.

- [x] **Step 4: Syntax check the script**

Run: `bash -n tools/bootstrap_ubuntu24_jazzy.sh`

Expected: exit code 0.

### Task 2: Documentation

**Files:**
- Create: `docs/deployment_ubuntu24_jazzy.md`

- [x] **Step 1: Write user-facing deployment instructions**

Document the one-command usage, common options, what the script installs, how to resume after failure, and what remains hardware/site-specific.

- [x] **Step 2: Cite the official ROS 2 Jazzy installation source**

Reference the official ROS 2 Jazzy Ubuntu deb package docs because the install flow depends on current ROS apt packaging.

### Task 3: Portability Fix

**Files:**
- Modify: `start_navigation.sh`
- Modify: `start_mapping.sh`

- [x] **Step 1: Replace hard-coded FastDDS config default**

Change `/home/ubuntu/.config/fastdds_shm.xml` defaults to `$HOME/.config/fastdds_shm.xml` so a workspace deployed under a non-`ubuntu` account can still use the generated FastDDS profile.

- [x] **Step 2: Shell syntax check**

Run: `bash -n start_navigation.sh start_mapping.sh start_websocket.sh tools/bootstrap_ubuntu24_jazzy.sh`

Expected: exit code 0.

### Task 4: Verification

**Files:**
- Verify the full diff with `git diff --check`
- Confirm changed files with `git status --short`

- [x] **Step 1: Static verification**

Run shell syntax checks and whitespace checks. Do not run the bootstrap script on the current machine because it installs system packages and mutates apt sources.

- [x] **Step 2: Report limitations**

State that the script was statically verified locally, while full end-to-end verification must happen on a fresh Ubuntu 24.04 machine with network and sudo access.
