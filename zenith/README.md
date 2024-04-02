# Zenith

Navigator's launch GUI.

## Usage

Run the following in `navigator` root.

```bash
# xhost +local:root on Ubuntu to allow access to display server
docker compose up zenith
```

## Development (Quad)

<details open>
<summary>
Pre-requisites
</summary> <br />
Navigator needs to be built for Zenith server to detect systems.

```bash
# In navigator root:
docker compose run navigator_carla
# Inside navigator_carla container:
colcon build --symlink-install
```

</details>

<details open>
<summary>
Running Zenith
</summary> <br />

1. Run Zenith backend dev container and setup

```bash
# From navigator root:
docker compose run zenith_backend_dev

# Install dependencies
python3 -m venv/bin/activate
pip install -r requirements.txt

# Source navigator build
. /navigator/install/setup.bash

# Run
npm run dev:api

# Alternatively:
uvicorn api:app --reload --log-config=log_conf.yaml
```

2. Run Zenith GUI container and setup

```bash
# Run container
docker compose up zenith_dev

# Install dependencies
npm install

# Run GUI
npm run tauri dev
```

</details>

## Development (local)

<details open>
<summary>
Pre-requisites
</summary> <br />
To be able to start development on Zenith, make sure that you have the following prerequisites installed:

- [Node.js (v18+)](https://nodejs.org/en/download)
- [Python 3.10](https://www.python.org/downloads/)
- [Rust Toolchain](https://www.rust-lang.org/tools/install)
- [Docker](https://docs.docker.com/engine/install/)
- [Git](https://git-scm.com/downloads)
</details>

<details open>
<summary>
Running Zenith
</summary> <br />

1. Setup and run backend server

```bash
cd zenith/server
# Setup Python venv
python -m venv venv
source venv/bin/activate # Or OS equivalent (different for Windows and MacOS).
# Install dependencies
pip install -r requirements.txt
# Run development server
NAVIGATOR_SRC=/CHANGE/navigator/src npm run dev:api
```

2. Run Tauri GUI

```bash
cd zenith
# Install dependencies
npm install
# Run GUI
LAUNCH_DIR=/CHANGEME/navigator/launches npm run tauri dev

# LAUNCH_DIR=/navigator/launches when running zenith_backend via Docker
```

</details>

## Architecture

Zenith uses simple client-server architecture. The server serves as a simple way for GUI to request ROS2 information, read and modify launch files, and create interactive terminals.

```bash
.
├── server                  # API server
│   ├── api.py              # API entrypoint
│   ├── launch              # Launch file parsing and generation
│   ├── routers             # API routes
│   └── subsystem.py        # Subsystem parsing
├── src                     # Web server
│   ├── lib
│   │   ├── api             # API bindings
│   │   ├── modules         # Large UI components (launch editor, list, terminal, etc.)
│   │   ├── ui              # UI components library
│   │   ├── stores          # Applications states
│   │   └── utils.ts        # Utility functions
│   └── routes              # Routes
│       ├── +layout.svelte
│       ├── +layout.ts
│       └── +page.svelte
├── src-tauri               # GUI Rust entrypoint
│   └── tauri.conf.json     # Tauri configurations
├── static                  # Svelte static assets
```
