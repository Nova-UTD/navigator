# Zenith

Navigator's launch GUI.

## Usage

Run the following in `navigator` root.

```bash
docker compose up zenith
```

## Development

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
npm run dev:api
```

2. Run Tauri GUI

```bash
cd zenith
# Install dependencies
npm install
# Run GUI
npm run tauri dev
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
│   │   ├── components      # UI components
│   │   │   └── ui
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
