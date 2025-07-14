# Zenith Server

The backend server for Zenith. It handles launch parsing, creation, and subsystem detection.

### Running via Docker

```bash
docker compose run zenith_backend_dev

# Within Docker container run:
python3 -m venv venv
source venv/bin/activate # Different for MacOS and Windows
pip install -r requirements.txt

npm run dev:api
# OR
uvicorn api:app --reload --log-config=log_conf.yaml
```

### Running Locally

<details open>
<summary>
Pre-requisites
</summary> <br />
To be able to start development on Zenith, make sure that you have the following prerequisites installed:

- [Node.js (v18+)](https://nodejs.org/en/download) (if running via NPM, optional)
- [Python 3.10](https://www.python.org/downloads/)
</details>

<details open>
<summary>
Running Zenith
</summary> <br />

```bash
python3 -m venv venv
source venv/bin/activate # Different for MacOS and Windows
pip install -r requirements.txt

npm run dev:api
# OR
uvicorn api:app --reload --log-config=log_conf.yaml
```

</details>
