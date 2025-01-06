# s_mapper_api

API for controlling and viewing live that coming from the SMapper device

## Install

### Using pip

```sh
python3 -m venv .venv && source .venv/bin/activate # Optional, requires python3-venv

pip install -r requirements.txt
```

### Using UV

First, make sure you have [UV](https://docs.astral.sh/uv/getting-started/installation/) installed.

Then,

```sh
uv sync && uv venv
source .venv/bin/activate
```

## Usage

For development purposes use

```sh
fastapi dev app/main.py
```

For production purposes use

```sh
fastapi run app/main.py
```
