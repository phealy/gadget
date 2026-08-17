# Team Gadget Python code

This repository hosts Python code for FLL Team Gadget.

> **Note:** This code will not function in the LEGO Education SPIKE app. It depends on the VS Code extension’s ability to merge the required `gadget.py` file into the program.

## Setup

Install [uv](https://docs.astral.sh/uv/) on Windows:

```powershell
winget install --id=astral-sh.uv -e
```

Install the project’s Python version, create a virtual environment, and install the pip package:

```powershell
uv python install
uv venv
uv pip install .
```

Activate the environment:

```powershell
.venv\Scripts\Activate.ps1
```

## Visual Studio Code Extensions

Download and install the following VS Code extensions.

From the extension marketplace:

- [Python](https://marketplace.visualstudio.com/items?itemName=ms-python.python)
- [Pylance](https://marketplace.visualstudio.com/items?itemName=ms-python.vscode-pylance)

Manually, as a VS Code VSIX file:

- [LEGO SPIKE Prime/MINDSTORMS](https://github.com/phealy/lego-spikeprime-mindstorms-vscode/releases/latest/)
