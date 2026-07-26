#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
venv_root="${project_root}/.venv"
python_bin="${PYTHON_BIN:-python3}"

cd "${project_root}"

"${python_bin}" -c '
import sys
if not ((3, 10) <= sys.version_info[:2] < (3, 13)):
    raise SystemExit(
        f"Python 3.10-3.12 required, found {sys.version.split()[0]}"
    )
'

if [[ ! -x "${venv_root}/bin/python" ]]; then
    "${python_bin}" -m venv "${venv_root}"
fi

"${venv_root}/bin/python" -m pip install --upgrade pip setuptools wheel
"${venv_root}/bin/python" -m pip install --editable .
"${venv_root}/bin/python" -m unittest discover -s tests -v
"${venv_root}/bin/python" -m examples.smoke_test

printf '\nInstallation complete.\n'
printf 'Start simulation: %q arm_dashboard.py --simulate\n' "${venv_root}/bin/python"
printf 'Start hardware UI: %q arm_dashboard.py\n' "${venv_root}/bin/python"
