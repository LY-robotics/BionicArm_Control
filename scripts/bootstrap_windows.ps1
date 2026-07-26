$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $PSScriptRoot
$VenvRoot = Join-Path $ProjectRoot ".venv"
$VenvPython = Join-Path $VenvRoot "Scripts\python.exe"

function Invoke-Checked {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Program,
        [Parameter(ValueFromRemainingArguments = $true)]
        [string[]]$Arguments
    )

    & $Program @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed with exit code ${LASTEXITCODE}: $Program $Arguments"
    }
}

Set-Location $ProjectRoot

if (-not (Test-Path $VenvPython)) {
    $PythonLauncher = Get-Command "py" -ErrorAction SilentlyContinue
    if ($null -ne $PythonLauncher) {
        Invoke-Checked "py" "-3.12" "-m" "venv" $VenvRoot
    }
    else {
        Invoke-Checked "python" "-m" "venv" $VenvRoot
    }
}

Invoke-Checked $VenvPython "-m" "pip" "install" "--upgrade" "pip" "setuptools" "wheel"
Invoke-Checked $VenvPython "-m" "pip" "install" "--editable" "."
Invoke-Checked $VenvPython "-m" "unittest" "discover" "-s" "tests" "-v"
Invoke-Checked $VenvPython "-m" "examples.smoke_test"

Write-Host ""
Write-Host "Installation complete."
Write-Host "Start simulation: $VenvPython arm_dashboard.py --simulate"
Write-Host "Start hardware UI: $VenvPython arm_dashboard.py"
