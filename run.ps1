# ============================================
# DISCLAIMER !!!!!!!!!!:
# THIS POWERSHELL SCRIPT IS AI MADE. BECAUSE I HAVE NO ABSOLUTE IDEA 
# ABOUT THE SYNTAX OF POWERSHELL. MIGHT LEARN IN THE NEAR FUTURE. 
# OTHERWISE, USE AT YOUR OWN RISK!
# ============================================

# --------------------------------------------
# Define paths
# --------------------------------------------
$ScriptDir        = Split-Path -Parent $MyInvocation.MyCommand.Path
$ProjectRoot      = Join-Path $ScriptDir "GUI_SRC_CODE"
$EnvDir           = Join-Path $ProjectRoot ".conda_env"
$MambaDir         = Join-Path $ScriptDir "Library\bin"
$MambaExe         = Join-Path $MambaDir "micromamba.exe"
$MainScriptPath   = Join-Path $ProjectRoot "src\main.py"
$ReqFile          = Join-Path $ProjectRoot "requirements.txt"

# --------------------------------------------
# Download micromamba if missing
# --------------------------------------------
if (-not (Test-Path $MambaExe)) {
    Write-Host "[INFO] Downloading micromamba..." -ForegroundColor Cyan
    if (-not (Test-Path $MambaDir)) {
        New-Item -ItemType Directory -Path $MambaDir -Force | Out-Null
    }

    $TarPath = Join-Path $ScriptDir "micromamba.tar.bz2"
    Invoke-WebRequest -Uri "https://micro.mamba.pm/api/micromamba/win-64/latest" -OutFile $TarPath

    # Use tar (built into Windows 10+)
    tar -xvjf $TarPath -C $ScriptDir
    Remove-Item $TarPath

    # Move exe if it extracted to the wrong spot
    if (Test-Path (Join-Path $ScriptDir "micromamba.exe")) {
        Move-Item (Join-Path $ScriptDir "micromamba.exe") $MambaExe -Force
    }
}

# --------------------------------------------
# Create/Verify Python environment
# --------------------------------------------
$PythonExe = Get-ChildItem -Path $EnvDir -Filter "python.exe" -Recurse | Select-Object -First 1 -ExpandProperty FullName

if (-not $PythonExe) {
    Write-Host "[INFO] Creating environment with Micromamba..." -ForegroundColor Cyan
    
    # Use the most basic, robust argument set:
    # -y (yes), -p (prefix path), -c (channel), then packages
    & $MambaExe create -y -p "$EnvDir" -c conda-forge python=3.12 pip matplotlib numpy pandas
    
    if ($LASTEXITCODE -ne 0) {
        Write-Host "`n[ERROR] Micromamba failed with exit code $LASTEXITCODE" -ForegroundColor Red
        Write-Host "[DEBUG] Trying a fallback manual init..." -ForegroundColor Yellow
        # Fallback: Just try to get Python 3.12 alone if the full list failed
        & $MambaExe create -y -p "$EnvDir" -c conda-forge python=3.12
    }

    # Re-scan for the executable
    $PythonExe = Get-ChildItem -Path $EnvDir -Filter "python.exe" -Recurse | Select-Object -First 1 -ExpandProperty FullName
}

# --------------------------------------------
# Install pip requirements
# --------------------------------------------
if (Test-Path $ReqFile) {
    Write-Host "[INFO] Syncing dependencies from requirements.txt ..." -ForegroundColor Cyan
    & $PythonExe -m pip install -r $ReqFile
}

# --------------------------------------------
# Run MiR-GUI
# --------------------------------------------
Write-Host "[INFO] Environment ready. Python: $PythonExe" -ForegroundColor Green
Write-Host "[INFO] Running MiR-GUI... DO NOT CLOSE THIS WINDOW!" -ForegroundColor Yellow

& $PythonExe -u $MainScriptPath

# --------------------------------------------
# Keep window open on crash/exit
# --------------------------------------------
Write-Host "`n[INFO] Script finished. Press any key to exit..." -ForegroundColor Red
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")