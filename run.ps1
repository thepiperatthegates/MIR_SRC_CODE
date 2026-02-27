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
    Write-Host "[INFO] Downloading micromamba binary directly..." -ForegroundColor Cyan
    
    # Ensure the destination directory (Library\bin) exists
    if (-not (Test-Path $MambaDir)) {
        New-Item -ItemType Directory -Path $MambaDir -Force | Out-Null
    }

    # Direct link to the latest standalone Windows 64-bit executable
    $DirectUrl = "https://github.com/mamba-org/micromamba-releases/releases/latest/download/micromamba-win-64"

    try {
        Invoke-WebRequest -Uri $DirectUrl -OutFile $MambaExe -ErrorAction Stop
        Write-Host "[SUCCESS] Micromamba binary ready at $MambaExe" -ForegroundColor Green
    }
    catch {
        Write-Host "[ERROR] Failed to download micromamba: $($_.Exception.Message)" -ForegroundColor Red
        Write-Host "Please check your internet connection or GitHub access." -ForegroundColor Yellow
        pause
        exit 1
    }
}

# --------------------------------------------
# Create/Verify Python environment
# --------------------------------------------
# Check if python.exe exists in the environment directory
$PythonExe = Get-ChildItem -Path $EnvDir -Filter "python.exe" -Recurse | Select-Object -First 1 -ExpandProperty FullName

if (-not $PythonExe) {
    Write-Host "[INFO] Creating environment at $EnvDir ..." -ForegroundColor Cyan
    
    # Run creation: -y (yes), -p (prefix), -c (channel)
    & $MambaExe create -y -p "$EnvDir" -c conda-forge python=3.12 pip matplotlib numpy pandas
    
    if ($LASTEXITCODE -ne 0) {
        Write-Host "[ERROR] Micromamba failed. Attempting minimal fallback (Python only)..." -ForegroundColor Yellow
        & $MambaExe create -y -p "$EnvDir" -c conda-forge python=3.12
    }

    # Re-scan for the executable
    $PythonExe = Get-ChildItem -Path $EnvDir -Filter "python.exe" -Recurse | Select-Object -First 1 -ExpandProperty FullName
}

# --------------------------------------------
# Install pip requirements
# --------------------------------------------
if ($PythonExe -and (Test-Path $ReqFile)) {
    Write-Host "[INFO] Syncing dependencies from requirements.txt ..." -ForegroundColor Cyan
    & $PythonExe -m pip install -r $ReqFile
}

# --------------------------------------------
# Run MiR-GUI
# --------------------------------------------
if ($PythonExe) {
    Write-Host "[INFO] Environment ready. Python: $PythonExe" -ForegroundColor Green
    Write-Host "[INFO] Running MiR-GUI... DO NOT CLOSE THIS WINDOW!" -ForegroundColor Yellow
    & $PythonExe -u $MainScriptPath
} else {
    Write-Host "[ERROR] Python environment could not be initialized." -ForegroundColor Red
}

# --------------------------------------------
# Keep window open on crash/exit
# --------------------------------------------
Write-Host "`n[INFO] Script finished. Press any key to exit..." -ForegroundColor Red
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")