# ============================================
# USE AT YOUR OWN RISK!
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



# ==================== Update Check Logic =======================================
Write-Host "[INFO] Checking GitHub for new commits..." -ForegroundColor Cyan

# ======================= Fetch the latest metadata from GitHub without downloading files yet ====================
git fetch origin main -q 2>$null


# Check the last exit code with $LASTEXITCODE
if ($LASTEXITCODE -eq 0) {
    # ============== Compare the local HEAD to the remote tracking branch ==================
    $LocalHash  = git rev-parse HEAD
    $RemoteHash = git rev-parse origin/main

if ($LocalHash -ne $RemoteHash) {
        Write-Host "=========================================" -ForegroundColor Yellow
        Write-Host "[UPDATE] New update detected on GitHub!" -ForegroundColor Yellow
        
        # --- NEW: User Input Logic ---
        $Confirmation = Read-Host "Would you like to update now? (y/n)" -ForegroundColor DarkYellow
        
        if ($Confirmation -eq 'y') {
            Write-Host "[INFO] Updating files... please wait and do not exit." -ForegroundColor Cyan
            git pull origin main
            Write-Host "[SUCCESS] Update complete!" -ForegroundColor Green
        } else {
            Write-Host "[SKIP] Update declined. Running current version..." -ForegroundColor Gray
        }
        # -----------------------------
        Write-Host "========================================="
    } else {
        Write-Host "[INFO] You are up to date." -ForegroundColor Green
    }
} else {
    Write-Host "[WARN] Could not reach GitHub or folder is not a Git repo." -ForegroundColor Gray
}


# --------------------------------------------
# Run MIR-GUI
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