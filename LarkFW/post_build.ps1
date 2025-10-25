<#
.SYNOPSIS
    A utility to automate the firmware artifact generation and staging process.

.DESCRIPTION
    This script provides a one-command solution for post-build packaging.
    It automatically discovers the project's .elf artifact, converts it to
    a "Full" HEX file (entire chip) and a "Slim" HEX file (program data only),
    and stages them in a release directory.

.PARAMETER BuildConfiguration
    Specifies the build configuration (e.g., 'Debug' or 'Release') passed by the IDE.
#>

# ============================================================================
# ==                       SCRIPT PARAMETERS                                ==
# ============================================================================
param(
    [string]$BuildConfiguration = 'Release'
)

# ============================================================================
# ==                       CONFIGURATION SECTION                            ==
# ============================================================================
$ScriptVersion        = "2.6-FullSlimNaming" # Version for the script header
$PSScriptRoot         = Split-Path -Parent $MyInvocation.MyCommand.Path
$BuildDirectory       = Get-Location

# --- Tool and Staging Locations ---
$ToolsDirectory       = Join-Path -Path $PSScriptRoot -ChildPath "Tools"
$HexToolPath          = Join-Path -Path $ToolsDirectory -ChildPath "hex_tool.exe"
$DestinationDirectory = Join-Path -Path $PSScriptRoot -ChildPath "public_release"

# --- Visual Flair ---
$Flair = @{
    SummaryColor = "Yellow"
}

# ============================================================================
# ==                       HELPER FUNCTIONS                                 ==
# ============================================================================

function Write-ScriptHeader {
    param(
        [string]$Title,
        [string]$Author = "HamSlices 2025",
        [string]$Version
    )
    $width = 65; $titleColor = $Flair.SummaryColor; $textColor = "White";
    $topBorder = "+$("=" * ($width - 2))+"
    $separator = "+$("-" * ($width - 2))+"
    $bottomBorder = $topBorder
    $paddingWidth = $width - 2 - $Title.Length
    $leftPad = [math]::Floor($paddingWidth / 2)
    $rightPad = [math]::Ceiling($paddingWidth / 2)
    $titleLine = "|{0}{1}{2}|" -f (' ' * $leftPad), $Title, (' ' * $rightPad)
    $infoLine = " Author: $Author | Version: $Version "
    $infoPadding = ' ' * ($width - 2 - $infoLine.Length)
    $formattedInfoLine = "|{0}{1}|" -f $infoLine, $infoPadding
    Write-Host ""; Write-Host $topBorder -ForegroundColor $titleColor; Write-Host $titleLine -ForegroundColor $textColor; Write-Host $separator -ForegroundColor $titleColor; Write-Host $formattedInfoLine -ForegroundColor $textColor; Write-Host $bottomBorder -ForegroundColor $titleColor; Write-Host ""
}

function Get-ProgramSizeFromElf {
    param ([string]$Elf)
    
    if (-not (Get-Command "arm-none-eabi-objdump" -ErrorAction SilentlyContinue)) {
        throw "arm-none-eabi-objdump not found in PATH."
    }

    $quotedElfPath = if ($Elf -like '* *') { "`"$Elf`"" } else { $Elf }
    $objdumpOutput = & arm-none-eabi-objdump -t $quotedElfPath 2>&1
    if ($LASTEXITCODE -ne 0) { throw "arm-none-eabi-objdump failed: $objdumpOutput" }

    function Get-SymbolAddress {
        param ($SymbolName, [string[]]$Lines)
        $pattern = "^([0-9a-f]{8})\s.*\s$($SymbolName)\s*$"
        foreach ($line in $Lines) {
            if ($line -match $pattern) {
                return [Convert]::ToInt32($matches[1], 16)
            }
        }
        throw "Symbol '$SymbolName' not found in ELF file."
    }

    $sidataAddr = Get-SymbolAddress -SymbolName "_sidata" -Lines $objdumpOutput
    $sdataAddr  = Get-SymbolAddress -SymbolName "_sdata"  -Lines $objdumpOutput
    $edataAddr  = Get-SymbolAddress -SymbolName "_edata"  -Lines $objdumpOutput
    
    $startAddress = 0x08000000
    $dataSizeBytes = $edataAddr - $sdataAddr
    $endAddress = $sidataAddr + $dataSizeBytes
    $totalSize = $endAddress - $startAddress
    
    return $totalSize
}

function Invoke-CommandAndCheck {
    param ([string]$Command, [string[]]$Arguments, [switch]$CaptureOutput)
    $CommandName = Split-Path -Path $Command -Leaf
    $quotedArgs = @()
    foreach ($arg in $Arguments) {
        if ($arg -like '* *' -and -not ($arg.StartsWith('"') -and $arg.EndsWith('"'))) {
            $quotedArgs += "`"$arg`""
        } else {
            $quotedArgs += $arg
        }
    }
    Write-Host "    -> Executing: $CommandName $($quotedArgs -join ' ')" -ForegroundColor Gray
    if ($CaptureOutput) {
        $process = Start-Process -FilePath $Command -ArgumentList $quotedArgs -Wait -NoNewWindow -PassThru -RedirectStandardOutput "temp_stdout.txt"
        $exitCode = $process.ExitCode
        $output = Get-Content "temp_stdout.txt" -Raw
        Remove-Item "temp_stdout.txt"
    } else {
        $process = Start-Process -FilePath $Command -ArgumentList $quotedArgs -Wait -NoNewWindow -PassThru
        $exitCode = $process.ExitCode
    }
    if ($exitCode -ne 0) { throw "$CommandName failed with exit code $exitCode. Halting script." }
    if ($CaptureOutput) { return $output }
}

# ============================================================================
# ==                      MAIN SCRIPT LOGIC                                 ==
# ============================================================================
# Define temp file paths as null initially for the 'finally' block
$TempFullHexFile = $null
$TempSlimHexFile = $null
$TempBinFile = $null

try {
    Clear-Host
    Write-ScriptHeader -Title "Lark Post-Build & Package Utility" -Version $ScriptVersion
    Write-Host "[INFO] Starting process for '$BuildConfiguration' configuration." -ForegroundColor "Gray"

    # === STEP 1: AUTO-DISCOVER ELF ARTIFACT AND DEFINE FILENAMES ===
    Write-Host "[1/4] Discovering build artifact..."
    $elfFiles = Get-ChildItem -Path $BuildDirectory -Filter "*.elf"
    if ($elfFiles.Count -eq 0) { throw "Build failed: No .elf file found in the output directory '$BuildDirectory'." }
    if ($elfFiles.Count -gt 1) { throw "Build ambiguous: Found multiple .elf files in '$BuildDirectory'. Please clean the project and rebuild." }
    
    $SourceElfFile = $elfFiles[0].FullName
    $ElfBaseName   = $elfFiles[0].BaseName
    Write-Host "    (OK) Found artifact: $ElfBaseName.elf" -ForegroundColor Green

    # --- CHANGE: Use "Full" and "Slim" naming convention ---
    $TempFullHexFile = Join-Path -Path $BuildDirectory -ChildPath "$($ElfBaseName)_Full.hex"
    $TempSlimHexFile = Join-Path -Path $BuildDirectory -ChildPath "$($ElfBaseName)_Slim.hex"
    $TempBinFile = Join-Path -Path $BuildDirectory -ChildPath "$($ElfBaseName)_Slim.bin" # Intermediate file
    $StagedFullHexFile = Join-Path -Path $DestinationDirectory -ChildPath "$($ElfBaseName)_Full.hex"
    $StagedSlimHexFile = Join-Path -Path $DestinationDirectory -ChildPath "$($ElfBaseName)_Slim.hex"

    # === STEP 2: VERIFY TOOLS ===
    Write-Host "[2/4] Verifying tools..."
    if (-not (Test-Path $HexToolPath)) { throw "'$HexToolPath' not found. Please create a 'Tools' folder and place it there." }
    Write-Host "    (OK) All required tools found." -ForegroundColor Green

    # === STEP 3: CONVERT ELF, GENERATE HASH, AND CREATE PARTIAL HEX ===
    Write-Host "[3/4] Generating firmware artifacts and capturing build hash..."
    # 3a. Create the full HEX file from the ELF
    Invoke-CommandAndCheck -Command "arm-none-eabi-objcopy" -Arguments @("-O", "ihex", $SourceElfFile, $TempFullHexFile)
    
    $ProgramSize = Get-ProgramSizeFromElf -Elf $SourceElfFile
    if ($ProgramSize -le 0) { throw "Invalid program size calculated: $ProgramSize" }
    $SizeHex = ("0x{0:X}" -f $ProgramSize)
    Write-Host "    (INFO) Calculated program size: $SizeHex"
    
    # 3b. Create the slim BIN file (and get the hash) using the custom tool
    $hexToolOutput = Invoke-CommandAndCheck -Command $HexToolPath -Arguments @($TempFullHexFile, $TempBinFile, "0x08000000", $SizeHex) -CaptureOutput
    if ($hexToolOutput -match 'Generated Hash:\s*(0x[0-9A-Fa-f]+)') {
        $buildHash = $matches[1]
        Write-Host "    (OK) Captured build-time hash: $buildHash" -ForegroundColor Green
    } else {
        throw "Could not parse hash from hex_tool.exe output. Ensure it prints 'Generated Hash: 0x...'"
    }

    # 3c. Convert the slim BIN file back into a slim HEX file
    $StartAddress = "0x08000000"
    Invoke-CommandAndCheck -Command "arm-none-eabi-objcopy" -Arguments @("-I", "binary", "-O", "ihex", "--change-address", $StartAddress, $TempBinFile, $TempSlimHexFile)
    # --- CHANGE: Update log message ---
    Write-Host "    (OK) Generated slim HEX file." -ForegroundColor Green

    # === STEP 4: STAGE FIRMWARE ARTIFACTS ===
    Write-Host "[4/4] Staging firmware in '$DestinationDirectory'..."
	if (-not (Test-Path $DestinationDirectory)) {
		New-Item -ItemType Directory -Force -Path $DestinationDirectory | Out-Null
	}
	Copy-Item -Path $TempFullHexFile -Destination $StagedFullHexFile -Force
	Copy-Item -Path $TempSlimHexFile -Destination $StagedSlimHexFile -Force
    # --- CHANGE: Update log message ---
    Write-Host "    (OK) Staged Full and Slim HEX files successfully." -ForegroundColor Green

    Write-Host ""
    Write-Host "--- Post-Build Step Successful ---" -ForegroundColor Cyan
    exit 0
}
catch {
    Write-Host ""
    Write-Host "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" -ForegroundColor Red
    Write-Host "!!! POST-BUILD STEP FAILED !!!" -ForegroundColor Red
    Write-Host "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" -ForegroundColor Red
    Write-Host "REASON: $($_.Exception.Message)" -ForegroundColor Red
    exit 1
}
finally {
    # This block is guaranteed to run, ensuring cleanup even on failure.
    Write-Host ""
    Write-Host "--- Final Cleanup Step ---" -ForegroundColor Gray
    if ($TempFullHexFile -and (Test-Path $TempFullHexFile)) {
        Write-Host "    -> Deleting temporary file: $(Split-Path $TempFullHexFile -Leaf)" -ForegroundColor Gray
        Remove-Item -Path $TempFullHexFile -ErrorAction SilentlyContinue
    }
    # --- CHANGE: Use new variable name for cleanup ---
    if ($TempSlimHexFile -and (Test-Path $TempSlimHexFile)) {
        Write-Host "    -> Deleting temporary file: $(Split-Path $TempSlimHexFile -Leaf)" -ForegroundColor Gray
        Remove-Item -Path $TempSlimHexFile -ErrorAction SilentlyContinue
    }
    if ($TempBinFile -and (Test-Path $TempBinFile)) {
        Write-Host "    -> Deleting temporary file: $(Split-Path $TempBinFile -Leaf)" -ForegroundColor Gray
        Remove-Item -Path $TempBinFile -ErrorAction SilentlyContinue
    }
}