param(
    [Parameter(Mandatory = $true)]
    [string]$InputPath,

    [Parameter(Mandatory = $true)]
    [string]$OutputPath
)

$ErrorActionPreference = "Stop"

$resolvedInput = (Resolve-Path -LiteralPath $InputPath).Path
$resolvedOutput = [System.IO.Path]::GetFullPath($OutputPath)
$outputDir = Split-Path -Parent $resolvedOutput

if ($outputDir -and -not (Test-Path -LiteralPath $outputDir)) {
    New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
}

$records = Get-Content -LiteralPath $resolvedInput
$image = New-Object 'System.Collections.Generic.Dictionary[int,byte]'
$baseAddress = 0
$maxAddress = -1

foreach ($line in $records) {
    if ([string]::IsNullOrWhiteSpace($line)) {
        continue
    }

    if (-not $line.StartsWith(":")) {
        throw "Invalid Intel HEX record: $line"
    }

    $byteCount = [Convert]::ToInt32($line.Substring(1, 2), 16)
    $address = [Convert]::ToInt32($line.Substring(3, 4), 16)
    $recordType = [Convert]::ToInt32($line.Substring(7, 2), 16)
    $dataStart = 9

    switch ($recordType) {
        0 {
            for ($i = 0; $i -lt $byteCount; $i++) {
                $value = [Convert]::ToByte($line.Substring($dataStart + ($i * 2), 2), 16)
                $absolute = $baseAddress + $address + $i
                $image[$absolute] = $value
                if ($absolute -gt $maxAddress) {
                    $maxAddress = $absolute
                }
            }
        }
        1 {
            break
        }
        2 {
            $segment = [Convert]::ToInt32($line.Substring($dataStart, 4), 16)
            $baseAddress = $segment -shl 4
        }
        4 {
            $segment = [Convert]::ToInt32($line.Substring($dataStart, 4), 16)
            $baseAddress = $segment -shl 16
        }
        default {
            continue
        }
    }
}

if ($maxAddress -lt 0) {
    throw "No data records found in $resolvedInput"
}

$bytes = New-Object byte[] ($maxAddress + 1)
foreach ($addr in $image.Keys) {
    $bytes[$addr] = $image[$addr]
}

[System.IO.File]::WriteAllBytes($resolvedOutput, $bytes)
