$ErrorActionPreference = 'Stop'

try {
    $root = Split-Path -Parent $PSScriptRoot
    $project = Join-Path $root 'MDK-ARM\FOC_Project.uvprojx'
    $scatter = Join-Path $root 'MDK-ARM\foc_reserved_flash.sct'

    if (!(Test-Path -LiteralPath $project) -or
        !(Test-Path -LiteralPath $scatter)) {
        throw 'Project or reserved scatter file not found. No files changed.'
    }

    # Require the existing scatter file to reserve the final 2 KB.
    $scatterText = [IO.File]::ReadAllText($scatter)
    $regions = [regex]::Matches(
        $scatterText,
        '(?im)^\s*(LR_IROM1|ER_IROM1)\s+0x0?8000000\s+0x0*7F800\b'
    )
    if ($regions.Count -ne 2) {
        throw 'Unexpected reserved Flash range. No files changed.'
    }

    $original = [IO.File]::ReadAllText($project)
    [xml]$xml = $original

    # Refuse ambiguous targets rather than changing another build target.
    if ($xml.SelectNodes('//ScatterFile').Count -ne 1 -or
        $xml.SelectNodes('//AfterMake').Count -ne 1) {
        throw 'Unexpected project structure. No files changed.'
    }

    $pattern = '<ScatterFile>[^<]*</ScatterFile>'
    if ([regex]::Matches($original, $pattern).Count -ne 1) {
        throw 'ScatterFile is not unique. No files changed.'
    }

    $oldScatter = $xml.SelectSingleNode('//ScatterFile').InnerText
    if ($oldScatter -notin @('stm32g474xx_flash.sct',
                            '.\stm32g474xx_flash.sct',
                            'foc_reserved_flash.sct',
                            '.\foc_reserved_flash.sct')) {
        throw 'Unrecognized ScatterFile selection. No files changed.'
    }

    $updated = [regex]::Replace(
        $original,
        $pattern,
        '<ScatterFile>.\foc_reserved_flash.sct</ScatterFile>'
    )

    # Preserve real user commands; only disable an empty After Build Run #2.
    $after = $xml.SelectSingleNode('//AfterMake')
    $commandNodes = $after.SelectNodes('UserProg2Name')
    $runNodes = $after.SelectNodes('RunUserProg2')
    if ($commandNodes.Count -ne 1 -or $runNodes.Count -ne 1) {
        throw 'Incomplete AfterMake structure. No files changed.'
    }

    if ([string]::IsNullOrWhiteSpace($commandNodes[0].InnerText)) {
        $match = [regex]::Match(
            $updated, '(?s)<AfterMake>.*?</AfterMake>'
        )
        if (!$match.Success -or
            [regex]::Matches(
                $match.Value, '<RunUserProg2>[01]</RunUserProg2>'
            ).Count -ne 1) {
            throw 'After Build Run #2 is not unique. No files changed.'
        }

        $block = [regex]::Replace(
            $match.Value,
            '<RunUserProg2>[01]</RunUserProg2>',
            '<RunUserProg2>0</RunUserProg2>'
        )
        $updated = $updated.Remove(
            $match.Index, $match.Length
        ).Insert($match.Index, $block)
    }

    # Parse the edited XML before writing anything.
    [xml]$check = $updated
    if ($check.SelectSingleNode('//ScatterFile').InnerText -ne
        '.\foc_reserved_flash.sct') {
        throw 'ScatterFile verification failed. No files changed.'
    }

    if ($updated -ceq $original) {
        Write-Host 'OK: Configuration already correct. No files changed.'
        exit 0
    }

    # Preserve UTF-8 BOM, whitespace and line endings of the project file.
    $bytes = [IO.File]::ReadAllBytes($project)
    $hasBom = ($bytes.Length -ge 3 -and
               $bytes[0] -eq 239 -and
               $bytes[1] -eq 187 -and
               $bytes[2] -eq 191)
    $encoding = [Text.UTF8Encoding]::new($hasBom)

    $temp = "$project.cubemx-fix.tmp"
    $backup = "$project.before-fix.$(Get-Date -Format 'yyyyMMdd_HHmmss_fff').bak"

    [IO.File]::WriteAllText($temp, $updated, $encoding)
    if ([IO.File]::ReadAllText($project) -cne $original) {
        throw 'Project changed during processing. Original file retained.'
    }
    [IO.File]::Replace($temp, $project, $backup)

    Write-Host 'OK: ScatterFile restored; empty After Build Run #2 disabled.'
    Write-Host "Project: $project"
    Write-Host "Backup: $backup"
}
catch {
    [Console]::Error.WriteLine("FAILED: $($_.Exception.Message)")
    exit 1
}
