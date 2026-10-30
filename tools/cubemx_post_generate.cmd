@echo off
setlocal
set "hook_log=%~dp0cubemx_post_generate.log"
> "%hook_log%" echo Started: %date% %time%
>> "%hook_log%" echo Working directory: %cd%
"%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe" -NoLogo -NoProfile -NonInteractive -ExecutionPolicy Bypass -File "%~dp0cubemx_post_generate.ps1" >> "%hook_log%" 2>&1
set "hook_exit=%errorlevel%"
>> "%hook_log%" echo Exit code: %hook_exit%
type "%hook_log%"
exit /b %hook_exit%
