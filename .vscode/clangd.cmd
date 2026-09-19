@echo off
setlocal

for /d %%D in ("%LOCALAPPDATA%\stm32cube\bundles\st-arm-clangd\*") do (
  if exist "%%~fD\bin\starm-clangd.exe" (
    "%%~fD\bin\starm-clangd.exe" %*
    exit /b %errorlevel%
  )
)

echo STM32Cube clangd was not found under %%LOCALAPPDATA%%\stm32cube\bundles\st-arm-clangd. 1>&2
exit /b 1
