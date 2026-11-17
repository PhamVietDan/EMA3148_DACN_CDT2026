@echo off
:: Reset all agent statuses to idle and clear task/result files
:: Usage: reset.cmd [agent]
::   agent: antigravity | kiro | all (default: all)

set AGENT=%1
if "%AGENT%"=="" set AGENT=all

set SCRIPT_DIR=%~dp0
set AGENTS_DIR=%SCRIPT_DIR%..

if "%AGENT%"=="all" (
  call :reset_agent antigravity
  call :reset_agent kiro
) else (
  call :reset_agent %AGENT%
)

echo Done.
exit /b 0

:reset_agent
set NAME=%1
set DIR=%AGENTS_DIR%\%NAME%

if not exist "%DIR%" (
  echo Warning: agent folder not found: %DIR%
  exit /b 0
)

:: Reset status
echo # %NAME% Status> "%DIR%\status.md"
echo.>> "%DIR%\status.md"
echo **Status:** idle>> "%DIR%\status.md"

:: Clear task
echo ^<!-- Cleared by reset.cmd --^>> "%DIR%\task.md"

:: Clear result
echo ^<!-- Cleared by reset.cmd --^>> "%DIR%\result.md"

echo Reset: %NAME%
exit /b 0
