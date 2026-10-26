@echo off
:: Archive completed task/result pairs to .agents/history/
:: Usage: archive.cmd <agent> <task-id>
::   agent:   antigravity | kiro
::   task-id: timestamp or unique id used in the task file

set AGENT=%1
set TASK_ID=%2

if "%AGENT%"=="" goto usage
if "%TASK_ID%"=="" goto usage
goto start

:usage
echo Usage: archive.cmd ^<agent^> ^<task-id^>
echo   agent:   antigravity ^| kiro
echo   task-id: timestamp or unique id
exit /b 1

:start
set SCRIPT_DIR=%~dp0
set AGENTS_DIR=%SCRIPT_DIR%..
set HISTORY_DIR=%AGENTS_DIR%\history
set SOURCE_DIR=%AGENTS_DIR%\%AGENT%

if not exist "%SOURCE_DIR%" (
  echo Error: agent folder not found: %SOURCE_DIR%
  exit /b 1
)

if not exist "%HISTORY_DIR%" mkdir "%HISTORY_DIR%"

:: Archive task
if exist "%SOURCE_DIR%\task.md" (
  copy /y "%SOURCE_DIR%\task.md" "%HISTORY_DIR%\%TASK_ID%_%AGENT%_task.md" >nul
  echo Archived: %TASK_ID%_%AGENT%_task.md
)

:: Archive result
if exist "%SOURCE_DIR%\result.md" (
  copy /y "%SOURCE_DIR%\result.md" "%HISTORY_DIR%\%TASK_ID%_%AGENT%_result.md" >nul
  echo Archived: %TASK_ID%_%AGENT%_result.md
)

echo Done.
