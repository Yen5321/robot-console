$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
dotnet publish (Join-Path $projectRoot 'windows-client/RobotConsole/RobotConsole.csproj') -c Release -r win-x64 --self-contained true -o (Join-Path $projectRoot 'releases/RobotConsole-v8.1-PIPER-L-CPV') -p:PublishSingleFile=false
if ($LASTEXITCODE -ne 0) { throw 'Windows publish failed' }
