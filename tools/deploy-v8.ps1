param([string]$SshTarget = 'tieta@10.126.122.113')
$ErrorActionPreference = 'Stop'
if ($SshTarget -notmatch '^[A-Za-z0-9_.-]+@[A-Za-z0-9.-]+$') { throw 'Use user@host without shell characters.' }
$releasePath = Join-Path (Split-Path $PSScriptRoot -Parent) 'releases'
if (Test-Path (Join-Path $PSScriptRoot 'RobotConsole-v8.1-RobotUpdate.zip')) { $releasePath = $PSScriptRoot }
Write-Host 'Stop the old bridge with Ctrl+C first. This installs a separate v8.1 directory; it does not start motion.'
ssh $SshTarget 'mkdir -p ~/robot-console-v8.1-upload'
if ($LASTEXITCODE -ne 0) { throw 'SSH upload directory failed' }
scp (Join-Path $releasePath 'RobotConsole-v8.1-RobotUpdate.zip') (Join-Path $releasePath 'install_v81.py') "${SshTarget}:robot-console-v8.1-upload/"
if ($LASTEXITCODE -ne 0) { throw 'Upload failed' }
ssh $SshTarget 'cd ~/robot-console-v8.1-upload && ~/robot-bridge-track-b/.venv/bin/python install_v81.py --archive RobotConsole-v8.1-RobotUpdate.zip --legacy ~/robot-console-v8 --target ~/robot-console-v8.1'
if ($LASTEXITCODE -ne 0) { throw 'Installation stopped. Read the remote reason; original bridge is unchanged.' }
Write-Host 'Uploaded and installed. Next follow robot-console-v8.1/docs/V8.1-DEPLOYMENT.md; no service was started.'
