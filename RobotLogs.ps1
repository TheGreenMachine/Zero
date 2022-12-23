$rio = '10.18.16.2'
$logdir = $env:BADLOG_DIR
if ($null -eq $logdir){
    Write-Host "Required environment variable BADLOG_DIR missing" -ForegroundColor Red
    exit 1
} 
Write-Host "Copying logs from roboRio"
& scp $rio`:/home/lvuser/*.bag "$($logdir)\"
Write-Host "Removing logs from roboRio"
#& ssh $rio "rm /home/lvuser/*.bag" *> $null
Write-Host "Creating html files"
Get-ChildItem $logdir\*.bag | ForEach-Object { & "$logdir\badlogvis.exe" $_ }
Remove-Item $logdir\*.bag *> $null
& $(Get-ChildItem $logdir\*.html | Sort-Object LastWriteTime | Select-Object -last 1)