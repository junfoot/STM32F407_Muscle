param(
    [string]$PortName = 'COM3',
    [int]$RecordSeconds = 10
)

function New-Command([byte]$Command, [byte[]]$Payload) {
    $length = 8 + $Payload.Length
    $frame = [byte[]]::new($length)
    $frame[0] = 0xA5
    $frame[1] = [byte]($length -band 0xFF)
    $frame[2] = [byte](($length -shr 8) -band 0xFF)
    $frame[4] = $Command
    $frame[5] = $frame[1] -bxor $frame[2] -bxor $frame[3] -bxor $frame[4]
    $check = [byte]0
    for ($index = 0; $index -lt $Payload.Length; $index++) {
        $frame[6 + $index] = $Payload[$index]
        $check = $check -bxor $Payload[$index]
    }
    $frame[$length - 2] = $check
    $frame[$length - 1] = 0x5A
    return $frame
}

function Read-Available($Port, $Destination, [int]$WaitMilliseconds) {
    $watch = [Diagnostics.Stopwatch]::StartNew()
    $buffer = New-Object byte[] 4096
    while ($watch.ElapsedMilliseconds -lt $WaitMilliseconds) {
        try {
            $count = $Port.Read($buffer, 0, $buffer.Length)
            for ($index = 0; $index -lt $count; $index++) {
                $Destination.Add($buffer[$index])
            }
        }
        catch [System.TimeoutException] {}
    }
}

function Get-Frames([byte[]]$InputBytes) {
    $offset = 0
    while ($offset -le $InputBytes.Length - 8) {
        if ($InputBytes[$offset] -ne 0xAA) { $offset++; continue }
        $length = [int]$InputBytes[$offset + 1] + 256 * [int]$InputBytes[$offset + 2]
        if ($length -lt 8 -or $offset + $length -gt $InputBytes.Length) {
            $offset++; continue
        }
        $headerCheck = $InputBytes[$offset + 1] -bxor $InputBytes[$offset + 2] `
                     -bxor $InputBytes[$offset + 3] -bxor $InputBytes[$offset + 4]
        $dataCheck = [byte]0
        for ($index = $offset + 6; $index -lt $offset + $length - 2; $index++) {
            $dataCheck = $dataCheck -bxor $InputBytes[$index]
        }
        if ($InputBytes[$offset + 5] -eq $headerCheck -and
            $InputBytes[$offset + $length - 2] -eq $dataCheck -and
            $InputBytes[$offset + $length - 1] -eq 0x55) {
            $frame = [byte[]]$InputBytes[$offset..($offset + $length - 1)]
            Write-Output -NoEnumerate $frame
            $offset += $length
        }
        else { $offset++ }
    }
}

$port = [System.IO.Ports.SerialPort]::new(
    $PortName, 460800, [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One)
$port.ReadTimeout = 25
$preReceived = [System.Collections.Generic.List[byte]]::new()
$recordReceived = [System.Collections.Generic.List[byte]]::new()
$postReceived = [System.Collections.Generic.List[byte]]::new()
$hostStart = New-Command 0x04 ([byte[]](1))
$hostStop = New-Command 0x04 ([byte[]](0))
$sdStart = New-Command 0x07 ([byte[]](1))
$sdStop = New-Command 0x07 ([byte[]](0))
$sdQuery = New-Command 0x07 ([byte[]]@())

try {
    $port.Open()
    $port.DiscardInBuffer()
    $port.Write($sdStart, 0, $sdStart.Length)
    Read-Available $port $preReceived 150
    $port.Write($hostStart, 0, $hostStart.Length)
    $recordWatch = [Diagnostics.Stopwatch]::StartNew()
    Read-Available $port $recordReceived ($RecordSeconds * 1000)
    $elapsed = $recordWatch.Elapsed.TotalSeconds
    $port.Write($hostStop, 0, $hostStop.Length)
    $port.Write($sdStop, 0, $sdStop.Length)
    for ($attempt = 0; $attempt -lt 20; $attempt++) {
        Read-Available $port $postReceived 200
        $port.Write($sdQuery, 0, $sdQuery.Length)
    }
    Read-Available $port $postReceived 200
    Read-Available $port $postReceived 200
}
finally {
    if ($port.IsOpen) { $port.Close() }
}

$recordFrames = @(Get-Frames $recordReceived.ToArray())
$preFrames = @(Get-Frames $preReceived.ToArray())
$postFrames = @(Get-Frames $postReceived.ToArray())
$frames = @($preFrames) + @($recordFrames) + @($postFrames)
$rawFrames = @($recordFrames | Where-Object { ($_[4] -band 0x7F) -eq 0x06 })
$sdReplies = @($frames | Where-Object { ($_[4] -band 0x7F) -eq 0x07 })
$samples = 0
foreach ($frame in $rawFrames) {
    $samples += [Math]::Floor(($frame.Length - 8) / 8)
}

$stateNames = @('idle', 'active', 'stopping', 'error')
$replyLines = @()
foreach ($frame in $sdReplies) {
    $state = [int]$frame[6]
    $dropped = [BitConverter]::ToUInt32($frame, 7)
    $written = [BitConverter]::ToUInt32($frame, 11)
    $nameBytes = [byte[]]$frame[15..25]
    $filename = [Text.Encoding]::ASCII.GetString($nameBytes).Trim([char]0)
    $stateName = if ($state -ge 0 -and $state -lt $stateNames.Count) {
        $stateNames[$state]
    } else { "unknown_$state" }
    $replyLines += "$stateName,dropped=$dropped,written=$written,file=$filename"
}

'elapsed_s=' + [Math]::Round($elapsed, 4)
'raw_frames=' + $rawFrames.Count
'samples=' + $samples
'measured_sps=' + [Math]::Round($samples / $elapsed, 2)
'sd_replies=' + ($replyLines -join ';')
'final_sd_state=' + $(if ($replyLines.Count) { $replyLines[-1] } else { 'no_reply' })
