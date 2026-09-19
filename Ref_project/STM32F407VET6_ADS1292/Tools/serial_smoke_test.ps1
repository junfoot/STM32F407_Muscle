param(
    [string]$PortName = 'COM3',
    [int]$DurationSeconds = 10
)

$port = [System.IO.Ports.SerialPort]::new(
    $PortName, 460800, [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One)
$port.ReadTimeout = 30
$data = [System.Collections.Generic.List[byte]]::new()
$after = [System.Collections.Generic.List[byte]]::new()
$startCommand = [byte[]](0xA5,0x09,0x00,0x00,0x04,0x0D,0x01,0x01,0x5A)
$stopCommand = [byte[]](0xA5,0x09,0x00,0x00,0x04,0x0D,0x00,0x00,0x5A)

try {
    $port.Open()
    $port.DiscardInBuffer()
    $watch = [Diagnostics.Stopwatch]::StartNew()
    $port.Write($startCommand, 0, $startCommand.Length)
    $buffer = New-Object byte[] 4096
    while ($watch.Elapsed.TotalSeconds -lt $DurationSeconds) {
        try {
            $count = $port.Read($buffer, 0, $buffer.Length)
            for ($index = 0; $index -lt $count; $index++) {
                $data.Add($buffer[$index])
            }
        }
        catch [System.TimeoutException] {}
    }
    $port.Write($stopCommand, 0, $stopCommand.Length)
    $elapsed = $watch.Elapsed.TotalSeconds
    Start-Sleep -Milliseconds 150
    while ($true) {
        try {
            $count = $port.Read($buffer, 0, $buffer.Length)
            for ($index = 0; $index -lt $count; $index++) {
                $after.Add($buffer[$index])
            }
        }
        catch [System.TimeoutException] {
            break
        }
    }
    Start-Sleep -Milliseconds 500
    $silentBytes = $port.BytesToRead
    $port.Close()
}
finally {
    if ($port.IsOpen) {
        $port.Close()
    }
}

function Get-RawFrames([byte[]]$InputBytes) {
    $offset = 0
    while ($offset -le $InputBytes.Length - 8) {
        if ($InputBytes[$offset] -ne 0xAA) {
            $offset++
            continue
        }
        $length = [int]$InputBytes[$offset + 1] + ([int]$InputBytes[$offset + 2] * 256)
        if ($length -lt 8 -or $offset + $length -gt $InputBytes.Length) {
            $offset++
            continue
        }
        if ($InputBytes[$offset + $length - 1] -eq 0x55 -and
            ($InputBytes[$offset + 4] -band 0x7F) -eq 0x06) {
            $frame = [byte[]]$InputBytes[$offset..($offset + $length - 1)]
            Write-Output -NoEnumerate $frame
            $offset += $length
        }
        else {
            $offset++
        }
    }
}

$rawFrames = @(Get-RawFrames $data.ToArray())
$postStopFrames = @(Get-RawFrames $after.ToArray())
$samples = 0
$loffCounts = @{}
$min1 = [double]::PositiveInfinity
$max1 = [double]::NegativeInfinity
$min2 = [double]::PositiveInfinity
$max2 = [double]::NegativeInfinity

foreach ($frame in $rawFrames) {
    $count = [Math]::Floor(($frame.Length - 8) / 8)
    $samples += $count
    $loffIndex = 6 + 8 * $count
    $loff = $frame[$loffIndex]
    if (!$loffCounts.ContainsKey($loff)) {
        $loffCounts[$loff] = 0
    }
    $loffCounts[$loff]++
    for ($sampleIndex = 0; $sampleIndex -lt $count; $sampleIndex++) {
        $value1 = [BitConverter]::ToSingle($frame, 6 + 8 * $sampleIndex)
        $value2 = [BitConverter]::ToSingle($frame, 10 + 8 * $sampleIndex)
        if ($value1 -lt $min1) { $min1 = $value1 }
        if ($value1 -gt $max1) { $max1 = $value1 }
        if ($value2 -lt $min2) { $min2 = $value2 }
        if ($value2 -gt $max2) { $max2 = $value2 }
    }
}

'elapsed_s=' + [Math]::Round($elapsed, 4)
'raw_frames=' + $rawFrames.Count
'samples=' + $samples
'measured_sps=' + [Math]::Round($samples / $elapsed, 2)
'ch1_min_max=' + $min1 + ',' + $max1
'ch2_min_max=' + $min2 + ',' + $max2
'loff=' + (($loffCounts.GetEnumerator() | Sort-Object Name | ForEach-Object {
    '0x' + [Convert]::ToString([int]$_.Name, 16).PadLeft(2, '0') + ':' + $_.Value
}) -join ',')
'post_stop_raw_frames=' + $postStopFrames.Count
'bytes_waiting_after_500ms=' + $silentBytes
