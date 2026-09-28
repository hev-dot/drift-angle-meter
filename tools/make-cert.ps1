# Creates a self-signed development certificate (tools/dev-cert.pfx) valid for localhost
# and this PC's LAN addresses, so phones on the same Wi-Fi can open the app over https.
# Usage: powershell -ExecutionPolicy Bypass -File tools/make-cert.ps1

$ips = Get-NetIPAddress -AddressFamily IPv4 |
  Where-Object { $_.IPAddress -notlike '127.*' -and $_.IPAddress -notlike '169.254.*' } |
  Select-Object -ExpandProperty IPAddress
$san = '2.5.29.17={text}DNS=localhost&IPAddress=127.0.0.1' + (($ips | ForEach-Object { "&IPAddress=$_" }) -join '')

$cert = New-SelfSignedCertificate -Subject 'CN=drift-meter-dev' -TextExtension @($san) `
  -CertStoreLocation 'Cert:\CurrentUser\My' -NotAfter (Get-Date).AddYears(1) `
  -KeyExportPolicy Exportable -KeyAlgorithm RSA -KeyLength 2048
$pwd = ConvertTo-SecureString -String 'drift-dev' -Force -AsPlainText
Export-PfxCertificate -Cert $cert -FilePath (Join-Path $PSScriptRoot 'dev-cert.pfx') -Password $pwd | Out-Null
Remove-Item "Cert:\CurrentUser\My\$($cert.Thumbprint)"
Write-Output "Created tools/dev-cert.pfx for: localhost $($ips -join ' ')"
