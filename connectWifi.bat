SET SSID_NAME="DIRECT-%1%-RC"
SET INTF_NAME=%2
netsh wlan disconnect interface=%INTF_NAME%
timeout /t 1 /nobreak
netsh wlan refresh
timeout /t 1 /nobreak
netsh wlan show networks
netsh wlan connect name=%SSID_NAME% ssid=%SSID_NAME% interface=%INTF_NAME%