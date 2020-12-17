To add a service that is the last thing to start. Requires the Controller to be already paired
and automatically connect at startup (CONTROLLER ON AND SEARCHING BEFORE PI STARTUP).

sudo raspi-config
sudo apt-get update
sudo apt-get upgrade
sudo apt install python3-pip
pip3 install evdev
pip3 install pyserial

To pair the bluetooth controller:

sudo bluetoothctl
	scan on
	- get the mac address of the correct controller
	connect XX:XX:XX:XX:XX:XX
	pair XX:XX:XX:XX:XX:XX
	trust XX:XX:XX:XX:XX:XX
	exit

To add a service and make it run on startup

sudo nano /etc/systemd/system/remoteControl.service

----------------------------------------------------------
[Unit]
Description=Service for Bluetooth Remote Control
After=getty.target

[Service]
ExecStart=sh launcher.sh
WorkingDirectory=/home/pi/scripts
StandardOutput=inherit
StandardError=inherit
Restart=always
User=pi

[Install]
WantedBy=multi-user.target

---------------------------------------------------------

sudo chmod a+r /etc/systemd/system/remoteControl.service

sudo systemctl daemon-reload

sudo systemctl start remoteControl.service
- check everything is working
sudo systemctl stop remoteControl.service
sudo systemctl enable remoteControl.service

sudo reboot
- everything should work

- to help debug:
sudo systemctl status remoteControl.service
- to tail the cmd line output of the service...
journalctl -f -u remoteControl.service 