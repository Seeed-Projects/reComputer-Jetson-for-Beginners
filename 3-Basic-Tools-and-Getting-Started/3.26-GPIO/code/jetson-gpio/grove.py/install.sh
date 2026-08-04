#!/bin/bash
#
: <<'EOF'
The MIT License (MIT)

Seeed-Studio Raspberry Pi Hats.
  Peter Yang, turmary@126.com
Copyright (C) 2018 Seeed Technology Co.,Ltd.
EOF


_DEBUG=0

_package_name=grove.py
_seeed_source_list=/etc/apt/sources.list.d/seeed.list
_seeed_apt_key="BB8F 40F3"
_repo_package_url=https://github.com/Seeed-Studio/$_package_name/archive/master.zip

BLACKLIST=/etc/modprobe.d/raspi-blacklist.conf
CONFIG=/boot/config.txt
FIRMWARE_CONFIG=/boot/firmware/config.txt
if [ $_DEBUG -ne 0 ]; then
	BLACKLIST=./raspi-blacklist.conf
	CONFIG=./config.txt
fi


set_config_var() {
	lua - "$1" "$2" "$3" <<EOF > "$3.bak"
local key=assert(arg[1])
local value=assert(arg[2])
local fn=assert(arg[3])
local file=assert(io.open(fn))
local made_change=false
for line in file:lines() do
	if line:match("^#?%s*"..key.."=.*$") then
		line=key.."="..value
		made_change=true
	end
	print(line)
end

if not made_change then
	print(key.."="..value)
end
EOF

	mv "$3.bak" "$3"
}

get_i2c() {
	# Check if the file /boot/firmware/config.txt exists
	config_file=$FIRMWARE_CONFIG
	if [ ! -f $config_file ]; then
		config_file=$CONFIG
	fi
	egrep -q "^(device_tree_param|dtparam)=([^,]*,)*i2c(_arm)?(=(on|true|yes|1))?(,.*)?$" $config_file
	echo $?
}


alias  pip=' pip --no-cache-dir'
alias pip3='pip3 --no-cache-dir'

function pip_install() {
	local fields
	pkg_name=$1
	pip_cmd=${2}

	fields=( $pip_cmd )
	for ((i = 0; i < 3; i++)); {
		$pip_cmd
		pkg_status=$(${fields[0]} list --format=columns | egrep "$pkg_name " | awk '{ printf "%s", $1; }')
		[ "$pkg_status" == "$pkg_name" ] && return 0
	}
	return 1
}

function platform_get() {
	local dts_model platform

	dts_model=$(strings /proc/device-tree/model)

	case "$dts_model" in
	TI\ AM335x*)
		platform=bbb;;
	Raspberry\ Pi*)
		platform=rpi;;
	Freescale\ i\.MX8MQ\ Phanbell)
		platform=coral;;
	NVIDIA\ Jetson\ Nano*)
		platform=jetson_nano;;
	*)
		platform="unknown";;
	esac
	echo $platform
}



# Get platform type
platform=$(platform_get)

do_uninstall=false
if [ "$1" == "uninstall" ]; then
	do_uninstall=true
fi

### Uninstall this package ###
if [ "$do_uninstall" == "true" ]; then
	# To remove all users installaon
	pip3 uninstall -y $_package_name
	exit 0
fi

# install dependencies
### add repository
if [ ! -f $_seeed_source_list ]; then
	case "$platform" in
	coral)
		code_name="mendel-beaker";;
	rpi)
		code_name="stretch";;
	jetson_nano)
		code_name="bionic";;
	*)
		code_name="unknown";;
	esac
	echo "deb https://seeed-studio.github.io/pi_repo/ $code_name main" | tee $_seeed_source_list
fi

### add public GPG key
# if ! apt-key list | egrep "$_seeed_apt_key" > /dev/null; then
# 	curl https://seeed-studio.github.io/pi_repo/public.key | apt-key add -
# fi



## Initial installation
r=0

command -v python3 >/dev/null 2>&1 || { echo "Executable \"python3\" couldn't be found. Error occurred with RFR_Tools installation." >&2; exit 4; }
command -v pip3 >/dev/null 2>&1 || { echo "Executable \"pip3\" couldn't be found. Error occurred with RFR_Tools installation." >&2; exit 5; }

### Check i2c 
if [ $(get_i2c) -ne 0 ]; then
	echo I2C is not enabled.
	echo "Please enable I2C by running raspi-config and selecting Interfacing Options -> I2C"
	exit 1
fi

## install library rpi.gpio

if [ "X$platform" == "Xrpi" ]; then
	(( r == 0 )) && { pip_install RPi.GPIO 'pip3 install RPi.GPIO'; r=$?; }
fi

## install library rpi-ws281x
if [ "X$platform" == "Xrpi" ]; then
	(( r == 0 )) && { pip_install rpi-ws281x 'pip3 install rpi-ws281x'; r=$?; }
fi

## install library smbus
(( r == 0 )) && { pip_install smbus  'pip3 install smbus'; r=$?; }

## install library smbus2
(( r == 0 )) && { pip_install smbus2 'pip3 install smbus2'; r=$?; }

## install library bme680

(( r == 0 )) && { pip_install bme680 'pip3 install bme680'; r=$?; }

## install library bmm150

(( r == 0 )) && { pip_install bmm150 'pip3 install bmm150'; r=$?; }


## install library sgp30
(( r == 0 )) && { pip_install sgp30 'pip3 install sgp30'; r=$?; }
# install this python repository
(( r == 0 )) && { pip_install Seeed-grove.py "pip3 install --upgrade $_repo_package_url"; r=$?; }
(( r == 0 )) && { which grove_button > /dev/null; r=$?; }

(( r != 0 )) && {
	echo "-------------------------------------------------------"
	echo "     Grove.py installation FAILED, FAILED, FAILED      "
	echo "-------------------------------------------------------"
	exit 1
}

sync
sleep 1
sync 

echo "#######################################################"
echo "  Lastest Grove.py from github install complete   !!!!!"
echo "#######################################################"
exit 0

