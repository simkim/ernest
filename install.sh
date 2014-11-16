mkdir /etc/servo/
cp ernest.ini /etc/servo
cp 98-ernest.rules /etc/udev/rules.d
/etc/init.d/udev reload
