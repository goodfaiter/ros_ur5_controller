# UR5 Controller 


## Setup
If eth0 exists, configure it
```
sudo ip addr add 192.168.137.1/24 dev eth0
sudo ip link set eth0 up
```
Verify it's configured
```
ip addr show eth0
```

Now try pinging the UR5
````
ping 192.168.137.3
```