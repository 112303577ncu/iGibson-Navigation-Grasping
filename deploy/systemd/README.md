# X3Plus `/dev/myserial` ownership

`grasp-service.service` and `x3plus-navigation.service` are mutually exclusive.
They must never run at the same time because both open the Rosmaster controller
through `/dev/myserial`.

Install on the Jetson:

```bash
sudo install -m 0644 x3plus-navigation.service /etc/systemd/system/
sudo install -d -m 0755 /etc/systemd/system/grasp-service.service.d
sudo install -m 0644 grasp-service.service.d/serial-owner.conf \
  /etc/systemd/system/grasp-service.service.d/
sudo systemctl daemon-reload
```

Switch to navigation mode (the ROS master must already be running):

```bash
sudo systemctl start x3plus-navigation.service
```

Switch back to the resident grasp mode:

```bash
sudo systemctl start grasp-service.service
sudo systemctl start grasp-vision.service
```

Select exactly one boot owner. The deployed default remains grasp mode:

```bash
# Grasp at boot (current default)
sudo systemctl disable x3plus-navigation.service
sudo systemctl enable grasp-service.service grasp-vision.service

# Navigation at boot (only after roscore is also made persistent)
sudo systemctl disable grasp-service.service grasp-vision.service
sudo systemctl enable x3plus-navigation.service
```

Verify that there is exactly one owner:

```bash
sudo fuser -v /dev/myserial
systemctl --no-pager --full status \
  x3plus-navigation.service grasp-service.service grasp-vision.service
```
