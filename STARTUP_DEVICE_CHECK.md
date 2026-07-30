# X3Plus 開機後設備檢查流程

目的：每次 Jetson 剛開機後，先確認 USB 序列埠、相機、Rosmaster symlink 沒有跑掉，再進 Phase 1/2/3 或實機測試。

## 1. 進入專案與虛擬環境

```bash
cd ~/Documents/deploy_jetson2
source ~/grasp_venv/bin/activate
```

## 2. 跑自動檢查

```bash
python3 startup_device_check.py
```

通過時最後應該看到：

```text
PASS: Rosmaster symlink is correct
PASS: device mapping looks ready
```

## 3. 正確對應關係

**序列埠用晶片認，不是認 ttyUSB 號**（號碼會跳）：

```text
/dev/myserial -> ch341 / QinHeng HL-340 = Rosmaster 板（控手臂/底盤一律用 /dev/myserial）
cp210x / Silicon Labs CP2102 = YDLIDAR TG30（ROS driver 用 /dev/rplidar udev 別名）
```

2026-07-16 當次枚舉是 Rosmaster=`ttyUSB1`、TG30=`ttyUSB0`；這只是一個開機快照，
不可寫進正式命令。

**相機用序號認，不是認 video 號**（索引會跳，曾 video0→video2）：

```text
後鏡頭   = SN0001 的 Sonix camera
手臂相機 = 另一顆 Sonix（非 SN0001）
```

`startup_device_check.py` 會直接把每個 `/dev/video*` 標成 `ARM camera` / `REAR camera (SN0001)`，
並在最後印「distinct cameras by serial」。**以那個標籤為準，不要假設 video0=手臂。**

重點：控制手臂/底盤/整合流程的 port 預設用 `/dev/myserial`，不要硬寫任何
`/dev/ttyUSB0/1`。

## 4. 如果 Rosmaster 指錯

先確認：

```bash
ls -l /dev/myserial
ls -l /dev/ttyUSB*
for d in /dev/ttyUSB*; do echo "== $d =="; udevadm info -q property -n $d | grep -E 'ID_MODEL|ID_USB_DRIVER|ID_VENDOR'; done
```

如果 `/dev/myserial` 沒指到 ch341/Rosmaster，先不要跑 `--real`。應修好 udev 規則；
只做診斷時可暫時指定當次查到的 ch341 stable by-id 路徑，例如目前是：

```bash
--port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
```

## 5. 手臂固定到 Phase 1/2/3 姿勢

```bash
cd ~/Documents/deploy_jetson2/grasp
python3 move_arm.py --pose nav
```

如果要只測夾爪：

```bash
python3 move_arm.py --deg 90,140,0,0,90,180 --run-ms 1500
python3 move_arm.py --deg 90,140,0,0,90,30 --run-ms 1500
```

## 6. Phase 1 相機內參

確認設備檢查通過後再開始校相機。**`--source` 的號碼要用 check 標成 ARM/REAR 的那個 `/dev/videoN` 的 N**
（索引會跳，別照抄 0/1；把下面 `<ARM_N>`/`<REAR_N>` 換成當下報出來的索引，例如手臂曾是 video2 → `--source 2`）：

```bash
cd ~/Documents/deploy_jetson2/detection/calibration
python3 calibrate_intrinsics.py --source <ARM_N>  --cols 9 --rows 6 --square-mm 20 --shots 18 --out ../../arm_cam_intrinsics.json
python3 calibrate_intrinsics.py --source <REAR_N> --cols 9 --rows 6 --square-mm 20 --shots 18 --out ../../rear_cam_intrinsics.json
```

> 提醒：手臂相機**畸變不可忽略**（Phase 1 實測 bbox-bottom-center 位移 9px），校完內參後，YOLO
> bbox 底邊中點進距離/座標換算前要先 `cv2.undistortPoints`；後鏡頭（1.79px）可先忽略。
