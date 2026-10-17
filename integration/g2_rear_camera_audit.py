#!/usr/bin/env python3
"""Bounded rear RGB recording for patrol visibility review; no robot commands."""
import argparse
import json
import os
import socket
import subprocess
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',required=True)
    p.add_argument('--seconds',type=int,default=600)
    p.add_argument('--camera',default='/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0')
    args=p.parse_args()
    if not 10<=args.seconds<=900:p.error('seconds must be 10..900')
    os.makedirs(args.output,exist_ok=True)
    if subprocess.call(['fuser',os.path.realpath(args.camera)],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)==0:
        raise RuntimeError('rear camera already in use')
    import cv2
    cv2.setNumThreads(1)
    cap=cv2.VideoCapture(args.camera,cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,640);cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,1)
    report={'camera':args.camera,'opencv':cv2.__version__,'depth_verified':False,'frames':[],'failures':0,'ok':False}
    start=time.time();last=0
    try:
        if not cap.isOpened():raise RuntimeError('rear RGB camera open failed')
        while time.time()-start<args.seconds and len(report['frames'])<200:
            with open('/proc/meminfo') as f:
                ram=next(float(x.split()[1])/1024 for x in f if x.startswith('MemAvailable:'))
            report['min_available_mb']=min(ram,report.get('min_available_mb',ram))
            if ram<1024:raise RuntimeError('stop optional camera audit under RAM pressure')
            ok,frame=cap.read()
            if not ok:
                report['failures']+=1
                if report['failures']>=5:raise RuntimeError('rear RGB camera repeated read failures')
                time.sleep(.1);continue
            now=time.time()
            if now-last>=3:
                name='frame_%03d.jpg'%len(report['frames'])
                if not cv2.imwrite(os.path.join(args.output,name),frame,[cv2.IMWRITE_JPEG_QUALITY,80]):
                    raise RuntimeError('image write failed')
                item={'file':name,'time':now,'size':list(frame.shape),'available_mb':ram}
                s=socket.socket(socket.AF_UNIX);s.settimeout(1)
                try:
                    s.connect('/tmp/grasp_health.sock');s.sendall(b'health\n')
                    with s.makefile('r') as f:h=json.loads(f.readline())
                    item['odom']=h.get('odom');item['chassis']=h.get('chassis')
                except Exception as exc:item['health_error']=str(exc)
                finally:s.close()
                report['frames'].append(item);last=now
                with open(os.path.join(args.output,'manifest.json'),'w') as f:json.dump(report,f,indent=2)
            time.sleep(.04)
        report['ok']=True
    except Exception as exc:report['error']=str(exc)
    finally:
        cap.release()
        with open(os.path.join(args.output,'manifest.json'),'w') as f:json.dump(report,f,indent=2)
    print(json.dumps({k:report.get(k) for k in ('ok','error','failures','min_available_mb')}))
    return 0 if report['ok'] else 1


if __name__=='__main__':raise SystemExit(main())
