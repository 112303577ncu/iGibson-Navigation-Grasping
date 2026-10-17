import socket,json,time,threading

def rpc(cmd,path="/tmp/grasp_service.sock",timeout=120):
 s=socket.socket(socket.AF_UNIX);s.settimeout(timeout)
 try:
  s.connect(path);s.sendall((cmd+"\n").encode())
  return json.loads(s.makefile().readline())
 finally:s.close()

report={"ok":False,"samples":[],"started_at":time.time()};done=threading.Event()
def watch():
 while not done.is_set():
  try:
   mem=next(float(x.split()[1])/1024 for x in open("/proc/meminfo") if x.startswith("MemAvailable:"))
   h=rpc("health","/tmp/grasp_health.sock",2)
   report["samples"].append({"time":time.time(),"available_mb":mem,"chassis":h["chassis"],"odom":h["odom"]})
  except Exception as e:report.setdefault("monitor_errors",[]).append(str(e))
  done.wait(.1)
try:
 before=rpc("status");report["before"]=before
 assert before["chassis"]["maintenance"] and not before["chassis"]["moving"]
 assert before["servo_deg"][5]<=33 and before["bin_rim_height_m"]==.06
 assert next(float(x.split()[1])/1024 for x in open("/proc/meminfo") if x.startswith("MemAvailable:"))>=1024
 assert rpc("maintenance-unlock").get("ok")
 t=threading.Thread(target=watch);t.daemon=True;t.start()
 report["result"]=rpc("release-preview")
 report["after"]=rpc("status");report["ok"]=bool(report["result"].get("ok"))
except Exception as e:report["error"]=str(e)
finally:
 try:report["maintenance_lock"]=rpc("maintenance-lock")
 except Exception as e:report["lock_error"]=str(e)
 done.set()
 if "t" in locals():t.join(3)
 report["ended_at"]=time.time()
 if report["samples"]:report["min_available_mb"]=min(x["available_mb"] for x in report["samples"])
 with open("/tmp/release_preview_result.json","w") as f:json.dump(report,f,indent=2)
print(json.dumps({k:report.get(k) for k in ["ok","error","result","min_available_mb"]}))
