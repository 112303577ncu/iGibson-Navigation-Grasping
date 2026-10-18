import sys,math,dataclasses
sys.path.insert(0,'/home/jetson/Documents/deploy_jetson2/integration')
import g2_quarter_turn_calibration as q
Base=q.EnvelopeGate
class SessionGate(Base):
 def check(self,scan):
  def retain(p):
   a,r=p;t=math.radians(a+180);x=.10+r*math.cos(t);y=r*math.sin(t)
   return not(-.215<=x<=-.18 and -.205<=y<=-.135)
  return super().check(dataclasses.replace(scan,points=tuple(p for p in scan.points if retain(p))))
q.EnvelopeGate=SessionGate
if __name__=='__main__':q.main()
