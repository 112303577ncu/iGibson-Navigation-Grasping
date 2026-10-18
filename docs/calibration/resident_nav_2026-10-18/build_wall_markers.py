"""Reproducible vector ArUco print set; OpenCV decodes the rendered PDF.

Run from repository root. Requires reportlab, OpenCV contrib, PyMuPDF.
"""
import hashlib
import json
from pathlib import Path
import cv2
import fitz
import numpy as np
from reportlab.pdfgen import canvas
from reportlab.lib.units import mm
from reportlab.lib.pagesizes import A4,landscape
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont

ROOT=Path(__file__).resolve().parents[3]
OUT=ROOT/'output/pdf';OUT.mkdir(parents=True,exist_ok=True)
EVIDENCE=Path(__file__).resolve().parent/'aruco_print';EVIDENCE.mkdir(exist_ok=True)
dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
font=Path('C:/Windows/Fonts/msjh.ttc')
pdfmetrics.registerFont(TTFont('CJK',str(font),subfontIndex=0))

# Room coordinates for placement instructions, not ROS map coordinates.
MARKERS=[
 dict(id=0,wall='A',distance_m=.5,room_xy_m=[.5,2.],description='紙盒後方牆，從紙盒角落沿牆 50 cm'),
 dict(id=2,wall='A',distance_m=1.5,room_xy_m=[1.5,2.],description='紙盒後方牆，從紙盒角落沿牆 150 cm'),
 dict(id=3,wall='B',distance_m=.5,room_xy_m=[0.,1.5],description='紙盒左側牆，從紙盒角落沿牆 50 cm'),
 dict(id=4,wall='B',distance_m=1.5,room_xy_m=[0.,.5],description='紙盒左側牆，從紙盒角落沿牆 150 cm'),
 dict(id=5,wall='C',distance_m=.5,room_xy_m=[.5,0.],description='A 的對面牆，從 B 側端點沿牆 50 cm'),
 dict(id=1,wall='C',distance_m=1.5,room_xy_m=[1.5,0.],description='A 的對面牆，從 B 側端點沿牆 150 cm'),
 dict(id=6,wall='D',distance_m=.5,room_xy_m=[2.,.5],description='B 的對面牆，從 C 側端點沿牆 50 cm'),
 dict(id=7,wall='D',distance_m=1.5,room_xy_m=[2.,1.5],description='B 的對面牆，從 C 側端點沿牆 150 cm'),
]


def label(c,x,y,text,size=10,color=(0,0,0),center=False):
    c.setFillColorRGB(*color);c.setFont('CJK',size)
    (c.drawCentredString if center else c.drawString)(x*mm,y*mm,text)


path=OUT/'aruco_walls_8_120mm_A4_two_per_page.pdf'
c=canvas.Canvas(str(path),pagesize=A4,pageCompression=1)
c.setTitle('2m x 2m ArUco wall set - 120 mm - 8 unique markers')
c.setViewerPreference('PrintScaling','None')
for page in range(4):
    pair=MARKERS[page*2:page*2+2]
    label(c,105,288,'A4｜實際大小 100%｜黑框 120 × 120 mm｜勿符合頁面縮放',9,center=True)
    for item,bottom,text_y in zip(pair,[158.5,17.5],[147.,6.]):
        pattern=cv2.aruco.generateImageMarker(dictionary,item['id'],6)
        c.setFillColorRGB(0,0,0)
        for row in range(6):
            for col in range(6):
                if pattern[row,col]==0:
                    c.rect((45+col*20)*mm,(bottom+(5-row)*20)*mm,20*mm,20*mm,fill=1,stroke=0)
        label(c,105,text_y,'ID %d｜牆 %s｜%s cm｜DICT_4X4_50'%(item['id'],item['wall'],int(item['distance_m']*100)),9,center=True)
    c.showPage()
c.save()

guide=OUT/'aruco_walls_placement_2m.pdf'
c=canvas.Canvas(str(guide),pagesize=landscape(A4),pageCompression=1)
c.setTitle('ArUco wall placement and print instructions - 2m x 2m')
label(c,16,193,'2 × 2 公尺場地：四面各兩張，共 8 張不同 ID',18)
label(c,16,181,'這是安裝示意圖；尚未量測登錄，圖上座標不是 ROS map 座標。',10,color=(.3,.3,.3))
x0,y0,size=32,37,116
def point(x,y):return x0+x/2*size,y0+y/2*size
c.setLineWidth(2);c.setStrokeColorRGB(.25,.35,.45);c.rect(x0*mm,y0*mm,size*mm,size*mm,fill=0)
label(c,90,165,'A：紙盒後方牆',11,center=True)
label(c,90,25,'C：A 的對面牆',11,center=True)
label(c,18,97,'B',13,center=True);label(c,160,97,'D',13,center=True)
c.setFillColorRGB(.83,.68,.44);px,py=point(.25,1.75);c.rect((px-5)*mm,(py-4)*mm,10*mm,8*mm,fill=1,stroke=0)
label(c,px+7,py-1,'紙盒',9)
for item in MARKERS:
    x,y=point(*item['room_xy_m']);c.setFillColorRGB(.05,.45,.55);c.circle(x*mm,y*mm,2.7*mm,fill=1,stroke=0)
    wall=item['wall'];dx,dy={'A':(0,-10),'B':(10,-2),'C':(0,7),'D':(-10,-2)}[wall]
    label(c,x+dx,y+dy,'ID %d'%item['id'],10,center=True)
label(c,90,88,'箭頭／字樣只作配置說明',9,center=True,color=(.4,.4,.4))
label(c,90,79,'每張正面朝向場地內側',11,center=True)
label(c,174,163,'安裝與列印',13)
for y,text in zip([150,138,126,114,102,90,78,66,54,42],[
    '1. 每頁兩張；A4，實際大小 100%。',
    '2. 黑框應為 120 × 120 mm，印後用尺量。',
    '3. 保留黑框外至少 8 mm 白邊，勿裁到。',
    '4. 貼在硬背板，保持平整、不反光。',
    '5. 中心先與後相機鏡頭同高。',
    '6. 軟牆太低時，用場外固定支架升高。',
    '7. A、B 從紙盒角落量 50／150 cm。',
    '8. C 從 B 側、D 從 C 側量 50／150 cm。',
    '9. ID 0 靠投放區；ID 1 靠其對角區。',
    '10. 貼妥後量位置、朝向，再登錄定位。',
]):label(c,174,y,text,9)
label(c,16,13,'兩張／面能增加可見機會，不能保證任何朝向都看得到；找不到時應停車、限角轉向搜尋，再核對定位。',9)
c.showPage();c.save()

# Verify actual page geometry, each rendered ID, and vector black-square size.
doc=fitz.open(str(path));detector=cv2.aruco.ArucoDetector(dictionary)
checks=[]
for page in doc:
    pix=page.get_pixmap(matrix=fitz.Matrix(200/72,200/72),alpha=False)
    rgb=np.frombuffer(pix.samples,dtype=np.uint8).reshape(pix.height,pix.width,3)
    gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY)
    corners,ids,_=detector.detectMarkers(gray)
    expected=[m['id'] for m in MARKERS[page.number*2:page.number*2+2]]
    actual=sorted(ids.flatten().tolist()) if ids is not None else []
    assert actual==sorted(expected),(actual,expected)
    for row,top in enumerate([18.5,159.5]):
        black=[d['rect'] for d in page.get_drawings() if d.get('fill')==(0.,0.,0.) and top-.1<=d['rect'].y0/mm<=top+120.1]
        assert black
        width=(max(r.x1 for r in black)-min(r.x0 for r in black))/mm
        height=(max(r.y1 for r in black)-min(r.y0 for r in black))/mm
        assert abs(width-120)<.01 and abs(height-120)<.01,(width,height)
        checks.append({'page':page.number+1,'id':expected[row],'black_mm':[width,height]})
    assert abs(page.rect.width/mm-210)<.01 and abs(page.rect.height/mm-297)<.01
    page.get_pixmap(matrix=fitz.Matrix(1.2,1.2),alpha=False).save(str(EVIDENCE/('print_page_%d.png'%(page.number+1))))
doc.close()
doc=fitz.open(str(guide));doc[0].get_pixmap(matrix=fitz.Matrix(1.5,1.5),alpha=False).save(str(EVIDENCE/'placement.png'));doc.close()
result={'dictionary':'DICT_4X4_50','black_marker_m':.120,'page_count':4,'markers':MARKERS,'geometry_and_decode_checks':checks,
        'physical_detection_verified':False,'camera_calibration_verified':False,'map_registration_verified':False,
        'outputs':[{ 'path':str(p.relative_to(ROOT)),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in [path,guide]]}
(EVIDENCE/'print_manifest.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({'pdfs':[str(path),str(guide)],'decoded_ids':[r['id'] for r in checks],'black_mm':120}))
