"""M5Stack PaperColor 縦置き猫耳ネックストラップケース (CadQuery)
座標: 本体(公式STL) x 0.1..70.1, y 0..103(上), z -8.5(背面)..0(表示面)
"""
import math, cadquery as cq

# ---- device & parameters
DX0,DX1,DY0,DY1,DZ0,DZ1=0.1,70.1,0.0,103.0,-8.5,0.0
# Side clearance per side. Real prints measured ~0.6mm lateral play with 0.15/0.15,
# so the fit is tightened by 0.25mm per side (expected play ~0.1mm).
CL_L,CL_R,CL_B,CL_Z=-0.1,-0.1,0.3,0.3
W=2.0; BACK=2.4; LIP_T=1.2; LIP_W=1.4
cx0,cx1,cy0=DX0-CL_L,DX1+CL_R,DY0-CL_B
cz0,cz1=DZ0-CL_Z,DZ1+0.2
ox0,ox1,oy0=cx0-W,cx1+W,cy0-W
oz0,oz1=cz0-BACK,cz1+LIP_T
PLATE_TOP=104.0
YEND=60.0; R_END=5.0          # side walls end (latch position), rounded end
EAR_H=20.0; EAR_R=3.0; EAR_IN=22.0   # ear height, apex radius, inner base x
HOLE_D=5.0; HOLE_DY=6.5
CHAMFER=0.6
SLOT=0.7
LATCH_Y0={'L':36.0,'R':42.5}
BUMP={'L':0.5,'R':0.5}   # = clearance + ~0.6mm engagement into the side recess
BZ0,BZ1=-8.1,-6.7

def box(x0,x1,y0,y1,z0,z1):
    return cq.Workplane("XY").box(x1-x0,y1-y0,z1-z0,centered=False).translate((x0,y0,z0))
def rbox(x0,x1,y0,y1,z0,z1,r):
    return box(x0,x1,y0,y1,z0,z1).edges("|Z").fillet(r)

# ---- back plate + ears
def ear_pts(xb0,xb1):
    xc=(xb0+xb1)/2; yc=PLATE_TOP+EAR_H-EAR_R
    def tan(px,py,pick):
        dx,dy=px-xc,py-yc; d=math.hypot(dx,dy); phi=math.atan2(dy,dx); a=math.acos(EAR_R/d)
        c=[(xc+EAR_R*math.cos(phi+s*a),yc+EAR_R*math.sin(phi+s*a)) for s in (1,-1)]
        return min(c,key=lambda p:p[0]) if pick=='L' else max(c,key=lambda p:p[0])
    return xc,tan(xb0,PLATE_TOP,'L'),(xc,yc+EAR_R),tan(xb1,PLATE_TOP,'R')
def ear(xb0,xb1):
    xc,tl,top,tr=ear_pts(xb0,xb1)
    return (cq.Workplane("XY").workplane(offset=oz0)
            .moveTo(xb0,PLATE_TOP-4).lineTo(xb0,PLATE_TOP).lineTo(*tl)
            .threePointArc(top,tr).lineTo(xb1,PLATE_TOP).lineTo(xb1,PLATE_TOP-4).close()
            .extrude(cz0-oz0)),xc
earL,xcL=ear(ox0,EAR_IN)
earR,xcR=ear(ox0+ox1-EAR_IN,ox1)
plate=rbox(ox0,ox1,oy0,PLATE_TOP,oz0,cz0,4.0).union(earL).union(earR)

# ---- walls + lips
walls=rbox(ox0,ox1,oy0,YEND+20,cz0,oz1,4.0)
walls=walls.cut(rbox(cx0,cx1,cy0,YEND+60,cz0,cz1,2.5))
walls=walls.cut(rbox(DX0+LIP_W,DX1-LIP_W,DY0+LIP_W,YEND+60,cz1-0.01,oz1+1,2.0))
body=plate.union(walls)
# trim walls at YEND with rounded front corner (side view)
X0,X1=ox0-1,ox1+1
trim=box(X0,X1,YEND,YEND+40,cz0,oz1+1)
corner=box(X0,X1,YEND-R_END,YEND+0.01,oz1-R_END,oz1+1).cut(
    cq.Workplane("YZ").workplane(offset=X0).center(YEND-R_END,oz1-R_END).circle(R_END).extrude(X1-X0))
body=body.cut(trim).cut(corner)

# ---- 0.6mm chamfer on front/back face edges
def on_z(e,z,tol=1e-3):
    bb=e.BoundingBox(); return abs(bb.zmin-z)<tol and abs(bb.zmax-z)<tol
sel=[e for e in body.edges().vals()
     if on_z(e,oz0) or on_z(e,oz1) or (on_z(e,cz0) and e.BoundingBox().ymin>YEND+0.5)]
body=body.newObject(sel).chamfer(CHAMFER)

# ---- lanyard holes (chamfered)
HY=PLATE_TOP+HOLE_DY
for xc in (xcL,xcR):
    body=body.cut(cq.Workplane("XY").workplane(offset=oz0-1).center(xc,HY).circle(HOLE_D/2).extrude(cz0-oz0+2))
hole_edges=[e for e in body.edges().vals() if e.geomType()=="CIRCLE" and abs(e.radius()-HOLE_D/2)<1e-3]
body=body.newObject(hole_edges).chamfer(CHAMFER)

# ---- cutouts
Zw0,Zw1=cz0+0.8,cz1-1.6
body=body.cut(box(ox0-1,cx0+0.01,7.5,35.5,Zw0,Zw1))      # left: power, slot
body=body.cut(box(cx1-0.01,ox1+1,30.0,41.5,Zw0,Zw1))     # right: slots
# left: relief groove so the power button (protrudes ~0.5mm, z -6.2..-2.5) is not
# pressed while the device slides in from the top past the latch section
GROOVE=0.9
body=body.cut(box(cx0-GROOVE,cx0+0.01,35.0,YEND+1,-6.8,-1.9))
body=body.cut(box(4.0,18.0,oy0-1,cy0+0.01,cz0+0.4,oz1+1))  # USB-C
body=body.cut(box(52.0,66.0,oy0-1,cy0+0.01,cz0-0.01,-2.5)) # Grove
body=body.cut(box(52.0,66.0,oy0-1,10.0,oz0-1,cz0+0.01))    # Grove cable (back)

# ---- latches (cantilever wall end, separated from back plate)
for side in ('L','R'):
    y0=LATCH_Y0[side]
    if side=='L': xs0,xs1=cx0,cx0+SLOT; g0,g1=ox0-1,xs1
    else:         xs0,xs1=cx1-SLOT,cx1; g0,g1=xs0,ox1+1
    xm=(xs0+xs1)/2; zc=oz0-1; hz=cz0-oz0+1.01
    body=body.cut(box(xs0,xs1,y0+0.8,YEND+SLOT,zc,zc+hz))
    body=body.cut(box(g0,g1,YEND,YEND+SLOT,zc,zc+hz))
    body=body.cut(cq.Workplane("XY").workplane(offset=zc).center(xm,y0+0.8).circle(0.8).extrude(hz))
    h=BUMP[side]
    if side=='L': pts=[(cx0-0.01,48.0),(cx0+h,52.0),(cx0+h,55.0),(cx0-0.01,59.0)]
    else:         pts=[(cx1+0.01,48.0),(cx1-h,52.0),(cx1-h,55.0),(cx1+0.01,59.0)]
    body=body.union(cq.Workplane("XY").workplane(offset=BZ0).polyline(pts).close().extrude(BZ1-BZ0))

if __name__=="__main__":
    # print orientation: back face on bed (z=0)
    out=body.translate((0,0,-oz0))
    cq.exporters.export(out,"PaperColor_NekoCase.step")
    cq.exporters.export(out,"PaperColor_NekoCase.stl",tolerance=0.02,angularTolerance=0.1)
    bb=out.val().BoundingBox(); print("valid",out.val().isValid(),"bbox",round(bb.xlen,2),round(bb.ylen,2),round(bb.zlen,2),"vol",round(out.val().Volume()/1000,2),"cm3")
