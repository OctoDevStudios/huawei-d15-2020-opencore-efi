import csv, math, cmath, sys
SR=48000.0
def load(f): return [(float(r['freq']),float(r['level_db'])) for r in csv.DictReader(open(f))]
a=load('fr_spk_raw.csv'); b=load('fr_spk_raw2.csv')
m=[(f,(x+y)/2) for (f,x),(_,y) in zip(a,b)]
# 1/2-octave smoothing in power
def smooth(m,oct=0.5):
    out=[]
    for f,_ in m:
        v=[10**(l/10) for g,l in m if abs(math.log2(g/f))<=oct/2]
        out.append((f,10*math.log10(sum(v)/len(v))))
    return out
sm=smooth(m)
def biq(t,f0,g,q):
    A=10**(g/40); w=2*math.pi*f0/SR; c=math.cos(w); s=math.sin(w); al=s/(2*q)
    if t=='pk': b0,b1,b2,a0,a1,a2=1+al*A,-2*c,1-al*A,1+al/A,-2*c,1-al/A
    elif t=='hs':
        sq=2*math.sqrt(A)*al
        b0=A*((A+1)+(A-1)*c+sq); b1=-2*A*((A-1)+(A+1)*c); b2=A*((A+1)+(A-1)*c-sq)
        a0=(A+1)-(A-1)*c+sq; a1=2*((A-1)-(A+1)*c); a2=(A+1)-(A-1)*c-sq
    elif t=='ls':
        sq=2*math.sqrt(A)*al
        b0=A*((A+1)-(A-1)*c+sq); b1=2*A*((A-1)-(A+1)*c); b2=A*((A+1)-(A-1)*c-sq)
        a0=(A+1)+(A-1)*c+sq; a1=-2*((A-1)+(A+1)*c); a2=(A+1)+(A-1)*c-sq
    elif t=='hp': b0,b1,b2,a0,a1,a2=(1+c)/2,-(1+c),(1+c)/2,1+al,-2*c,1-al
    return [b0/a0,b1/a0,b2/a0,a1/a0,a2/a0]
def mag(cs,f):
    z=cmath.exp(-1j*2*math.pi*f/SR); h=1
    for b0,b1,b2,a1,a2 in cs: h*=(b0+b1*z+b2*z*z)/(1+a1*z+a2*z*z)
    return 20*math.log10(abs(h))
EQ=eval(sys.argv[1]) if len(sys.argv)>1 else []
cs=[biq(*e) for e in EQ]
ref=[l for f,l in sm if 500<=f<=5000]; ref=sum(ref)/len(ref)
print(" freq   raw  smooth   eq    result")
for (f,l),(_,s) in zip(m,sm):
    e=mag(cs,f) if cs else 0
    print(f"{f:7.0f} {l-ref:6.1f} {s-ref:6.1f} {e:6.1f} {s-ref+e:6.1f}  "+"#"*max(0,int((s-ref+e+30)/1.5)))
print("peak eq gain", max(mag(cs,f) for f,_ in m) if cs else 0)
