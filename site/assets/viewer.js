import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
const el=document.getElementById('viewer'),tip=document.getElementById('tip');
const scene=new THREE.Scene();
const cam=new THREE.PerspectiveCamera(40,el.clientWidth/el.clientHeight,1,5000);
cam.position.set(620,380,900);
const r=new THREE.WebGLRenderer({antialias:true,alpha:true});r.setPixelRatio(devicePixelRatio);r.setSize(el.clientWidth,el.clientHeight);el.appendChild(r.domElement);
const ctl=new OrbitControls(cam,r.domElement);ctl.target.set(0,285,0);ctl.update();
scene.add(new THREE.HemisphereLight(0xffffff,0x334455,1.1));
const dl=new THREE.DirectionalLight(0xffffff,1.6);dl.position.set(300,600,400);scene.add(dl);
const grid=new THREE.GridHelper(600,30,0x445566,0x2a3340);scene.add(grid);
const G={servo:new THREE.Group(),compute:new THREE.Group(),power:new THREE.Group(),sensor:new THREE.Group(),frame:new THREE.Group()};
Object.values(G).forEach(g=>scene.add(g));
const COL={servo:0x7b3fe4,compute:0x1f6fe0,power:0xe8710a,sensor:0x1a9a55,frame:0x8a96a3};
// robot frame (x fwd, y left, z up, mm) -> three (x, z, -y)
const P=(x,y,z)=>new THREE.Vector3(x,z,-y);
const picks=[];
function box(group,name,spec,dx,dy,dz,c,color,price){ // sizes in robot axes
  const m=new THREE.Mesh(new THREE.BoxGeometry(dx,dz,dy),new THREE.MeshStandardMaterial({color,roughness:.55,metalness:.1,transparent:true,opacity:.92}));
  m.position.copy(P(...c));
  m.add(new THREE.LineSegments(new THREE.EdgesGeometry(m.geometry),new THREE.LineBasicMaterial({color:0x000000,transparent:true,opacity:.5})));
  m.userData={name,spec,price};group.add(m);picks.push(m);return m;
}
// ST3215 envelope 45.2 x 24.7 x 35 (long x thin x shaft-axis)
function servo(id,name,c,axis,long){
  const d={x:0,y:0,z:0};const axes=['x','y','z'];
  const other=axes.find(a=>a!==axis&&a!==long);
  d[axis]=35;d[long]=45.2;d[other]=24.7;
  box(G.servo,`ID ${id}: ${name}`,'Feetech ST3215, 45.2 x 24.7 x 35 mm, 55 g, 12 V, 30 kg·cm stall',d.x,d.y,d.z,c,COL.servo,'$19.90');
}
const side=[['R',-1,[1,2,3,4,5,6]],['L',1,[7,8,9,10,11,12]]];
for(const [s,sg,ids] of side){
  const y=38*sg;
  servo(ids[5],'ankle roll ('+s+')',[0,y,47],'x','z');
  servo(ids[4],'ankle pitch ('+s+')',[0,y,100],'y','z');
  servo(ids[3],'knee pitch ('+s+')',[0,y,165],'y','z');
  servo(ids[2],'hip pitch ('+s+')',[0,y,252],'y','z');
  servo(ids[1],'hip roll ('+s+')',[-38,y,252],'x','z');
  servo(ids[0],'hip yaw ('+s+')',[-26,y,305],'z','x');
}
for(const [s,sg,ids] of [['R',-1,[13,14,15,16]],['L',1,[17,18,19,20]]]){
  servo(ids[0],'shoulder pitch ('+s+')',[0,64*sg,425],'y','z');
  servo(ids[1],'shoulder roll ('+s+')',[0,100*sg,398],'x','z');
  servo(ids[2],'elbow pitch ('+s+')',[30,100*sg,335],'y','z');
  servo(ids[3],'gripper ('+s+')',[30,100*sg,280],'y','z');
}
servo(21,'neck yaw',[-10,0,470],'z','x');
servo(22,'neck pitch',[0,0,500],'y','z');
const ENV=' (envelope, not vendor CAD)';
box(G.compute,'Raspberry Pi 5','85 x 56 mm board; height ~17 mm assumed'+ENV,17,56,85,[14,0,415],COL.compute,'$77.50');
box(G.compute,'XIAO ESP32-S3','21 x 17.8 mm board'+ENV,3.5,17.8,21,[-5,25,340],COL.compute,'$7.90');
box(G.compute,'Waveshare bus adapter A','42 x 33 mm assumed'+ENV,8,33,42,[-5,-30,345],COL.compute,'$4.99');
box(G.compute,'Waveshare bus adapter B','42 x 33 mm assumed'+ENV,8,33,42,[-5,-30,392],COL.compute,'$4.99');
box(G.power,'3S LiPo 2200 mAh','~105 x 35 x 25 mm assumed (size NOT verified)'+ENV,25,35,105,[-22,0,395],COL.power,'$19.99');
box(G.power,'Pololu D24V50F5','17.8 x 20.3 x 8.8 mm'+ENV,8.8,20.3,17.8,[-5,30,380],COL.power,'$32.95');
box(G.power,'Relay + fuse','assumed ~30 x 20 x 20 mm'+ENV,20,20,30,[-30,22,350],COL.power,'~$12');
const es=new THREE.Mesh(new THREE.CylinderGeometry(11,11,18,24),new THREE.MeshStandardMaterial({color:0xd62728}));es.position.copy(P(-46,0,450));es.rotation.z=Math.PI/2;es.userData={name:'E-stop (NC latching mushroom)',spec:'~22 mm panel hole, envelope only',price:'$7.50'};G.power.add(es);picks.push(es);
box(G.sensor,'Adafruit BNO085 IMU','25.6 x 22.7 mm board'+ENV,4.6,22.7,25.6,[0,0,336],COL.sensor,'$29.50');
box(G.sensor,'Pi Camera Module 3','25 x 24 x 11.5 mm'+ENV,11.5,24,25,[26,0,548],COL.sensor,'$25');
box(G.sensor,'I2S MEMS mic','small breakout'+ENV,2,12.7,16.7,[0,22,548],COL.sensor,'$6.95');
const sp=new THREE.Mesh(new THREE.CylinderGeometry(20,20,15,32),new THREE.MeshStandardMaterial({color:0x1a9a55}));sp.position.copy(P(40,0,370));sp.rotation.z=Math.PI/2;sp.userData={name:'40 mm speaker',spec:'40 mm dia, envelope only',price:'$4.95'};G.sensor.add(sp);picks.push(sp);
// frame outline (phase 1: outline only)
const fr=new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(80,132,96)),new THREE.LineDashedMaterial({color:0x8a96a3}));fr.position.copy(P(0,0,394));G.frame.add(fr);
const pl=new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(70,104,40)),new THREE.LineBasicMaterial({color:0x8a96a3}));pl.position.copy(P(-10,0,305));G.frame.add(pl);

// schematic link rods (printed links; placeholder geometry until CAD phase)
function rod(a,b,rad=7){const A=P(...a),B=P(...b),d=B.clone().sub(A);const m=new THREE.Mesh(new THREE.CylinderGeometry(rad,rad,d.length(),12),new THREE.MeshStandardMaterial({color:0x8a96a3,roughness:.7}));m.position.copy(A.clone().add(B).multiplyScalar(.5));m.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0),d.normalize());m.userData={name:'Printed link (schematic rod)',spec:'Placeholder: real printed links come with the CAD phase',price:'-'};G.frame.add(m);picks.push(m)}
for(const sg of [-1,1]){const y=38*sg;
  rod([0,y,30],[0,y,47]);rod([0,y,47],[0,y,100]);rod([0,y,100],[0,y,165]);rod([0,y,165],[0,y,252]);rod([0,y,252],[-38,y,252]);rod([-38,y,252],[-26,y,305]);rod([-26,y,305],[0,0,320]);
  const a=100*sg;rod([0,64*sg,425],[0,64*sg,445]);rod([0,64*sg,425],[0,100*sg,398]);rod([0,100*sg,398],[30,100*sg,335]);rod([30,100*sg,335],[30,100*sg,280]);rod([0,64*sg,425],[0,0,425]);
  const f=new THREE.Mesh(new THREE.BoxGeometry(80,4,50),new THREE.MeshStandardMaterial({color:0x5b6773}));f.position.copy(P(20,y,14));f.userData={name:'Foot plate (schematic)',spec:'80 x 50 mm placeholder',price:'-'};G.frame.add(f);picks.push(f);
}
rod([0,0,320],[0,0,440],10);rod([-10,0,470],[0,0,440]);rod([0,0,500],[26,0,548]);
const hd=new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(60,70,50)),new THREE.LineDashedMaterial({color:0x8a96a3}));hd.position.copy(P(10,0,545));G.frame.add(hd);
document.querySelectorAll('[data-g]').forEach(b=>b.onclick=()=>{b.classList.toggle('on');G[b.dataset.g].visible=b.classList.contains('on')});
const ray=new THREE.Raycaster(),mp=new THREE.Vector2();let hov=null,sel=null;
function setHov(o){if(hov&&hov!==sel)hov.material.emissive?.setHex(0);hov=o;if(o){o.material.emissive?.setHex(0x333333)}}
r.domElement.addEventListener('pointermove',e=>{const b=r.domElement.getBoundingClientRect();mp.set((e.clientX-b.left)/b.width*2-1,-(e.clientY-b.top)/b.height*2+1);ray.setFromCamera(mp,cam);
  const h=ray.intersectObjects(picks.filter(p=>p.parent.visible))[0];setHov(h?h.object:null);
  if(h){const u=h.object.userData;tip.innerHTML=`<b>${u.name}</b><br>${u.spec}<br><span style="color:#9aa7b4">${u.price}</span>`}});
addEventListener('resize',()=>{cam.aspect=el.clientWidth/el.clientHeight;cam.updateProjectionMatrix();r.setSize(el.clientWidth,el.clientHeight)});
(function loop(){requestAnimationFrame(loop);ctl.update();r.render(scene,cam)})();
