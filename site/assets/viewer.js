import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {GLTFLoader} from 'three/addons/loaders/GLTFLoader.js';
import {toCreasedNormals} from 'three/addons/utils/BufferGeometryUtils.js';
const el=document.getElementById('viewer'),tip=document.getElementById('tip');
const Q=new URLSearchParams(location.search);
const scene=new THREE.Scene();
const cam=new THREE.PerspectiveCamera(32,el.clientWidth/el.clientHeight,10,8000);
const r=new THREE.WebGLRenderer({antialias:true,alpha:true});
r.setPixelRatio(Math.min(devicePixelRatio,2));r.setSize(el.clientWidth,el.clientHeight);
r.shadowMap.enabled=true;r.shadowMap.type=THREE.PCFSoftShadowMap;
r.toneMapping=THREE.ACESFilmicToneMapping;r.toneMappingExposure=1.05;
el.appendChild(r.domElement);
const ctl=new OrbitControls(cam,r.domElement);ctl.target.set(0,300,0);ctl.enableDamping=true;ctl.minDistance=150;ctl.maxDistance=2500;
// views: robot frame x forward, y left, z up -> three (x, z, -y)
const VIEWS={front:[1500,300,0],side:[0,300,1500],back:[-1500,300,0],'34':[1000,520,900],top:[1,1700,0]};
function setView(n){const v=VIEWS[n]||VIEWS['34'];cam.position.set(...v);ctl.target.set(0,300,0);ctl.update()}
setView(Q.get('view')||'34');
// lighting: soft sky/ground ambient + key + fill + rim, key casts shadows
scene.add(new THREE.HemisphereLight(0xdfe8ff,0x2a2f3a,1.1));
const key=new THREE.DirectionalLight(0xffffff,1.9);key.position.set(500,900,700);key.castShadow=true;
key.shadow.mapSize.set(2048,2048);Object.assign(key.shadow.camera,{left:-500,right:500,top:900,bottom:-200,near:100,far:2500});key.shadow.bias=-0.0006;key.shadow.normalBias=1.5;scene.add(key);
const fill=new THREE.DirectionalLight(0xaec6ff,0.9);fill.position.set(-700,300,500);scene.add(fill);
const rim=new THREE.DirectionalLight(0xffe0c0,0.8);rim.position.set(-300,500,-800);scene.add(rim);
const floor=new THREE.Mesh(new THREE.CircleGeometry(700,64),new THREE.MeshStandardMaterial({color:0x1a212b,roughness:.9,metalness:0}));floor.rotation.x=-Math.PI/2;floor.position.y=-1;floor.receiveShadow=true;scene.add(floor);
const grid=new THREE.GridHelper(1000,40,0x3a4656,0x252d39);grid.position.y=-.5;scene.add(grid);
const root=new THREE.Group();root.rotation.x=-Math.PI/2;scene.add(root);
const G={cover:new THREE.Group(),frame:new THREE.Group(),servo:new THREE.Group(),electronics:new THREE.Group(),envelope:new THREE.Group()};
Object.values(G).forEach(g=>root.add(g));
G.envelope.visible=false;
const picks=[];
const KIND={real:['REAL vendor CAD','#3fb950'],generated:['GENERATED in this repo (parametric CadQuery)','#58a6ff'],envelope:['ENVELOPE: size-only box, not CAD','#d29922']};
const COLS={cover:0xf3f5f8,printed:0x8f9bb0,servo:0x30343c,compute:0x2f7de1,power:0xf0861a,sensor:0x22b36a};
const creased=new Map(),edgesCache=new Map();
function smooth(g){if(!creased.has(g)){const c=toCreasedNormals(g.index?g.toNonIndexed():g,THREE.MathUtils.degToRad(35));creased.set(g,c)}return creased.get(g)}
function edges(g,ang){if(!edgesCache.has(g))edgesCache.set(g,new THREE.EdgesGeometry(g,ang));return edgesCache.get(g)}
tip.textContent='Loading CAD model...';
Promise.all([fetch('assets/robot_info.json').then(x=>x.json()),new Promise((ok,bad)=>new GLTFLoader().load('assets/robot.gltf',ok,undefined,bad))]).then(([info,gltf])=>{
  const SI={};for(const k in info)SI[THREE.PropertyBinding.sanitizeNodeName(k)]=info[k]; // three strips ':' '.' and spaces from node names
  const nodes=[];gltf.scene.traverse(o=>{if(o.isMesh)nodes.push(o)});
  for(const m of nodes){
    const meta=SI[m.name]||SI[m.parent?.name]||info[m.name];if(!meta)continue;
    m.updateWorldMatrix(true,false);const wm=m.matrixWorld.clone();
    const env=meta.kind==='envelope'&&!meta.show;     // e-stop and charge port are visible parts of the product; other envelopes stay hidden by default
    const grp=env?G.envelope:(meta.group==='cover'?G.cover:meta.group==='printed'?G.frame:meta.group==='servo'?G.servo:G.electronics);
    const col=meta.show?(/E-stop/.test(meta.name)?0xd62828:0xf2c200):(COLS[meta.group]||0xcccccc);
    const g=env?m.geometry:smooth(m.geometry);
    const mat=env?new THREE.MeshStandardMaterial({color:col,transparent:true,opacity:.35,roughness:.8,depthWrite:false})
      :new THREE.MeshStandardMaterial({color:col,roughness:(meta.group==='printed'||meta.group==='cover')?.55:.4,metalness:meta.group==='servo'?.35:.05});
    const mesh=new THREE.Mesh(g,mat);wm.decompose(mesh.position,mesh.quaternion,mesh.scale);
    mesh.castShadow=!env;mesh.receiveShadow=!env;mesh.userData=meta;grp.add(mesh);picks.push(mesh);
    // crisp outlines make plates, pockets and bosses readable
    const lc=env?0xe3b341:meta.group==='servo'?0x7d8594:(meta.group==='printed'||meta.group==='cover')?0x56606e:0x1c222b;
    const e=new THREE.LineSegments(edges(m.geometry,env?20:32),new THREE.LineBasicMaterial({color:lc,transparent:true,opacity:env?.9:.55}));
    e.position.copy(mesh.position);e.quaternion.copy(mesh.quaternion);e.scale.copy(mesh.scale);grp.add(e);mesh.userData.edge=e;
  }
  const n={real:0,generated:0,envelope:0};picks.forEach(p=>n[p.userData.kind]++);
  tip.innerHTML=`${picks.length} bodies: <b style="color:#3fb950">${n.real} real vendor CAD</b> (servos, Pi 5, IMU, amp, speaker), <b style="color:#58a6ff">${n.generated} generated printed parts</b>, <b style="color:#d29922">${n.envelope} size-only envelopes (hidden, use the toggle)</b>. Hover a part.`;
  window.__b22={cam,ctl,G,setView,ready:true};
}).catch(e=>{tip.textContent='Could not load the 3D model ('+e+')'});
function sync(){for(const p of picks){const g=p.parent;if(p.userData.edge)p.userData.edge.visible=g.visible}}
document.querySelectorAll('[data-g]').forEach(b=>b.onclick=()=>{b.classList.toggle('on');G[b.dataset.g].visible=b.classList.contains('on')});
document.querySelectorAll('[data-v]').forEach(b=>b.onclick=()=>setView(b.dataset.v));
if(Q.get('env')==='1'){G.envelope.visible=true}
const ray=new THREE.Raycaster(),mp=new THREE.Vector2();let hov=null;
function setHov(o){if(hov)hov.material.emissive?.setHex(0);hov=o;if(o)o.material.emissive?.setHex(0x223049)}
r.domElement.addEventListener('pointermove',e=>{const b=r.domElement.getBoundingClientRect();mp.set((e.clientX-b.left)/b.width*2-1,-(e.clientY-b.top)/b.height*2+1);ray.setFromCamera(mp,cam);
  const h=ray.intersectObjects(picks.filter(p=>p.parent.visible))[0];setHov(h?h.object:null);
  if(h){const u=h.object.userData,k=KIND[u.kind]||['',''];tip.innerHTML=`<b>${u.name}</b><br><span style="color:${k[1]}">${k[0]}</span><br>${u.spec}<br><span style="color:#9aa7b4">${u.price||''}${u.src?' &middot; '+u.src:''}</span>`}});
addEventListener('resize',()=>{cam.aspect=el.clientWidth/el.clientHeight;cam.updateProjectionMatrix();r.setSize(el.clientWidth,el.clientHeight)});
(function loop(){requestAnimationFrame(loop);ctl.update();sync();r.render(scene,cam)})();
