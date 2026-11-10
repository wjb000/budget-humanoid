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
const labelsLayer=document.createElement('div');labelsLayer.id='labels';el.appendChild(labelsLayer);
Object.values(G).forEach(g=>root.add(g));
G.envelope.visible=false;
const picks=[];
const KIND={real:['REAL vendor CAD','#3fb950'],modeled:['MODELLED from datasheet dimensions (see spec for what is approximate)','#a371f7'],generated:['GENERATED in this repo (parametric CadQuery)','#58a6ff'],envelope:['ENVELOPE: size-only box, not CAD','#d29922']};
const BUILD='v3-droid';   // bump with every model export so browsers never mix a new page with an old model/caption
const COLS={accent:0xc8742a,cover:0xe6e1d2,printed:0x4a5059,servo:0x1f2227,compute:0x2f7de1,power:0xf0861a,sensor:0x22b36a};
const creased=new Map(),edgesCache=new Map();
function smooth(g){if(!creased.has(g)){const c=toCreasedNormals(g.index?g.toNonIndexed():g,THREE.MathUtils.degToRad(35));creased.set(g,c)}return creased.get(g)}
function edges(g,ang){if(!edgesCache.has(g))edgesCache.set(g,new THREE.EdgesGeometry(g,ang));return edgesCache.get(g)}
tip.textContent='Loading CAD model...';
Promise.all([fetch('assets/robot_info.json?v='+BUILD,{cache:'no-cache'}).then(x=>x.json()),new Promise((ok,bad)=>new GLTFLoader().load('assets/robot.gltf?v='+BUILD,ok,undefined,bad))]).then(([info,gltf])=>{
  const SI={};for(const k in info)SI[THREE.PropertyBinding.sanitizeNodeName(k)]=info[k]; // three strips ':' '.' and spaces from node names
  const nodes=[];gltf.scene.traverse(o=>{if(o.isMesh)nodes.push(o)});
  for(const m of nodes){
    const meta=SI[m.name]||SI[m.parent?.name]||info[m.name];if(!meta)continue;
    m.updateWorldMatrix(true,false);const wm=m.matrixWorld.clone();
    const env=meta.kind==='envelope'&&!meta.show;     // e-stop and charge port are visible parts of the product; other envelopes stay hidden by default
    const grp=env?G.envelope:((meta.group==='cover'||meta.group==='accent')?G.cover:meta.group==='printed'?G.frame:meta.group==='servo'?G.servo:G.electronics);
    const col=meta.show?(/E-stop/.test(meta.name)?0xd62828:0xf2c200):(meta.color?new THREE.Color(meta.color):(COLS[meta.group]||0xcccccc));
    const g=env?m.geometry:smooth(m.geometry);
    const mat=env?new THREE.MeshStandardMaterial({color:col,transparent:true,opacity:.35,roughness:.8,depthWrite:false})
      :new THREE.MeshStandardMaterial({color:col,roughness:(meta.group==='printed'||meta.group==='cover')?.42:.4,metalness:meta.group==='servo'?.35:.05});
    const mesh=new THREE.Mesh(g,mat);wm.decompose(mesh.position,mesh.quaternion,mesh.scale);
    mesh.castShadow=!env;mesh.receiveShadow=!env;mesh.userData=meta;grp.add(mesh);picks.push(mesh);
    // crisp outlines make plates, pockets and bosses readable
    const lc=env?0xe3b341:meta.group==='servo'?0x7d8594:(meta.group==='printed'||meta.group==='cover')?0x56606e:0x1c222b;
    const e=new THREE.LineSegments(edges(m.geometry,env?20:32),new THREE.LineBasicMaterial({color:lc,transparent:true,opacity:env?.9:.55}));
    e.position.copy(mesh.position);e.quaternion.copy(mesh.quaternion);e.scale.copy(mesh.scale);grp.add(e);mesh.userData.edge=e;
  }
  const n={real:0,modeled:0,generated:0,envelope:0};picks.forEach(p=>n[p.userData.kind]=(n[p.userData.kind]||0)+1);
  tip.innerHTML=`${picks.length} bodies: <b style="color:#3fb950">${n.real} real vendor CAD</b>, <b style="color:#a371f7">${n.modeled} modelled from datasheets</b>, <b style="color:#58a6ff">${n.generated} generated printed parts</b>${n.envelope?`, <b style="color:#d29922">${n.envelope} size-only envelopes (hidden)</b>`:''}. Hover or click a part, or pick one in the list.`;
  buildList();if(n.envelope){const eb=document.getElementById('env-toggle');if(eb)eb.style.display=''}
  window.__b22={cam,ctl,G,setView,ready:true,select};
}).catch(e=>{tip.textContent='Could not load the 3D model ('+e+')'});
function sync(){for(const p of picks){const g=p.parent;if(p.userData.edge)p.userData.edge.visible=g.visible}}
document.querySelectorAll('[data-g]').forEach(b=>b.onclick=()=>{b.classList.toggle('on');const on=b.classList.contains('on');b.dataset.g.split(',').forEach(g=>{G[g].visible=on})});
document.getElementById('lbl-toggle')?.addEventListener('click',e=>e.currentTarget.classList.toggle('on'));
document.querySelectorAll('[data-v]').forEach(b=>b.onclick=()=>setView(b.dataset.v));
if(Q.get('env')==='1'){G.envelope.visible=true}
const ray=new THREE.Raycaster(),mp=new THREE.Vector2();let hov=null;
function setHov(o){const i=o?ENT.findIndex(x=>x.meshes.includes(o)):-1;if(i!==hovI)hover(i)}
r.domElement.addEventListener('pointermove',e=>{const b=r.domElement.getBoundingClientRect();mp.set((e.clientX-b.left)/b.width*2-1,-(e.clientY-b.top)/b.height*2+1);ray.setFromCamera(mp,cam);
  const h=ray.intersectObjects(picks.filter(p=>p.parent.visible))[0];setHov(h?h.object:null);
  if(h){const u=h.object.userData,k=KIND[u.kind]||['',''];tip.innerHTML=`<b>${u.name}</b><br><span style="color:${k[1]}">${k[0]}</span><br>${u.spec}<br><span style="color:#9aa7b4">${u.price||''}${u.src?' &middot; '+u.src:''}</span>`}});
addEventListener('resize',()=>{cam.aspect=el.clientWidth/el.clientHeight;cam.updateProjectionMatrix();r.setSize(el.clientWidth,el.clientHeight)});


// ---------------- component list, 3D labels, selection ----------------
const ENT=[];                                   // {key,title,spec,price,kind,meshes,anchor,label,row}
function entryKey(u){
  const nm=u.name||'';
  let m=nm.match(/^Servo ID \d+: (.+)$/);
  if(m){const j=m[1],s=j[0]==='r'?'Right':j[0]==='l'?'Left':'';
    const leg=/hip|knee|ankle/.test(j),neck=/neck/.test(j);
    return neck?['servo:neck','Neck servo']:[`servo:${j[0]}${leg?'leg':'arm'}`,`${s} ${leg?'leg':'arm'} servos`]}
  m=nm.match(/^Printed part: (.+)$/);
  if(m){const p=m[1].replace(/^[lr]_/,'');return ['part:'+p,p.replace(/_/g,' ').replace(/^./,c=>c.toUpperCase())+(/^[lr]_/.test(m[1])?' (left + right)':'')]}
  return ['c:'+nm,nm];
}
function buildList(){
  const map=new Map();
  for(const p of picks){const u=p.userData,[k,t]=entryKey(u);
    if(!map.has(k)){map.set(k,{key:k,title:t,spec:u.spec,price:u.price,kind:u.kind,group:u.group,meshes:[],src:u.src});ENT.push(map.get(k))}
    map.get(k).meshes.push(p)}
  const order={compute:0,power:1,sensor:2,servo:3,cover:4,accent:4,printed:4};
  ENT.sort((a,b)=>(order[a.group]??5)-(order[b.group]??5)||a.title.localeCompare(b.title));
  ENT.forEach((e,i)=>{
    e.n=i+1;
    const box=new THREE.Box3();e.meshes.forEach(m=>{m.updateWorldMatrix(true,false);box.expandByObject(m)});
    e.box=box;e.anchor=box.getCenter(new THREE.Vector3());
    if(e.meshes.length>1&&e.group==='printed'||e.meshes.length>1&&e.group==='cover'){const b1=new THREE.Box3().setFromObject(e.meshes[0]);e.anchor=b1.getCenter(new THREE.Vector3())}
    const L=document.createElement('div');L.className='lbl';L.textContent=e.n;L.dataset.i=i;labelsLayer.appendChild(L);e.label=L;
    L.onclick=()=>select(i,true);L.onmouseenter=()=>hover(i);L.onmouseleave=()=>hover(-1);
  });
  const list=document.getElementById('vlist');if(!list)return;
  list.innerHTML=ENT.map((e,i)=>{const k=KIND[e.kind]||['',''];return `<div class="vitem" data-i="${i}"><span class="vnum">${e.n}</span><div><b>${e.title}</b><small>${e.spec||''}</small><small style="color:${k[1]}">${k[0]?k[0].split(' (')[0]:''}${e.price?' &middot; '+e.price:''}</small></div></div>`}).join('');
  list.querySelectorAll('.vitem').forEach(r=>{const i=+r.dataset.i;r.onclick=()=>select(i,true);r.onmouseenter=()=>hover(i);r.onmouseleave=()=>hover(-1)});
}
let selI=-1,hovI=-1;
function paint(){
  picks.forEach(p=>p.material.emissive&&p.material.emissive.setHex(0));
  const hi=(i,c)=>{if(i<0)return;ENT[i].meshes.forEach(p=>p.material.emissive&&p.material.emissive.setHex(c))};
  hi(selI,0x7a4a00);hi(hovI,0x223c6e);
  document.querySelectorAll('.vitem').forEach(r=>r.classList.toggle('sel',+r.dataset.i===selI));
  document.querySelectorAll('.vitem').forEach(r=>r.classList.toggle('hov',+r.dataset.i===hovI));
  ENT.forEach((e,i)=>{if(e.label){e.label.classList.toggle('sel',i===selI);e.label.classList.toggle('hov',i===hovI)}});
}
function hover(i){hovI=i;paint()}
let anim=null;
function select(i,focus){
  selI=(i===selI&&!focus)?-1:i;paint();
  if(i<0)return;const e=ENT[i];
  const row=document.querySelector(`.vitem[data-i="${i}"]`);if(row)row.scrollIntoView({block:'nearest',behavior:'smooth'});
  if(focus){
    const size=e.box.getSize(new THREE.Vector3()).length();
    const c=e.meshes.length>1&&(e.group==='printed'||e.group==='cover')?e.anchor.clone():e.box.getCenter(new THREE.Vector3());
    const dir=cam.position.clone().sub(ctl.target).normalize();
    const dist=Math.max(260,size*2.6);
    anim={t0:performance.now(),from:{p:cam.position.clone(),t:ctl.target.clone()},to:{t:c,p:c.clone().add(dir.multiplyScalar(dist))}};
  }
  const u=e.meshes[0].userData,k=KIND[e.kind]||['',''];
  tip.innerHTML=`<b>${e.title}</b>${e.meshes.length>1?` (${e.meshes.length} bodies)`:''}<br><span style="color:${k[1]}">${k[0]}</span><br>${e.spec||''}<br><span style="color:#9aa7b4">${e.price||''}${e.src?' &middot; '+e.src:''}</span>`;
}
function stepAnim(){if(!anim)return;const t=Math.min(1,(performance.now()-anim.t0)/450),k=t*t*(3-2*t);
  ctl.target.lerpVectors(anim.from.t,anim.to.t,k);cam.position.lerpVectors(anim.from.p,anim.to.p,k);if(t>=1)anim=null}
function placeLabels(){
  const show=document.getElementById('lbl-toggle')?.classList.contains('on')!==false;
  const w=el.clientWidth,h=el.clientHeight,v=new THREE.Vector3();
  for(const e of ENT){
    const vis=show&&e.meshes.some(m=>m.parent.visible);
    if(!vis){e.label.style.display='none';continue}
    v.copy(e.anchor).project(cam);
    if(v.z>1||v.z<-1){e.label.style.display='none';continue}
    e.label.style.display='';e.label.style.transform=`translate(${(v.x*.5+.5)*w}px,${(-v.y*.5+.5)*h}px) translate(-50%,-50%)`;
  }
}
// viewer picking: click selects (and focuses on double-click), hover highlights the list row
r.domElement.addEventListener('click',e=>{const b=r.domElement.getBoundingClientRect();mp.set((e.clientX-b.left)/b.width*2-1,-(e.clientY-b.top)/b.height*2+1);ray.setFromCamera(mp,cam);
  const h=ray.intersectObjects(picks.filter(p=>p.parent.visible))[0];if(!h){selI=-1;paint();return}
  const i=ENT.findIndex(x=>x.meshes.includes(h.object));if(i>=0)select(i,false)});
document.getElementById('reset-view')?.addEventListener('click',()=>{selI=-1;paint();setView('34')});

(function loop(){requestAnimationFrame(loop);stepAnim();ctl.update();sync();placeLabels();r.render(scene,cam)})();
