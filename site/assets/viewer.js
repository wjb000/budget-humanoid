import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {GLTFLoader} from 'three/addons/loaders/GLTFLoader.js';
const el=document.getElementById('viewer'),tip=document.getElementById('tip');
const scene=new THREE.Scene();
const cam=new THREE.PerspectiveCamera(38,el.clientWidth/el.clientHeight,1,6000);
cam.position.set(520,420,820);
const r=new THREE.WebGLRenderer({antialias:true,alpha:true});r.setPixelRatio(Math.min(devicePixelRatio,2));r.setSize(el.clientWidth,el.clientHeight);el.appendChild(r.domElement);
const ctl=new OrbitControls(cam,r.domElement);ctl.target.set(0,290,0);ctl.update();
scene.add(new THREE.HemisphereLight(0xffffff,0x334455,1.2));
const dl=new THREE.DirectionalLight(0xffffff,1.8);dl.position.set(300,700,500);scene.add(dl);
const dl2=new THREE.DirectionalLight(0xffffff,.6);dl2.position.set(-400,200,-300);scene.add(dl2);
scene.add(new THREE.GridHelper(600,30,0x445566,0x2a3340));
// glTF is in robot frame (mm, x fwd, y left, z up); three is y-up: rotate -90 deg about x
const root=new THREE.Group();root.rotation.x=-Math.PI/2;scene.add(root);
const G={frame:new THREE.Group(),servo:new THREE.Group(),compute:new THREE.Group(),power:new THREE.Group(),sensor:new THREE.Group()};
Object.values(G).forEach(g=>root.add(g));
const picks=[];
const KIND={real:['REAL vendor CAD','#3fb950'],generated:['GENERATED in this repo (parametric CadQuery)','#58a6ff'],envelope:['ENVELOPE: size-only box, not CAD','#d29922']};
const COLS={compute:0x1f6fe0,power:0xe8710a,sensor:0x1a9a55};
tip.textContent='Loading CAD model...';
Promise.all([fetch('assets/robot_info.json').then(x=>x.json()),new Promise((ok,bad)=>new GLTFLoader().load('assets/robot.gltf',ok,undefined,bad))]).then(([info,gltf])=>{
  const SI={};for(const k in info)SI[THREE.PropertyBinding.sanitizeNodeName(k)]=info[k]; // three strips ':' '.' and spaces from node names
  const nodes=[];gltf.scene.traverse(o=>{if(o.isMesh)nodes.push(o)});
  for(const m of nodes){
    let meta=SI[m.name]||SI[m.parent?.name]||info[m.name];
    if(!meta){continue}
    m.updateWorldMatrix(true,false);
    const wm=m.matrixWorld.clone();
    // detach with world transform baked into the object
    const g=m.geometry;
    let mat,grp=G[meta.group==='printed'?'frame':meta.group]||G.frame;
    if(meta.kind==='envelope'){
      mat=new THREE.MeshStandardMaterial({color:COLS[meta.group]||0x888888,transparent:true,opacity:.45,roughness:.8,depthWrite:false,flatShading:true});
    }else{
      const c=meta.group==='printed'?0xcfd6df:meta.group==='servo'?0x4a505c:(COLS[meta.group]||0xcfd6df);
      mat=new THREE.MeshStandardMaterial({color:c,roughness:.55,metalness:.12,flatShading:true});
    }
    const mesh=new THREE.Mesh(g,mat);
    wm.decompose(mesh.position,mesh.quaternion,mesh.scale);
    mesh.userData=meta;grp.add(mesh);picks.push(mesh);
    if(meta.kind==='envelope'){
      const e=new THREE.LineSegments(new THREE.EdgesGeometry(g,20),new THREE.LineDashedMaterial({color:0xe3b341,dashSize:3,gapSize:2}));
      e.computeLineDistances();e.position.copy(mesh.position);e.quaternion.copy(mesh.quaternion);e.scale.copy(mesh.scale);grp.add(e);
    }
  }
  const n={real:0,generated:0,envelope:0};picks.forEach(p=>n[p.userData.kind]++);
  tip.innerHTML=`Loaded ${picks.length} bodies: <b style="color:#3fb950">${n.real} real vendor CAD</b>, <b style="color:#58a6ff">${n.generated} generated printed parts</b>, <b style="color:#d29922">${n.envelope} size-only envelopes</b>. Hover a part.`;
}).catch(e=>{tip.textContent='Could not load the 3D model ('+e+')'});
document.querySelectorAll('[data-g]').forEach(b=>b.onclick=()=>{b.classList.toggle('on');G[b.dataset.g].visible=b.classList.contains('on')});
const ray=new THREE.Raycaster(),mp=new THREE.Vector2();let hov=null;
function setHov(o){if(hov)hov.material.emissive?.setHex(0);hov=o;if(o)o.material.emissive?.setHex(0x2a3340)}
r.domElement.addEventListener('pointermove',e=>{const b=r.domElement.getBoundingClientRect();mp.set((e.clientX-b.left)/b.width*2-1,-(e.clientY-b.top)/b.height*2+1);ray.setFromCamera(mp,cam);
  const h=ray.intersectObjects(picks.filter(p=>p.parent.visible))[0];setHov(h?h.object:null);
  if(h){const u=h.object.userData,k=KIND[u.kind]||['',''];tip.innerHTML=`<b>${u.name}</b><br><span style="color:${k[1]}">${k[0]}</span><br>${u.spec}<br><span style="color:#9aa7b4">${u.price||''}${u.src?' &middot; '+u.src:''}</span>`}});
addEventListener('resize',()=>{cam.aspect=el.clientWidth/el.clientHeight;cam.updateProjectionMatrix();r.setSize(el.clientWidth,el.clientHeight)});
(function loop(){requestAnimationFrame(loop);ctl.update();r.render(scene,cam)})();
