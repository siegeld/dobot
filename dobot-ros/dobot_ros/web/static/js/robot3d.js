// ── Dobot CR5 3D Visualization ─────────────────────────────
// Three.js scene with URDF-based kinematic chain and Collada meshes.
// Driven by real-time joint angles from the WebSocket feed.

import * as THREE from 'three';
import { ColladaLoader } from 'three/addons/loaders/ColladaLoader.js';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const MESH_PATH = '/static/meshes/cr5/';

// URDF joint origins and fixed rotations (from cr5_robot.urdf)
// Each entry: { xyz: [x,y,z], rpy: [roll, pitch, yaw] }
const JOINTS = [
  { xyz: [0, 0, 0.147],       rpy: [0, 0, 0] },                // joint1: base_link -> Link1
  { xyz: [0, 0, 0],           rpy: [Math.PI/2, Math.PI/2, 0] },// joint2: Link1 -> Link2
  { xyz: [-0.427, 0, 0],      rpy: [0, 0, 0] },                // joint3: Link2 -> Link3
  { xyz: [-0.357, 0, 0.141],  rpy: [0, 0, -Math.PI/2] },       // joint4: Link3 -> Link4
  { xyz: [0, -0.116, 0],      rpy: [Math.PI/2, 0, 0] },        // joint5: Link4 -> Link5
  { xyz: [0, 0.105, 0],       rpy: [-Math.PI/2, 0, 0] },       // joint6: Link5 -> Link6
];

const LINK_FILES = ['Link1.dae', 'Link2.dae', 'Link3.dae', 'Link4.dae', 'Link5.dae', 'Link6.dae'];

// Per-link material overrides (Dobot CR5 color scheme)
// base_link = dark charcoal, Link1 = dark joint cover, Links 2-3 = white arm, Links 4-6 = lighter wrist
const LINK_COLORS = {
  'base_link': { color: 0x8a8a94, metalness: 0.4, roughness: 0.5 },
  'Link1':     { color: 0x9a9aa4, metalness: 0.35, roughness: 0.45 },
  'Link2':     { color: 0xd8dce2, metalness: 0.15, roughness: 0.35 },
  'Link3':     { color: 0xd8dce2, metalness: 0.15, roughness: 0.35 },
  'Link4':     { color: 0x9a9aa4, metalness: 0.35, roughness: 0.45 },
  'Link5':     { color: 0x9a9aa4, metalness: 0.35, roughness: 0.45 },
  'Link6':     { color: 0xa5a5ae, metalness: 0.3,  roughness: 0.4 },
};

let scene, camera, renderer, controls;
let jointPivots = [];   // 6 Group objects — rotation.z drives each joint
let robotRoot;
let animationId;

function init() {
  const container = document.getElementById('robot-3d-container');
  if (!container) return;

  // Scene
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0d1117);

  // Camera
  const w = container.clientWidth;
  const h = container.clientHeight;
  camera = new THREE.PerspectiveCamera(40, w / h, 0.01, 10);
  camera.position.set(0.9, 0.7, 1.1);

  // Renderer
  renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
  renderer.setSize(w, h);
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 0.9;
  container.appendChild(renderer.domElement);

  // Controls
  controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0.35, 0);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.minDistance = 0.3;
  controls.maxDistance = 3;
  controls.update();

  // Lights — hemisphere for ambient sky/ground tint, then directional for definition
  scene.add(new THREE.HemisphereLight(0x8ec8ff, 0x1a1a2e, 1.2));

  const key = new THREE.DirectionalLight(0xffffff, 1.8);
  key.position.set(2, 4, 3);
  scene.add(key);

  const fill = new THREE.DirectionalLight(0xb0d0ff, 0.6);
  fill.position.set(-3, 2, -1);
  scene.add(fill);

  const rim = new THREE.DirectionalLight(0x58a6ff, 0.8);
  rim.position.set(-1, 1, -3);
  scene.add(rim);

  // Ground grid
  const grid = new THREE.GridHelper(2, 20, 0x30363d, 0x1c2129);
  scene.add(grid);

  // Robot root — URDF is Z-up, Three.js is Y-up
  robotRoot = new THREE.Group();
  robotRoot.rotation.x = -Math.PI / 2;
  scene.add(robotRoot);

  loadRobot();
  loadSavedCamera();

  // Resize handling
  const ro = new ResizeObserver(() => onResize(container));
  ro.observe(container);

  animate();
}

// Replace Collada materials with PBR MeshStandardMaterial and strip embedded lights/cameras
function upgradeMaterials(object, linkName) {
  const props = LINK_COLORS[linkName] || { color: 0xcccccc, metalness: 0.2, roughness: 0.4 };
  const toRemove = [];
  object.traverse((child) => {
    if (child.isLight || child.isCamera) {
      toRemove.push(child);
      return;
    }
    if (!child.isMesh) return;
    child.material = new THREE.MeshStandardMaterial({
      color: props.color,
      metalness: props.metalness,
      roughness: props.roughness,
    });
    child.castShadow = true;
    child.receiveShadow = true;
  });
  toRemove.forEach((obj) => obj.parent && obj.parent.remove(obj));
}

function loadRobot() {
  const loader = new ColladaLoader();

  // Base link mesh
  loader.load(MESH_PATH + 'base_link.dae', (col) => {
    upgradeMaterials(col.scene, 'base_link');
    // DAE files are Z_UP; ColladaLoader auto-converts to Y_UP, but our
    // kinematic chain is in URDF (Z_UP) coords — undo the conversion.
    const wrap = new THREE.Group();
    wrap.rotation.x = Math.PI / 2;
    wrap.add(col.scene);
    robotRoot.add(wrap);
  });

  // Build kinematic chain: jointGroup (origin transform) -> pivot (animated rotation) -> ...
  let parent = robotRoot;

  LINK_FILES.forEach((file, i) => {
    const jd = JOINTS[i];

    // Fixed transform from URDF <joint><origin>
    const jointGroup = new THREE.Group();
    jointGroup.position.set(jd.xyz[0], jd.xyz[1], jd.xyz[2]);
    jointGroup.quaternion.setFromEuler(
      new THREE.Euler(jd.rpy[0], jd.rpy[1], jd.rpy[2], 'ZYX')
    );

    // Animated pivot — all URDF axes are [0,0,1] so rotation.z = joint angle
    const pivot = new THREE.Group();
    jointPivots.push(pivot);

    jointGroup.add(pivot);
    parent.add(jointGroup);

    // Load link mesh into pivot (counter-rotate to undo ColladaLoader Z_UP→Y_UP)
    const linkName = file.replace('.dae', '');
    loader.load(MESH_PATH + file, (col) => {
      upgradeMaterials(col.scene, linkName);
      const wrap = new THREE.Group();
      wrap.rotation.x = Math.PI / 2;
      wrap.add(col.scene);
      pivot.add(wrap);
    });

    parent = pivot;
  });
}

function updateJoints(anglesDeg) {
  if (!anglesDeg || anglesDeg.length < 6) return;
  for (let i = 0; i < 6; i++) {
    if (jointPivots[i]) {
      jointPivots[i].rotation.z = anglesDeg[i] * (Math.PI / 180);
    }
  }
}

function getCameraState() {
  if (!camera || !controls) return null;
  return {
    position: { x: camera.position.x, y: camera.position.y, z: camera.position.z },
    target: { x: controls.target.x, y: controls.target.y, z: controls.target.z },
  };
}

function applyCameraState(state) {
  if (!camera || !controls || !state) return;
  camera.position.set(state.position.x, state.position.y, state.position.z);
  controls.target.set(state.target.x, state.target.y, state.target.z);
  controls.update();
}

async function saveCamera() {
  const state = getCameraState();
  if (!state) return;
  await fetch('/api/camera', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(state),
  });
}

async function resetCamera() {
  try {
    const res = await fetch('/api/camera');
    const state = await res.json();
    if (state && state.position) {
      applyCameraState(state);
      return;
    }
  } catch (e) { /* fall through to defaults */ }
  // Default view
  if (!camera || !controls) return;
  camera.position.set(0.9, 0.7, 1.1);
  controls.target.set(0, 0.35, 0);
  controls.update();
}

async function loadSavedCamera() {
  try {
    const res = await fetch('/api/camera');
    const state = await res.json();
    if (state && state.position) applyCameraState(state);
  } catch (e) { /* use defaults */ }
}

function onResize(container) {
  if (!camera || !renderer) return;
  const w = container.clientWidth;
  const h = container.clientHeight;
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
  renderer.setSize(w, h);
}

function animate() {
  animationId = requestAnimationFrame(animate);
  if (controls) controls.update();
  if (renderer && scene && camera) renderer.render(scene, camera);
}

// ── Bootstrap ────────────────────────────────────────────────

if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', init);
} else {
  init();
}

// Expose to non-module app.js
window.updateRobotJoints = updateJoints;
window.resetRobotCamera = resetCamera;
window.saveRobotCamera = saveCamera;
