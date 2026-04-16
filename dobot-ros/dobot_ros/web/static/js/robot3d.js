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

// Gripper state
let gripperGroup = null;     // Group attached after Link6
let gripperFingerL = null;
let gripperFingerR = null;
let gripperTargetSpread = 0; // 0..0.0525 (half of 105mm max stroke, in meters)
let gripperCurrentSpread = 0;

// Simulation state
let simulating = false;
let simSteps = [];
let simIndex = 0;
let simStartTime = 0;
let simStepDuration = 1.0;  // seconds per step
let simGhostRoot = null;     // transparent clone of robot for "current real pose"
let simGhostPivots = [];
let simGhostGripperL = null;
let simGhostGripperR = null;
let simOnComplete = null;
let simBadge = null;
let simLabel = null;
let simLiveFrozenJoints = null;
let simLiveFrozenGripper = 0;

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
  createSimBadge(container);

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

    // After the last link, attach the gripper.
    if (i === LINK_FILES.length - 1) {
      buildGripper(pivot);
    }

    parent = pivot;
  });
}

function updateJoints(anglesDeg) {
  if (!anglesDeg || anglesDeg.length < 6) return;
  // During simulation the live feed drives the ghost, not the main robot.
  if (simulating) {
    simLiveFrozenJoints = anglesDeg.slice();
    updateGhostJoints(anglesDeg);
    return;
  }
  for (let i = 0; i < 6; i++) {
    if (jointPivots[i]) {
      jointPivots[i].rotation.z = anglesDeg[i] * (Math.PI / 180);
    }
  }
}

function setJointsDirect(anglesDeg) {
  // Set main robot joints directly (used by simulation, bypasses live-guard).
  for (let i = 0; i < 6; i++) {
    if (jointPivots[i]) {
      jointPivots[i].rotation.z = anglesDeg[i] * (Math.PI / 180);
    }
  }
}

function updateGripperPosition(pos01) {
  // pos01: 0 = closed, 1 = fully open (maps to 0-52.5mm half-stroke each side)
  gripperTargetSpread = pos01 * 0.0525;
}

function updateGripper(pos0to1000) {
  // From WebSocket gripper_position (0-1000).
  if (simulating) {
    simLiveFrozenGripper = pos0to1000;
    return;
  }
  updateGripperPosition(pos0to1000 / 1000.0);
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

// (old animate/bootstrap removed — enhanced versions are below)

// ── Gripper geometry ────────────────────────────────────────────
// AG-105: 105mm stroke (0 closed → 105mm open). Flange-to-tip ~203mm.
// Procedural: mount block + two finger pads that translate ±X.

function buildGripper(parent) {
  gripperGroup = new THREE.Group();
  // Flush against the Link6 flange face. The joint6 pivot is at the
  // origin; the Link6 mesh extends a short distance in +Z. Setting
  // the gripper origin at Z=0 (or a tiny nudge) seats it flush.
  gripperGroup.position.set(0, 0, 0);

  // Colors match the wrist/Link6 scheme so it looks integrated, not floating.
  const bodyMat = new THREE.MeshStandardMaterial({
    color: 0xb0b0b8, metalness: 0.35, roughness: 0.4,
  });
  const fingerMat = new THREE.MeshStandardMaterial({
    color: 0xc8c8d0, metalness: 0.3, roughness: 0.35,
  });

  // AG-105 dimensions: 203mm flange-to-tip total.
  // Mounting adapter: ~15mm, body/housing: ~100mm, fingers: ~88mm.

  // Mounting plate (thin disc flush to flange)
  const plateGeo = new THREE.CylinderGeometry(0.028, 0.028, 0.015, 16);
  const plate = new THREE.Mesh(plateGeo, bodyMat);
  plate.rotation.x = Math.PI / 2;
  plate.position.set(0, 0, 0.0075);
  gripperGroup.add(plate);

  // Gripper body (housing) — 100mm long
  const bodyGeo = new THREE.BoxGeometry(0.055, 0.038, 0.10);
  const body = new THREE.Mesh(bodyGeo, bodyMat);
  body.position.set(0, 0, 0.065);
  gripperGroup.add(body);

  // Fingers — 88mm long, tips reach Z=0.203 (203mm total)
  const fingerGeo = new THREE.BoxGeometry(0.010, 0.022, 0.088);
  gripperFingerL = new THREE.Mesh(fingerGeo, fingerMat);
  gripperFingerL.position.set(-0.005, 0, 0.159);
  gripperGroup.add(gripperFingerL);

  gripperFingerR = new THREE.Mesh(fingerGeo, fingerMat);
  gripperFingerR.position.set(0.005, 0, 0.159);
  gripperGroup.add(gripperFingerR);

  parent.add(gripperGroup);
}

function animateGripperFingers() {
  if (!gripperFingerL || !gripperFingerR) return;
  // Lerp toward target spread for smooth animation.
  gripperCurrentSpread += (gripperTargetSpread - gripperCurrentSpread) * 0.15;
  // Fingers slide apart symmetrically in ±X from center.
  gripperFingerL.position.x = -(0.005 + gripperCurrentSpread);
  gripperFingerR.position.x =  (0.005 + gripperCurrentSpread);
}


// ── Simulation mode ─────────────────────────────────────────────

function createSimBadge(container) {
  simBadge = document.createElement('div');
  simBadge.style.cssText =
    'position:absolute; top:8px; left:8px; z-index:10; display:none; ' +
    'background:rgba(255,165,0,0.85); color:#000; font-weight:700; ' +
    'padding:3px 10px; border-radius:4px; font-size:13px; pointer-events:none;';
  simBadge.textContent = 'SIMULATION';
  container.style.position = 'relative';
  container.appendChild(simBadge);

  simLabel = document.createElement('div');
  simLabel.style.cssText =
    'position:absolute; bottom:8px; left:8px; z-index:10; display:none; ' +
    'background:rgba(0,0,0,0.7); color:#fff; padding:2px 8px; ' +
    'border-radius:4px; font-size:12px; pointer-events:none;';
  container.appendChild(simLabel);
}

function buildGhostRobot() {
  // Clone the kinematic chain as a transparent "ghost" showing the real pose.
  if (simGhostRoot) { robotRoot.parent.remove(simGhostRoot); }
  simGhostRoot = robotRoot.clone(true);
  simGhostPivots = [];

  // Make all meshes transparent.
  simGhostRoot.traverse((child) => {
    if (child.isMesh) {
      child.material = child.material.clone();
      child.material.transparent = true;
      child.material.opacity = 0.25;
      child.material.depthWrite = false;
    }
  });

  // Find the pivots in the cloned tree (same order as original).
  // The structure is: robotRoot > jointGroup > pivot > ... repeated.
  // We walk depth-first and collect Groups that are children of Groups
  // and have children of their own — same pattern as the original build.
  function collectPivots(node, depth, acc) {
    // jointGroup is at even depth, pivot at odd depth in the chain.
    node.children.forEach(child => {
      if (child.isGroup) {
        // Heuristic: pivot has rotation capability; jointGroup has position offset.
        // In the original, jointPivots are at indices 0,1,2,3,4,5.
        if (depth % 2 === 1 && acc.length < 6) {
          acc.push(child);
        }
        collectPivots(child, depth + 1, acc);
      }
    });
    return acc;
  }
  simGhostPivots = collectPivots(simGhostRoot, 0, []);

  robotRoot.parent.add(simGhostRoot);
}

function updateGhostJoints(anglesDeg) {
  if (!simGhostPivots.length) return;
  for (let i = 0; i < 6 && i < simGhostPivots.length; i++) {
    simGhostPivots[i].rotation.z = anglesDeg[i] * (Math.PI / 180);
  }
}

function removeGhost() {
  if (simGhostRoot && simGhostRoot.parent) {
    simGhostRoot.parent.remove(simGhostRoot);
  }
  simGhostRoot = null;
  simGhostPivots = [];
}

function lerp(a, b, t) { return a + (b - a) * t; }
function lerpArray(a, b, t) { return a.map((v, i) => lerp(v, b[i] || v, t)); }

function simulationTick() {
  if (!simulating || simSteps.length === 0) return;

  const elapsed = (performance.now() / 1000) - simStartTime;
  const totalDur = simSteps.length * simStepDuration;

  if (elapsed >= totalDur) {
    // Simulation complete — show final pose, call callback.
    const last = simSteps[simSteps.length - 1];
    setJointsDirect(last.joints);
    updateGripperPosition((last.gripper_pos || 0) / 1000.0);
    if (simLabel) simLabel.textContent = `Step ${simSteps.length}/${simSteps.length}: ${last.label || 'done'}`;
    if (simOnComplete) setTimeout(simOnComplete, 300);
    return;
  }

  // Find current step and interpolation fraction within it.
  const rawIdx = elapsed / simStepDuration;
  const idx = Math.min(Math.floor(rawIdx), simSteps.length - 1);
  const frac = rawIdx - idx;

  const cur = simSteps[idx];
  const next = simSteps[Math.min(idx + 1, simSteps.length - 1)];

  const interpJoints = lerpArray(cur.joints, next.joints, Math.min(frac, 1.0));
  setJointsDirect(interpJoints);

  const interpGripper = lerp(cur.gripper_pos || 0, next.gripper_pos || 0, Math.min(frac, 1.0));
  updateGripperPosition(interpGripper / 1000.0);

  if (simLabel) {
    simLabel.textContent = `Step ${idx + 1}/${simSteps.length}: ${cur.label || ''}`;
    simLabel.style.display = '';
  }
}

/**
 * Start motion simulation in the 3D view.
 * @param {Array} steps — [{joints: [J1..J6], gripper_pos: 0-1000, label: "approach"}, ...]
 * @param {Object} opts — {stepDuration: 1.0, onComplete: fn}
 */
function startSimulation(steps, opts = {}) {
  if (!steps || steps.length === 0) return;
  simSteps = steps;
  simStepDuration = opts.stepDuration || 1.0;
  simOnComplete = opts.onComplete || null;

  // Freeze the current live pose for the ghost.
  simLiveFrozenJoints = [];
  for (let i = 0; i < 6; i++) {
    simLiveFrozenJoints.push(
      jointPivots[i] ? jointPivots[i].rotation.z * (180 / Math.PI) : 0
    );
  }
  simLiveFrozenGripper = gripperCurrentSpread / 0.0525 * 1000;

  buildGhostRobot();
  updateGhostJoints(simLiveFrozenJoints);

  simStartTime = performance.now() / 1000;
  simulating = true;

  if (simBadge) simBadge.style.display = '';
  if (simLabel) simLabel.style.display = '';

  // Set main robot to first step immediately.
  setJointsDirect(steps[0].joints);
  updateGripperPosition((steps[0].gripper_pos || 0) / 1000.0);
}

function stopSimulation() {
  simulating = false;
  removeGhost();
  if (simBadge) simBadge.style.display = 'none';
  if (simLabel) simLabel.style.display = 'none';
  simSteps = [];
  // Reconnect to live — next updateJoints call from WebSocket will drive the robot.
}

// ── Animate loop (enhanced with gripper + simulation) ───────────

function animate() {
  animationId = requestAnimationFrame(animate);
  if (controls) controls.update();
  animateGripperFingers();
  if (simulating) simulationTick();
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
window.updateRobotGripper = updateGripper;
window.resetRobotCamera = resetCamera;
window.saveRobotCamera = saveCamera;
window.simulateRobotMotion = startSimulation;
window.stopRobotSimulation = stopSimulation;
