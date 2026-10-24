// 用真实 three.js、STL 及 /model.json 数据执行页面脚本；仅替代 DOM/WebGL/文件传输边界。
// 运行：ARM_PYTHON=<python路径> node --test tests/web_scene.test.cjs
const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const { spawnSync } = require('node:child_process');
const THREE = require('../web/vendor/three.min.js');
const root = path.resolve(__dirname, '..');
const html = fs.readFileSync(path.join(root, 'web/index.html'), 'utf8');
const script = [...html.matchAll(/<script>([\s\S]*?)<\/script>/g)].map(m => m[1]).join('\n');
const result = spawnSync(process.env.ARM_PYTHON || 'python', ['-c',
  'import json; from tools.serve import build_backend; b=build_backend(None, {}); print(json.dumps({"model":b.model_json(), "meshRoot":str(b.model.package_root)}))'],
  { cwd: root, encoding: 'utf8' });
assert.equal(result.status, 0, result.stderr);
const fixture = JSON.parse(result.stdout);

function element() {
  const listeners = new Map();
  return {
    style: {}, textContent: '', className: '', value: '', disabled: false, children: [],
    appendChild(child) { this.children.push(child); },
    addEventListener(type, fn) { if (!listeners.has(type)) listeners.set(type, []); listeners.get(type).push(fn); },
    removeEventListener(type, fn) { listeners.set(type, (listeners.get(type) || []).filter(f => f !== fn)); },
    dispatch(type, data = {}) {
      const event = { type, pointerId: 1, button: 0, preventDefault() {}, stopPropagation() {}, stopImmediatePropagation() {}, ...data };
      for (const fn of listeners.get(type) || []) fn(event);
      return event;
    },
    getBoundingClientRect() { return { left: 0, top: 0, width: 800, height: 600 }; },
    setPointerCapture(id) { this.captured = id; },
    hasPointerCapture(id) { return this.captured === id; },
    releasePointerCapture(id) { if (this.captured === id) this.captured = null; },
    set innerHTML(value) {
      this.html = value;
      if (value.includes('<input')) {
        const input = element();
        input.value = (value.match(/value="([^"]*)"/) || [])[1] || '';
        input.nextElementSibling = element();
        this.input = input;
      }
    },
    get innerHTML() { return this.html || ''; },
    querySelector(tag) { if (tag === 'input') return this.input; return this.body || (this.body = element()); },
  };
}

function page({ failMesh = false, noCamera = false, aspect = 800/600 } = {}) {
  const nodes = new Map([...html.matchAll(/<[^>]+\bid="([^"]+)"[^>]*>/g)].map(m => [m[1], Object.assign(element(), {
    checked: /\bchecked\b/.test(m[0]), disabled: /\bdisabled\b/.test(m[0]),
    value: (m[0].match(/value="([^"]*)"/) || [])[1] || '',
  })]));
  const requests = [], timers = [], active = new Map();
  const canvas = element();
  let clock = 0, responder = () => new Promise(() => {});
  const context = vm.createContext({
    THREE: { ...THREE }, window: Object.assign(element(), { confirm: () => true }), AbortController, console, model: structuredClone(fixture.model), canvas,
    document: {
      getElementById: id => nodes.get(id) || null, createElement: element,
      querySelector: selector => {
        const id = (selector.match(/data-servo="(\d+)"/) || [])[1];
        return nodes.get('sliders').children.find(row => row.innerHTML.includes('data-servo="' + id + '"'))?.input || null;
      },
      querySelectorAll: () => nodes.get('sliders').children.map(row => row.input),
    },
    fetch: (url, options) => { requests.push([url, options]); return responder(url, options); },
    sceneAspect: aspect,
    performance: { now: () => clock },
    setTimeout: (fn, delay = 0) => { timers.push(fn); active.set(timers.length, { fn, at: clock + delay }); return timers.length; },
    clearTimeout: id => active.delete(id),
    requestAnimationFrame: fn => { timers.push(fn); active.set(timers.length, { fn, at: clock + 16 }); return timers.length; },
    cancelAnimationFrame: id => active.delete(id),
  });
  if (noCamera) context.model.optional_components = {};
  vm.runInContext(fs.readFileSync(path.join(root, 'web/vendor/STLLoader.js'), 'utf8'), context);
  // STLLoader 本身仍使用真实解析器，仅将 HTTP 读取替换为本地文件读取。
  context.THREE.STLLoader.prototype.load = function(url, success, progress, failure) {
    setImmediate(() => {
      if (failMesh && url.endsWith('/l_out_link.STL')) return failure(new Error('测试网格不可用'));
      try {
        const bytes = fs.readFileSync(path.join(fixture.meshRoot, url));
        success(this.parse(bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength)));
      } catch (error) { failure(error); }
    });
  };
  const modulePath = path.join(root, 'web/trajectory.js');
  if (fs.existsSync(modulePath)) vm.runInContext(fs.readFileSync(modulePath, 'utf8'), context);
  vm.runInContext(script, context); // boot 等待 model 请求，不连接任何串口
  const run = code => vm.runInContext(code, context);
  run(`initThree = () => {
    scene = new THREE.Scene(); camera = new THREE.PerspectiveCamera(45, sceneAspect, .01, 20);
    camera.position.set(.5, .42, .55); camera.lookAt(0, .14, 0); camera.updateMatrixWorld(true);
    renderer = { domElement: canvas };
    controls = { enabled: true, target: new THREE.Vector3(0, .14, 0), update() {} };
  };`); // 仅替代 GPU / OrbitControls 的 DOM 边界，射线和数学使用真实 three
  requests.length = 0;
  active.clear(); timers.length = 0; // 不运行此隔离夹具启动时刻意悬挂的 model 请求
  return {
    run, context, nodes, requests, timers, canvas,
    respond(fn = () => ({})) {
      responder = async (url, options) => {
        const data = await fn(url, options);
        return { ok: true, json: async () => ({motion_epoch:'epoch-0', ...data}) };
      };
    },
    async tick(ms) {
      const end = clock + ms;
      while (true) {
        const next = [...active.entries()].filter(([, t]) => t.at <= end).sort((a, b) => a[1].at - b[1].at)[0];
        if (!next) break;
        clock = next[1].at; active.delete(next[0]); next[1].fn(clock);
        // 排空跨 VM / fetch / json 的完整微任务链，再推进虚拟时钟；固定 8 次会跳过续订 poll。
        await new Promise(setImmediate);
      }
      clock = end;
      await new Promise(setImmediate);
    },
  };
}

async function ready(options) {
  const p = page(options);
  p.run('state.model = model');
  await p.run('buildScene(model)');
  return p;
}

function localTip(p, name, y) {
  const point = p.run(`linkNodes['${name}'].localToWorld(new THREE.Vector3(.006, ${y}, .035))`);
  return p.run('linkNodes.gripper_link').worldToLocal(point);
}

test('模型加载完成才返回，夹爪六个零件都有实际几何', async () => {
  const p = page();
  const pending = p.run('buildScene(model)');
  assert.ok(pending && typeof pending.then === 'function', '必须等待 STL 而非仅建空运动树');
  await pending;
  for (const name of ['l_link', 'l_in_link', 'l_out_link', 'r_link', 'r_in_link', 'r_out_link']) {
    const node = p.run(`linkNodes['${name}']`);
    assert.ok(node.children.some(c => c.isMesh && c.geometry.attributes.position.count > 0), name);
  }
});

test('夹爪六关节一起转动，开口增大且指尖保持平行', async () => {
  const p = await ready();
  p.run('setServoAngle(1, 0); scene.updateMatrixWorld(true)');
  const gap0 = localTip(p, 'l_out_link', .007).y - localTip(p, 'r_out_link', -.007).y;
  const joints = p.run('model.joints.filter(j => /^(l_|r_)/.test(j.name))');
  const initial = joints.map(j => j.__holder.matrix.clone());
  p.run('setServoAngle(1, 30); scene.updateMatrixWorld(true)');
  const gap30 = localTip(p, 'l_out_link', .007).y - localTip(p, 'r_out_link', -.007).y;
  assert.ok(gap30 > gap0 + .02, `张开角度应增大间距：${gap0} → ${gap30}`);
  for (let i = 0; i < joints.length; i++) assert.ok(!initial[i].equals(joints[i].__holder.matrix), joints[i].name);
  const baseQ = p.run('linkNodes.gripper_link.getWorldQuaternion(new THREE.Quaternion())');
  for (const name of ['l_out_link', 'r_out_link']) {
    const q = p.run(`linkNodes['${name}'].getWorldQuaternion(new THREE.Quaternion())`);
    assert.ok(1 - Math.abs(q.dot(baseQ)) < .00001, name + ' 不应随内杆倾倒');
  }
  assert.equal(p.run('state.targets[1]'), 30, '显示符号不得改变发送目标');
  assert.equal(p.requests.length, 0);
});

test('摄像头默认隐藏，切换整组且不影响夹爪或安排运动', async () => {
  const p = await ready();
  const input = p.nodes.get('cameraToggle');
  assert.ok(input, '需要独立的相机复选框');
  assert.equal(input.checked, false);
  const camera = p.run('linkNodes.camera_connet_link');
  assert.equal(camera.visible, false);
  let meshes = 0;
  camera.traverse(n => { if (n.isMesh) meshes++; });
  assert.ok(meshes >= 5, '复用外壳并增加镜头示意');
  const children = camera.children.length;
  p.run('state.linked = true'); // 仅在隔离环境模拟联动状态，真实页面不联动
  const targets = JSON.stringify(p.run('state.targets'));
  for (const enabled of [true, false, true, false]) {
    input.checked = enabled;
    input.onchange();
    assert.equal(camera.visible, enabled);
    assert.equal(p.run('state.cameraVisible'), enabled);
    assert.equal(p.run('linkNodes.gripper_link.visible'), true);
    assert.equal(camera.children.length, children, '切换不应重复创建几何');
  }
  assert.equal(JSON.stringify(p.run('state.targets')), targets);
  assert.equal(p.requests.length, 0, '组件显示不得发送 API 请求');
  assert.equal(p.timers.length, 0, '组件显示不得安排延迟运动');
});

test('相机与腕部一起运动，重复启用保留其正确位置', async () => {
  const p = await ready();
  p.run('setCameraVisible(true); scene.updateMatrixWorld(true)');
  const initial = p.run('linkNodes.camera_connet_link.matrixWorld.clone()');
  p.run('setCameraVisible(false); setServoAngle(3, 25); setCameraVisible(true); scene.updateMatrixWorld(true)');
  const moved = p.run('linkNodes.camera_connet_link.matrixWorld');
  assert.ok(!initial.equals(moved));
  assert.equal(p.requests.length, 0);
});

test('缺少可选相机的自定义模型仍能显示，开关禁用', async () => {
  const p = await ready({ noCamera: true });
  assert.equal(p.nodes.get('cameraToggle').disabled, true);
  assert.equal(p.nodes.get('cameraToggle').checked, false);
});

test('网格失败明确中止加载，不冒充完整模型', async () => {
  const p = page({ failMesh: true });
  await assert.rejects(async () => await p.run('buildScene(model)'), /l_out_link.STL/);
});

function baseHeading(p) {
  const holder = p.run('state.actuated[6].holder');
  const rotation = holder.userData.base.clone().invert().multiply(holder.matrix);
  return new THREE.Vector3(1, 0, 0).transformDirection(rotation);
}

test('显式设置底座零位使用正90度补偿，纠正左右相反且不发送运动', async () => {
  const p = await ready();
  p.run('setServoAngle(6, 0)');
  assert.ok(baseHeading(p).distanceTo(new THREE.Vector3(0, 1, 0)) < 1e-9);
  assert.equal(p.run('state.targets[6]'), 0);
  assert.equal(p.requests.length, 0);
  assert.equal(p.timers.length, 0);
});

test('底座修改角度再复位仍保留零位补偿，不累计也不修改其他关节', async () => {
  const p = await ready();
  const otherJoints = p.run('model.joints.filter(j => j.servo_id !== 6)');
  const initial = otherJoints.map(j => j.__holder.matrix.clone());
  p.run('state.linked = true'); // 隔离环境，无真实串口
  const basePlate = p.run('scene.updateMatrixWorld(true); linkNodes.base_link.matrixWorld.clone()');
  for (const [angle, x, y] of [[30, -.5, .8660254037844386], [0, 0, 1],
                              [-30, .5, .8660254037844386], [0, 0, 1]]) {
    p.run(`setServoAngle(6, ${angle}); scene.updateMatrixWorld(true)`);
    assert.ok(basePlate.equals(p.run('linkNodes.base_link.matrixWorld')), '底板不能跟着零位补偿旋转');
    assert.ok(baseHeading(p).distanceTo(new THREE.Vector3(x, y, 0)) < 1e-9);
    assert.equal(p.run('state.targets[6]'), angle, '显示补偿不得写入运动目标');
  }
  otherJoints.forEach((j, i) => assert.ok(initial[i].equals(j.__holder.matrix), j.name));
  assert.equal(p.requests.length, 0);
  assert.equal(p.timers.length, 0);
});

test('回读底座原始位置保持角度数值，模型应用相同零位补偿', async () => {
  const p = await ready();
  const tbody = { innerHTML: '' };
  p.nodes.get('stateTable').querySelector = () => tbody;
  p.run('renderStatus({servos: {6: {ok: true, name: "底座", position: 625, temperature: null, loaded: true}}})');
  assert.ok(Math.abs(p.run('state.targets[6]') - 30) < 1e-9);
  assert.ok(tbody.innerHTML.includes('30.0°'));
  assert.ok(baseHeading(p).distanceTo(new THREE.Vector3(-.5, .8660254037844386, 0)) < 1e-9);
  p.run('renderStatus({servos: {6: {ok: true, name: "底座", position: 500, temperature: null, loaded: true}}})');
  assert.equal(p.run('state.targets[6]'), 0);
  assert.ok(baseHeading(p).distanceTo(new THREE.Vector3(0, 1, 0)) < 1e-9);
  assert.equal(p.requests.length, 0);
  assert.equal(p.timers.length, 0);
});

async function interactive(options) {
  const p = await ready(options);
  p.run('state.motionEpoch="epoch-0"; buildSliders(model); initTrajectory(); bindPanel(); updateModeUI()');
  p.respond();
  return p;
}
const json = value => JSON.parse(JSON.stringify(value));
const writes = p => p.requests.filter(([, o]) => o?.method === 'POST');

// 缺少默认待机 / 滑块同步会使模型与待发目标不一致。
test('默认待机、滑块一致，离线待机只改两轴，复位只改相机并取消尾帧', async () => {
  const p = await interactive();
  assert.deepEqual(json(p.run('state.targets')), {1: 0, 2: 0, 3: -90, 4: 0, 5: 0, 6: -90});
  assert.equal(p.run('document.querySelector(\'input[data-servo="6"]\').value'), '-90.0');
  p.run('setServoAngle(2, 25); setServoAngle(3, 0); setServoAngle(6, 0)');
  await p.nodes.get('standbyBtn').onclick();
  assert.equal(p.run('state.targets[2]'), 25);
  assert.equal(p.run('state.targets[3]'), -90);
  assert.equal(p.run('state.targets[6]'), -90);
  p.run('state.linked = true; changeMode("manual"); scheduleMove()');
  const before = json(p.run('state.targets'));
  p.nodes.get('resetBtn').onclick();
  await p.tick(500);
  assert.deepEqual(json(p.run('state.targets')), before);
  assert.equal(writes(p).length, 0);
});

// 移除 generation/mode 检查会把规划角度通过既有 timer 发给真机。
test('联动待发 timer 进入规划后无写请求，规划滑块和录制都仅本地', async () => {
  const p = await interactive();
  p.run('state.linked = true; changeMode("manual"); scheduleMove(); enterPlanning()');
  p.run('document.querySelector(\'input[data-servo="2"]\').value = "35"; document.querySelector(\'input[data-servo="2"]\').dispatch("input")');
  await p.tick(1000);
  assert.equal(p.run('state.targets[2]'), 35);
  assert.equal(writes(p).length, 0);
  p.nodes.get('recordBtn').onclick();
  await p.tick(120);
  p.run('document.querySelector(\'input[data-servo="2"]\').value = "40"; document.querySelector(\'input[data-servo="2"]\').dispatch("input")');
  p.nodes.get('recordStopBtn').onclick();
  assert.equal(p.run('state.mode'), 'planning');
  assert.equal(p.run('state.recorder.points.at(-1).angles[2]'), 40);
  assert.equal(writes(p).length, 0);
});

// 丢失指针必须复原相机；拾取使用真实 Torus/STL 射线而非伪造命中。
test('真实旋转环拾取、跨 wrap 拖动、轴方向和底板不变，cancel 恢复 OrbitControls', async () => {
  const p = await interactive();
  p.run(`state.dragger.select(6); scene.updateMatrixWorld(true);
    globalThis.frame = state.dragger.frame();
    camera.position.copy(frame.center).addScaledVector(frame.axis, .5);
    camera.up.copy(frame.v); camera.lookAt(frame.center); camera.updateMatrixWorld(true);
    globalThis.pixel = angle => {
      const pt = frame.center.clone().addScaledVector(frame.u, Math.cos(angle)*state.dragger.radius)
        .addScaledVector(frame.v, Math.sin(angle)*state.dragger.radius).project(camera);
      return {clientX:(pt.x+1)*400, clientY:(1-pt.y)*300};
    };
    globalThis.plate = linkNodes.base_link.matrixWorld.clone();
    state.linked = true; changeMode('manual'); scheduleMove();`);
  // 厂商 joint1 自带约 0.4° 安装倾角；Z-up→Y-up 还必须保留这段 base 旋转。
  assert.ok(p.run('frame.axis').distanceTo(new THREE.Vector3(-.000002294874253, .999975630824132, .006981242913213)) < 1e-9,
    'URDF 轴转换必须同时保留机器人旋转和原始安装倾角');
  assert.ok(p.run('frame.center').distanceTo(p.run('state.actuated[6].holder.getWorldPosition(new THREE.Vector3())')) < 1e-9);
  p.canvas.dispatch('pointerdown', p.run('pixel(179*Math.PI/180)'));
  assert.equal(p.run('controls.enabled'), false);
  assert.equal(p.canvas.captured, 1);
  p.canvas.dispatch('pointermove', p.run('pixel(-179*Math.PI/180)'));
  assert.ok(Math.abs(p.run('state.targets[6]') + 88) < .001);
  p.canvas.dispatch('pointercancel');
  assert.equal(p.run('controls.enabled'), true);
  assert.equal(p.canvas.captured, null);
  assert.ok(p.run('scene.updateMatrixWorld(true); plate.equals(linkNodes.base_link.matrixWorld)'));
  await p.tick(500);
  assert.equal(writes(p).length, 0);
  p.canvas.dispatch('pointerdown', p.run('pixel(0)'));
  p.canvas.dispatch('lostpointercapture');
  assert.equal(p.run('controls.enabled'), true);
});

test('模型肢段与夹爪 follower 可拾取到所属轴；负 scale、clamp 和平行视角安全', async () => {
  const p = await interactive();
  assert.equal(p.run('ArmTrajectory.servoForObject(linkNodes.r_out_link, state.actuated)'), 1);
  assert.equal(p.run('ArmTrajectory.servoForObject(linkNodes.base_link, state.actuated)'), null);
  p.run(`state.dragger.select(1); setServoAngle(1, 20); scene.updateMatrixWorld(true);
    globalThis.frame = state.dragger.frame();
    camera.position.copy(frame.center).addScaledVector(frame.axis, .5);
    camera.up.copy(frame.v); camera.lookAt(frame.center); camera.updateMatrixWorld(true);
    globalThis.pixel = angle => {
      const pt = frame.center.clone().addScaledVector(frame.u, Math.cos(angle)*state.dragger.radius)
        .addScaledVector(frame.v, Math.sin(angle)*state.dragger.radius).project(camera);
      return {clientX:(pt.x+1)*400, clientY:(1-pt.y)*300};
    };`);
  p.canvas.dispatch('pointerdown', p.run('pixel(0)'));
  p.canvas.dispatch('pointermove', p.run('pixel(-10*Math.PI/180)'));
  assert.ok(Math.abs(p.run('state.targets[1]') - 30) < .001, 'scale -1 不得与 offset 混用');
  p.canvas.dispatch('pointerup');
  p.run('setServoAngle(1, 1e6)');
  assert.equal(p.run('state.targets[1]'), p.run('state.actuated[1].joint.slider_max_deg'));
  p.run('setServoAngle(1, NaN)');
  assert.ok(Number.isFinite(p.run('state.targets[1]')));
  p.run('camera.position.copy(frame.center).addScaledVector(frame.u, .5); camera.lookAt(frame.center); camera.updateMatrixWorld(true)');
  p.canvas.dispatch('pointerdown', {clientX:400, clientY:300});
  p.canvas.dispatch('pointermove', {clientX:402, clientY:300});
  p.canvas.dispatch('pointercancel');
  assert.equal(p.run('controls.enabled'), true);
  assert.ok(Number.isFinite(p.run('state.targets[1]')));
});

// 深拷贝、末点与硬上限必须由生产录制器约束，而不是只限制界面。
test('录制起点深拷贝、100ms采样、末点、速度时长与容量/总时长上限', async () => {
  const p = await interactive();
  p.run(`globalThis.rec = new ArmTrajectory.Recorder();
    globalThis.pose = {1:0,2:0,3:0,4:0,5:0,6:0}; rec.start(pose, 0);
    pose[2]=3; rec.sample(pose, 99);`);
  assert.equal(p.run('rec.points.length'), 1);
  p.run('rec.sample(pose, 100); pose[2]=6; rec.stop(pose, 130)');
  assert.equal(p.run('rec.points.length'), 3);
  assert.equal(p.run('rec.points[0].angles[2]'), 0);
  assert.equal(p.run('rec.points.at(-1).angles[2]'), 6);
  assert.equal(p.run('rec.points.at(-1).duration_ms'), 100);
  p.run('pose[2]=50');
  assert.equal(p.run('rec.points.at(-1).angles[2]'), 6);
  p.run('rec.start(pose, 0); for(let i=1;i<700;i++){pose[2]=i%2;rec.sample(pose,i*100)}');
  assert.equal(p.run('rec.points.length'), 600);
  assert.equal(p.run('rec.active'), false);
  assert.ok(p.run('rec.points.every(p=>Number.isInteger(p.duration_ms)&&p.duration_ms>=80&&p.duration_ms<=30000)'));
  p.run('rec = new ArmTrajectory.Recorder(); rec.start(pose,0); for(let i=1;i<30;i++){pose[2]=i;rec.sample(pose,i*30000)}');
  assert.ok(p.run('rec.total <= 600000'));
  assert.equal(p.run('rec.active'), false);
});

test('参考点路径遵循真实 FK 且不改当前姿态，六轴预览经过点并可取消', async () => {
  const p = await interactive();
  p.nodes.get('recordBtn').onclick();
  await p.tick(200);
  p.run('setServoAngle(2,30); setServoAngle(3,-60); sampleRecording(true)');
  p.nodes.get('recordStopBtn').onclick();
  const before = json(p.run('state.targets'));
  p.run('updateTrajectoryPath(); scene.updateMatrixWorld(true)');
  const tip = p.run('linkNodes.gripper_link.getWorldPosition(new THREE.Vector3())');
  const array = p.run('state.pathLine.geometry.attributes.position.array');
  assert.ok(new THREE.Vector3(...Array.from(array).slice(-3)).distanceTo(tip) < 1e-6);
  assert.deepEqual(json(p.run('state.targets')), before);
  p.nodes.get('previewBtn').onclick();
  await p.tick(700); // 已记录起点200ms停留，再经过限速运动段的500ms
  assert.ok(Math.abs(p.run('state.targets[2]') - 15) < 1);
  assert.ok(Math.abs(p.run('state.targets[3]') + 75) < 1);
  p.nodes.get('previewStopBtn').onclick();
  const stopped = json(p.run('state.targets'));
  await p.tick(2000);
  assert.deepEqual(json(p.run('state.targets')), stopped);
  assert.equal(p.run('state.mode'), 'planning');
  assert.equal(writes(p).length, 0);
});

test('执行需确认且等待已发帧；busy 门禁、停止无尾帧、联动待机只发后端待机', async () => {
  const p = await interactive();
  let release;
  p.respond((url) => {
    if (url === '/api/move') return new Promise(resolve => { release = resolve; });
    if (url === '/api/trajectory/start' || url === '/api/standby') return {state:'running', index:0,total:2,run_id:1,current_angles:{},error:null};
    if (url === '/api/stop') return {state:'stopped',index:0,total:2,run_id:1,current_angles:{},error:null};
    return {};
  });
  p.run('state.linked=true; changeMode("manual"); scheduleMove()');
  await p.tick(200);
  assert.equal(writes(p).length, 1);
  p.nodes.get('recordBtn').onclick();
  p.run('setServoAngle(2, 20)');
  p.nodes.get('recordStopBtn').onclick();
  p.run('window.confirm = () => false');
  await p.nodes.get('executeBtn').onclick();
  assert.equal(writes(p).length, 1);
  p.run('window.confirm = () => true');
  const start = p.nodes.get('executeBtn').onclick();
  await p.tick(0);
  assert.equal(writes(p).length, 1, '先等待已经发送的 manual 帧完成');
  release({}); await start;
  assert.equal(writes(p)[1][0], '/api/trajectory/start');
  const payload = JSON.parse(writes(p)[1][1].body);
  assert.deepEqual(payload.points[0].angles, {1:0,2:0,3:-90,4:0,5:0,6:-90});
  assert.equal(payload.points.at(-1).angles[2], 20);
  for (const id of ['readBtn','loadBtn','standbyBtn','relinkBtn','recordBtn','axisSel']) assert.equal(p.nodes.get(id).disabled, true, id);
  await p.nodes.get('readBtn').onclick();
  await p.nodes.get('standbyBtn').onclick();
  await p.nodes.get('loadBtn').onclick();
  assert.equal(writes(p).length, 2);
  await p.nodes.get('stopBtn').onclick();
  await p.tick(500);
  assert.equal(writes(p).at(-1)[0], '/api/stop');
  assert.equal(p.run('state.mode'), 'planning');
  await p.nodes.get('standbyBtn').onclick();
  assert.equal(writes(p).at(-1)[0], '/api/standby');
  assert.deepEqual(JSON.parse(writes(p).at(-1)[1].body), {duration_ms:600, motion_epoch:'epoch-0'});
});

test('启动 GET status 恢复已有联动和后台忙碌，不写入、不覆盖默认规划姿态', async () => {
  const p = page();
  p.respond(url => {
    if (url === '/model.json') return structuredClone(fixture.model);
    if (url === '/api/ports') return {ports:[]};
    if (url === '/api/status') return {linked:true,motion_enabled:true,servos:{6:{position:500}},trajectory:{state:'running',index:1,total:3,run_id:7,current_angles:{6:10},error:null}};
    return {};
  });
  await p.run('boot()');
  assert.equal(p.run('state.linked'), true);
  assert.equal(p.run('state.mode'), 'executing');
  assert.equal(p.run('state.targets[6]'), -90);
  assert.equal(p.nodes.get('readBtn').disabled, true);
  assert.equal(writes(p).length, 0);
});

test('恢复滑块联动必须显式回读，失败保持规划；stop 取消未发 timer', async () => {
  const p = await interactive();
  p.run('state.linked=true; setServoAngle(6,50)');
  p.respond(url => {
    if(url === '/api/read') return {servos:Object.fromEntries([1,2,3,4,5,6].map(id=>[id,{ok:true,position:id===1?700:500,temperature:null,loaded:id!==2}]))};
    return {};
  });
  await p.nodes.get('relinkBtn').onclick();
  assert.equal(p.run('state.mode'), 'manual');
  assert.equal(p.run('state.targets[6]'), 0);
  assert.match(p.nodes.get('stateTable').querySelector('tbody').innerHTML, /卸力/);
  assert.deepEqual(writes(p).map(r=>r[0]), ['/api/read']);
  p.run('scheduleMove()');
  await p.nodes.get('stopBtn').onclick(); await p.tick(500);
  assert.deepEqual(writes(p).map(r=>r[0]), ['/api/read','/api/stop']);
  p.respond(() => {throw new Error('fake offline');});
  await p.nodes.get('relinkBtn').onclick();
  assert.equal(p.run('state.mode'), 'planning');
});

test('复位视图不暂停预览，也不丢失已发执行请求的状态', async () => {
  const p = await interactive();
  p.nodes.get('recordBtn').onclick();
  p.run('setServoAngle(2,30)');
  p.nodes.get('recordStopBtn').onclick();
  p.nodes.get('previewBtn').onclick();
  await p.tick(300);
  p.nodes.get('resetBtn').onclick();
  await p.tick(900);
  assert.equal(p.run('state.mode'), 'planning');
  assert.equal(p.run('state.targets[2]'), 30);
  let release;
  p.respond(() => new Promise(resolve => { release = resolve; }));
  p.run('state.linked=true');
  const start = p.nodes.get('executeBtn').onclick();
  await p.tick(0);
  p.nodes.get('resetBtn').onclick();
  release({state:'running',index:0,total:2,run_id:9,current_angles:{},error:null});
  await start;
  assert.equal(p.run('state.trajectory.run_id'), 9);
  assert.equal(p.run('state.mode'), 'executing');
});

test('启动中刷新状态不允许旧 idle 状态解除门禁', async () => {
  const p = await interactive();
  p.nodes.get('recordBtn').onclick();
  p.nodes.get('recordStopBtn').onclick();
  let release;
  p.respond(url => url === '/api/trajectory/start' ? new Promise(resolve => {release=resolve;}) :
    {linked:true,motion_enabled:true,servos:{},trajectory:{state:'idle',index:0,total:0,error:null}});
  p.run('state.linked=true');
  const start = p.nodes.get('executeBtn').onclick();
  await p.tick(0);
  await p.nodes.get('syncStatusBtn').onclick();
  assert.equal(p.run('state.mode'), 'executing');
  release({state:'running',index:0,total:1,run_id:10,current_angles:{},error:null}); await start;
  assert.equal(p.run('state.trajectory.run_id'), 10);
  assert.equal(p.nodes.get('readBtn').disabled, true);
});

test('停止立即越过悬挂 start；旧请求带旧 epoch，迟到响应不得解除停止', async () => {
  const p = await interactive();
  p.run('state.linked=true; changeMode("manual"); scheduleMove()');
  const staleTimer = p.timers.at(-1);
  p.nodes.get('recordBtn').onclick();
  p.nodes.get('recordStopBtn').onclick();
  staleTimer(); await p.tick(0);
  assert.equal(writes(p).length, 0);
  let release;
  p.respond(url => url === '/api/trajectory/start' ? new Promise(resolve => {release=resolve;}) :
    {motion_epoch:'epoch-1',trajectory:{state:'stopped',index:0,total:1,run_id:11,current_angles:{},error:null}});
  const start = p.nodes.get('executeBtn').onclick(); await p.tick(0);
  const stop = p.nodes.get('stopBtn').onclick(); await p.tick(0);
  assert.deepEqual(writes(p).map(r=>r[0]), ['/api/trajectory/start','/api/stop']);
  assert.equal(JSON.parse(writes(p)[0][1].body).motion_epoch, 'epoch-0');
  await stop;
  assert.equal(p.run('state.motionEpoch'), 'epoch-1');
  release({motion_epoch:'epoch-0',state:'running',index:0,total:1,run_id:11,current_angles:{},error:null});
  await start; await p.tick(1000);
  assert.equal(writes(p).length, 2);
  assert.equal(p.run('state.mode'), 'planning');
  assert.equal(p.run('state.trajectory.state'), 'stopped');
  assert.equal(p.run('state.motionEpoch'), 'epoch-1');
  p.respond(() => ({state:'preparing',index:0,total:1,run_id:12}));
  await p.nodes.get('executeBtn').onclick();
  assert.equal(JSON.parse(writes(p).at(-1)[1].body).motion_epoch, 'epoch-1');
});

test('真 STL 射线点选夹爪肢段，六轴选择器覆盖遮挡关节且始终本地', async () => {
  const p = await interactive();
  p.run(`state.linked=true; changeMode('manual'); scheduleMove();
    scene.updateMatrixWorld(true);
    globalThis.pick = null;
    const mesh = linkNodes.r_out_link.children.find(n=>n.isMesh);
    const center = new THREE.Box3().setFromObject(mesh).getCenter(new THREE.Vector3());
    for(const offset of [[.2,0,0],[-.2,0,0],[0,.2,0],[0,-.2,0],[0,0,.2],[0,0,-.2]]) {
      camera.position.copy(center).add(new THREE.Vector3(...offset)); camera.up.set(0,1,0);
      camera.lookAt(center); camera.updateMatrixWorld(true);
      const ray = new THREE.Raycaster();
      for(let x=300; x<=500 && !pick; x+=10) for(let y=200; y<=400 && !pick; y+=10) {
        ray.setFromCamera(new THREE.Vector2(x/400-1,1-y/300),camera);
        const hit=ray.intersectObject(state.robotGroup,true)[0];
        if(hit?.object.name==='r_out_link_mesh') pick={clientX:x,clientY:y};
      }
      if(pick) break;
    }`);
  assert.ok(p.run('pick'), '至少一条实际射线命中外指 STL');
  p.canvas.dispatch('pointerdown', p.run('pick'));
  assert.equal(p.run('state.dragger.id'), 1);
  assert.equal(p.nodes.get('axisSel').value, '1');
  assert.equal(p.run('state.mode'), 'planning');
  assert.equal(p.nodes.get('axisSel').children.length, 6);
  for (const id of [1,2,3,4,5,6]) {
    p.nodes.get('axisSel').value = String(id); p.nodes.get('axisSel').onchange();
    assert.equal(p.run('state.dragger.id'), id);
    assert.ok(p.run('state.dragger.ring.visible'));
  }
  await p.tick(300); assert.equal(writes(p).length, 0);
});

test('预览按各段时长插值全部六轴、经过全部点、结束后保持规划', async () => {
  const p = await interactive();
  p.run(`state.recorder.points = [
    {angles:{1:0,2:0,3:0,4:0,5:0,6:0},duration_ms:80},
    {angles:{1:10,2:20,3:30,4:20,5:10,6:30},duration_ms:1000},
    {angles:{1:0,2:0,3:0,4:0,5:0,6:0},duration_ms:1000}]; state.recorder.total=2080;`);
  assert.deepEqual(json(p.run('ArmTrajectory.poseAt(state.recorder.points,580)')), {1:5,2:10,3:15,4:10,5:5,6:15});
  assert.deepEqual(json(p.run('ArmTrajectory.poseAt(state.recorder.points,1080)')), {1:10,2:20,3:30,4:20,5:10,6:30});
  assert.deepEqual(json(p.run('ArmTrajectory.poseAt(state.recorder.points,1580)')), {1:5,2:10,3:15,4:10,5:5,6:15});
  p.nodes.get('previewBtn').onclick(); await p.tick(2200);
  assert.deepEqual(json(p.run('state.targets')), {1:0,2:0,3:0,4:0,5:0,6:0});
  assert.equal(p.run('state.mode'), 'planning');
  assert.equal(writes(p).length, 0);
});

test('执行时只读轮询不覆盖规划点；请求失败保持门禁并允许停止断连', async () => {
  const p = await interactive();
  p.run(`state.linked=true; state.recorder.start(state.targets,0); state.recorder.stop(state.targets,0);
    applyTrajectoryStatus({state:'running',index:1,total:2,run_id:12,current_angles:{6:90},error:null}); startPolling();`);
  const points = json(p.run('state.recorder.points'));
  p.respond(url => { if(url === '/api/trajectory/status') throw new Error('offline'); return {linked:false,motion_enabled:false}; });
  const before = json(p.run('state.targets'));
  const slider = p.run('document.querySelector(\'input[data-servo="2"]\')');
  slider.value = 30; slider.dispatch('input');
  assert.deepEqual(json(p.run('state.targets')), before);
  await p.tick(510);
  assert.equal(p.run('state.backendUnknown'), true);
  assert.equal(p.nodes.get('readBtn').disabled, true);
  assert.equal(p.nodes.get('linkBtn').disabled, false);
  assert.deepEqual(json(p.run('state.recorder.points')), points);
  await p.nodes.get('linkBtn').onclick();
  assert.equal(p.run('state.linked'), false);
  assert.equal(p.run('state.mode'), 'planning');
  assert.deepEqual(writes(p).map(r=>r[0]), ['/api/link']);
});

test('状态请求失败离开 manual；恢复网络也不会跳过真机同步重新联动', async () => {
  const p = await interactive();
  p.run('state.linked=true; changeMode("manual"); scheduleMove()');
  p.respond(() => { throw new Error('offline'); });
  await p.nodes.get('syncStatusBtn').onclick();
  assert.equal(p.run('state.mode'), 'planning');
  assert.equal(p.run('state.backendUnknown'), true);
  p.respond(() => ({linked:true,motion_enabled:true,servos:{},trajectory:{state:'idle',index:0,total:0,error:null}}));
  await p.nodes.get('syncStatusBtn').onclick();
  assert.equal(p.run('state.mode'), 'planning');
  await p.tick(500);
  assert.equal(writes(p).length, 0);
});

test('初始连接状态未知时仍可明确断开，不会反向开启联动', async () => {
  const p = await interactive();
  p.respond(() => { throw new Error('offline'); });
  await p.nodes.get('syncStatusBtn').onclick();
  assert.equal(p.nodes.get('linkBtn').disabled, false);
  p.respond(() => ({linked:false,motion_enabled:false,trajectory:{state:'idle',index:0,total:0,error:null}}));
  await p.nodes.get('linkBtn').onclick();
  assert.deepEqual(writes(p).map(r=>[r[0],JSON.parse(r[1].body)]), [['/api/link',{enabled:false}]]);
  assert.equal(p.run('state.linked'), false);
  assert.equal(p.run('state.backendUnknown'), false);
});

for (const action of ['executeBtn', 'loadBtn', 'linkBtn']) {
  test(`${action} 悬挂时断连立即发送，旧响应不恢复联动`, async () => {
    const p = await interactive();
    p.run(`state.linked=${action !== 'linkBtn'}; state.recorder.start(state.targets,0); state.recorder.stop(state.targets,0)`);
    let release;
    p.respond((url, options) => JSON.parse(options?.body || '{}').enabled === false ?
      {linked:false,motion_epoch:'epoch-1',trajectory:{state:'idle',index:0,total:0}} : new Promise(resolve => {release=resolve;}));
    const pending = p.nodes.get(action).onclick(); await p.tick(0);
    const close = p.nodes.get('linkBtn').onclick(); await p.tick(0);
    assert.equal(writes(p).length, 2);
    assert.deepEqual(JSON.parse(writes(p)[1][1].body), {enabled:false});
    assert.equal(JSON.parse(writes(p)[0][1].body).motion_epoch, 'epoch-0');
    await close;
    release({linked:true,motion_epoch:'epoch-0',state:'running',index:0,total:1}); await pending;
    assert.equal(p.run('state.linked'), false);
    assert.equal(p.run('state.motionEpoch'), 'epoch-1');
    assert.equal(p.run('state.mode'), 'planning');
  });
}

test('10秒超时仅结束等待：保留未知门禁，停止断连和只读刷新仍可用', async () => {
  const p = await interactive();
  p.run('state.linked=true; changeMode("manual"); scheduleMove()');
  p.respond(() => new Promise(() => {}));
  await p.tick(150);
  const signal = writes(p)[0][1].signal;
  await p.tick(10001);
  assert.equal(p.run('state.inFlight.size'), 0);
  assert.equal(signal.aborted, true);
  assert.equal(p.run('state.backendUnknown'), true);
  assert.equal(p.run('state.mode'), 'planning');
  assert.equal(p.nodes.get('executeBtn').disabled, true);
  assert.equal(p.nodes.get('linkBtn').disabled, false);
  assert.equal(p.nodes.get('stopBtn').disabled, false);
  assert.equal(p.nodes.get('syncStatusBtn').disabled, false);
  assert.match(p.nodes.get('toast').textContent, /超时.*未确认.*取消/);
  p.respond(() => ({linked:true,trajectory:{state:'idle',index:0,total:0}}));
  await p.nodes.get('syncStatusBtn').onclick();
  assert.equal(p.run('state.mode'), 'planning');
  assert.equal(p.run('state.backendUnknown'), true, '同代际空闲不是运动取消确认');
  assert.equal(p.nodes.get('executeBtn').disabled, true);
});

test('执行准备等待旧 move 超时后不能偷偷发送 start，必须保持未知门禁', async () => {
  const p = await interactive();
  p.run('state.linked=true; changeMode("manual"); scheduleMove()');
  p.respond(() => new Promise(() => {}));
  await p.tick(150);
  p.nodes.get('recordBtn').onclick(); p.nodes.get('recordStopBtn').onclick();
  const start = p.nodes.get('executeBtn').onclick();
  await p.tick(10001);
  assert.deepEqual(writes(p).map(r=>r[0]), ['/api/move']);
  await start;
  assert.equal(p.run('state.backendUnknown'), true);
  assert.equal(p.run('state.mode'), 'planning');
});

test('外部 STOP 导致刷新 epoch 改变时离开 manual 并取消旧待发 timer', async () => {
  const p = await interactive();
  p.run('state.linked=true; changeMode("manual"); scheduleMove()');
  p.respond(() => ({motion_epoch:'epoch-peer-stop',linked:true,trajectory:{state:'idle',index:0,total:0}}));
  await p.nodes.get('syncStatusBtn').onclick(); await p.tick(500);
  assert.equal(p.run('state.mode'), 'planning');
  assert.equal(writes(p).length, 0);
});

test('没有 epoch 的旧服务器禁止开启联动与执行并明确要求重启', async () => {
  const p = await interactive();
  p.respond(() => ({motion_epoch:undefined,linked:false,trajectory:{state:'idle',index:0,total:0}}));
  await p.nodes.get('syncStatusBtn').onclick();
  assert.equal(p.run('state.motionEpoch'), null);
  assert.match(p.nodes.get('toast').textContent, /重启/);
  await p.nodes.get('executeBtn').onclick();
  await p.nodes.get('loadBtn').onclick();
  await p.nodes.get('linkBtn').onclick();
  assert.ok(writes(p).every(([, options]) => JSON.parse(options.body).enabled === false));
});

// 只替代传输边界；请求超时、generation、录制/预览和门禁均执行真实页面代码。
function backendReply(epoch = 'epoch-0', phase = 'idle', linked = true) {
  const trajectory = {motion_epoch:epoch, state:phase, index:0, total:1, run_id:21, current_angles:{}, error:null};
  return {...trajectory, linked, motion_enabled:linked, servos:{}, trajectory};
}
async function hangingMotion(kind = 'move') {
  const p = await interactive();
  p.nodes.get('recordBtn').onclick(); p.nodes.get('recordStopBtn').onclick();
  p.run(`state.linked=${kind !== 'link'}`);
  const paths = {move:'/api/move', start:'/api/trajectory/start', standby:'/api/standby',
    load:'/api/load', link:'/api/link', stop:'/api/stop', disconnect:'/api/link'};
  p.respond(url => url === paths[kind] ? new Promise(() => {}) : backendReply());
  let pending;
  if (kind === 'move') {
    p.run('changeMode("manual"); scheduleMove()'); await p.tick(150);
  } else {
    const buttons = {start:'executeBtn', standby:'standbyBtn', load:'loadBtn', link:'linkBtn', stop:'stopBtn', disconnect:'linkBtn'};
    pending = p.nodes.get(buttons[kind]).onclick(); await p.tick(0);
  }
  assert.deepEqual(writes(p).map(r => r[0]), [paths[kind]]);
  return {p, pending};
}
async function assertMotionLocked(p) {
  assert.equal(p.run('state.backendUnknown'), true);
  assert.notEqual(p.run('state.mode'), 'manual');
  for (const id of ['executeBtn','standbyBtn','loadBtn','unloadBtn','readBtn','relinkBtn','recordBtn','previewBtn','axisSel']) {
    assert.equal(p.nodes.get(id).disabled, true, id + ' 必须保持门禁');
  }
  for (const id of ['stopBtn','linkBtn','syncStatusBtn']) assert.equal(p.nodes.get(id).disabled, false, id);
  assert.match(p.nodes.get('modeLabel').textContent, /未确认.*停止.*断连.*同.*不解除/);
  const count = writes(p).length;
  for (const id of ['executeBtn','standbyBtn','loadBtn','unloadBtn','relinkBtn','readBtn','recordBtn','previewBtn']) {
    await p.nodes.get(id).onclick();
  }
  p.run('scheduleMove()'); await p.tick(200);
  assert.equal(writes(p).length, count, '不能只禁用按钮，真实 handler/timer 也必须拒绝运动');
}

test('运动未知：manual move 悬挂→复位→超时仍离开 manual 并封锁所有运动', async () => {
  const {p} = await hangingMotion();
  p.nodes.get('resetBtn').onclick();
  await p.tick(10001);
  assert.equal(p.run('state.mode'), 'planning');
  p.respond(() => backendReply());
  await assertMotionLocked(p);
});

test('运动未知：过期 move 超时自动结束录制，停止录制不能解除门禁', async () => {
  const {p} = await hangingMotion();
  p.nodes.get('recordBtn').onclick();
  await p.tick(10001);
  assert.equal(p.run('state.mode'), 'planning');
  assert.equal(p.run('state.recorder.active'), false);
  assert.equal(p.run('state.recordTimer'), null);
  const points = json(p.run('state.recorder.points'));
  p.nodes.get('recordStopBtn').onclick();
  p.respond(() => backendReply());
  await p.tick(30000);
  assert.deepEqual(json(p.run('state.recorder.points')), points, '不继续录制无数停留点');
  await assertMotionLocked(p);
});

test('运动未知：过期 move 超时取消本地预览，不再更新模型', async () => {
  const {p} = await hangingMotion();
  p.run('state.recorder.points[0].duration_ms=30000; state.recorder.total=30000');
  p.nodes.get('previewBtn').onclick();
  assert.equal(p.run('state.mode'), 'preview');
  await p.tick(10001);
  assert.equal(p.run('state.mode'), 'planning');
  assert.equal(p.run('state.previewFrame'), null);
  const pose = json(p.run('state.targets'));
  p.respond(() => backendReply());
  await p.tick(1000);
  assert.deepEqual(json(p.run('state.targets')), pose);
  await assertMotionLocked(p);
});

for (const kind of ['start', 'standby', 'load', 'link', 'stop', 'disconnect']) {
  test(`运动未知：${kind} 传输超时后同 epoch 空闲刷新和轮询均不能解锁`, async () => {
    const {p, pending} = await hangingMotion(kind);
    await p.tick(10001); await pending;
    p.respond(() => backendReply());
    await p.nodes.get('syncStatusBtn').onclick();
    await p.tick(1600);
    assert.ok(p.requests.some(([url]) => url === '/api/trajectory/status'), '未知期间只读轮询可继续');
    assert.equal(p.run('state.mode'), 'planning');
    assert.match(p.nodes.get('trajectoryStatus').textContent, /空闲.*未确认/);
    await assertMotionLocked(p);
    assert.equal(writes(p).length, 1, '不得自动停止或断连');
  });
}

for (const phase of ['running', 'completed', 'stopped']) {
  test(`运动未知：同 epoch ${phase} 状态不能当作请求取消确认`, async () => {
    const {p, pending} = await hangingMotion('start');
    await p.tick(10001); await pending;
    p.respond(() => backendReply('epoch-0', phase));
    await p.nodes.get('syncStatusBtn').onclick(); await p.tick(600);
    await assertMotionLocked(p);
  });
}

for (const source of ['stopBtn', 'linkBtn', 'syncStatusBtn', 'poll']) {
  test(`运动未知：${source} 确认不同 epoch 后解锁且保持规划`, async () => {
    const {p, pending} = await hangingMotion('start');
    await p.tick(10001); await pending;
    p.respond(() => backendReply());
    await p.nodes.get('syncStatusBtn').onclick();
    await assertMotionLocked(p);
    p.respond(() => backendReply('epoch-1', 'idle', source !== 'linkBtn'));
    if (source === 'poll') await p.tick(600);
    else await p.nodes.get(source).onclick();
    assert.equal(p.run('state.motionEpoch'), 'epoch-1');
    assert.equal(p.run('state.backendUnknown'), false);
    assert.equal(p.run('busy()'), false);
    assert.equal(p.run('state.mode'), 'planning');
    assert.equal(p.nodes.get('recordBtn').disabled, false);
    assert.doesNotMatch(p.nodes.get('modeLabel').textContent, /未确认/);
    assert.deepEqual(writes(p).map(r => r[0]), ['/api/trajectory/start',
      ...(source === 'stopBtn' ? ['/api/stop'] : source === 'linkBtn' ? ['/api/link'] : [])]);
    if (source !== 'linkBtn') {
      await p.nodes.get('loadBtn').onclick();
      assert.equal(JSON.parse(writes(p).at(-1)[1].body).motion_epoch, 'epoch-1');
    }
  });
}

for (const kind of ['move', 'start']) {
  for (const source of ['stopBtn', 'linkBtn']) {
    test(`运动未知：${source} 新 epoch 后旧 ${kind} 迟到超时不得再锁或改 UI`, async () => {
      const {p, pending} = await hangingMotion(kind);
      p.respond(() => backendReply('epoch-1', 'idle', source !== 'linkBtn'));
      await p.nodes.get(source).onclick();
      assert.equal(p.run('state.motionEpoch'), 'epoch-1');
      assert.equal(p.run('state.mode'), 'planning');
      const ui = ['modeLabel','trajectoryStatus','toast'].map(id => p.nodes.get(id).textContent);
      await p.tick(10001); await pending;
      assert.equal(p.run('state.backendUnknown'), false);
      assert.equal(p.run('busy()'), false);
      assert.equal(p.run('state.mode'), 'planning');
      assert.deepEqual(['modeLabel','trajectoryStatus','toast'].map(id => p.nodes.get(id).textContent), ui);
      assert.equal(p.nodes.get('recordBtn').disabled, false);
    });
  }
}

for (const action of ['syncStatusBtn', 'readBtn']) {
  test(`只读失败：${action} 超时经有效同 epoch 状态可恢复，不要求 STOP`, async () => {
    const p = await interactive();
    p.run('state.linked=true');
    p.respond(() => new Promise(() => {}));
    const pending = p.nodes.get(action).onclick();
    await p.tick(0); await p.tick(10001); await pending;
    assert.equal(p.run('state.backendUnknown'), true);
    p.respond(() => backendReply());
    await p.nodes.get('syncStatusBtn').onclick();
    assert.equal(p.run('state.backendUnknown'), false);
    assert.equal(p.run('busy()'), false);
    assert.equal(p.run('state.mode'), 'planning');
    assert.ok(writes(p).every(([url]) => url === '/api/read'));
  });
}

for (const missing of [undefined, null]) {
  test(`运动未知：缺失 epoch (${missing}) 再出现同旧 epoch 不得误解除`, async () => {
    const {p, pending} = await hangingMotion('start');
    await p.tick(10001); await pending;
    p.respond(() => ({...backendReply(), motion_epoch:missing}));
    await p.nodes.get('syncStatusBtn').onclick();
    assert.equal(p.run('state.motionEpoch'), null);
    p.respond(() => backendReply());
    await p.nodes.get('syncStatusBtn').onclick(); await p.tick(600);
    await assertMotionLocked(p);
  });
}

test('运动未知：从未取得 epoch 的 STOP 超时不能被首次空闲状态误解除', async () => {
  const p = await interactive();
  p.run('state.motionEpoch=null; state.backendUnknown=true');
  p.respond(() => new Promise(() => {}));
  const stop = p.nodes.get('stopBtn').onclick(); await p.tick(0);
  await p.tick(10001); await stop;
  p.respond(() => backendReply());
  await p.nodes.get('syncStatusBtn').onclick();
  await assertMotionLocked(p);
  p.respond(() => backendReply('epoch-1', 'stopped'));
  await p.nodes.get('stopBtn').onclick();
  assert.equal(p.run('busy()'), false);
  assert.equal(p.run('state.mode'), 'planning');
});

test('运动未知：move 超时不应吞掉已在途 STOP 的成功确认', async () => {
  const {p} = await hangingMotion();
  await p.tick(9000);
  let release;
  p.respond(url => url === '/api/stop' ? new Promise(resolve => {release=resolve;}) : backendReply());
  const stop = p.nodes.get('stopBtn').onclick(); await p.tick(0);
  await p.tick(1001);
  assert.equal(p.run('state.backendUnknown'), true);
  release(backendReply('epoch-1', 'stopped')); await stop;
  assert.equal(p.run('state.motionEpoch'), 'epoch-1');
  assert.equal(p.run('busy()'), false);
  assert.equal(p.run('state.mode'), 'planning');
});

test('运动未知：STOP 后暂缺 epoch 也不能让旧 move 超时污染新会话', async () => {
  const {p} = await hangingMotion();
  p.respond(() => backendReply('epoch-1', 'stopped'));
  await p.nodes.get('stopBtn').onclick();
  p.respond(() => ({...backendReply(), motion_epoch:null}));
  await p.nodes.get('syncStatusBtn').onclick();
  assert.equal(p.run('state.motionEpoch'), null);
  await p.tick(10001);
  p.respond(() => backendReply('epoch-1'));
  await p.nodes.get('syncStatusBtn').onclick();
  assert.equal(p.run('busy()'), false);
  assert.equal(p.run('state.mode'), 'planning');
});

test('运动未知：普通 read 或 HTTP200 旧动作成功响应不能清除门禁', async () => {
  const {p} = await hangingMotion();
  await p.tick(500);
  let release;
  p.respond(() => new Promise(resolve => {release=resolve;}));
  // 两个已在途请求：只替代 HTTP 边界，不替代门禁和状态应用函数。
  const oldAction = p.run('trackedApi("/api/trajectory/start", {points:state.recorder.points})');
  p.respond(() => backendReply());
  await p.tick(9501);
  release(backendReply());
  p.context.oldStatus = await oldAction;
  p.run('applyTrajectoryStatus(oldStatus)');
  await p.run('api("/api/read", {})');
  await assertMotionLocked(p);
});

test('仅有未保存本地录制时阻止无提示刷新，清空后不再拦截', async () => {
  const p = await interactive();
  let prevented = false;
  const unload = () => p.context.window.dispatch('beforeunload', {preventDefault(){prevented=true;}});
  unload(); assert.equal(prevented, false);
  p.nodes.get('recordBtn').onclick();
  const event = unload();
  assert.equal(prevented, true);
  assert.equal(event.returnValue, '');
  p.nodes.get('recordStopBtn').onclick();
  p.nodes.get('clearBtn').onclick();
  prevented = false; unload();
  assert.equal(prevented, false);
  assert.equal(writes(p).length, 0);
});

for (const aspect of [420/714, 1.8]) {
  test(`实际STL宽窄取景 ${aspect} 留边且复位只改相机`, async () => {
    const p = await interactive({aspect});
    const checkProjection = () => {
      const bounds = p.run(`(() => {
        scene.updateMatrixWorld(true); camera.updateMatrixWorld(true);
        const bounds = {x:0,y:0,z:0,count:0};
        state.robotGroup.traverseVisible(mesh => {
          if(!mesh.isMesh) return;
          const vertices = mesh.geometry.attributes.position;
          for(let i=0;i<vertices.count;i++) {
            const pt = new THREE.Vector3().fromBufferAttribute(vertices,i).applyMatrix4(mesh.matrixWorld).project(camera);
            bounds.x=Math.max(bounds.x,Math.abs(pt.x)); bounds.y=Math.max(bounds.y,Math.abs(pt.y));
            bounds.z=Math.max(bounds.z,Math.abs(pt.z)); bounds.count++;
          }
        });
        return bounds;
      })()`);
      assert.ok(bounds.count > 1000);
      assert.ok(bounds.x <= .87 && bounds.y <= .87 && bounds.z < 1, JSON.stringify(bounds));
    };
    checkProjection();
    p.run('setServoAngle(6,40); setServoAngle(2,20); camera.position.set(0,.1,.1)');
    const before = json(p.run('state.targets'));
    p.nodes.get('resetBtn').onclick();
    checkProjection();
    assert.deepEqual(json(p.run('state.targets')), before);
    assert.equal(writes(p).length, 0);
  });
}

test('HTTP200 串口失败显示具体错误而非成功断开或连接', async () => {
  const p = await interactive();
  p.respond(() => ({linked:false,motion_enabled:false,error:'OSError: FAKE port unavailable',trajectory:{state:'idle',index:0,total:0}}));
  await p.nodes.get('linkBtn').onclick();
  assert.match(p.nodes.get('toast').textContent, /FAKE port unavailable/);
  assert.doesNotMatch(p.nodes.get('toast').textContent, /已连接|已断开联动/);
  assert.equal(p.run('state.mode'), 'planning');
  assert.equal(p.run('state.linked'), false);
});

test('停留录制保持起点10秒、独立运动段和尾部10秒，停止时间不丢失', async () => {
  const p = await interactive();
  p.run(`globalThis.rec = new ArmTrajectory.Recorder();
    globalThis.pose = {1:0,2:0,3:0,4:0,5:0,6:0}; rec.start(pose,0);
    for(let t=100;t<=10000;t+=100) rec.sample(pose,t);
    pose[6]=3; rec.sample(pose,10100);
    for(let t=10200;t<20100;t+=100) rec.sample(pose,t);
    rec.stop(pose,20100);`);
  assert.equal(p.run('ArmTrajectory.poseAt(rec.points,5000)[6]'), 0);
  assert.equal(p.run('ArmTrajectory.poseAt(rec.points,10050)[6]'), 1.5);
  assert.equal(p.run('ArmTrajectory.poseAt(rec.points,15000)[6]'), 3);
  assert.equal(p.run('rec.total'), 20100);
  assert.equal(p.run('rec.points.length'), 3);
  assert.deepEqual(json(p.run('rec.points.map(p=>p.duration_ms)')), [10000,100,10000]);
  p.run(`for(let i=1;i<rec.points.length;i++) {
    const a=rec.points[i-1],b=rec.points[i];
    if(Math.abs(a.angles[6]-b.angles[6])*1000/b.duration_ms>30) throw Error('超过计划限速');
  }`);
});

test('长停留有界拆分合并，600点与600000ms上限和短尾均合法', async () => {
  const p = await interactive();
  p.run(`globalThis.rec = new ArmTrajectory.Recorder();
    globalThis.pose = {1:0,2:0,3:0,4:0,5:0,6:0}; rec.start(pose,0);
    rec.sample(pose,30001);`);
  assert.equal(p.run('rec.total'), 30001);
  assert.equal(p.run('rec.points.length'), 2);
  assert.ok(p.run('rec.points.every(p=>p.duration_ms>=80&&p.duration_ms<=30000)'));
  p.run('for(let t=30100;t<=610000;t+=100) rec.sample(pose,t);');
  assert.equal(p.run('rec.total'), 600000);
  assert.equal(p.run('rec.points.length'), 20);
  assert.equal(p.run('rec.active'), false);
  assert.ok(p.run('rec.points.every(p=>Number.isInteger(p.duration_ms)&&p.duration_ms>=80&&p.duration_ms<=30000)'));
});

for (const invalid of [32767, 500.5, null, '500', 'missing', 'false-ok', 'fake-ok']) {
  test(`非法六轴回读 ${invalid} 不夹紧、不部分覆盖、不恢复 manual`, async () => {
    const p = await interactive();
    p.run('state.linked=true; setServoAngle(6,50)');
    const before = json(p.run('state.targets'));
    const servos = Object.fromEntries([1,2,3,4,5,6].map(id => [id,{ok:true,position:id===1?700:500,loaded:id!==2}]));
    if (invalid === 'missing') delete servos[6];
    else if (invalid === 'false-ok') servos[6].ok = false;
    else if (invalid === 'fake-ok') servos[6].ok = 1;
    else servos[6].position = invalid;
    p.respond(() => ({servos}));
    await p.nodes.get('relinkBtn').onclick();
    assert.equal(p.run('state.mode'), 'planning');
    assert.deepEqual(json(p.run('state.targets')), before);
    assert.match(p.nodes.get('toast').textContent, /回读/);
    p.run('setServoAngle(2,20); scheduleMove()'); await p.tick(500);
    assert.deepEqual(writes(p).map(r => r[0]), ['/api/read']);
  });
}

// 真机示教：只替代 HTTP，仍走真实页面状态机、three/STL、定时器与轨迹模块。
const teachingIds = ['teachStartBtn', 'captureBtn', 'teachEndBtn', 'teachDuration', 'teachingPoints'];
function assertTeachingUI(p) {
  for (const id of teachingIds) assert.ok(p.nodes.get(id), '缺少逐点示教控件 ' + id);
}
function readReply(raw = {1:213,2:167,3:36,4:500,5:500,6:625}, epoch = 'epoch-0') {
  return {...backendReply(epoch), servos:Object.fromEntries(Object.entries(raw).map(([id, position]) =>
    [id, {ok:true, position, loaded:false, temperature:25}]))};
}
async function teachingPage() {
  const p = await interactive(); assertTeachingUI(p);
  p.run('state.linked=true'); p.respond(() => backendReply());
  await p.nodes.get('teachStartBtn').onclick();
  assert.equal(p.run('state.mode'), 'teaching');
  return p;
}
const snapshot = p => json(p.run('({points:state.recorder.points,total:state.recorder.total,targets:state.targets})'));

test('示教 appendPoint 原子深拷贝、同姿态独立点、容量与整数时长边界', async () => {
  const p = await interactive();
  assert.equal(p.run('typeof state.recorder.appendPoint'), 'function', '需要独立逐点追加，不能复用 sample');
  p.run(`globalThis.point = {angles:{1:0,2:0,3:0,4:0,5:0,6:0},
    positions:{1:700,2:500,3:500,4:500,5:500,6:500},duration_ms:80};
    state.recorder.appendPoint(point); state.recorder.appendPoint(point);
    point.angles[6]=30; point.positions[6]=625;`);
  assert.equal(p.run('state.recorder.points.length'), 2);
  assert.equal(p.run('state.recorder.total'), 160);
  assert.equal(p.run('state.recorder.points[0].angles[6]'), 0);
  assert.equal(p.run('state.recorder.points[1].positions[6]'), 500);
  for (const duration of ['NaN', '80.5', '0', '79', '30001', 'null', 'true', '"1000"']) {
    const before = snapshot(p);
    assert.throws(() => p.run(`state.recorder.appendPoint({...point,duration_ms:${duration}})`));
    assert.deepEqual(snapshot(p), before);
  }
  for (const [duration, count] of [[80,600],[30000,20]]) {
    p.run(`state.recorder.clear(); point.duration_ms=${duration};
      for(let i=0;i<${count};i++) state.recorder.appendPoint(point);`);
    const before = snapshot(p);
    assert.throws(() => p.run('state.recorder.appendPoint(point)'), /上限|容量|时长/);
    assert.deepEqual(snapshot(p), before);
  }
  p.run('globalThis.copied=ArmTrajectory.copyPoint(state.recorder.points[0]); copied.positions[6]=700; copied.angles[6]=40;');
  assert.equal(p.run('state.recorder.points[0].positions[6]'), 625);
  assert.equal(p.run('state.recorder.points[0].angles[6]'), 30);
});

for (const block of ['offline','no-epoch','busy','unknown','uncertain','recording','preview','invalid-mode']) {
  test(`示教开始门禁 ${block} 不能清点或发送任何请求`, async () => {
    const p = await interactive(); assertTeachingUI(p);
    p.run('state.linked=true; state.recorder.start(state.targets,0); state.recorder.stop(state.targets,0)');
    const setups = {offline:'state.linked=false', 'no-epoch':'state.motionEpoch=null', busy:'state.trajectory.state="preparing"',
      unknown:'state.backendUnknown=true', uncertain:'state.uncertainMotion={epoch:"epoch-0"}',
      recording:'state.mode="recording"',preview:'state.mode="preview"','invalid-mode':'state.mode="other"'};
    p.run(setups[block] + '; updateModeUI()');
    const before = snapshot(p);
    assert.equal(p.nodes.get('teachStartBtn').disabled, true);
    await p.nodes.get('teachStartBtn').onclick();
    assert.deepEqual(snapshot(p), before); assert.equal(p.requests.length, 0);
  });
}

test('示教覆盖取消不改原轨迹；准备等待旧请求且失败保留，成功不读首点或写硬件', async () => {
  const p = await interactive(); assertTeachingUI(p);
  p.run('state.linked=true; state.recorder.start(state.targets,0); state.recorder.stop(state.targets,0)');
  const before = snapshot(p);
  p.context.window.confirm = () => false;
  await p.nodes.get('teachStartBtn').onclick();
  assert.deepEqual(snapshot(p), before); assert.equal(p.requests.length, 0);
  p.context.window.confirm = () => true;
  let release;
  p.respond(url => url === '/api/move' ? new Promise(resolve => {release=resolve;}) : backendReply());
  p.run('changeMode("manual"); scheduleMove()'); await p.tick(150);
  const pending = p.nodes.get('teachStartBtn').onclick(); await p.tick(0);
  assert.equal(p.run('state.mode'), 'planning'); assert.equal(p.run('state.actionBusy'), true);
  assert.deepEqual(snapshot(p), before);
  assert.deepEqual(p.requests.map(r=>r[0]), ['/api/move']);
  p.respond(() => ({...backendReply(), error:'准备失败'}));
  release({}); await pending;
  assert.deepEqual(snapshot(p), before); assert.equal(p.run('state.mode'), 'planning');
  p.respond(() => backendReply()); await p.nodes.get('syncStatusBtn').onclick();
  p.run('changeMode("manual"); scheduleMove(); state.dragger.drag={id:1,controlsEnabled:true}; controls.enabled=false');
  p.requests.length = 0;
  await p.nodes.get('teachStartBtn').onclick(); await p.tick(200);
  assert.equal(p.run('state.mode'), 'teaching'); assert.equal(p.run('state.recorder.points.length'), 0);
  assert.equal(p.run('state.dragger.drag'), null); assert.equal(p.run('controls.enabled'), true);
  assert.deepEqual(json(p.run('state.targets')), before.targets);
  assert.deepEqual(p.requests.map(r=>r[0]), ['/api/status']);
});

for (const bad of ['disconnected','epoch','missing-epoch','running','bad-trajectory']) {
  test(`示教准备新 status ${bad} 保留旧点`, async () => {
    const p = await interactive(); assertTeachingUI(p);
    p.run('state.linked=true; state.recorder.start(state.targets,0); state.recorder.stop(state.targets,0)');
    const before = snapshot(p), response = backendReply();
    if (bad === 'disconnected') response.linked = false;
    if (bad === 'epoch') response.motion_epoch = 'epoch-1';
    if (bad === 'missing-epoch') response.motion_epoch = null;
    if (bad === 'running') response.trajectory.state = 'running';
    if (bad === 'bad-trajectory') response.trajectory = {};
    p.respond(() => response); await p.nodes.get('teachStartBtn').onclick();
    assert.notEqual(p.run('state.mode'), 'teaching'); assert.deepEqual(snapshot(p), before);
    assert.deepEqual(p.requests.map(r=>r[0]), ['/api/status']);
  });
}

test('示教多次新 READ 冻结点击时长、接受卸力实值不取模型，预览与执行保留 raw', async () => {
  const p = await teachingPage();
  const before = json(p.run('state.targets')); let release;
  p.respond(url => url === '/api/read' ? new Promise(resolve => {release=resolve;}) : backendReply());
  assert.equal(p.nodes.get('teachDuration').value, '1000');
  p.nodes.get('teachDuration').value = '1234';
  const capture = p.nodes.get('captureBtn').onclick(); await p.tick(0);
  p.nodes.get('teachDuration').value = '2345';
  const reply = readReply(); release(reply); await capture;
  assert.notDeepEqual(json(p.run('state.targets')), before);
  assert.deepEqual(json(p.run('state.recorder.points[0].positions')), {1:213,2:167,3:36,4:500,5:500,6:625});
  assert.equal(p.run('state.recorder.points[0].duration_ms'), 1234);
  assert.ok(Math.abs(p.run('state.recorder.points[0].angles[1]') - 87.66) < 1e-9);
  assert.ok(Math.abs(p.run('state.recorder.points[0].angles[2]') - 79.92) < 1e-9);
  assert.ok(Math.abs(p.run('state.recorder.points[0].angles[3]') + 111.36) < 1e-9);
  reply.servos[1].position = 700;
  p.respond(() => readReply()); await p.nodes.get('captureBtn').onclick();
  assert.deepEqual(json(p.run('state.recorder.points.map(p=>p.duration_ms)')), [1234,2345]);
  assert.equal(p.run('state.recorder.total'), 3579);
  assert.equal(p.run('state.recorder.points[0].positions[1]'), 213);
  p.respond(() => readReply({1:700,2:500,3:500,4:600,5:500,6:500}));
  await p.nodes.get('captureBtn').onclick();
  assert.equal(p.run('state.recorder.points[2].positions[4]'), 600);
  assert.equal(p.run('state.targets[4]'), 24);
  assert.deepEqual(writes(p).map(r=>r[0]), ['/api/read','/api/read','/api/read']);
  assert.match(p.nodes.get('teachingPoints').textContent, /1234.*213/s);
  const points = json(p.run('state.recorder.points'));
  await p.nodes.get('teachEndBtn').onclick(); assert.equal(p.run('state.mode'), 'planning');
  p.nodes.get('previewBtn').onclick(); await p.tick(100); p.nodes.get('previewStopBtn').onclick();
  assert.deepEqual(json(p.run('state.recorder.points')), points);
  p.respond(() => backendReply('epoch-0','preparing').trajectory);
  await p.nodes.get('executeBtn').onclick();
  assert.deepEqual(JSON.parse(writes(p).at(-1)[1].body).points, points);
  assert.equal(writes(p).at(-1)[0], '/api/trajectory/start');
});

for (const invalid of ['', 'NaN', '0', '80.5', '30001', '79']) {
  test(`示教时长 ${invalid} 拒绝而非默认或夹紧`, async () => {
    const p = await teachingPage(), before = snapshot(p);
    p.nodes.get('teachDuration').value = invalid;
    await p.nodes.get('captureBtn').onclick();
    assert.deepEqual(snapshot(p), before); assert.equal(writes(p).length, 0);
    assert.match(p.nodes.get('toast').textContent, /整数|时长/);
  });
}

for (const invalid of ['missing', 'null-item', 'false-ok', 'fake-ok', null, '', '500', 500.5, true, 32767, -1, 'claw-range', 'roll-range', 'cached-offline', 'error', 'epoch', 'missing-epoch', 'bad-trajectory', 'running']) {
  test(`示教非法 READ ${invalid} 不入点且不部分修改模型`, async () => {
    const p = await teachingPage(), before = snapshot(p), reply = readReply();
    if (invalid === 'missing') delete reply.servos[6];
    else if (invalid === 'null-item') reply.servos[6] = null;
    else if (invalid === 'false-ok') reply.servos[6].ok = false;
    else if (invalid === 'fake-ok') reply.servos[6].ok = 1;
    else if (invalid === 'claw-range') reply.servos[1].position = 199;
    else if (invalid === 'roll-range') reply.servos[2].position = 876;
    else if (invalid === 'cached-offline') reply.linked = false;
    else if (invalid === 'error') reply.error = '串口读取失败';
    else if (invalid === 'epoch') reply.motion_epoch = 'epoch-other';
    else if (invalid === 'missing-epoch') reply.motion_epoch = null;
    else if (invalid === 'bad-trajectory') reply.trajectory = {state:'unknown'};
    else if (invalid === 'running') reply.trajectory.state = 'running';
    else reply.servos[6].position = invalid;
    p.respond(() => reply); await p.nodes.get('captureBtn').onclick();
    assert.deepEqual(snapshot(p), before);
    assert.equal(p.run('state.uncertainMotion'), null);
    if (['cached-offline','error','epoch','missing-epoch','bad-trajectory','running'].includes(invalid)) {
      assert.notEqual(p.run('state.mode'), 'teaching');
    }
  });
}

test('示教容量失败不提交点或模型，同姿态也占独立点', async () => {
  const p = await teachingPage();
  p.run(`for(let i=0;i<600;i++) state.recorder.appendPoint({angles:state.targets,duration_ms:80})`);
  const before = snapshot(p); p.respond(() => readReply());
  await p.nodes.get('captureBtn').onclick();
  assert.deepEqual(snapshot(p), before); assert.match(p.nodes.get('toast').textContent, /上限|容量/);
});

test('示教禁止按钮与直接 handler/拖动/滑块绕过，结束始终可用', async () => {
  const p = await teachingPage(), before = snapshot(p), count = p.requests.length;
  for (const id of ['recordBtn','recordStopBtn','clearBtn','previewBtn','previewStopBtn','executeBtn','standbyBtn','relinkBtn','readBtn','loadBtn','unloadBtn','axisSel']) {
    assert.equal(p.nodes.get(id).disabled, true, id);
    if (p.nodes.get(id).onclick) await p.nodes.get(id).onclick();
  }
  p.run('enterPlanning(); startRecording(); startPreview(); readPose(true); beginExecution("/api/standby",{}); scheduleMove(); sendMove()');
  const slider = p.run('document.querySelector(\'input[data-servo="6"]\')');
  slider.value=60; slider.dispatch('input'); slider.dispatch('change');
  p.run('state.dragger.change(6,45)'); p.canvas.dispatch('pointerdown', {clientX:400,clientY:300});
  await p.tick(200);
  assert.equal(p.run('state.mode'), 'teaching'); assert.deepEqual(snapshot(p), before);
  assert.equal(p.requests.length, count);
  assert.equal(p.nodes.get('teachEndBtn').disabled, false);
});

for (const source of ['poll','refresh']) {
  test(`示教 ${source} 持续只读完整状态但不覆盖点或模型`, async () => {
    const p = await teachingPage(); p.respond(() => readReply());
    await p.nodes.get('captureBtn').onclick(); const before = snapshot(p);
    p.respond(() => readReply({1:700,2:500,3:500,4:500,5:500,6:500}));
    if (source === 'refresh') await p.nodes.get('syncStatusBtn').onclick();
    await p.tick(2100);
    assert.ok(p.requests.filter(([url])=>url==='/api/status').length >= 4,
      JSON.stringify({requests:p.requests.map(r=>r[0]), mode:p.run('state.mode'), poll:p.run('state.pollTimer'), unknown:p.run('state.backendUnknown')}));
    assert.ok(!p.requests.some(([url])=>url==='/api/trajectory/status'));
    assert.deepEqual(snapshot(p), before); assert.equal(p.run('state.mode'), 'teaching');
  });
}

for (const interrupt of ['end','stop','disconnect','epoch','running','preparing','offline','unknown']) {
  for (const late of ['success','error']) {
    test(`示教 capture 双击与 ${interrupt}，迟到 ${late} 不污染新会话`, async () => {
      const p = await teachingPage(); p.respond(() => readReply()); await p.nodes.get('captureBtn').onclick();
      const before = snapshot(p); let resolve, reject;
      p.respond(url => url === '/api/read' ? new Promise((a,b) => {resolve=a;reject=b;}) : backendReply());
      const pending = p.nodes.get('captureBtn').onclick(); await p.tick(0);
      await p.nodes.get('captureBtn').onclick();
      assert.equal(writes(p).filter(([url])=>url==='/api/read').length, 2);
      assert.equal(p.nodes.get('teachEndBtn').disabled, false);
      if (interrupt === 'end') await p.nodes.get('teachEndBtn').onclick();
      else if (interrupt === 'stop' || interrupt === 'disconnect') {
        p.respond(() => backendReply('epoch-1','stopped',interrupt !== 'disconnect'));
        const action = p.nodes.get(interrupt==='stop'?'stopBtn':'linkBtn').onclick();
        await p.tick(0);
        assert.equal(writes(p).at(-1)[0], interrupt==='stop'?'/api/stop':'/api/link');
        await action;
      } else {
        p.respond(() => {
          if (interrupt === 'unknown') throw new Error('状态丢失');
          return backendReply(interrupt==='epoch'?'epoch-1':'epoch-0', ['running','preparing'].includes(interrupt)?interrupt:'idle', interrupt!=='offline');
        });
        await p.tick(510);
      }
      assert.notEqual(p.run('state.mode'), 'teaching'); assert.deepEqual(snapshot(p), before);
      // 下一次开始仍等待旧 read；旧错误不应改 UI、锁门或清除下一次 captureBusy。
      const ui = ['toast','trajectoryStatus'].map(id=>p.nodes.get(id).textContent);
      if (late==='success') resolve(readReply()); else reject(new Error('旧回读失败'));
      await pending;
      assert.deepEqual(snapshot(p), before);
      assert.deepEqual(['toast','trajectoryStatus'].map(id=>p.nodes.get(id).textContent), ui);
      p.respond(() => backendReply('epoch-1')); await p.nodes.get('syncStatusBtn').onclick();
      await p.nodes.get('teachStartBtn').onclick(); assert.equal(p.run('state.mode'),'teaching');
      let next;
      p.respond(() => new Promise(resolve=>{next=resolve;}));
      const newCapture=p.nodes.get('captureBtn').onclick(); await p.tick(0);
      assert.equal(p.run('state.captureBusy'),true);
      next(readReply(undefined,'epoch-1')); await newCapture;
      assert.equal(p.run('state.recorder.points.length'),1);
      assert.equal(p.run('state.captureBusy'),false);
    });
  }
}

test('示教 read 超时结束教学，仅普通 backendUnknown，同 epoch 刷新可恢复', async () => {
  const p = await teachingPage();
  p.respond(() => readReply()); await p.nodes.get('captureBtn').onclick(); const before=snapshot(p);
  p.respond(url => url==='/api/read' ? new Promise(()=>{}) : backendReply());
  const capture=p.nodes.get('captureBtn').onclick(); await p.tick(0); await p.tick(10001); await capture;
  assert.deepEqual(snapshot(p),before); assert.equal(p.run('state.mode'),'planning');
  assert.equal(p.run('state.backendUnknown'),true); assert.equal(p.run('state.uncertainMotion'),null);
  assert.equal(p.nodes.get('captureBtn').disabled,true);
  p.respond(() => backendReply()); await p.nodes.get('syncStatusBtn').onclick();
  assert.equal(p.run('state.backendUnknown'),false); assert.equal(p.run('state.mode'),'planning');
});

test('示教 read HTTP409 立即结束并锁门，刷新完成前不可再采集', async () => {
  const p = await teachingPage(), before=snapshot(p); let refresh;
  p.context.fetch=(url,options)=>{
    p.requests.push([url,options]);
    if(url==='/api/read') return Promise.resolve({ok:false,status:409,json:async()=>({error:'busy'})});
    return new Promise(resolve=>{refresh=resolve;});
  };
  const capture=p.nodes.get('captureBtn').onclick(); await p.tick(0);
  assert.equal(p.run('state.mode'),'planning'); assert.equal(p.run('state.backendUnknown'),true);
  assert.equal(p.nodes.get('captureBtn').disabled,true); assert.equal(p.nodes.get('teachStartBtn').disabled,true);
  assert.ok(refresh,'409 必须触发新状态刷新');
  await p.nodes.get('captureBtn').onclick();
  refresh({ok:true,json:async()=>backendReply('epoch-1','running')}); await capture; await p.tick(0);
  assert.equal(p.run('state.mode'),'executing'); assert.deepEqual(snapshot(p),before);
  assert.equal(p.run('state.uncertainMotion'),null);
});

// 状态的迟到响应也必须隔离，不能让教学旧 poll 覆盖较新的 epoch / 后台忙碌。
for (const change of ['epoch','running','missing-epoch']) {
  test(`示教 capture 发现 ${change} 后旧 poll 不得回滚安全状态`, async () => {
    const p=await teachingPage(), before=snapshot(p); let oldPoll;
    p.respond(url=>url==='/api/status' ? new Promise(resolve=>{oldPoll=resolve;}) : backendReply());
    await p.tick(500); assert.ok(oldPoll);
    const reply=readReply();
    if(change==='epoch') reply.motion_epoch='epoch-1';
    if(change==='running') reply.trajectory.state='running';
    if(change==='missing-epoch') reply.motion_epoch=null;
    p.respond(()=>reply); await p.nodes.get('captureBtn').onclick();
    assert.notEqual(p.run('state.mode'),'teaching');
    const safety=json(p.run('({epoch:state.motionEpoch,phase:state.trajectory.state,unknown:state.backendUnknown,mode:state.mode})'));
    oldPoll(backendReply()); await p.tick(0);
    assert.deepEqual(json(p.run('({epoch:state.motionEpoch,phase:state.trajectory.state,unknown:state.backendUnknown,mode:state.mode})')),safety);
    assert.deepEqual(snapshot(p),before);
  });
}

test('示教准备发现后台 running 后继续只读跟踪，不会永远卡在 executing', async () => {
  const p=await interactive(); assertTeachingUI(p); p.run('state.linked=true');
  p.respond(()=>backendReply('epoch-0','running'));
  await p.nodes.get('teachStartBtn').onclick();
  assert.equal(p.run('state.mode'),'executing');
  p.respond(()=>backendReply('epoch-0','completed').trajectory); await p.tick(510);
  assert.equal(p.run('state.mode'),'planning');
  assert.equal(p.run('state.trajectory.state'),'completed');
  assert.equal(p.run('state.recorder.points.length'),0);
  assert.equal(writes(p).length,0);
});

for (const late of ['success','error']) {
  test(`示教超时旧 HTTP 迟到 ${late} 时，新 capture 的 busy/token/点不受影响`, async () => {
    const p=await teachingPage(); let resolveOld,rejectOld;
    p.respond(url=>url==='/api/read'?new Promise((a,b)=>{resolveOld=a;rejectOld=b;}):backendReply());
    const old=p.nodes.get('captureBtn').onclick(); await p.tick(0); await p.tick(10001); await old;
    p.respond(()=>backendReply('epoch-1')); await p.nodes.get('syncStatusBtn').onclick();
    await p.nodes.get('teachStartBtn').onclick();
    let next;
    p.respond(url=>url==='/api/read'?new Promise(resolve=>{next=resolve;}):backendReply('epoch-1'));
    const pending=p.nodes.get('captureBtn').onclick(); await p.tick(0);
    const before=snapshot(p), token=p.run('state.captureToken');
    const ui=['toast','modeLabel','trajectoryStatus'].map(id=>p.nodes.get(id).textContent);
    if(late==='success') resolveOld(readReply()); else rejectOld(new Error('迟到失败'));
    await p.tick(0);
    assert.equal(p.run('state.captureBusy'),true); assert.equal(p.run('state.captureToken'),token);
    assert.deepEqual(snapshot(p),before);
    assert.deepEqual(['toast','modeLabel','trajectoryStatus'].map(id=>p.nodes.get(id).textContent),ui);
    next(readReply(undefined,'epoch-1')); await pending;
    assert.equal(p.run('state.recorder.points.length'),1); assert.equal(p.run('state.captureBusy'),false);
  });
}

for (const action of ['stopBtn','linkBtn']) {
  test(`示教开始等待旧请求时 ${action} 立即取消准备，迟到新 status 不能清轨迹`, async () => {
    const p=await interactive(); assertTeachingUI(p);
    p.run('state.linked=true; state.recorder.start(state.targets,0); state.recorder.stop(state.targets,0)');
    const before=snapshot(p); let old;
    p.respond(()=>new Promise(resolve=>{old=resolve;}));
    const start=p.nodes.get('teachStartBtn').onclick(); await p.tick(0);
    p.respond(()=>backendReply('epoch-1','stopped',action!=='linkBtn'));
    await p.nodes.get(action).onclick();
    old(backendReply()); await start;
    assert.deepEqual(snapshot(p),before); assert.equal(p.run('state.mode'),'planning');
    assert.equal(p.run('state.motionEpoch'),'epoch-1');
    assert.equal(p.run('state.actionBusy'),false);
  });
}

test('示教开始等待旧 move 超时不清点、不自动 read，独立 uncertainMotion 继续锁门', async () => {
  const p=await interactive(); assertTeachingUI(p);
  p.run('state.linked=true; state.recorder.start(state.targets,0); state.recorder.stop(state.targets,0)');
  const before=snapshot(p);
  p.respond(()=>new Promise(()=>{})); p.run('changeMode("manual"); scheduleMove()'); await p.tick(150);
  const start=p.nodes.get('teachStartBtn').onclick(); await p.tick(10001); await start;
  assert.deepEqual(snapshot(p),before); assert.equal(p.run('state.mode'),'planning');
  assert.ok(p.run('state.uncertainMotion')); assert.equal(p.nodes.get('teachStartBtn').disabled,true);
  assert.deepEqual(writes(p).map(r=>r[0]),['/api/move']);
  p.respond(()=>backendReply()); await p.nodes.get('syncStatusBtn').onclick();
  assert.equal(p.nodes.get('teachStartBtn').disabled,true); await p.nodes.get('teachStartBtn').onclick();
  assert.deepEqual(snapshot(p),before);
});
