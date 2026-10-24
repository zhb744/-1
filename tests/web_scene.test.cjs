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

function page({ failMesh = false, noCamera = false } = {}) {
  const nodes = new Map([...html.matchAll(/<[^>]+\bid="([^"]+)"[^>]*>/g)].map(m => [m[1], {
    checked: /\bchecked\b/.test(m[0]), disabled: /\bdisabled\b/.test(m[0]),
    style: {}, textContent: '', className: '',
  }]));
  const requests = [], timers = [];
  const context = vm.createContext({
    THREE: { ...THREE }, window: {}, console, model: structuredClone(fixture.model),
    document: { getElementById: id => nodes.get(id) || null, querySelector: () => null },
    fetch: (...args) => { requests.push(args); return new Promise(() => {}); },
    setTimeout: fn => { timers.push(fn); return timers.length; }, clearTimeout() {},
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
  vm.runInContext(script, context); // boot 等待 model 请求，不连接任何串口
  const run = code => vm.runInContext(code, context);
  run('initThree = () => { scene = new THREE.Scene(); };'); // 无 GPU；运动树和几何代码保持原样
  requests.length = 0;
  return { run, context, nodes, requests, timers };
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

test('首次加载底座零位使用正90度补偿，纠正左右相反且不发送运动', async () => {
  const p = await ready();
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
  p.run('renderStatus({servos: {6: {name: "底座", position: 625, temperature: null, loaded: true}}})');
  assert.ok(Math.abs(p.run('state.targets[6]') - 30) < 1e-9);
  assert.ok(tbody.innerHTML.includes('30.0°'));
  assert.ok(baseHeading(p).distanceTo(new THREE.Vector3(-.5, .8660254037844386, 0)) < 1e-9);
  p.run('renderStatus({servos: {6: {name: "底座", position: 500, temperature: null, loaded: true}}})');
  assert.equal(p.run('state.targets[6]'), 0);
  assert.ok(baseHeading(p).distanceTo(new THREE.Vector3(0, 1, 0)) < 1e-9);
  assert.equal(p.requests.length, 0);
  assert.equal(p.timers.length, 0);
});
