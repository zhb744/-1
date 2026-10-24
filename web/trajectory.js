/* 本地关节规划：不访问网络，不使用末端 IK。依赖本地 three r128。 */
const ArmTrajectory = (() => {
  'use strict';
  const DEG = Math.PI / 180;
  const DEFAULT_LIMITS = Object.freeze({ max_points: 600, min_duration_ms: 80,
    max_duration_ms: 30000, max_total_ms: 600000, max_speed_deg_s: 30 });
  const copyPose = pose => Object.fromEntries([1, 2, 3, 4, 5, 6].map(id => {
    const value = pose[id];
    if (!Number.isFinite(value)) throw new Error('轨迹必须包含六轴有限角度');
    return [id, value];
  }));
  const distance = (a, b) => Math.max(...Object.keys(a).map(id => Math.abs(a[id] - b[id])));
  const wrap = radians => Math.atan2(Math.sin(radians), Math.cos(radians));

  function jointMatrices(entry, degrees) {
    const angle = degrees * DEG * (entry.joint.servo_scale ?? 1) + (entry.joint.servo_offset || 0);
    const matrix = (holder, radians) => holder.userData.base.clone()
      .multiply(new THREE.Matrix4().makeRotationAxis(holder.userData.axis, radians));
    return [[entry.holder, matrix(entry.holder, angle)], ...entry.followers.map(f =>
      [f.holder, matrix(f.holder, angle * f.ratio + f.offset)])];
  }

  class Recorder {
    constructor(limits = {}) { this.limits = { ...DEFAULT_LIMITS, ...limits }; this.clear(); }
    clear() { this.points = []; this.total = 0; this.active = false; this.reason = ''; }
    start(pose, now) {
      this.clear();
      this.points.push({ angles: copyPose(pose), duration_ms: this.limits.min_duration_ms });
      this.total = this.limits.min_duration_ms;
      this.lastTime = now;
      this.holdStart = 0; // 首点是驻留占位；运动后的驻留必须另建同姿态段。
      this.holdTime = 0;
      this.active = true;
    }
    sample(pose, now, force = false) {
      if (!this.active || (!force && now - this.lastTime < 100)) return false;
      const angles = copyPose(pose), last = this.points[this.points.length - 1];
      if (distance(last.angles, angles) < 1e-7) {
        const elapsed = Math.max(0, Math.ceil(now - this.lastTime));
        if (!elapsed) return false;
        this.lastTime = now;
        this.holdTime += elapsed;
        const limits = this.limits;
        const prefixTotal = this.points.slice(0, this.holdStart).reduce((sum, p) => sum + p.duration_ms, 0);
        const requested = Math.max(limits.min_duration_ms, this.holdTime);
        const duration = Math.min(requested, limits.max_total_ms - prefixTotal,
          (limits.max_points - this.holdStart) * limits.max_duration_ms);
        if (duration < limits.min_duration_ms) {
          this.active = false;
          this.reason = '录制已到上限，最后有效点已保存。';
          return false;
        }
        const holds = [];
        for (let remaining = duration; remaining > 0;) {
          let part = Math.min(limits.max_duration_ms, remaining);
          const rest = remaining - part;
          if (rest > 0 && rest < limits.min_duration_ms) part -= limits.min_duration_ms - rest;
          holds.push({ angles: copyPose(angles), duration_ms: part });
          remaining -= part;
        }
        // 只替换连续停留尾段，不能拉长前一个有角差的运动段。
        this.points.splice(this.holdStart, this.points.length - this.holdStart, ...holds);
        this.total = prefixTotal + duration;
        if (duration < requested || this.total === limits.max_total_ms || this.points.length === limits.max_points) {
          this.active = false;
          this.reason = '录制已到上限，最后有效点已保存。';
        }
        return true;
      }
      const speedTime = Math.ceil(distance(last.angles, angles) * 1000 / this.limits.max_speed_deg_s);
      const duration = Math.max(this.limits.min_duration_ms, speedTime,
        Math.min(this.limits.max_duration_ms, Math.ceil(now - this.lastTime)));
      if (duration > this.limits.max_duration_ms || this.total + duration > this.limits.max_total_ms ||
          this.points.length >= this.limits.max_points) {
        this.active = false;
        this.reason = '已达轨迹容量或时长上限；保留最后有效点，后续姿态未录入。';
        return false;
      }
      this.points.push({ angles, duration_ms: duration });
      this.total += duration;
      this.lastTime = now;
      this.holdStart = this.points.length;
      this.holdTime = 0;
      if (this.points.length === this.limits.max_points || this.total === this.limits.max_total_ms) {
        this.active = false;
        this.reason = '录制已到上限，最后有效点已保存。';
      }
      return true;
    }
    stop(pose, now) { this.sample(pose, now, true); this.active = false; }
  }

  // duration_ms 是到达该点的段时长。预览首点驻留；真机另从回读实位过渡至首点。
  function poseAt(points, elapsed) {
    if (!points.length) return null;
    let previous = points[0].angles;
    for (const point of points) {
      if (elapsed <= point.duration_ms) {
        const t = Math.max(0, elapsed / point.duration_ms);
        return Object.fromEntries(Object.keys(previous).map(id =>
          [id, previous[id] + (point.angles[id] - previous[id]) * t]));
      }
      elapsed -= point.duration_ms;
      previous = point.angles;
    }
    return copyPose(previous);
  }

  function forwardPoint(reference, pose, entries) {
    const matrices = new Map(Object.entries(entries).flatMap(([id, entry]) => jointMatrices(entry, pose[id])));
    const point = new THREE.Vector3();
    for (let node = reference; node; node = node.parent) {
      if (node.matrixAutoUpdate) node.updateMatrix();
      point.applyMatrix4(matrices.get(node) || node.matrix);
    }
    return point;
  }
  function pathPositions(reference, points, entries) {
    const positions = [];
    for (let i = 0; i < points.length; i++) {
      const a = points[Math.max(0, i - 1)].angles, b = points[i].angles;
      const steps = Math.max(1, Math.ceil(distance(a, b) / 3));
      for (let step = 1; step <= steps; step++) {
        const pose = Object.fromEntries(Object.keys(a).map(id => [id, a[id] + (b[id] - a[id]) * step / steps]));
        positions.push(forwardPoint(reference, pose, entries));
      }
    }
    return positions;
  }

  function servoForObject(object, entries) {
    for (let node = object; node; node = node.parent) {
      for (const [id, entry] of Object.entries(entries)) {
        if (entry.holder === node || entry.followers.some(f => f.holder === node)) return Number(id);
      }
    }
    return null;
  }
  function visible(object) {
    for (let node = object; node; node = node.parent) if (!node.visible) return false;
    return true;
  }

  class JointDrag {
    constructor({ scene, camera, canvas, controls, robot, entries, targets, canEdit, enter, change, finish, selected, hint }) {
      Object.assign(this, { scene, camera, canvas, controls, robot, entries, targets, canEdit, enter, change, finish, selected, hint });
      this.radius = .045;
      this.raycaster = new THREE.Raycaster();
      this.ring = new THREE.Mesh(new THREE.TorusGeometry(this.radius, .0025, 12, 96),
        new THREE.MeshBasicMaterial({ color: 0x66ceff, depthTest: false, transparent: true, opacity: .9 }));
      this.ring.name = 'joint_rotation_ring';
      this.ring.renderOrder = 10;
      this.ring.visible = false;
      scene.add(this.ring);
      // 捕获阶段先于 OrbitControls 的 bubble listener，防止同一指针同时转相机。
      canvas.addEventListener('pointerdown', event => this.down(event), true);
      canvas.addEventListener('pointermove', event => this.move(event), true);
      for (const type of ['pointerup', 'pointercancel', 'lostpointercapture']) {
        canvas.addEventListener(type, event => { if (this.drag && event.pointerId === this.drag.id) this.end(); }, true);
      }
    }
    select(id) {
      if (!this.entries[id]) return;
      this.id = Number(id);
      this.ring.visible = true;
      this.update();
      this.selected(this.id);
    }
    frame() {
      const holder = this.entries[this.id].holder;
      holder.parent.updateWorldMatrix(true, false);
      const base = holder.parent.matrixWorld.clone().multiply(holder.userData.base);
      const axis = holder.userData.axis.clone().transformDirection(base);
      const center = new THREE.Vector3().setFromMatrixPosition(base);
      const helper = Math.abs(axis.z) < .9 ? new THREE.Vector3(0, 0, 1) : new THREE.Vector3(0, 1, 0);
      const u = helper.cross(axis).normalize(), v = axis.clone().cross(u).normalize();
      return { center, axis, u, v };
    }
    update() {
      if (!this.id) return;
      const f = this.frame();
      this.ring.position.copy(f.center);
      this.ring.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(f.u, f.v, f.axis));
      this.ring.updateMatrixWorld(true);
    }
    ray(event) {
      const rect = this.canvas.getBoundingClientRect();
      this.camera.updateMatrixWorld(true);
      this.raycaster.setFromCamera(new THREE.Vector2(
        (event.clientX - rect.left) / rect.width * 2 - 1,
        -(event.clientY - rect.top) / rect.height * 2 + 1), this.camera);
      return this.raycaster.ray;
    }
    angle(ray, frame) {
      if (Math.abs(ray.direction.dot(frame.axis)) < .08) return null;
      const point = ray.intersectPlane(new THREE.Plane().setFromNormalAndCoplanarPoint(frame.axis, frame.center), new THREE.Vector3());
      if (!point || point.distanceToSquared(frame.center) < 1e-12) return null;
      point.sub(frame.center);
      const angle = Math.atan2(point.dot(frame.v), point.dot(frame.u));
      return Number.isFinite(angle) ? angle : null;
    }
    down(event) {
      if (event.button !== 0 || this.drag || !this.canEdit()) return;
      this.scene.updateMatrixWorld(true);
      this.update();
      const ray = this.ray(event);
      if (this.ring.visible && this.raycaster.intersectObject(this.ring).length) {
        this.enter();
        event.preventDefault(); event.stopImmediatePropagation();
        const frame = this.frame(), angle = this.angle(ray, frame);
        const scale = this.entries[this.id].joint.servo_scale ?? 1;
        if (angle === null || !Number.isFinite(scale) || Math.abs(scale) < 1e-9) {
          this.hint('旋转面接近侧视，请转动相机后再拖动旋转环。'); return;
        }
        this.drag = { id: event.pointerId, frame, angle, scale, controlsEnabled: this.controls.enabled };
        this.controls.enabled = false;
        try { this.canvas.setPointerCapture(event.pointerId); }
        catch (_) { this.end(); }
        return;
      }
      const hit = this.raycaster.intersectObject(this.robot, true).find(hit => visible(hit.object));
      const id = hit && servoForObject(hit.object, this.entries);
      if (id != null) {
        this.enter(); this.select(id);
        event.preventDefault(); event.stopImmediatePropagation();
      }
    }
    move(event) {
      if (!this.drag || event.pointerId !== this.drag.id) return;
      event.preventDefault(); event.stopImmediatePropagation();
      const angle = this.angle(this.ray(event), this.drag.frame);
      if (angle === null) return;
      const delta = wrap(angle - this.drag.angle) / DEG / this.drag.scale;
      this.drag.angle = angle;
      this.change(this.id, this.targets[this.id] + delta);
      this.update();
    }
    end() {
      if (!this.drag) return;
      const drag = this.drag;
      this.drag = null;
      this.controls.enabled = drag.controlsEnabled;
      if (this.canvas.hasPointerCapture(drag.id)) this.canvas.releasePointerCapture(drag.id);
      this.finish();
    }
  }
  return { DEFAULT_LIMITS, copyPose, jointMatrices, Recorder, poseAt, forwardPoint, pathPositions, servoForObject, JointDrag };
})();
