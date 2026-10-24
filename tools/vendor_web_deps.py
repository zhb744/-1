"""从 GitHub Contents API 拉取 three.js r128 的三个前端文件到 web/vendor/。

本机网络只放通了 api.github.com，jsdelivr / cdnjs / unpkg / raw.githubusercontent 全部被重置，
所以走 API 的 base64 内容接口，把依赖固化进仓库，页面从此不依赖外网 CDN。
"""

import base64
import json
import sys
import urllib.request
from pathlib import Path

REF = "r128"
TARGETS = {
    "three.min.js": "build/three.min.js",
    "STLLoader.js": "examples/js/loaders/STLLoader.js",
    "OrbitControls.js": "examples/js/controls/OrbitControls.js",
}

out_dir = Path(__file__).resolve().parent.parent / "web" / "vendor"
out_dir.mkdir(parents=True, exist_ok=True)

for name, path in TARGETS.items():
    url = f"https://api.github.com/repos/mrdoob/three.js/contents/{path}?ref={REF}"
    request = urllib.request.Request(url, headers={"User-Agent": "armcomm-dev", "Accept": "application/vnd.github+json"})
    with urllib.request.urlopen(request, timeout=60) as response:
        payload = json.load(response)
    data = base64.b64decode(payload["content"].replace("\n", ""))
    target = out_dir / name
    target.write_bytes(data)
    print(f"{name:20} {len(data):>8} 字节  sha={payload['sha'][:12]}")
    if not data:
        sys.exit(f"{name} 为空")
