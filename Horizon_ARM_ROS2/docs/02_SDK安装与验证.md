# 02 SDK 安装与验证（wheel 发行）

- SDK 包：`horizon_control_sdk-<ver>-py3-none-any.whl`
- 推荐放置于：`<ws>/sdk-wheel/dist/`

安装（仅需一次）：
```bash
python3 -m pip install --user <wheel>
```

验证安装与来源（确认未使用本地源码）：
```bash
python3 - <<'PY'
import simple_sdk, inspect
from Control_Core.motor_controller_modular import ZDTMotorControllerModular
print('simple_sdk path =', inspect.getsourcefile(simple_sdk) or simple_sdk.__file__)
print('Control_Core path =', inspect.getsourcefile(ZDTMotorControllerModular) or ZDTMotorControllerModular.__module__)
PY
```
输出路径若位于 `site-packages` 即说明来自已安装的 wheel。

卸载与重装：
```bash
python3 -m pip uninstall -y horizon-control-sdk
python3 -m pip install --user <wheel>
```
