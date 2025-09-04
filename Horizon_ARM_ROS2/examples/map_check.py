#!/usr/bin/env python3
# 读取 mapping.yaml 并校验关键字段

import yaml
from pathlib import Path


def main():
    ws = Path(__file__).resolve().parents[1]
    mapping = ws / 'arm-ros2' / 'mapping.yaml'
    data = yaml.safe_load(mapping.read_text(encoding='utf-8'))
    keys = ['send_space', 'pos_limit_deg', 'vel_limit_deg_s', 'servo_rate_hz']
    print('mapping file =', mapping)
    for k in keys:
        v = data.get(k)
        print(f'- {k}:', v)
    ss = str(data.get('send_space', '')).strip().lower()
    if ss not in ('output', 'motor'):
        print('! WARN: send_space 应为 "output" 或 "motor"，当前 =', ss)


if __name__ == '__main__':
    main()
