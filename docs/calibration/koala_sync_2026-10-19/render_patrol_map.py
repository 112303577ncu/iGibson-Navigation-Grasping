"""Render the active centre mask and route; no ROS or robot connection."""
import hashlib
import json
import math
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.colors import ListedColormap
from matplotlib.patches import Polygon, Patch
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from integration.g2_patrol_policy import Policy


def main():
    scene = ROOT / 'maps/demo_box_20261018_patrol'
    layout = json.loads((scene / 'scene_layout.json').read_text(encoding='utf-8'))
    route_file = scene / layout['preferred_patrol']['route_file']
    route = np.asarray([r['point'] for r in json.loads(route_file.read_text())])
    policy = Policy(str(scene))
    assert all(policy.segment_allowed(a, b) for a, b in zip(route, route[1:]))
    font = Path('C:/Windows/Fonts/msjh.ttc')
    if font.exists():
        font_manager.fontManager.addfont(str(font))
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=str(font)).get_name()
    plt.rcParams['axes.unicode_minus'] = False
    corner = np.asarray(layout['room_polygon'][0])
    yaw = layout['room_heading_rad']
    rotation = np.asarray([[math.cos(yaw), -math.sin(yaw)],
                           [math.sin(yaw), math.cos(yaw)]])

    def uv(points):
        return (np.asarray(points) - corner) @ rotation

    w, h, res = policy.width, policy.height, policy.resolution
    cols, rows = np.meshgrid(np.arange(w + 1), np.arange(h + 1))
    edges = np.stack([policy.origin[0] + cols * res,
                      policy.origin[1] + (h - rows) * res], axis=-1)
    edges = uv(edges)
    cells = np.asarray(policy.cells).reshape(h, w)
    raw = np.asarray(Image.open(scene / 'demo_box.pgm'))
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_facecolor('#fff5f3')
    ax.pcolormesh(edges[:, :, 0], edges[:, :, 1], cells > 250,
                  cmap=ListedColormap(['#f6c7c2', '#e9f2f5']), shading='flat', rasterized=True)
    # These are actual occupied samples, separate from the ideal fitted walls.
    rr, cc = np.where(raw < 80)
    points = uv(np.column_stack([policy.origin[0] + (cc + .5) * res,
                                policy.origin[1] + (h - rr - .5) * res]))
    ax.scatter(points[:, 0], points[:, 1], s=3, color='#59616b', alpha=.55)
    room = uv(layout['room_polygon'])
    ax.add_patch(Polygon(room, closed=True, fill=False, ec='#222', lw=2.3))
    allowed = uv(layout['center_allowed_polygon'])
    ax.add_patch(Polygon(allowed, closed=True, fill=False, ec='#c05751', lw=1.3, ls='--'))
    line = uv(route)
    ax.plot(line[:, 0], line[:, 1], '-o', color='#12835e', lw=3, ms=7)
    for i, (a, b) in enumerate(zip(line, line[1:])):
        start = a + .38 * (b - a)
        end = a + .61 * (b - a)
        ax.annotate('', xy=end, xytext=start,
                    arrowprops=dict(arrowstyle='-|>', color='#12835e', lw=2, mutation_scale=17))
        ax.annotate('P%d' % i, a, xytext=(-20, -18), textcoords='offset points',
                    fontsize=12, color='#07563d', weight='bold')
    prep = uv(layout['bin']['navigation_preparation']['point_map'])
    ax.scatter(*prep, s=190, marker='*', color='#2465b8', zorder=6)
    ax.annotate('原投放準備點\n需另驗證相機交接距離', prep, xytext=(-125, -40),
                textcoords='offset points', fontsize=10, color='#164777',
                arrowprops=dict(arrowstyle='-', color='#2465b8'))
    box = layout['bin']['placement_update']['midpoint_pose_map_candidate']
    bx, by = uv([box['x'], box['y']])
    ax.add_patch(Polygon([[bx-.0675, by-.09], [bx+.0675, by-.09],
                          [bx+.0675, by+.09], [bx-.0675, by+.09]],
                         ec='#b46b08', fc='#ffdb92', ls='--', lw=2, zorder=5))
    ax.text(bx-.02, by+.14, '紙盒估計位置\n未精測登錄', ha='center', fontsize=11, color='#8a5005')
    ax.annotate('', xy=(.4, .17), xytext=(0, .17),
                arrowprops=dict(arrowstyle='<->', color='#ac3e35', lw=1.5))
    ax.text(.2, .21, '40 cm', ha='center', color='#ac3e35', fontsize=11)
    ax.text(.99, .69, '主要巡航線\n車體中心離牆約 55 cm',
            ha='center', fontsize=13, color='#07563d')
    ax.set(xlim=(-.08, 2.10), ylim=(-.08, 2.14), aspect='equal',
           xlabel='沿牆距離（m）', ylabel='沿牆距離（m）',
           title='目前巡航路徑與禁航區｜2026-10-19\n約 2 × 2 m 場地・逆時鐘巡航')
    ax.set_xticks(np.arange(0, 2.01, .25)); ax.set_yticks(np.arange(0, 2.01, .25))
    ax.grid(alpha=.2)
    ax.legend(handles=[Patch(fc='#e9f2f5', label='允許車體中心通行'),
                       Patch(fc='#f6c7c2', label='中心禁航區：牆邊／盒角／未知格'),
                       Patch(fc='#ffdb92', ec='#b46b08', label='紙盒候選位置')],
              loc='upper left', fontsize=10)
    fig.text(.5, .025, '禁航距離以車體中心計算；圖中不代表即時位置。\n'
             '禁航遮罩目前由地圖導航執行，視覺接近與直接 TCP 命令未共用此全域遮罩。',
             ha='center', fontsize=10, color='#5d6269')
    fig.tight_layout(rect=(0, .065, 1, 1))
    output = ROOT / 'output/png/patrol_keepout_latest_2026-10-19.png'
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=170)
    metadata = dict(scene=str(scene.relative_to(ROOT)), route=str(route_file.relative_to(ROOT)),
                    route_sha256=hashlib.sha256(route_file.read_bytes()).hexdigest(),
                    mask_sha256=hashlib.sha256((scene/'navigation_center_mask.pgm').read_bytes()).hexdigest(),
                    wall_margin_m=.4, patrol_inset_m=.55, route_segments_valid=True,
                    box_registration_verified=False, hardware_changed=False)
    output.with_suffix('.json').write_text(json.dumps(metadata, indent=2)+'\n', encoding='utf-8')
    print(output)


if __name__ == '__main__':
    main()
