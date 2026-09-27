"""Сайт проекта: главная страница с разделами + дашборд метрик + 3D-плеер, статические файлы без сервера.

    python tools/site.py                                  # -> site/index.html, metrics.html, demo.html
    python tools/site.py --metrics gui/dashboard.html     # метрики из другой сборки дашборда
    python tools/site.py --video media/rviz.mp4           # видео прогона в разделе «Демонстрация»

Цифры на главной берутся из данных, встроенных в страницу метрик (tools/dashboard.py), а отрывки --
из 3D-плеера (tools/demo.py): главная всегда согласована с тем, что лежит рядом. По умолчанию метрики --
results/metrics.html, если это полный прогон (есть вся new_data), иначе gui/dashboard.html из git.
Папка site/ открывается двойным щелчком по index.html или выкладывается как есть (GitHub Pages, Netlify).
"""
import argparse
import json
import shutil
import sys
from datetime import date
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from bags import ROOT                     # noqa: E402
from dashboard import git_rev             # noqa: E402

TEMPLATE = ROOT / 'gui' / 'site_template.html'
RESULTS = ROOT / 'results'
MARK = 'const DATA = '


def page_data(path):
    text = path.read_text(encoding='utf-8')
    i = text.index(MARK) + len(MARK)
    data, _ = json.JSONDecoder().raw_decode(text[i:])
    return data


def is_full(metrics):
    return bool(metrics.get('quality', {}).get('nd_full'))


def pick_metrics(arg):
    if arg:
        return Path(arg)
    fresh = RESULTS / 'metrics.html'
    if fresh.exists() and is_full(page_data(fresh)):
        return fresh
    return ROOT / 'gui' / 'dashboard.html'


def pick_demo(arg):
    if arg:
        return Path(arg)
    for p in (RESULTS / 'demo.html', ROOT / 'gui' / 'demo.html'):
        if p.exists():
            return p
    return None


def quality_summary(q):
    """Только то, что показывает главная: без покадровых рядов (они есть на странице метрик)."""
    out = {k: q[k] for k in ('obstacle', 'empty', 'nd_segments', 'approach') if k in q}
    if 'nd_full' in q:
        out['nd_full'] = {k: v for k, v in q['nd_full'].items() if k not in ('per_min', 'per_100')}
    if 'dist_err' in q:
        out['dist_err'] = {k: v for k, v in q['dist_err'].items() if k != 'values'}
    if 'empty' in out:
        out['empty'] = {k: v for k, v in out['empty'].items() if k != 'bags'}
    return out


def clip_summary(demo):
    out = []
    for c in demo.get('clips', []):
        f = c['frames']
        dist = sorted(x['distance_m'] for x in f if x['obstacle'] and x['distance_m'] is not None)
        out.append({'bag': c['bag'], 'start': c['start'], 'seconds': f[-1]['t'] if f else 0,
                    'alarm': [1 if x['obstacle'] else 0 for x in f],
                    'dist': dist[len(dist) // 2] if dist else None})
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--metrics', help='страница метрик (tools/dashboard.py)')
    ap.add_argument('--demo', help='3D-плеер (tools/demo.py)')
    ap.add_argument('--video', help='видео прогона для раздела «Демонстрация» (mp4/webm)')
    ap.add_argument('--out', type=Path, default=ROOT / 'site')
    args = ap.parse_args()

    metrics_path, demo_path = pick_metrics(args.metrics), pick_demo(args.demo)
    metrics = page_data(metrics_path)
    if not is_full(metrics):
        print(f'внимание: {metrics_path} -- не полный прогон (нет всей new_data), часть цифр будет пустой')
    args.out.mkdir(parents=True, exist_ok=True)

    data = {'generated': date.today().isoformat(), 'commit': git_rev(),
            'metrics_generated': metrics.get('generated'),
            'metrics_src': f'{metrics_path.relative_to(ROOT) if metrics_path.is_relative_to(ROOT) else metrics_path}'
                           f', commit {metrics.get("commit", "?")}',
            'quality': quality_summary(metrics.get('quality', {})),
            'e2e': {k: v for k, v in metrics.get('speed', {}).get('e2e', {}).items()
                    if k in ('e2e_ms_p50', 'e2e_ms_p95', 'latency_ms_p50', 'latency_ms_p95', 'processed', 'received')},
            'clips': [], 'video': None}

    shutil.copyfile(metrics_path, args.out / 'metrics.html')
    if demo_path:
        shutil.copyfile(demo_path, args.out / 'demo.html')
        data['clips'] = clip_summary(page_data(demo_path))
    else:
        print('3D-плеера нет: python tools/demo.py --out results/demo.html')
    if args.video:
        v = Path(args.video)
        (args.out / 'media').mkdir(exist_ok=True)
        shutil.copyfile(v, args.out / 'media' / v.name)
        data['video'] = f'media/{v.name}'

    html = TEMPLATE.read_text(encoding='utf-8').replace(
        '/*__DATA__*/null', json.dumps(data, ensure_ascii=False, separators=(',', ':')))
    (args.out / 'index.html').write_text(html, encoding='utf-8')

    print(f'метрики: {metrics_path} ({metrics.get("generated")}, commit {metrics.get("commit")})')
    print(f'плеер:   {demo_path}')
    for p in sorted(args.out.rglob('*')):
        if p.is_file():
            print(f'  {p.relative_to(ROOT) if p.is_relative_to(ROOT) else p}  {p.stat().st_size / 1e6:.1f} МБ')


if __name__ == '__main__':
    main()
