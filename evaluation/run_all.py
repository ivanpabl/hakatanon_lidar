"""Полный прогон на датасете одной командой: тесты, метрики, дашборд и демонстрация.
Работает на Windows, Linux и macOS без ROS 2 и Docker (на Windows запускается из run_all.bat).

    python -m evaluation all                         # всё: ~20-60 мин, результат в output/report
    python -m evaluation all --quick                 # проверка окружения: пара минут, output/report_quick
    python -m evaluation all --data D:/lidar/data    # данные не в ./data
    python -m evaluation all --skip-full             # без прогона всей new_data (самый долгий шаг)

Результат:
    output/report/index.html     сайт: главная с разделами (site)
    output/report/metrics.html   дашборд метрик (dashboard)
    output/report/demo.html      плеер: детектор по кадрам записей в 3D (demo)
    output/runs/                 csv/json каждого шага, output/runs/logs/<шаг>.log -- их вывод

Шаги метрик идут параллельно, число процессов подбирается по числу ядер (--jobs). Каждый процесс
считает numpy в один поток. Прогоны качества -- текущая версия детектора (config по умолчанию);
исходная версия для сравнения, e2e в Docker и замер C++-разбора без компилятора -- из evaluation/baseline.json.
"""
import argparse
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
TOOLS = ROOT / 'tools'
TAG = 'P_ev_f20'
FOR_HACKATHON = ['doubleT_obstacle', 'doubleT_platform', 'roundT_doubleT', 'roundT_pressureGate_roundT',
                 'roundT_squareT_pressureGate_squareT', 'squareT_platform_squareT_switch']
QUICK = ['doubleT_obstacle', 'roundT_doubleT']
QUICK_APPROACH = 'roundT_pressureGate_roundT'


def data_dir(args):
    return Path(args.data or os.environ.get('TUNNEL_OD_DATA') or ROOT / 'data')


def check_data(data, quick, new_data=True):
    """Какие записи есть: каталог с metadata.yaml."""
    need = QUICK + [QUICK_APPROACH] if quick else FOR_HACKATHON + (['new_data'] if new_data else [])
    found, missing = {}, []
    for b in need:
        for p in (data / b, data / 'Датасет' / 'archive' / 'for_hackathon' / b):
            if (p / 'metadata.yaml').is_file():
                found[b] = p
                break
        else:
            missing.append(b)
    return found, missing


def has_compiler():
    sys.path.insert(0, str(ROOT / 'ros2_ws' / 'src' / 'tunnel_od_preproc'))
    from tunnel_od_preproc.build_host import ensure
    return ensure() is not None


class Step:
    def __init__(self, name, argv, logs):
        self.name, self.argv, self.log = name, argv, logs / f'{name}.log'
        self.proc = self.t0 = self.dt = None

    def start(self, env):
        self.t0 = time.time()
        self.fh = open(self.log, 'w', encoding='utf-8')
        self.proc = subprocess.Popen([sys.executable, *self.argv], cwd=ROOT, env=env, stdout=self.fh,
                                     stderr=subprocess.STDOUT)
        print(f'  запущен  {self.name}', flush=True)

    def poll(self):
        if self.proc is None or self.dt is not None:
            return self.dt is not None
        if self.proc.poll() is None:
            return False
        self.dt = time.time() - self.t0
        self.fh.close()
        ok = 'готово' if self.proc.returncode == 0 else f'ОШИБКА (код {self.proc.returncode}), см. {self.log}'
        print(f'  {self.name}: {ok}, {self.dt / 60:.1f} мин', flush=True)
        return True

    @property
    def ok(self):
        return self.proc is not None and self.proc.returncode == 0


def run_parallel(steps, env):
    for s in steps:
        s.start(env)
    t0 = time.time()
    last = t0
    while not all(s.poll() for s in steps):
        time.sleep(2)
        if time.time() - last > 120:
            last = time.time()
            left = [s.name for s in steps if s.dt is None]
            print(f'  ... {(time.time() - t0) / 60:.0f} мин, идут: {", ".join(left)}', flush=True)
    return all(s.ok for s in steps)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--data', help='каталог данных (по умолчанию ./data или TUNNEL_OD_DATA)')
    ap.add_argument('--out', type=Path, help='куда положить отчёт (по умолчанию output/report, для --quick output/report_quick)')
    ap.add_argument('--runs', type=Path, help='каталог результатов шагов (по умолчанию output/runs, для --quick output/runs/quick)')
    ap.add_argument('--jobs', type=int, default=2,
                    help='сколько процессов детектора одновременно (по умолчанию 2; каждый -- одно ядро, до 1,3 ГБ памяти)')
    ap.add_argument('--quick', action='store_true', help='короткий прогон для проверки окружения')
    ap.add_argument('--skip-full', action='store_true', help='не прогонять всю new_data (~11 тыс. кадров)')
    ap.add_argument('--skip-new-data', action='store_true', help='без new_data совсем (84 ГБ): только 6 записей')
    ap.add_argument('--skip-tests', action='store_true')
    args = ap.parse_args()

    data = data_dir(args)
    runs = (args.runs or ROOT / 'output' / 'runs' / ('quick' if args.quick else '')).resolve()
    args.out = args.out or ROOT / 'output' / ('report_quick' if args.quick else 'report')
    logs = runs / 'logs'
    logs.mkdir(parents=True, exist_ok=True)
    args.out.mkdir(parents=True, exist_ok=True)
    env = {**os.environ, 'TUNNEL_OD_DATA': str(data), 'TUNNEL_OD_RUNS': str(runs), 'PYTHONUTF8': '1',
           'PYTHONIOENCODING': 'utf-8', 'OMP_NUM_THREADS': '1', 'OPENBLAS_NUM_THREADS': '1', 'MKL_NUM_THREADS': '1'}
    T0 = time.time()
    print(f'Python {sys.version.split()[0]}, {os.cpu_count()} потоков, процессов детектора: {args.jobs}')
    print(f'данные: {data}\nрезультаты шагов: {runs}')

    found, missing = check_data(data, args.quick, not args.skip_new_data)
    for b, p in found.items():
        print(f'  запись {b:<38} {p}')
    if missing:
        print(f'\nНЕТ ЗАПИСЕЙ: {", ".join(missing)}\nОжидается data/Датасет/archive/for_hackathon/<запись>/metadata.yaml '
              f'и data/new_data/metadata.yaml (или --data <каталог>; без new_data -- --skip-new-data).', file=sys.stderr)
        sys.exit(2)
    fake_obj = not args.quick and (data / 'cloud_with_fake_obj' / 'metadata.yaml').is_file()
    print(f'запись организаторов cloud_with_fake_obj: {"есть" if fake_obj else "нет -- оценка по 10 объектам пропущена"}')
    compiler = has_compiler()
    print(f'компилятор C++ для библиотеки разбора: {"есть" if compiler else "нет -- проверка входа и замер разбора C++ из baseline.json"}')

    failed = []
    if not args.skip_tests:
        print('\n[1/4] тесты ядра (синтетический тоннель)')
        t = Step('pytest', ['-m', 'pytest', '-q', '-p', 'no:cacheprovider', 'tests'], logs)
        if not run_parallel([t], env):
            failed.append(t.name)

    print('\n[2/4] метрики качества')
    six = QUICK if args.quick else FOR_HACKATHON
    only_six = ['--bags', *six] if args.quick or args.skip_new_data else []
    j = args.jobs
    if args.quick:
        steps = [Step('alarms', ['evaluation/alarms.py', '--tag', TAG, '--bags', *six, '--workers', '2'], logs),
                 Step('approach', ['evaluation/eval_approach.py', '--tag', TAG, '--bags', QUICK_APPROACH, '--workers', '1'], logs)]
    else:
        w_alarms, w_approach = max(1, min(6, (j - 2) // 2)), max(1, j - 2 - max(1, min(6, (j - 2) // 2)))
        steps = [Step('alarms', ['evaluation/alarms.py', '--tag', TAG, '--bags', *six, '--workers', str(w_alarms)], logs),
                 Step('approach', ['evaluation/eval_approach.py', '--tag', TAG, '--workers', str(w_approach)], logs)]
        if not args.skip_new_data:
            steps.append(Step('alarms_nd', ['evaluation/alarms.py', '--tag', f'nd_{TAG}', '--bags', 'new_data', '--segments',
                                            '8', '--seglen', '400', '--workers', '1'], logs))
        if not args.skip_full and not args.skip_new_data:
            steps.append(Step('alarms_nd_full', ['evaluation/alarms.py', '--tag', 'nd_final', '--bags', 'new_data',
                                                 '--workers', '1'], logs))
        if fake_obj:
            steps.append(Step('alarms_fo', ['evaluation/alarms.py', '--tag', 'fo_final', '--bags', 'cloud_with_fake_obj',
                                            '--workers', '1', '--all-objects'], logs))
    if compiler:
        steps.append(Step('input_report', ['evaluation/input_report.py', *only_six], logs))
    if not run_parallel(steps, env):
        failed += [s.name for s in steps if not s.ok]
    after = []
    if fake_obj and 'alarms_fo' not in failed:
        after.append(Step('fake_obj', ['evaluation/eval_fake_obj.py', 'eval', '--tag', 'fo_final'], logs))
    if not args.quick and not args.skip_full and not args.skip_new_data and 'alarms_nd_full' not in failed:
        after.append(Step('selflabel', ['evaluation/selflabel.py', '--tag', 'nd_final'], logs))
    if after and not run_parallel(after, env):
        failed += [s.name for s in after if not s.ok]

    if compiler:
        print('\n[3/4] разбор C++ против Python: побитное совпадение и время')
        s = Step('check_preproc', ['evaluation/check_preproc.py', *only_six], logs)
        if not run_parallel([s], env):
            failed.append(s.name)
    else:
        print('\n[3/4] разбор C++: нет компилятора, пропущено')

    print('\n[4/4] дашборд и демонстрация')
    metrics, demo = args.out / 'metrics.html', args.out / 'demo.html'
    steps = [Step('dashboard', ['evaluation/dashboard.py', '--out', str(metrics)], logs),
             Step('demo', ['evaluation/demo.py', '--out', str(demo), '--workers', '2',
                           *(['--clip', 'doubleT_obstacle:90:30', '--clip', 'roundT_doubleT:0:30',
                              '--approach', f'{QUICK_APPROACH}:32:person:60'] if args.quick else [])],
                  logs)]
    if not run_parallel(steps, env):
        failed += [s.name for s in steps if not s.ok]
    if metrics.exists() and demo.exists():
        s = Step('site', ['evaluation/site.py', '--out', str(args.out)], logs)
        if not run_parallel([s], env):
            failed.append(s.name)

    print(f'\nвсего {(time.time() - T0) / 60:.1f} мин')
    for p in (metrics, demo):
        if p.exists():
            print(f'  {p}  {p.stat().st_size / 1e6:.1f} МБ')
    ref = [line for line in (logs / 'dashboard.log').read_text(encoding='utf-8').splitlines() if 'baseline' in line] \
        if (logs / 'dashboard.log').exists() else []
    for line in ref:
        print(' ', line.strip())
    if failed:
        print(f'\nС ОШИБКАМИ: {", ".join(failed)} -- логи в {logs}', file=sys.stderr)
        sys.exit(1)
    print('\nГотово. Файлы открываются в браузере двойным щелчком, сеть не нужна.')
    if shutil.which('explorer') and sys.platform == 'win32':
        subprocess.run(['explorer', str(args.out)])


if __name__ == '__main__':
    main()
