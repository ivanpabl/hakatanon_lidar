"""Проверка качества детектора офлайн: без ROS 2 и Docker, на записях из data/ (или --data / TUNNEL_OD_DATA).

    python -m evaluation all [--quick]         всё сразу: метрики, отчёт -> output/report/index.html
    python -m evaluation <команда> --help      параметры команды

Команды:
    all          все шаги подряд и отчёт (run_all.py)
    alarms       ложные тревоги на записях без вставок (alarms.py)
    approach     дальность при подъезде к синтетическому объекту (eval_approach.py)
    injection    доля найденных синтетических объектов по дальностям (eval_injection.py)
    benchmark    время обработки кадра (benchmark.py)
    preproc      C++-разбор облака против Python: совпадение и время (check_preproc.py)
    input        проверка входного потока по контракту (input_report.py)
    contract     перевод записи в формат контракта v1 (to_contract_v1.py)
    dashboard    страница метрик output/report/metrics.html (dashboard.py)
    demo         3D-плеер output/report/demo.html (demo.py)
    site         главная страница отчёта output/report/index.html (site.py)
    inject-demo  картинки кадра до и после вставки объекта (inject_demo.py)
    inject-bag   запись ros2 bag с синтетическим объектом: испытание подъезда для видео (inject_bag.py)
    fake-obj     объекты организаторов в cloud_with_fake_obj: первый СТОП, ложные кадры (eval_fake_obj.py)
    selflabel    эпизоды СТОП на new_data, доказанно ложные проездом (selflabel.py)
    trace        решение по каждому кадру подъезда: где СТОП и почему нет (approach_trace.py)
    compare      строка сводной таблицы вариантов (compare_row.py)

Результаты шагов -- в output/runs (TUNNEL_OD_RUNS). Нужны: pip install -e "core[eval]".
"""
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
COMMANDS = {
    'all': 'run_all', 'alarms': 'alarms', 'approach': 'eval_approach', 'injection': 'eval_injection',
    'benchmark': 'benchmark', 'preproc': 'check_preproc', 'input': 'input_report', 'contract': 'to_contract_v1',
    'dashboard': 'dashboard', 'demo': 'demo', 'site': 'site', 'inject-demo': 'inject_demo',
    'inject-bag': 'inject_bag', 'fake-obj': 'eval_fake_obj', 'selflabel': 'selflabel', 'trace': 'approach_trace',
    'compare': 'compare_row',
}


def main():
    if len(sys.argv) < 2 or sys.argv[1] in ('-h', '--help', 'help'):
        print(__doc__)
        return 0
    cmd = sys.argv[1]
    if cmd not in COMMANDS:
        print(f'неизвестная команда: {cmd}\n{__doc__}', file=sys.stderr)
        return 2
    return subprocess.call([sys.executable, str(HERE / f'{COMMANDS[cmd]}.py'), *sys.argv[2:]])


if __name__ == '__main__':
    sys.exit(main())
