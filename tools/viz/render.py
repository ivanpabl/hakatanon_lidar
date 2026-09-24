"""Картинка кадра: "выпрямленный" коридор вдоль оси пути (ближняя и дальняя часть),
профиль высоты и вид сверху. Используется разметкой (labeling/make_set.py) и
демонстрацией вставки (tools/inject_demo.py). Решений детектора не показывает --
только ось пути, по которой выпрямлен коридор."""
import numpy as np

from tunnel_od import TrackPath

LAT_VIEW = 3.0         # полуширина выпрямленного коридора на картинке, м
NEAR_VIEW = (2.0, 50.0)
FAR_VIEW = (50.0, 150.0)
H_VIEW = (-0.3, 2.5)   # цветовая шкала: высота над головкой рельса
CORRIDOR_HALF = 1.0    # линии на картинке -- только ориентир, не граница разметки


def render_frame(fig_path, x, y, z, snap: TrackPath, title):
    """Рисует кадр. Возвращает положение кликабельных панелей в пикселях картинки."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fwd = -y
    keep = (fwd > 0) & (fwd < FAR_VIEW[1])
    x, fwd, z = x[keep], fwd[keep], z[keep]
    lat = x - snap.center_at(fwd)
    h = z - snap.rail_top_at(fwd)

    fig = plt.figure(figsize=(18, 11), dpi=80)
    gs = fig.add_gridspec(2, 4, width_ratios=[1, 1, 1.25, 1.25], height_ratios=[1, 1],
                          left=0.04, right=0.985, top=0.93, bottom=0.06, wspace=0.28, hspace=0.25)
    fig.suptitle(title, fontsize=13)
    cmap = 'turbo'
    panels = []
    path_range = snap.path_range or 0.0

    for col, (f0, f1), label, size in ((0, NEAR_VIEW, 'ближняя зона', 2.0), (1, FAR_VIEW, 'дальняя зона', 4.0)):
        ax = fig.add_subplot(gs[:, col])
        s = (fwd >= f0) & (fwd <= f1) & (np.abs(lat) < LAT_VIEW) & (h > H_VIEW[0] - 0.5) & (h < 4.0)
        o = np.argsort(h[s])  # высокие точки поверх низких
        ax.scatter(lat[s][o], fwd[s][o], c=np.clip(h[s][o], *H_VIEW), s=size, cmap=cmap,
                   vmin=H_VIEW[0], vmax=H_VIEW[1], linewidths=0, rasterized=True)
        for v in (-CORRIDOR_HALF, CORRIDOR_HALF):
            ax.axvline(v, color='k', lw=0.8, ls='--')
        for v in (-0.76, 0.76):
            ax.axvline(v, color='gray', lw=0.5, ls=':')
        if path_range < f1:
            ax.axhspan(max(path_range, f0), f1, color='gray', alpha=0.15)
            ax.text(0, max(path_range, f0) + 1, 'ось пути здесь -- догадка', ha='center', fontsize=8, color='dimgray')
        ax.set_xlim(-LAT_VIEW, LAT_VIEW)
        ax.set_ylim(f0, f1)
        ax.set_xlabel('от оси пути вбок, м')
        ax.set_ylabel('вперёд, м')
        ax.set_title(f'Коридор, {label} (кликать сюда)', fontsize=10)
        ax.grid(alpha=0.25)
        panels.append((ax, f0, f1))

    ax = fig.add_subplot(gs[0, 2:])
    s = (fwd > NEAR_VIEW[0]) & (np.abs(lat) < 1.5) & (h > -1.0) & (h < 4.0)
    sc = ax.scatter(fwd[s], h[s], c=np.clip(h[s], *H_VIEW), s=1.5, cmap=cmap, vmin=H_VIEW[0], vmax=H_VIEW[1],
                    linewidths=0, rasterized=True)
    ax.axhline(0, color='k', lw=0.6)
    ax.set_xlim(0, FAR_VIEW[1]); ax.set_ylim(-1.0, 3.5)
    ax.set_xlabel('вперёд, м'); ax.set_ylabel('высота над головкой рельса, м')
    ax.set_title('Профиль: точки не дальше 1,5 м от оси', fontsize=10)
    ax.grid(alpha=0.25)
    fig.colorbar(sc, ax=ax, label='высота над рельсом, м', pad=0.01)

    ax = fig.add_subplot(gs[1, 2:])
    s = (np.abs(x) < 10) & (h > -1.0) & (h < 4.0)
    ax.scatter(fwd[s], x[s], c=np.clip(h[s], *H_VIEW), s=0.8, cmap=cmap, vmin=H_VIEW[0], vmax=H_VIEW[1],
               linewidths=0, rasterized=True)
    ff = np.linspace(2, FAR_VIEW[1], 200)
    cc = snap.center_at(ff)
    ax.plot(ff, cc, 'k-', lw=1)
    ax.plot(ff, cc - CORRIDOR_HALF, 'k--', lw=0.6)
    ax.plot(ff, cc + CORRIDOR_HALF, 'k--', lw=0.6)
    ax.set_xlim(0, FAR_VIEW[1]); ax.set_ylim(-10, 10)
    ax.set_xlabel('вперёд, м'); ax.set_ylabel('вбок, м')
    ax.set_title('Вид сверху как есть: проверить, что ось пути (линия) лежит на пути', fontsize=10)
    ax.grid(alpha=0.25)

    fig.canvas.draw()
    W, H = fig.canvas.get_width_height()
    meta = []
    for ax, f0, f1 in panels:
        bb = ax.get_window_extent()
        meta.append({'x0': bb.x0, 'x1': bb.x1, 'y_top': H - bb.y1, 'y_bottom': H - bb.y0,
                     'lat': [-LAT_VIEW, LAT_VIEW], 'fwd': [f0, f1]})
    fig.savefig(fig_path, dpi=80)
    plt.close(fig)
    return {'w': W, 'h': H, 'panels': meta}
