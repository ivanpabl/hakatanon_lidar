#include "tunnel_od_preproc/canonical.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <sstream>

namespace tunnel_od
{

namespace
{

inline float load_f32(const uint8_t * p)
{
  float v;
  std::memcpy(&v, p, sizeof(v));
  return v;
}

inline double load_f64(const uint8_t * p)
{
  double v;
  std::memcpy(&v, p, sizeof(v));
  return v;
}

inline double load_u16(const uint8_t * p)
{
  uint16_t v;
  std::memcpy(&v, p, sizeof(v));
  return v;
}

inline double load_u32(const uint8_t * p)
{
  uint32_t v;
  std::memcpy(&v, p, sizeof(v));
  return v;
}

double load_number(const uint8_t * p, uint8_t type)
{
  switch (type) {
    case kInt8: return static_cast<int8_t>(*p);
    case kUint8: return *p;
    case kInt16: {int16_t v; std::memcpy(&v, p, 2); return v;}
    case kUint16: {uint16_t v; std::memcpy(&v, p, 2); return v;}
    case kInt32: {int32_t v; std::memcpy(&v, p, 4); return v;}
    case kUint32: {uint32_t v; std::memcpy(&v, p, 4); return v;}
    case kFloat32: return load_f32(p);
    case kFloat64: {double v; std::memcpy(&v, p, 8); return v;}
    default: return std::numeric_limits<double>::quiet_NaN();
  }
}

uint32_t type_size(uint8_t type)
{
  switch (type) {
    case kInt8: case kUint8: return 1;
    case kInt16: case kUint16: return 2;
    case kInt32: case kUint32: case kFloat32: return 4;
    case kFloat64: return 8;
    default: return 0;
  }
}

// Граница обрезки в float так, чтобы условие в float было не уже условия в double.
float round_down(double v)
{
  float f = static_cast<float>(v);
  if (static_cast<double>(f) > v) {f = std::nextafter(f, -std::numeric_limits<float>::infinity());}
  return f;
}

float round_up(double v)
{
  float f = static_cast<float>(v);
  if (static_cast<double>(f) < v) {f = std::nextafter(f, std::numeric_limits<float>::infinity());}
  return f;
}

constexpr double kRadToDeg = 57.29577951308232;
constexpr uint32_t kAzSampleEvery = 127;    // азимут -- по каждой 127-й точке (127 и 128 взаимно просты)

}  // namespace

const char * to_string(Format f)
{
  switch (f) {
    case Format::Auto: return "auto";
    case Format::LegacyHesai: return "legacy_hesai";
    case Format::ContractV1: return "contract_v1";
    case Format::Generic: return "generic";
  }
  return "?";
}

const char * to_string(Axes a)
{
  switch (a) {
    case Axes::Auto: return "auto";
    case Axes::Legacy: return "legacy";
    case Axes::Rep103: return "rep103";
  }
  return "?";
}

bool parse_format(const std::string & s, Format * out)
{
  for (Format f : {Format::Auto, Format::LegacyHesai, Format::ContractV1, Format::Generic}) {
    if (s == to_string(f)) {*out = f; return true;}
  }
  return false;
}

bool parse_axes(const std::string & s, Axes * out)
{
  for (Axes a : {Axes::Auto, Axes::Legacy, Axes::Rep103}) {
    if (s == to_string(a)) {*out = a; return true;}
  }
  return false;
}

Layout make_layout(const std::vector<Field> & fields)
{
  Layout l;
  std::ostringstream sig;
  for (const auto & f : fields) {
    sig << f.name << ':' << f.offset << ':' << static_cast<int>(f.datatype) << ',';
    FieldRef ref{static_cast<int32_t>(f.offset), f.datatype};
    if (f.name == "x") {l.x = ref;}
    else if (f.name == "y") {l.y = ref;}
    else if (f.name == "z") {l.z = ref;}
    else if (f.name == "intensity") {l.intensity = ref;}
    else if (f.name == "ring") {l.ring = ref;}
    else if (f.name == "return_id") {l.return_id = ref;}
    else if (f.name == "t_offset_ns") {l.t_offset_ns = ref;}
    else if (f.name == "timestamp") {l.timestamp = ref;}
  }
  l.signature = sig.str();
  return l;
}

std::string validate(const CloudDesc & c, const Layout & l)
{
  if (!l.x.present() || !l.y.present() || !l.z.present()) {return "нет полей x, y, z";}
  if (l.x.type != kFloat32 || l.y.type != kFloat32 || l.z.type != kFloat32) {
    return "x, y, z должны быть float32";
  }
  if (c.is_bigendian) {return "is_bigendian = true не поддерживается";}
  if (c.point_step == 0) {return "point_step = 0";}
  for (const FieldRef * f : {&l.x, &l.y, &l.z, &l.intensity, &l.ring, &l.return_id, &l.t_offset_ns,
      &l.timestamp})
  {
    if (f->present() && (type_size(f->type) == 0 ||
      static_cast<uint64_t>(f->offset) + type_size(f->type) > c.point_step))
    {
      return "поле выходит за point_step или неизвестного типа";
    }
  }
  if (c.n_points() == 0) {return "";}
  if (c.row_step < static_cast<uint64_t>(c.width) * c.point_step) {return "row_step < width * point_step";}
  const uint64_t need = static_cast<uint64_t>(c.height - 1) * c.row_step +
    static_cast<uint64_t>(c.width) * c.point_step;
  if (c.data_size < need) {return "data короче height * row_step";}
  return "";
}

Format detect_format(const CloudDesc & c, const Layout & l)
{
  if (c.height == 1 && l.timestamp.present() && l.timestamp.type == kFloat64 &&
    !l.return_id.present() && c.width % (2 * kColumnHeight) == 0)
  {
    return Format::LegacyHesai;
  }
  if (c.height == kColumnHeight || l.return_id.present() || l.t_offset_ns.present()) {
    return Format::ContractV1;
  }
  return Format::Generic;
}

Axes default_axes(Format f)
{
  return f == Format::ContractV1 ? Axes::Rep103 : Axes::Legacy;
}

namespace
{

struct Ctx
{
  const uint8_t * data;
  const CloudDesc * c;
  const Layout * l;
  const ParseOptions * opt;
  Format format;
  Axes axes;
  bool organized;      // contract_v1 с height == 128: строки = ring, обход по столбцам
  bool dedupe;         // legacy: дубли по соседнему столбцу пары
  double stamp_s;
  float crop_lo, crop_hi;
};

struct Acc
{
  uint64_t n_out = 0, guard = 0, valid = 0;
  double t_min = std::numeric_limits<double>::infinity(), t_max = -std::numeric_limits<double>::infinity();
  double az_min = 1e9, az_max = -1e9, az360_min = 1e9, az360_max = -1e9, sum_c = 0, sum_s = 0;
  uint64_t n_az = 0;
};

// Один проход по кадру без ветвлений на каждую точку (пустых лучей 40-60%, и ветвление
// "есть отражение" предсказывается плохо). Выход пишется всегда, а счётчик растёт только
// для оставленных точек -- поэтому ёмкость выхода не меньше числа точек входа.
// kStats -- собирать статистику для проверки потока; kCrop -- резать по дальности.
template<bool kStats, bool kCrop>
void run(const Ctx & x, ParseStats * st, Acc & a, float * ox, float * oy, float * oz, size_t stride)
{
  const CloudDesc & c = *x.c;
  const Layout & l = *x.l;
  const uint32_t H = c.height, W = c.width, ps = c.point_step;
  const size_t rs = c.row_step;
  const size_t n = c.n_points();
  const int32_t xo = l.x.offset, yo = l.y.offset, zo = l.z.offset;
  const bool have_rid = l.return_id.present();
  const bool have_ring = l.ring.present();
  const bool use_toff = l.t_offset_ns.present();
  const bool use_ts = !use_toff && l.timestamp.present();
  const bool ring_u16 = l.ring.type == kUint16;
  const bool toff_u32 = l.t_offset_ns.type == kUint32;
  const bool ts_f64 = l.timestamp.type == kFloat64;
  const size_t pair_back = static_cast<size_t>(kColumnHeight) * ps;
  const Rotation & rot = x.opt->rotation;
  const Axes axes = rot.enabled ? rot.out_axes : x.axes;
  const bool rep = axes == Axes::Rep103;
  const float lo = x.crop_lo, hi = x.crop_hi;
  
  // счётчики -- в локальных переменных: через указатели компилятор не держит их в регистрах
  uint64_t c_n_nonfinite = 0, c_n_zero = 0, c_pairs_both_valid = 0, c_pairs_coincident = 0, c_n_dup = 0, c_ring_mismatch = 0, c_n_second = 0, c_n_second_valid = 0, c_n_second_coincident = 0, c_n_cropped = 0;
  uint64_t l_n_out = 0, l_guard = 0, l_valid = 0, l_n_az = 0;
  double l_t_min = a.t_min, l_t_max = a.t_max, l_az_min = a.az_min, l_az_max = a.az_max;
  double l_az360_min = a.az360_min, l_az360_max = a.az360_max, l_sum_c = 0, l_sum_s = 0;
  // contract: первое отражение предыдущего столбца той же строки (проверка M2)
  float prev_col[kColumnHeight][3] = {};
  bool prev_ok[kColumnHeight] = {false};
  bool first_valid[kColumnHeight] = {false};    // legacy: есть ли отражение в первом столбце пары
  uint32_t az_countdown = 1;

  // Кадр -- последовательность отрезков с постоянным шагом: столбцы по 128 точек
  // (legacy и generic с height == 1; contract_v1 -- столбец матрицы 128 x W с шагом
  // row_step) или строки облака (прочие height > 1).
  size_t n_seg, seg_len;
  if (x.organized) {n_seg = W; seg_len = kColumnHeight;}
  else if (H == 1) {n_seg = (n + kColumnHeight - 1) / kColumnHeight; seg_len = kColumnHeight;}
  else {n_seg = H; seg_len = W;}

  size_t k = 0;      // номер точки в порядке обхода
  for (size_t sgi = 0; sgi < n_seg; ++sgi) {
    const uint8_t * p;
    size_t step, len;
    if (x.organized) {p = x.data + sgi * ps; step = rs; len = seg_len;}
    else if (H == 1) {p = x.data + sgi * kColumnHeight * ps; step = ps; len = std::min(seg_len, n - k);}
    else {p = x.data + sgi * rs; step = ps; len = W;}
    const bool dedupe_seg = x.dedupe && (sgi & 1u);

    for (size_t j = 0; j < len; ++j, ++k, p += step) {
      float px = load_f32(p + xo), py = load_f32(p + yo), pz = load_f32(p + zo);
      const bool finite = (std::isfinite(px) + std::isfinite(py) + std::isfinite(pz)) == 3;
      const bool zero = (px == 0.0f) & (py == 0.0f) & (pz == 0.0f);
      bool valid = finite & !zero;
      if (kStats) {
        c_n_nonfinite += !finite;
        c_n_zero += finite & zero;
      }
      if (dedupe_seg) {
        // parse_pointcloud2: |x1-x0|, |y1-y0|, |z1-z0| < 0.01 в float32, по сырым значениям
        const uint8_t * q = p - pair_back;
        const float qx = load_f32(q + xo), qy = load_f32(q + yo), qz = load_f32(q + zo);
        const bool dup = (std::fabs(px - qx) < kDualReturnDupM) & (std::fabs(py - qy) < kDualReturnDupM) &
          (std::fabs(pz - qz) < kDualReturnDupM);
        if (kStats) {
          const bool both = valid & first_valid[j];
          c_pairs_both_valid += both;
          c_pairs_coincident += both & dup;
          c_n_dup += valid & dup;
        }
        valid = valid & !dup;
      } else if (kStats && x.dedupe) {
        first_valid[j] = valid;
      }

      if (kStats) {
        if (have_rid) {
          const bool second = load_number(p + l.return_id.offset, l.return_id.type) != 0.0;
          c_n_second += second;
          c_n_second_valid += second & valid;
          if (x.organized) {
            if (second) {
              c_n_second_coincident += valid & prev_ok[j] &
                (std::fabs(px - prev_col[j][0]) < kDualReturnDupM) &
                (std::fabs(py - prev_col[j][1]) < kDualReturnDupM) &
                (std::fabs(pz - prev_col[j][2]) < kDualReturnDupM);
            } else {
              prev_ok[j] = valid;
              prev_col[j][0] = px; prev_col[j][1] = py; prev_col[j][2] = pz;
            }
          }
        }
      }

      // оси: вход -> (поворот) -> оси ядра
      if (rot.enabled) {
        const double u = px, v = py, w = pz;
        px = static_cast<float>(rot.m[0] * u + rot.m[1] * v + rot.m[2] * w);
        py = static_cast<float>(rot.m[3] * u + rot.m[4] * v + rot.m[5] * w);
        pz = static_cast<float>(rot.m[6] * u + rot.m[7] * v + rot.m[8] * w);
      }
      const float cx = rep ? py : px;         // REP-103: влево y -> x ядра
      const float cy = rep ? -px : py;        //          вперёд x -> -y ядра
      const float fwd = -cy;

      l_guard += valid & (fwd > kFloorGuardNear) & (fwd < kFloorGuardFar) &
        (std::fabs(cx) < kFloorGuardHalfWidth);
      bool keep = valid;
      if (kCrop) {keep = keep & (fwd >= lo) & (fwd <= hi);}
      ox[l_n_out * stride] = cx;
      oy[l_n_out * stride] = cy;
      oz[l_n_out * stride] = pz;
      l_n_out += keep;
      l_valid += valid;

      if (kStats) {
        c_n_cropped += valid & !keep;
        // Остальное -- по выборке точек: порядок каналов, время точек, азимут (сектор и
        // его середина для M5 и проверки осей). Полностью по каждой точке это ещё +5 мс
        // на кадре 23 МБ, а для проверки контракта выборки хватает (~1 точка на столбец).
        if (--az_countdown == 0) {
          az_countdown = kAzSampleEvery;
          if (have_ring) {
            const size_t expect = x.organized ? j : (k % kColumnHeight);
            const double r = ring_u16 ? load_u16(p + l.ring.offset) : load_number(p + l.ring.offset, l.ring.type);
            c_ring_mismatch += r != static_cast<double>(expect);
          }
          if (valid) {
            if (use_toff | use_ts) {
              double t;
              if (use_toff) {
                t = (toff_u32 ? load_u32(p + l.t_offset_ns.offset) :
                  load_number(p + l.t_offset_ns.offset, l.t_offset_ns.type)) * 1e-9;
              } else {
                t = (ts_f64 ? load_f64(p + l.timestamp.offset) :
                  load_number(p + l.timestamp.offset, l.timestamp.type)) - x.stamp_s;
              }
              l_t_min = std::min(l_t_min, t);
              l_t_max = std::max(l_t_max, t);
            }

            const double h = std::hypot(static_cast<double>(cx), static_cast<double>(fwd));
            if (h > 0) {
              const double az = std::atan2(static_cast<double>(cx), static_cast<double>(fwd)) * kRadToDeg;
              const double az360 = az < 0 ? az + 360.0 : az;
              l_az_min = std::min(l_az_min, az); l_az_max = std::max(l_az_max, az);
              l_az360_min = std::min(l_az360_min, az360); l_az360_max = std::max(l_az360_max, az360);
              l_sum_c += fwd / h; l_sum_s += cx / h;
              ++l_n_az;
            }
          }
        }
      }
    }
  }
  if (kStats) {
    st->n_nonfinite += c_n_nonfinite;
    st->n_zero += c_n_zero;
    st->pairs_both_valid += c_pairs_both_valid;
    st->pairs_coincident += c_pairs_coincident;
    st->n_dup += c_n_dup;
    st->ring_mismatch += c_ring_mismatch;
    st->n_second += c_n_second;
    st->n_second_valid += c_n_second_valid;
    st->n_second_coincident += c_n_second_coincident;
    st->n_cropped += c_n_cropped;
  }
  a.n_out = l_n_out; a.guard = l_guard; a.valid = l_valid; a.n_az = l_n_az;
  a.t_min = l_t_min; a.t_max = l_t_max; a.az_min = l_az_min; a.az_max = l_az_max;
  a.az360_min = l_az360_min; a.az360_max = l_az360_max; a.sum_c = l_sum_c; a.sum_s = l_sum_s;
}

size_t run_any(const Ctx & x, bool crop, ParseStats * st, uint64_t * guard_count,
  float * ox, float * oy, float * oz, size_t stride)
{
  Acc a;
  if (st) {
    if (crop) {run<true, true>(x, st, a, ox, oy, oz, stride);} else {run<true, false>(x, st, a, ox, oy, oz, stride);}
    st->n_valid = a.valid;
    if (a.t_min <= a.t_max) {st->have_point_time = true; st->t_min = a.t_min; st->t_max = a.t_max;}
    st->n_az = a.n_az;
    if (a.n_az) {
      if (a.az360_max - a.az360_min < a.az_max - a.az_min) {
        st->az_min = a.az360_min > 180.0 ? a.az360_min - 360.0 : a.az360_min;
        st->az_max = st->az_min + (a.az360_max - a.az360_min);
      } else {
        st->az_min = a.az_min; st->az_max = a.az_max;
      }
      st->az_center = std::atan2(a.sum_s, a.sum_c) * kRadToDeg;
    }
  } else {
    if (crop) {run<false, true>(x, st, a, ox, oy, oz, stride);} else {run<false, false>(x, st, a, ox, oy, oz, stride);}
  }
  *guard_count = a.guard;
  return a.n_out;
}

}  // namespace

size_t parse_cloud(
  const uint8_t * data, const CloudDesc & cloud, const Layout & layout,
  const ParseOptions & opt, double stamp_s,
  float * out_x, float * out_y, float * out_z, size_t out_stride,
  ParseStats * stats, std::string * error)
{
  const std::string err = validate(cloud, layout);
  if (!err.empty()) {
    if (error) {*error = err;}
    return 0;
  }
  Format fmt = opt.format == Format::Auto ? detect_format(cloud, layout) : opt.format;
  Axes axes = opt.axes == Axes::Auto ? default_axes(fmt) : opt.axes;

  Ctx x;
  x.data = data;
  x.c = &cloud;
  x.l = &layout;
  x.opt = &opt;
  x.format = fmt;
  x.axes = axes;
  x.stamp_s = stamp_s;
  x.organized = fmt == Format::ContractV1 && cloud.height == kColumnHeight;
  // как в parse_pointcloud2: только если облако -- целое число пар столбцов по 128
  const size_t n = cloud.n_points();
  const bool contiguous = cloud.height == 1 ||
    cloud.row_step == static_cast<size_t>(cloud.width) * cloud.point_step;
  x.dedupe = fmt == Format::LegacyHesai && opt.dedupe_dual_return && n % (2 * kColumnHeight) == 0 &&
    contiguous;
  x.crop_lo = round_down(opt.crop.fwd_min);
  x.crop_hi = round_up(opt.crop.fwd_max);

  ParseStats local;
  ParseStats * st = opt.collect_stats ? &local : nullptr;
  uint64_t guard = 0;
  size_t n_out = run_any(x, opt.crop.enabled, st, &guard, out_x, out_y, out_z, out_stride);
  bool guard_hit = false;
  if (opt.crop.enabled && guard < kFloorGuardMin) {
    // мало точек пола: ядро возьмёт перцентиль по всем точкам -- не режем
    guard_hit = true;
    if (st) {local = ParseStats();}
    n_out = run_any(x, false, st, &guard, out_x, out_y, out_z, out_stride);
  }
  if (stats) {
    *stats = local;
    stats->format = fmt;
    stats->axes = opt.rotation.enabled ? opt.rotation.out_axes : axes;
    stats->n_in = n;
    stats->n_out = n_out;
    stats->dedupe_applied = x.dedupe;
    stats->crop_applied = opt.crop.enabled && !guard_hit;
    stats->crop_guard = guard_hit;
    stats->ring_checked = layout.ring.present();
    if (!opt.collect_stats) {stats->n_valid = 0;}
  }
  return n_out;
}

}  // namespace tunnel_od
