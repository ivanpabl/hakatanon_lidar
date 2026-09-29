// © 2026 команда «Голуби», github.com/ivanpabl/hakatanon_lidar. Все права защищены, условия — в файле LICENSE. GLB-K5-4660c47a8c6c
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace tunnel_od
{

constexpr uint32_t kColumnHeight = 128;
constexpr float kDualReturnDupM = 0.01f;
constexpr double kDedupeCellM = 0.01;

enum PointFieldType : uint8_t
{
  kInt8 = 1, kUint8 = 2, kInt16 = 3, kUint16 = 4, kInt32 = 5, kUint32 = 6, kFloat32 = 7, kFloat64 = 8
};

enum class Format { Auto, LegacyHesai, ContractV1, Generic };
enum class Axes { Auto, Legacy, Rep103 };

const char * to_string(Format f);
const char * to_string(Axes a);
bool parse_format(const std::string & s, Format * out);
bool parse_axes(const std::string & s, Axes * out);

struct Field
{
  std::string name;
  uint32_t offset = 0;
  uint8_t datatype = 0;
  uint32_t count = 1;
};

struct FieldRef
{
  int32_t offset = -1;
  uint8_t type = 0;
  bool present() const {return offset >= 0;}
};

struct Layout
{
  FieldRef x, y, z, intensity, ring, return_id, t_offset_ns, timestamp;
  std::string signature;
};

struct CloudDesc
{
  uint32_t height = 0;
  uint32_t width = 0;
  uint32_t point_step = 0;
  uint32_t row_step = 0;
  bool is_bigendian = false;
  bool is_dense = false;
  size_t data_size = 0;
  size_t n_points() const {return static_cast<size_t>(height) * width;}
};

Layout make_layout(const std::vector<Field> & fields);

std::string validate(const CloudDesc & cloud, const Layout & layout);

Format detect_format(const CloudDesc & cloud, const Layout & layout);

Axes default_axes(Format f);

struct CropParams
{
  bool enabled = false;
  double fwd_min = 2.0;
  double fwd_max = 250.0;
};
constexpr float kFloorGuardNear = 2.0f;
constexpr float kFloorGuardFar = 15.0f;
constexpr float kFloorGuardHalfWidth = 3.0f;
constexpr uint64_t kFloorGuardMin = 50;

struct Rotation
{
  bool enabled = false;
  double m[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  Axes out_axes = Axes::Rep103;
};

struct ParseOptions
{
  Format format = Format::Auto;
  Axes axes = Axes::Auto;
  bool dedupe_dual_return = true;
  CropParams crop;
  Rotation rotation;
  bool collect_stats = true;
};

struct ParseStats
{
  Format format = Format::Generic;
  Axes axes = Axes::Legacy;
  uint64_t n_in = 0;
  uint64_t n_nonfinite = 0;
  uint64_t n_zero = 0;
  uint64_t n_dup = 0;
  uint64_t n_valid = 0;
  uint64_t n_cropped = 0;
  uint64_t n_out = 0;
  bool dedupe_applied = false;
  bool dedupe_rounded = false;
  uint64_t n_dup_rounded = 0;
  bool crop_applied = false;
  bool crop_guard = false;
  uint64_t pairs_both_valid = 0;
  uint64_t pairs_coincident = 0;
  uint64_t n_second = 0;
  uint64_t n_second_valid = 0;
  uint64_t n_second_coincident = 0;
  bool ring_checked = false;
  uint64_t ring_mismatch = 0;
  bool have_point_time = false;
  double t_min = 0.0, t_max = 0.0;
  uint64_t n_az = 0;
  double az_min = 0.0, az_max = 0.0, az_center = 0.0;
};

size_t parse_cloud(
  const uint8_t * data, const CloudDesc & cloud, const Layout & layout,
  const ParseOptions & opt, double stamp_s,
  float * out_x, float * out_y, float * out_z, size_t out_stride,
  ParseStats * stats, std::string * error);

// Дубли облака не из пар столбцов Hesai (синтетика организаторов, generic, contract_v1): одна точка
// на ячейку сетки kDedupeCellM. Точно тот же отбор, что в tunnel_od.pointcloud.dedupe_rounded:
// ключ ячейки q(v) = (int64(rint(double(v) / cell)) + 32768) & 0xFFFF (rint -- к ближайшему чётному,
// как np.round), key = q(x) << 32 | q(y) << 16 | q(z); остаётся первое вхождение ключа, порядок точек --
// как во входе. Узел детектора тогда dedupe_rounded не вызывает (meta parse.dedupe_rounded).
// Нужен ли он кадру -- как tunnel_od_detector.util.unordered_raw(n_in, format == legacy_hesai).
bool needs_dedupe_rounded(Format format, uint64_t n_in);

uint64_t dedupe_cell_key(float x, float y, float z);

class RoundedDeduper
{
public:
  // На месте: x/y/z -- указатели на первую точку, шаг out_stride float'ов (в узле 3). Возвращает
  // число оставшихся точек; они уплотнены в начало. Таблица переиспользуется между кадрами.
  size_t run(float * x, float * y, float * z, size_t stride, size_t n);

private:
  std::vector<uint64_t> table_;
};

}
