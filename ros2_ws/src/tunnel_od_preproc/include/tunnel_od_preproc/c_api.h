// © 2026 команда «Голуби», github.com/ivanpabl/hakatanon_lidar. Все права защищены, условия — в файле LICENSE. GLB-K5-4660c47a8c6c

#ifndef TUNNEL_OD_PREPROC__C_API_H_
#define TUNNEL_OD_PREPROC__C_API_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TOD_API_VERSION 1

typedef struct tod_field
{
  const char * name;
  uint32_t offset;
  uint8_t datatype;
  uint32_t count;
} tod_field;

typedef struct tod_cloud
{
  uint32_t height, width, point_step, row_step;
  int32_t is_bigendian, is_dense;
  int64_t stamp_ns;
  const char * frame_id;
} tod_cloud;

typedef struct tod_options
{
  int32_t format, axes, dedupe, crop_enabled, collect_stats;
  double crop_min, crop_max;
} tod_options;

int32_t tod_api_version(void);

int64_t tod_parse(
  const uint8_t * data, size_t data_size, const tod_cloud * cloud,
  const tod_field * fields, size_t n_fields, const tod_options * opt,
  float * out_x, float * out_y, float * out_z, size_t out_stride,
  void * monitor, double recv_s, char * stats_json, size_t stats_len, char * err, size_t err_len);

void * tod_monitor_new(int32_t speed_expected, int32_t description_expected);
void tod_monitor_free(void * monitor);
void tod_monitor_tf(void * monitor, const char * parent, const char * child, const double * xyz,
  const double * rpy_deg);
void tod_monitor_speed(void * monitor, double stamp_s, double speed_mps);
void tod_monitor_description(void * monitor, const char * text);

size_t tod_monitor_summary(void * monitor, char * buf, size_t len);
size_t tod_monitor_new_warnings(void * monitor, char * buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif
