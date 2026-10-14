/* Minimal stand-in for libfprint/drivers_api.h for the macOS port. */
#pragma once
#include <glib.h>
typedef void FpDevice;
#define G_DECLARE_FINAL_TYPE(T, t, P, N, Parent) typedef struct _##T T;
#define G_DEFINE_TYPE(T, t, Parent)
#define FPI_DEVICE_GOODIX51A0(d) ((FpiDeviceGoodix51A0 *)(d))
#define fpi_device_get_cancellable(d) ((void)(d), (GCancellable *)NULL)
#define FP_DEVICE(x) ((FpDevice *)(x))
