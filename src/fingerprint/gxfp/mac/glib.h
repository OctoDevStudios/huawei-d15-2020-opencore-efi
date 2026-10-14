/* Mini-glib: just what the goodix51a0 sources need when ported to macOS. */
#pragma once
#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

typedef int gboolean;
typedef int gint;
typedef unsigned int guint;
typedef uint8_t guint8;
typedef uint16_t guint16;
typedef uint32_t guint32;
typedef int32_t gint32;
typedef int64_t gint64;
typedef uint64_t guint64;
typedef size_t gsize;
typedef ssize_t gssize;
typedef char gchar;
typedef double gdouble;
typedef void *gpointer;
typedef const void *gconstpointer;
typedef void (*GDestroyNotify)(gpointer);

#ifndef TRUE
#define TRUE 1
#define FALSE 0
#endif
#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif
#define CLAMP(x, lo, hi) (((x) > (hi)) ? (hi) : (((x) < (lo)) ? (lo) : (x)))
#define G_N_ELEMENTS(a) (sizeof(a) / sizeof((a)[0]))
#define G_STATIC_ASSERT(e) _Static_assert(e, #e)
#define G_USEC_PER_SEC 1000000
#define G_GINT64_CONSTANT(v) ((gint64)(v##LL))
#define G_MAXINT INT32_MAX
#define G_MAXUINT UINT32_MAX
#define G_MAXINT64 INT64_MAX
#define G_MAXSIZE SIZE_MAX
#define G_LIKELY(x) (x)
#define G_UNLIKELY(x) (x)

/* ---- memory ---- */
static inline gpointer g_malloc(gsize n) { void *p = malloc(n ? n : 1); if (!p) abort(); return p; }
static inline gpointer g_malloc0(gsize n) { void *p = calloc(1, n ? n : 1); if (!p) abort(); return p; }
static inline void g_free(gpointer p) { free(p); }
#define g_new(T, n) ((T *)g_malloc(sizeof(T) * (gsize)(n)))
#define g_new0(T, n) ((T *)g_malloc0(sizeof(T) * (gsize)(n)))
static inline gchar *g_strdup(const gchar *s) { return s ? strdup(s) : NULL; }
static inline gpointer g_memdup2(gconstpointer p, gsize n) { if (!p) return NULL; void *d = g_malloc(n); memcpy(d, p, n); return d; }
#define g_clear_pointer(pp, destroy) do { if (*(pp)) { destroy((void *)*(pp)); *(pp) = NULL; } } while (0)
static inline gpointer g_steal_pointer_impl(gpointer pp) { void **p = (void **)pp; void *r = *p; *p = NULL; return r; }
#define g_steal_pointer(pp) ((__typeof__(*(pp)))g_steal_pointer_impl(pp))

static inline void g__autofree_cleanup(void *pp) { free(*(void **)pp); }
#define g_autofree __attribute__((cleanup(g__autofree_cleanup)))
#define g_autoptr(T) __attribute__((cleanup(T##__autoptr_cleanup))) T *

/* ---- strings ---- */
static inline gsize g_strlcpy(gchar *d, const gchar *s, gsize n) { return strlcpy(d, s, n); }
static inline int g_strcmp0(const char *a, const char *b) { if (!a) return b ? -1 : 0; if (!b) return 1; return strcmp(a, b); }
static inline gchar *g_strdup_printf(const gchar *fmt, ...) __attribute__((format(printf, 1, 2)));
static inline gchar *g_strdup_printf(const gchar *fmt, ...) { va_list ap; va_start(ap, fmt); char *s = NULL; vasprintf(&s, fmt, ap); va_end(ap); return s; }
#define g_strerror strerror
static inline const gchar *g_getenv(const gchar *n) { return getenv(n); }

/* ---- time ---- */
static inline void g_usleep(unsigned long us) { usleep((useconds_t)us); }
static inline gint64 g_get_monotonic_time(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return (gint64)t.tv_sec * 1000000 + t.tv_nsec / 1000; }

/* ---- log ---- */
extern int gx_log_level;   /* 0 warn, 1 info, 2 debug */
#define fp_warn(...)  do { fprintf(stderr, "[warn] " __VA_ARGS__); fputc('\n', stderr); } while (0)
#define fp_info(...)  do { if (gx_log_level >= 1) { fprintf(stderr, "[info] " __VA_ARGS__); fputc('\n', stderr); } } while (0)
#define fp_dbg(...)   do { if (gx_log_level >= 2) { fprintf(stderr, "[dbg] " __VA_ARGS__); fputc('\n', stderr); } } while (0)
#define g_debug fp_dbg
#define g_return_val_if_fail(e, v) do { if (!(e)) return (v); } while (0)

/* ---- GError ---- */
typedef struct { int domain; int code; char *message; } GError;
#define G_IO_ERROR 1
#define G_IO_ERROR_FAILED 0
#define G_IO_ERROR_NOT_FOUND 1
static inline void g_set_error_literal(GError **e, int d, int c, const char *m) { if (e) { *e = g_new0(GError, 1); (*e)->domain = d; (*e)->code = c; (*e)->message = g_strdup(m); } else fp_warn("%s", m); }
#define g_set_error(e, d, c, ...) do { char *m_ = g_strdup_printf(__VA_ARGS__); g_set_error_literal(e, d, c, m_); free(m_); } while (0)
static inline void g_error_free(GError *e) { if (e) { free(e->message); free(e); } }
static inline void g_clear_error(GError **e) { if (e && *e) { g_error_free(*e); *e = NULL; } }
static inline void GError__autoptr_cleanup(GError **e) { g_clear_error(e); }

/* ---- GPtrArray ---- */
typedef struct { gpointer *pdata; guint len; guint cap; GDestroyNotify free_func; } GPtrArray;
static inline GPtrArray *g_ptr_array_new_with_free_func(GDestroyNotify f) { GPtrArray *a = g_new0(GPtrArray, 1); a->free_func = f; return a; }
static inline GPtrArray *g_ptr_array_new(void) { return g_ptr_array_new_with_free_func(NULL); }
static inline void g_ptr_array_add(GPtrArray *a, gpointer p) { if (a->len == a->cap) { a->cap = a->cap ? a->cap * 2 : 8; a->pdata = realloc(a->pdata, a->cap * sizeof(gpointer)); } a->pdata[a->len++] = p; }
#define g_ptr_array_index(a, i) ((a)->pdata[i])
static inline void g_ptr_array_set_free_func(GPtrArray *a, GDestroyNotify f) { a->free_func = f; }
static inline void g_ptr_array_unref(GPtrArray *a) { if (!a) return; if (a->free_func) for (guint i = 0; i < a->len; i++) if (a->pdata[i]) a->free_func(a->pdata[i]); free(a->pdata); free(a); }
static inline void GPtrArray__autoptr_cleanup(GPtrArray **a) { if (*a) g_ptr_array_unref(*a); }

/* ---- GByteArray ---- */
typedef struct { guint8 *data; guint len; guint cap; } GByteArray;
static inline GByteArray *g_byte_array_sized_new(guint n) { GByteArray *b = g_new0(GByteArray, 1); b->cap = n ? n : 16; b->data = g_malloc(b->cap); return b; }
static inline GByteArray *g_byte_array_new(void) { return g_byte_array_sized_new(16); }
static inline GByteArray *g_byte_array_append(GByteArray *b, const guint8 *d, guint n) { if (b->len + n > b->cap) { while (b->len + n > b->cap) b->cap *= 2; b->data = realloc(b->data, b->cap); } memcpy(b->data + b->len, d, n); b->len += n; return b; }
static inline void g_byte_array_unref(GByteArray *b) { if (b) { free(b->data); free(b); } }
static inline guint8 *g_byte_array_free(GByteArray *b, gboolean free_seg) { guint8 *d = b->data; if (free_seg) { free(d); d = NULL; } free(b); return d; }
static inline void GByteArray__autoptr_cleanup(GByteArray **b) { if (*b) g_byte_array_unref(*b); }

/* ---- once ---- */
#define g_once_init_enter(p) (*(p) == 0)
#define g_once_init_leave(p, v) (*(p) = (v))

/* ---- files ---- */
#define G_FILE_TEST_EXISTS 1
#define G_FILE_TEST_IS_DIR 2
#define G_FILE_SET_CONTENTS_CONSISTENT 1
#define G_FILE_SET_CONTENTS_DURABLE 2
typedef struct stat GStatBuf;
static inline gboolean g_file_test(const gchar *p, int t) { struct stat s; if (stat(p, &s)) return FALSE; if (t & G_FILE_TEST_IS_DIR) return S_ISDIR(s.st_mode); return TRUE; }
static inline gboolean g_file_get_contents(const gchar *p, gchar **out, gsize *len, GError **e)
{
    FILE *f = fopen(p, "rb"); if (!f) { g_set_error(e, G_IO_ERROR, 1, "open %s: %s", p, strerror(errno)); return FALSE; }
    fseek(f, 0, SEEK_END); long n = ftell(f); fseek(f, 0, SEEK_SET);
    char *b = g_malloc((gsize)n + 1); gsize r = fread(b, 1, (size_t)n, f); fclose(f); b[r] = 0;
    *out = b; if (len) *len = r; return TRUE;
}
static inline gboolean g_file_set_contents_full(const gchar *p, const gchar *d, gssize n, int flags, int mode, GError **e)
{
    (void)flags; char tmp[1024]; snprintf(tmp, sizeof tmp, "%s.tmp", p);
    int fd = open(tmp, O_WRONLY | O_CREAT | O_TRUNC, mode); if (fd < 0) { g_set_error(e, G_IO_ERROR, 1, "open %s", tmp); return FALSE; }
    if (n < 0) n = (gssize)strlen(d);
    if (write(fd, d, (size_t)n) != n) { close(fd); unlink(tmp); return FALSE; }
    fsync(fd); close(fd); return rename(tmp, p) == 0;
}
static inline int g_mkdir_with_parents(const gchar *p, int mode) { char b[1024]; snprintf(b, sizeof b, "%s", p); for (char *s = b + 1; *s; s++) if (*s == '/') { *s = 0; mkdir(b, mode); *s = '/'; } return (mkdir(b, mode) == 0 || errno == EEXIST) ? 0 : -1; }
#define g_chmod chmod
#define g_stat stat
#define g_unlink unlink

/* ---- cancellation (libfprint): never cancelled here ---- */
typedef void GCancellable;
#define g_cancellable_is_cancelled(c) ((void)(c), 0)
