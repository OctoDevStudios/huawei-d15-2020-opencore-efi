/*
 * MateBookDSP.h — output processing of the HAL plugin (48 kHz stereo, float).
 *
 * SPEAKERS profile (calibrated by measurement with the internal mic, tools/measure/):
 *   volume -> 120 Hz HPF (4th order) + virtual bass (150-500 Hz harmonics)
 *   -> EQ (-7 dB @820 Hz "boxiness", +6 dB @2.9 kHz presence, +7 dB shelf 6 kHz)
 *   -> +6 dB makeup -> bass limiter (<400 Hz, prevents speaker distortion)
 *   -> look-ahead peak limiter (ceiling -1 dBFS)
 * HEADPHONES profile: volume -> peak limiter only (-0.5 dBFS), otherwise flat
 * like Linux (flat SOF pipeline, no playback EQ).
 *
 * All state is static/preallocated: no malloc and no lock on the IO thread.
 */
#ifndef MATEBOOK_DSP_H
#define MATEBOOK_DSP_H

#include <math.h>
#include <string.h>

#define MBDSP_SR            48000.0
#define MBDSP_LA            120          /* limiter look-ahead: 2.5 ms */

/* ---- tuning (dB / Hz) ---- */
#define MBDSP_SPK_HPF_HZ        120.0
#define MBDSP_SPK_VB_GAIN       1.2      /* virtual bass */
#define MBDSP_SPK_MAKEUP_DB     6.0
#define MBDSP_SPK_BASS_SPLIT    400.0
#define MBDSP_SPK_BASS_THR_DB   (-15.0)  /* max bass-band peak before distortion */
#define MBDSP_SPK_CEIL_DB       (-1.0)
#define MBDSP_HP_CEIL_DB        (-0.5)

/* ---- volume: cubic curve (like PipeWire), dB = 60·log10(s) ---- */
static inline float MBVol_ScalarToDB(float s) {
    if (s <= 0.001f) return -60.0f;
    float d = 60.0f * log10f(s);
    return d < -60.0f ? -60.0f : (d > 0.0f ? 0.0f : d);
}
static inline float MBVol_DBToScalar(float d) {
    if (d <= -60.0f) return 0.0f;
    if (d >= 0.0f) return 1.0f;
    return powf(10.0f, d / 60.0f);
}
static inline float MBVol_Gain(float s) { return s * s * s; }

/* ---- biquad (RBJ), TDF-II in double precision ---- */
typedef struct { double b0, b1, b2, a1, a2; } MBBiq;
typedef struct { double z1, z2; } MBBiqState;

enum { MB_PK, MB_HS, MB_LS, MB_HP, MB_LP };

static MBBiq MBBiq_Make(int t, double f0, double gdb, double q) {
    double A = pow(10.0, gdb / 40.0), w = 2 * M_PI * f0 / MBDSP_SR;
    double c = cos(w), s = sin(w), al = s / (2 * q), sq = 2 * sqrt(A) * al;
    double b0 = 1, b1 = 0, b2 = 0, a0 = 1, a1 = 0, a2 = 0;
    switch (t) {
    case MB_PK: b0 = 1 + al * A; b1 = -2 * c; b2 = 1 - al * A; a0 = 1 + al / A; a1 = -2 * c; a2 = 1 - al / A; break;
    case MB_HS: b0 = A * ((A + 1) + (A - 1) * c + sq); b1 = -2 * A * ((A - 1) + (A + 1) * c);
                b2 = A * ((A + 1) + (A - 1) * c - sq); a0 = (A + 1) - (A - 1) * c + sq;
                a1 = 2 * ((A - 1) - (A + 1) * c); a2 = (A + 1) - (A - 1) * c - sq; break;
    case MB_LS: b0 = A * ((A + 1) - (A - 1) * c + sq); b1 = 2 * A * ((A - 1) - (A + 1) * c);
                b2 = A * ((A + 1) - (A - 1) * c - sq); a0 = (A + 1) + (A - 1) * c + sq;
                a1 = -2 * ((A - 1) + (A + 1) * c); a2 = (A + 1) + (A - 1) * c - sq; break;
    case MB_HP: b0 = (1 + c) / 2; b1 = -(1 + c); b2 = (1 + c) / 2; a0 = 1 + al; a1 = -2 * c; a2 = 1 - al; break;
    case MB_LP: b0 = (1 - c) / 2; b1 = 1 - c; b2 = (1 - c) / 2; a0 = 1 + al; a1 = -2 * c; a2 = 1 - al; break;
    }
    MBBiq r = { b0 / a0, b1 / a0, b2 / a0, a1 / a0, a2 / a0 };
    return r;
}
static inline double MBBiq_Run(const MBBiq *f, MBBiqState *s, double x) {
    double y = f->b0 * x + s->z1;
    s->z1 = f->b1 * x - f->a1 * y + s->z2;
    s->z2 = f->b2 * x - f->a2 * y;
    return y;
}

/* ---- look-ahead peak limiter (linked stereo, guaranteed no overshoot) ---- */
typedef struct {
    float  dl[2][MBDSP_LA];        /* audio delay line */
    float  req[MBDSP_LA];          /* required gain (window) */
    int    dq[MBDSP_LA];           /* deque indices for sliding min */
    int    dqh, dqn;
    float  box[MBDSP_LA];          /* boxcar over released gain */
    double boxSum;
    float  rel;                    /* gain after release */
    int    pos;
    float  ceil, relCoef;
} MBLimiter;

static void MBLim_Init(MBLimiter *l, double ceilDb, double releaseMs) {
    memset(l, 0, sizeof(*l));
    l->ceil = (float)pow(10.0, ceilDb / 20.0);
    l->relCoef = (float)(1.0 - exp(-1.0 / (releaseMs * 0.001 * MBDSP_SR)));
    l->rel = 1.0f;
    for (int i = 0; i < MBDSP_LA; i++) { l->req[i] = 1.0f; l->box[i] = 1.0f; }
    l->boxSum = MBDSP_LA;
}

static inline void MBLim_Run(MBLimiter *l, float *L, float *R) {
    const int N = MBDSP_LA, p = l->pos;
    float pk = fmaxf(fabsf(*L), fabsf(*R));
    float r = pk > l->ceil ? l->ceil / pk : 1.0f;
    l->req[p] = r;
    /* sliding min over N samples (monotonic deque, circular indices) */
    while (l->dqn > 0) {
        int last = l->dq[(l->dqh + l->dqn - 1) % N];
        if (l->req[last] >= r) l->dqn--; else break;
    }
    if (l->dqn > 0 && l->dq[l->dqh] == p) { l->dqh = (l->dqh + 1) % N; l->dqn--; }
    l->dq[(l->dqh + l->dqn) % N] = p; l->dqn++;
    float hold = l->req[l->dq[l->dqh]];
    /* exponential release, instant attack on the held value */
    if (hold < l->rel) l->rel = hold; else l->rel += (hold - l->rel) * l->relCoef;
    /* N-tap boxcar -> smooth attack ramp */
    l->boxSum += l->rel - l->box[p]; l->box[p] = l->rel;
    float g = (float)(l->boxSum / N);
    /* output delayed by N-1: the oldest sample in the window */
    int o = (p + 1) % N;
    float oL = l->dl[0][o], oR = l->dl[1][o];
    l->dl[0][p] = *L; l->dl[1][p] = *R;
    /* the element leaving the window on the next step */
    if (l->dqn > 0 && l->dq[l->dqh] == o) { l->dqh = (l->dqh + 1) % N; l->dqn--; }
    l->pos = o;
    *L = oL * g; *R = oR * g;
}

/* ---- global state ---- */
#define MBDSP_NEQ 3
typedef struct {
    MBBiq hp1, hp2, eq[MBDSP_NEQ], vbLp1, vbLp2, vbHp, vbLp3, xLp, xHp;
    MBBiqState sHp1[2], sHp2[2], sEq[MBDSP_NEQ][2], sVbLp1, sVbLp2, sVbHp, sVbLp3, sXLp1[2], sXLp2[2], sXHp1[2], sXHp2[2];
    float  makeup, bassThr, bassEnv, bassG, bassAtk, bassRel, envRel;
    MBLimiter limSpk, limHp;
    int    profile;          /* 0 = speakers, 1 = headphones */
    int    ready;
} MBDSP;

static void MBDSP_Init(MBDSP *d) {
    memset(d, 0, sizeof(*d));
    /* 4th-order Butterworth HPF (Q 0.541 + 1.307) */
    d->hp1 = MBBiq_Make(MB_HP, MBDSP_SPK_HPF_HZ, 0, 0.5412);
    d->hp2 = MBBiq_Make(MB_HP, MBDSP_SPK_HPF_HZ, 0, 1.3066);
    d->eq[0] = MBBiq_Make(MB_PK, 820.0, -7.0, 1.1);
    d->eq[1] = MBBiq_Make(MB_PK, 2900.0, 6.0, 1.0);
    d->eq[2] = MBBiq_Make(MB_HS, 6000.0, 7.0, 0.7);
    /* virtual bass: <150 Hz band -> |x| -> 150..500 Hz */
    d->vbLp1 = MBBiq_Make(MB_LP, 150.0, 0, 0.7071);
    d->vbLp2 = MBBiq_Make(MB_LP, 150.0, 0, 0.7071);
    d->vbHp  = MBBiq_Make(MB_HP, 150.0, 0, 0.7071);
    d->vbLp3 = MBBiq_Make(MB_LP, 500.0, 0, 0.7071);
    /* Linkwitz-Riley 4 crossover: LP²+HP² = all-pass (no coloration at g=1) */
    d->xLp = MBBiq_Make(MB_LP, MBDSP_SPK_BASS_SPLIT, 0, 0.7071);
    d->xHp = MBBiq_Make(MB_HP, MBDSP_SPK_BASS_SPLIT, 0, 0.7071);
    d->makeup  = (float)pow(10.0, MBDSP_SPK_MAKEUP_DB / 20.0);
    d->bassThr = (float)pow(10.0, MBDSP_SPK_BASS_THR_DB / 20.0);
    d->bassG = 1.0f;
    d->bassAtk = (float)(1.0 - exp(-1.0 / (0.002 * MBDSP_SR)));
    d->bassRel = (float)(1.0 - exp(-1.0 / (0.250 * MBDSP_SR)));
    d->envRel  = (float)exp(-1.0 / (0.050 * MBDSP_SR));
    MBLim_Init(&d->limSpk, MBDSP_SPK_CEIL_DB, 120.0);
    MBLim_Init(&d->limHp, MBDSP_HP_CEIL_DB, 80.0);
    d->ready = 1;
}

/* Process n interleaved frames in -> out (full-scale S32). */
static void MBDSP_Process(MBDSP *d, const float *in, int32_t *out, unsigned n, float gain, int profile) {
    if (!d->ready) MBDSP_Init(d);
    if (profile != d->profile) {           /* output changed: start clean */
        int keep = profile; MBDSP_Init(d); d->profile = keep;
    }
    for (unsigned i = 0; i < n; i++) {
        float L = in[2 * i] * gain, R = in[2 * i + 1] * gain;
        if (d->profile == 0) {
            /* virtual bass from the full-band signal (before HPF) */
            double m = 0.5 * (L + R);
            m = MBBiq_Run(&d->vbLp1, &d->sVbLp1, m);
            m = MBBiq_Run(&d->vbLp2, &d->sVbLp2, m);
            double h = fabs(m);
            h = MBBiq_Run(&d->vbHp, &d->sVbHp, h);
            h = MBBiq_Run(&d->vbLp3, &d->sVbLp3, h) * MBDSP_SPK_VB_GAIN;
            double ch[2] = { L, R };
            for (int c = 0; c < 2; c++) {
                double v = MBBiq_Run(&d->hp1, &d->sHp1[c], ch[c]);
                v = MBBiq_Run(&d->hp2, &d->sHp2[c], v) + h;
                for (int k = 0; k < MBDSP_NEQ; k++) v = MBBiq_Run(&d->eq[k], &d->sEq[k][c], v);
                ch[c] = v * d->makeup;
            }
            /* bass limiter: y = highs(LR4) + g·lows(LR4) */
            double lo[2], hi[2];
            for (int c = 0; c < 2; c++) {
                lo[c] = MBBiq_Run(&d->xLp, &d->sXLp2[c], MBBiq_Run(&d->xLp, &d->sXLp1[c], ch[c]));
                hi[c] = MBBiq_Run(&d->xHp, &d->sXHp2[c], MBBiq_Run(&d->xHp, &d->sXHp1[c], ch[c]));
            }
            double lo0 = lo[0], lo1 = lo[1];
            float pk = (float)fmax(fabs(lo0), fabs(lo1));
            d->bassEnv = pk > d->bassEnv ? pk : d->bassEnv * d->envRel;
            float tgt = d->bassEnv > d->bassThr ? d->bassThr / d->bassEnv : 1.0f;
            d->bassG += (tgt - d->bassG) * (tgt < d->bassG ? d->bassAtk : d->bassRel);
            L = (float)(hi[0] + lo0 * d->bassG);
            R = (float)(hi[1] + lo1 * d->bassG);
            MBLim_Run(&d->limSpk, &L, &R);
        } else {
            MBLim_Run(&d->limHp, &L, &R);
        }
        if (L > 1.0f) L = 1.0f; if (L < -1.0f) L = -1.0f;
        if (R > 1.0f) R = 1.0f; if (R < -1.0f) R = -1.0f;
        out[2 * i]     = (int32_t)(L * 2147483647.0f);
        out[2 * i + 1] = (int32_t)(R * 2147483647.0f);
    }
}

#endif
