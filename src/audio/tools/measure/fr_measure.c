/*
 * fr_measure — speaker -> internal mic frequency response (stepped sine).
 * Plays 1/6-octave sines (50 Hz..16 kHz) on the default output while
 * recording the default input (AudioQueue), then measures the level of
 * each fundamental (windowed Goertzel) + H2/H3 + noise floor.
 *
 *   fr_measure [amplitude_dBFS=-18] [out.csv] [rec.raw]
 */
#include <AudioToolbox/AudioToolbox.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define SR        48000.0
#define NSTEPS    50
#define TONE_S    0.40
#define GAP_S     0.06
#define PRE_S     1.00
#define POST_S    1.00
#define NBUF      3
#define BUF_FR    2048

static float  *gPlay;      /* mono play signal */
static long    gPlayLen, gPlayPos;
static float  *gRec;       /* mono (L) recorded */
static long    gRecLen, gRecPos;

static double stepFreq(int k) { return 50.0 * pow(2.0, k / 6.0); }

static void outCb(void *u, AudioQueueRef q, AudioQueueBufferRef b) {
    float *d = (float *)b->mAudioData;
    for (int i = 0; i < BUF_FR; i++) {
        float v = gPlayPos < gPlayLen ? gPlay[gPlayPos++] : 0.f;
        d[2 * i] = v; d[2 * i + 1] = v;
    }
    b->mAudioDataByteSize = BUF_FR * 8;
    AudioQueueEnqueueBuffer(q, b, 0, NULL);
}

static void inCb(void *u, AudioQueueRef q, AudioQueueBufferRef b,
                 const AudioTimeStamp *t, UInt32 n, const AudioStreamPacketDescription *p) {
    float *s = (float *)b->mAudioData;
    UInt32 fr = b->mAudioDataByteSize / 8;
    for (UInt32 i = 0; i < fr && gRecPos < gRecLen; i++) gRec[gRecPos++] = s[2 * i];
    AudioQueueEnqueueBuffer(q, b, 0, NULL);
}

/* Hann-windowed Goertzel amplitude (linear, sine peak) */
static double tone_amp(const float *x, long n, double f) {
    double w = 2 * M_PI * f / SR, c = 2 * cos(w), s1 = 0, s2 = 0, wsum = 0;
    for (long i = 0; i < n; i++) {
        double h = 0.5 - 0.5 * cos(2 * M_PI * i / (n - 1));
        double s0 = x[i] * h + c * s1 - s2; s2 = s1; s1 = s0; wsum += h;
    }
    double re = s1 - s2 * cos(w), im = s2 * sin(w);
    return 2 * sqrt(re * re + im * im) / wsum;
}
static double db(double a) { return 20 * log10(a + 1e-12); }

int main(int argc, char **argv) {
    double ampDb = argc > 1 ? atof(argv[1]) : -18.0;
    const char *csv = argc > 2 ? argv[2] : "fr.csv";
    const char *raw = argc > 3 ? argv[3] : NULL;
    double amp = pow(10, ampDb / 20);

    long toneN = (long)(TONE_S * SR), gapN = (long)(GAP_S * SR), preN = (long)(PRE_S * SR);
    gPlayLen = preN + NSTEPS * (toneN + gapN) + (long)(POST_S * SR);
    gPlay = calloc(gPlayLen, sizeof(float));
    long fadeN = (long)(0.01 * SR);
    for (int k = 0; k < NSTEPS; k++) {
        long st = preN + k * (toneN + gapN);
        double f = stepFreq(k);
        for (long i = 0; i < toneN; i++) {
            double e = 1;
            if (i < fadeN) e = (double)i / fadeN;
            if (i > toneN - fadeN) e = (double)(toneN - i) / fadeN;
            gPlay[st + i] = (float)(amp * e * sin(2 * M_PI * f * i / SR));
        }
    }
    gRecLen = gPlayLen + (long)(1.0 * SR);
    gRec = calloc(gRecLen, sizeof(float));

    AudioStreamBasicDescription fmt = {0};
    fmt.mSampleRate = SR; fmt.mFormatID = kAudioFormatLinearPCM;
    fmt.mFormatFlags = kAudioFormatFlagIsFloat | kAudioFormatFlagIsPacked;
    fmt.mChannelsPerFrame = 2; fmt.mBitsPerChannel = 32;
    fmt.mBytesPerFrame = 8; fmt.mFramesPerPacket = 1; fmt.mBytesPerPacket = 8;

    AudioQueueRef oq, iq;
    OSStatus e1 = AudioQueueNewOutput(&fmt, outCb, NULL, CFRunLoopGetCurrent(), kCFRunLoopCommonModes, 0, &oq);
    OSStatus e2 = AudioQueueNewInput(&fmt, inCb, NULL, CFRunLoopGetCurrent(), kCFRunLoopCommonModes, 0, &iq);
    if (e1 || e2) { fprintf(stderr, "queue err out=%d in=%d\n", (int)e1, (int)e2); return 1; }
    for (int i = 0; i < NBUF; i++) {
        AudioQueueBufferRef b;
        AudioQueueAllocateBuffer(oq, BUF_FR * 8, &b); outCb(NULL, oq, b);
        AudioQueueAllocateBuffer(iq, BUF_FR * 8, &b); AudioQueueEnqueueBuffer(iq, b, 0, NULL);
    }
    AudioQueueStart(iq, NULL);
    AudioQueueStart(oq, NULL);
    double total = (double)gRecLen / SR + 0.5;
    CFRunLoopRunInMode(kCFRunLoopDefaultMode, total, false);
    AudioQueueStop(oq, true); AudioQueueStop(iq, true);
    fprintf(stderr, "recorded %ld / %ld frames\n", gRecPos, gRecLen);
    if (raw) { FILE *r = fopen(raw, "wb"); fwrite(gRec, 4, gRecPos, r); fclose(r); }

    float peak = 0; for (long i = 0; i < gRecPos; i++) peak = fmaxf(peak, fabsf(gRec[i]));
    fprintf(stderr, "mic peak %.3f (%.1f dBFS)\n", peak, db(peak));

    /* alignment: find the offset from the ~1 kHz step (k=26..33) */
    long win = (long)(0.10 * SR), hop = (long)(0.005 * SR);
    long offs[8]; int no = 0;
    for (int k = 26; k < 34; k++) {
        double f = stepFreq(k), best = 0; long bt = 0;
        long exp0 = preN + k * (toneN + gapN);
        for (long t = exp0 - (long)(0.3 * SR); t < exp0 + (long)(0.6 * SR); t += hop) {
            if (t < 0 || t + win > gRecPos) continue;
            double a = tone_amp(gRec + t, win, f);
            if (a > best) { best = a; bt = t; }
        }
        offs[no++] = bt - (exp0 + (toneN - win) / 2);
    }
    /* median */
    for (int i = 0; i < no; i++) for (int j = i + 1; j < no; j++) if (offs[j] < offs[i]) { long t = offs[i]; offs[i] = offs[j]; offs[j] = t; }
    long off = offs[no / 2];
    fprintf(stderr, "latency offset %.1f ms\n", off * 1000.0 / SR);

    FILE *o = fopen(csv, "w");
    fprintf(o, "freq,level_db,h2_db,h3_db,noise_db\n");
    long mN = toneN - (long)(0.12 * SR);
    for (int k = 0; k < NSTEPS; k++) {
        double f = stepFreq(k);
        long st = preN + k * (toneN + gapN) + off + (long)(0.08 * SR);
        if (st < 0 || st + mN > gRecPos) { fprintf(o, "%.1f,nan,nan,nan,nan\n", f); continue; }
        double a = tone_amp(gRec + st, mN, f) / amp;
        double h2 = 2 * f < SR / 2 - 500 ? tone_amp(gRec + st, mN, 2 * f) / amp : 0;
        double h3 = 3 * f < SR / 2 - 500 ? tone_amp(gRec + st, mN, 3 * f) / amp : 0;
        long ns = (long)(0.1 * SR);
        double nf = tone_amp(gRec + ns, mN, f) / amp; /* silent pre-roll */
        fprintf(o, "%.1f,%.2f,%.2f,%.2f,%.2f\n", f, db(a), db(h2), db(h3), db(nf));
    }
    fclose(o);
    fprintf(stderr, "wrote %s\n", csv);
    return 0;
}
