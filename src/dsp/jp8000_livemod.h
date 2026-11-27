/* jp8000_livemod.h -- where the sound has its cutoff now (Pung's live_mod).
 *
 * The JP-8000's H8S runs the envelopes and LFOs and writes the ASICs'
 * coefficients ~100 times a second (if_mode 0x55: two 10-bit fields into
 * two program words). Measured on the v1.5 firmware with
 * src/tools/je_modscan.cpp (never shipped):
 *
 *  - Eight voices. Each has a cutoff word (knob, filter envelope, LFOs, key
 *    follow, velocity and the ribbon in it) and an amp word whose high
 *    field is the voice's amp envelope level (256: silent; it rises through
 *    the attack, peaks by velocity, falls through decay and release).
 *  - The cutoff word maps the knob monotonically (ordered by its low field,
 *    then its high field) and is the same for every filter type, slope and
 *    resonance; from knob 114 up it is saturated (fully open). The firmware
 *    writes it only while the voice sounds.
 *  - Which voices a part plays: in Single all eight are the upper part's
 *    (taken in order V0..V7; the lower part is silent); in Dual and Split
 *    the performance's Voice Assign divides them (2-6 .. 6-2: the upper part
 *    the first number, the lower the second), MIDI channel 1 playing the
 *    upper part and channel 2 the lower (the split point divides only the
 *    performance control channel). The addresses are the same for every
 *    patch.
 *
 * The DSP process watches the writes (jp_lm_capture, on the H8S thread),
 * the plugin turns them into knob units (jp_lm_cutoff). */
#pragma once
#include <stddef.h>
#include <stdint.h>

#define JP_LM_VOICES 8

/* each voice's words: asic, program word */
static const uint8_t  JP_LM_AMP_ASIC[JP_LM_VOICES] = {3, 3, 3, 3, 3, 3, 3, 3};
static const uint16_t JP_LM_AMP_ADDR[JP_LM_VOICES] = {138, 141, 161, 165, 185, 189, 209, 213};
static const uint8_t  JP_LM_CUT_ASIC[JP_LM_VOICES] = {2, 2, 2, 3, 3, 3, 2, 2};
static const uint16_t JP_LM_CUT_ADDR[JP_LM_VOICES] = {1024, 1072, 1120, 0, 46, 92, 86, 129};

/* the upper part's voices in Dual and Split for each Voice Assign (bit v:
 * voice v); the lower part has the others */
static const uint8_t JP_LM_UPPER[5] = {
    0x05, /* 2-6: V0 V2 */
    0xC1, /* 3-5: V0 V6 V7 */
    0xC5, /* 4-4: V0 V2 V6 V7 */
    0x3D, /* 5-3: V0 V2 V3 V4 V5 */
    0xF9, /* 6-2: V0 V3 V4 V5 V6 V7 */
};

/* the cutoff word at each knob position 0..127, as the firmware writes it
 * with nothing modulating (key follow 0, note 60) */
static const uint32_t JP_LM_CUT_TABLE[128] = {
    0x45900, 0x45d00, 0x46500, 0x46900, 0x47100, 0x47900, 0x48100, 0x48500,
    0x48d00, 0x49900, 0x4a100, 0x4a900, 0x4b500, 0x4bd00, 0x4c900, 0x4d500,
    0x4e500, 0x4f100, 0x50100, 0x50900, 0x51900, 0x53100, 0x54100, 0x55100,
    0x56900, 0x57900, 0x59100, 0x5a900, 0x5c900, 0x5e100, 0x40101, 0x41101,
    0x43101, 0x46101, 0x48101, 0x4a101, 0x4d101, 0x4f101, 0x52101, 0x55101,
    0x59101, 0x5c101, 0x40102, 0x42102, 0x46102, 0x4c102, 0x50102, 0x54102,
    0x5a102, 0x5e102, 0x44103, 0x4a103, 0x52103, 0x58103, 0x40104, 0x44104,
    0x4c104, 0x58104, 0x40105, 0x48105, 0x54105, 0x5c105, 0x48106, 0x54106,
    0x44107, 0x50107, 0x40108, 0x48108, 0x58108, 0x50109, 0x4010a, 0x5010a,
    0x4810b, 0x5810b, 0x5010c, 0x4810d, 0x4810e, 0x4010f, 0x40110, 0x50110,
    0x50111, 0x40113, 0x40114, 0x40115, 0x50116, 0x50117, 0x40119, 0x5011a,
    0x5011c, 0x4011e, 0x40120, 0x40121, 0x40123, 0x40126, 0x40128, 0x4012a,
    0x4012d, 0x4012f, 0x40132, 0x40135, 0x40139, 0x4013c, 0x40140, 0x40142,
    0x40146, 0x4014c, 0x40150, 0x40154, 0x4015a, 0x4015e, 0x40164, 0x4016a,
    0x40172, 0x40178, 0x5fd7f, 0x5fd7f, 0x5fd7f, 0x5fd7f, 0x5fd7f, 0x5fd7f,
    0x5fd7f, 0x5fd7f, 0x5fd7f, 0x5fd7f, 0x5fd7f, 0x5fd7f, 0x5fd7f, 0x5fd7f,
};
#define JP_LM_CUT_OPEN 0x5fd7fu /* saturated: knob 114 and up */
#define JP_LM_CUT_FULL 114

/* what the DSP process publishes in the shared memory (written there, read
 * here; 32-bit words, no lock) */
struct jp_livemod_t {
    volatile uint32_t cut[JP_LM_VOICES];  /* each voice's cutoff word, 0: not yet */
    volatile uint32_t amp[JP_LM_VOICES];  /* each voice's amp level (256: silent), 0: not yet */
    volatile int32_t  newest[2];          /* the newest voice of the upper, lower part; -1: none */
    volatile uint32_t started[JP_LM_VOICES]; /* each voice's start, in order (0: not yet) */
    volatile int32_t  part[JP_LM_VOICES];    /* the part it started for: 0 upper, 1 lower */
    volatile uint32_t ok;                 /* 1 once the words are the measured kind */
};

/* The firmware all of this was measured on: the JP-8000's v1.5, as the
 * emulator holds it once the dump is put together (the CRC-32 of the whole
 * image). The emulator also boots other firmware (another version, a
 * JP-8080's), whose program writes other words to other places: there the
 * module shows no live cutoff, where it would show a wrong one. */
#define JP_LM_ROM_CRC 0xa30ec332u /* 524288 bytes; `je_modscan <roms> rom` says it */

static inline uint32_t jp_lm_crc32(const uint8_t *d, size_t n) {
    uint32_t c = 0xffffffffu;
    for (size_t i = 0; i < n; i++) {
        c ^= d[i];
        for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xedb88320u & (0u - (c & 1u)));
    }
    return ~c;
}

/* the word's order: its low field first, then its high field */
static inline uint32_t jp_lm_order(uint32_t w) { return ((w & 0x3ffu) << 10) | (w >> 10); }

/* a cutoff word of the measured kind: both fields have bit 8 set */
static inline int jp_lm_cut_word(uint32_t w) { return (w & 0x100u) && ((w >> 10) & 0x100u); }

/* a cutoff word in knob units (0..127). Saturated: fully open, which is any
 * knob from 114 up: the knob itself if it is there. Between two knob steps
 * the place in the word's order, linearly. */
static inline float jp_lm_cutoff(uint32_t w, int knob) {
    if (w == JP_LM_CUT_OPEN) return knob >= JP_LM_CUT_FULL ? (float)knob : (float)JP_LM_CUT_FULL;
    const uint32_t k = jp_lm_order(w);
    if (k <= jp_lm_order(JP_LM_CUT_TABLE[0])) return 0.0f;
    for (int i = 1; i <= JP_LM_CUT_FULL; i++) {
        const uint32_t k1 = jp_lm_order(JP_LM_CUT_TABLE[i]);
        if (k <= k1) {
            const uint32_t k0 = jp_lm_order(JP_LM_CUT_TABLE[i - 1]);
            return (float)(i - 1) + (float)(k - k0) / (float)(k1 - k0);
        }
    }
    return (float)JP_LM_CUT_FULL;
}
