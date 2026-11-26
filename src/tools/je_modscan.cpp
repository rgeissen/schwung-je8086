/*
 * je_modscan — where the JP-8000 has its modulated controls now (research
 * for Pung's live modulation; never shipped).
 *
 * The H8S runs the envelopes and LFOs and writes the ASICs' coefficients
 * (if_mode 0x55: a 20-bit coefficient into two program words) at control
 * rate. This tool boots the JE-8086, sets a sound through the temp patch's
 * DT1s, plays a scripted session and logs every committed coefficient:
 *
 *   je_modscan <rom_dir> <mode> [out.csv] [key=value ...]
 *
 *   modes: lfo     a held note, LFO 1 on the cutoff
 *          knob    a held note, the cutoff knob stepped 0..127, no modulation
 *          voices  notes started one at a time, a filter envelope each
 *          env     one note, a filter envelope, released
 *          hold    just the keys given, a held note (key=value set first)
 *
 * key=value: any patch key of jp8000_params.h (displayed units), set on
 * both temp patches before the session.
 *
 * CSV: sample,asic,addr,coef (addr: the program word, coef: 20 bits) and
 * "# mark" lines where the session does something.
 */

#include <cstdio>
#include <cstring>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>
#include <array>
#include <unistd.h>

#include "jeLib/device.h"
#include "jeLib/je8086.h"
#include "jeLib/je8086devices.h"
#include "jeLib/romloader.h"
#include "jeLib/sysexRemoteControl.h"
#include "synthLib/midiTypes.h"
#include "synthLib/romLoader.h"
#include "synthLib/audioTypes.h"

#include "../dsp/jp8000_params.h"

using namespace jeLib;

static FILE *g_out = nullptr;
static int64_t g_sample = 0;
static uint8_t g_mode[4];
static uint8_t g_word[4][4];

static void capture(int asic, uint32_t address, uint8_t value) {
    address &= 0x3fff;
    g_word[asic][address & 3] = value;
    if (address == 0x2003) { g_mode[asic] = value; return; }
    if ((address & 3) != 3) return;
    const uint8_t *w = g_word[asic];
    if (g_mode[asic] == 0x55) {
        const uint32_t lo = w[0] | ((w[1] & 3u) << 8);
        const uint32_t hi = ((w[1] >> 2) & 0x3fu) | ((w[2] & 0xfu) << 6);
        fprintf(g_out, "%lld,%d,%u,%u\n", (long long)g_sample, asic, address >> 2, lo | (hi << 10));
    }
}

struct Session {
    Device &dev;
    std::vector<synthLib::SMidiEvent> in, out;
    SysexRemoteControl remote;
    explicit Session(Device &d) : dev(d) {}
    void run(int blocks) {
        constexpr size_t bs = 128;
        static std::array<std::vector<float>, 2> bufs;
        synthLib::TAudioInputs inputs{};
        synthLib::TAudioOutputs outputs{};
        for (size_t i = 0; i < 2; ++i) { bufs[i].resize(bs); outputs[i] = bufs[i].data(); }
        for (int b = 0; b < blocks; ++b) {
            dev.process(inputs, outputs, bs, in, out);
            g_sample += bs;
            in.clear();
            for (const auto &e : out) remote.receive(e);
            out.clear();
        }
    }
    void ms(int m) { run((int)(m * 88.2 / 128) + 1); }
    void on(int n, int v = 100, int ch = 0) { in.emplace_back(synthLib::MidiEventSource::Host, (uint8_t)(0x90 | ch), (uint8_t)n, (uint8_t)v); }
    void off(int n, int ch = 0) { in.emplace_back(synthLib::MidiEventSource::Host, (uint8_t)(0x80 | ch), (uint8_t)n, (uint8_t)0); }
    void sysex(const std::vector<uint8_t> &m) {
        synthLib::SMidiEvent e(synthLib::MidiEventSource::Host);
        e.sysex.assign(m.begin(), m.end());
        in.push_back(e);
    }
    /* a patch key in displayed units, on both temp patches */
    bool set(const char *key, int value) {
        for (int i = 0; i < JP_PARAM_COUNT; i++) {
            const jp_param_t *p = &jp_params[i];
            if (strcmp(p->key, key) != 0) continue;
            int raw = p->neg ? -value : value + p->off;
            std::vector<uint32_t> blocks;
            switch (p->area) {
            case JP_AREA_PATCH: blocks = {0x4000u, 0x4200u}; break;
            case JP_AREA_PART_UP: blocks = {0x1000u}; break;
            case JP_AREA_PART_LO: blocks = {0x1100u}; break;
            case JP_AREA_COMMON: blocks = {0x0000u}; break;
            default: fprintf(stderr, "%s: system key\n", key); return false;
            }
            for (uint32_t block : blocks) {
                uint32_t addr = 0x01000000u | block | (((p->lin / 128u) << 8) | (p->lin % 128u));
                std::vector<uint8_t> m = {0xF0, 0x41, 0x10, 0x00, 0x06, 0x12,
                    (uint8_t)((addr >> 24) & 0x7f), (uint8_t)((addr >> 16) & 0x7f), (uint8_t)((addr >> 8) & 0x7f), (uint8_t)(addr & 0x7f)};
                if (p->bit14) { m.push_back((raw >> 7) & 0x7f); m.push_back(raw & 0x7f); }
                else m.push_back(raw & 0x7f);
                unsigned s = 0;
                for (size_t k = 6; k < m.size(); k++) s += m[k];
                m.push_back((uint8_t)((128 - (s & 0x7f)) & 0x7f));
                m.push_back(0xF7);
                sysex(m);
            }
            run(4);
            return true;
        }
        fprintf(stderr, "%s: no such key\n", key);
        return false;
    }
};

static void mark(const char *what) { fprintf(g_out, "# %lld %s\n", (long long)g_sample, what); fflush(g_out); }

int main(int argc, char **argv) {
    if (argc < 3) { fprintf(stderr, "usage: %s <rom_dir> <mode> [out.csv] [key=value ...]\n", argv[0]); return 1; }
    const std::string mode = argv[2];
    g_out = fopen(argc >= 4 ? argv[3] : "modscan.csv", "w");
    char abspath[4096];
    std::string dir = realpath(argv[1], abspath) ? abspath : argv[1];
    if (chdir(argv[1]) != 0) return 1;
    synthLib::RomLoader::addSearchPath(dir);
    auto rom = RomLoader::findROM();
    if (!rom.isValid()) { fprintf(stderr, "no rom in %s\n", dir.c_str()); return 1; }

    synthLib::DeviceCreateParams params;
    params.romData = rom.getData();
    params.romName = rom.getName();
    /* the battery RAM is looked for at homePath + "/roms/ram_dump.bin":
     * the rom dir's parent (jp8000_render.cpp says why it matters) */
    params.homePath = dir.substr(0, dir.find_last_of('/'));
    params.hostSamplerate = 88200.0f;
    params.preferredSamplerate = 88200.0f;
    Device device(params);
    if (!device.isValid()) { fprintf(stderr, "device init failed\n"); return 1; }
    Session s(device);
    bool booted = false;
    s.remote.evLcdDdDataChanged.addListener([&](const std::array<char, 40> &lcd) {
        std::string t(lcd.begin(), lcd.end());
        if (t.find("PERFORM") != std::string::npos) booted = true;
    });
    for (int i = 0; i < 20000 && !booted; i++) s.run(1);
    fprintf(stderr, "[modscan] booted=%d after %lld samples\n", booted, (long long)g_sample);
    s.ms(500);

    /* a plain sound: no filter envelope, no key follow, no LFO on anything */
    s.set("filter_env_depth", 0); s.set("key_follow", 0);
    s.set("filter_lfo1_depth", 0); s.set("filter_lfo2_depth", 0);
    s.set("amp_lfo1_depth", 0); s.set("amp_lfo2_depth", 0);
    s.set("osc_lfo1_depth", 0); s.set("pitch_lfo2_depth", 0);
    s.set("resonance", 0); s.set("cutoff", 64);
    for (int i = 4; i < argc; i++) {
        std::string a = argv[i];
        auto eq = a.find('=');
        if (eq == std::string::npos) continue;
        s.set(a.substr(0, eq).c_str(), atoi(a.c_str() + eq + 1));
    }
    s.ms(300);

    devices::g_je_uc_write_capture = capture;
    mark("start");
    if (mode == "lfo") {
        s.set("lfo1_waveform", 1); s.set("lfo1_rate", 70); s.set("filter_lfo1_depth", 40);
        s.ms(200);
        mark("on 60"); s.on(60); s.ms(4000); mark("off 60"); s.off(60); s.ms(1500);
    } else if (mode == "knob") {
        mark("on 60"); s.on(60); s.ms(300);
        for (int k = 0; k <= 127; k++) {
            char b[32]; snprintf(b, sizeof b, "knob %d", k);
            s.set("cutoff", k); mark(b); s.ms(60);
        }
        mark("off 60"); s.off(60); s.ms(800);
    } else if (mode == "voices") {
        s.set("filter_env_depth", 50); s.set("filter_attack", 0); s.set("filter_decay", 60); s.set("filter_sustain", 0);
        s.set("cutoff", 30);
        s.ms(200);
        int notes[] = {48, 52, 55, 60, 64, 67, 72, 76, 79, 84};
        for (int n : notes) { char b[32]; snprintf(b, sizeof b, "on %d", n); mark(b); s.on(n); s.ms(400); }
        mark("off all"); for (int n : notes) s.off(n); s.ms(1500);
    } else if (mode == "parts") {
        s.set("filter_env_depth", 50); s.set("filter_attack", 0); s.set("filter_decay", 60); s.set("filter_sustain", 0);
        s.set("cutoff", 30);
        s.ms(200);
        for (int ch = 0; ch < 2; ch++)
            for (int i = 0; i < 6; i++) { char b[32]; snprintf(b, sizeof b, "on %d ch%d", 50 + i * 3 + ch, ch); mark(b); s.on(50 + i * 3 + ch, 100, ch); s.ms(400); }
        mark("off all"); for (int ch = 0; ch < 2; ch++) for (int i = 0; i < 6; i++) s.off(50 + i * 3 + ch, ch); s.ms(1500);
    } else if (mode == "assign") {
        /* every voice once: eight notes on each channel, a filter envelope on each */
        s.set("filter_env_depth", 50); s.set("filter_attack", 0); s.set("filter_decay", 60); s.set("filter_sustain", 0);
        s.set("cutoff", 30);
        s.ms(200);
        for (int ch = 0; ch < 2; ch++)
            for (int i = 0; i < 8; i++) { char b[32]; int n = 40 + i * 4 + ch * 2; snprintf(b, sizeof b, "on %d ch%d", n, ch); mark(b); s.on(n, 100, ch); s.ms(250); }
        mark("off all"); for (int ch = 0; ch < 2; ch++) for (int i = 0; i < 8; i++) s.off(40 + i * 4 + ch * 2, ch); s.ms(1500);
    } else if (mode == "pair") {
        /* one voice at a time: a note, released, silence; which cutoff word goes with which gate word */
        s.set("filter_env_depth", 50); s.set("filter_attack", 0); s.set("filter_decay", 40); s.set("filter_sustain", 0);
        s.set("filter_release", 0); s.set("amp_release", 0);
        s.set("cutoff", 30);
        s.ms(200);
        for (int ch = 0; ch < 2; ch++)
            for (int i = 0; i < 10; i++) { char b[32]; int n = 40 + i * 3 + ch; snprintf(b, sizeof b, "on %d ch%d", n, ch); mark(b); s.on(n, 100, ch); s.ms(300); s.off(n, ch); s.ms(900); }
    } else if (mode == "gate") {
        /* one note, held 1.5 s, released; then a retrigger while held (two notes, the second
         * stealing nothing), velocity low and high: how the voices' gate words move */
        mark("on 60"); s.on(60, 100); s.ms(1500); mark("off 60"); s.off(60); s.ms(1500);
        mark("on 62 v20"); s.on(62, 20); s.ms(800); mark("on 64 v127"); s.on(64, 127); s.ms(800);
        mark("off 62"); s.off(62); s.ms(300); mark("off 64"); s.off(64); s.ms(1500);
    } else if (mode == "patches") {
        /* program changes, LFO 1 on the cutoff in each: do the words stay? */
        for (int pc = 0; pc < 8; pc++) {
            char b[32]; snprintf(b, sizeof b, "pc %d", pc); mark(b);
            s.in.emplace_back(synthLib::MidiEventSource::Host, (uint8_t)0xC0, (uint8_t)pc, (uint8_t)0);
            s.ms(400);
            s.set("lfo1_rate", 80); s.set("filter_lfo1_depth", 50); s.set("filter_env_depth", 0); s.set("key_follow", 0);
            snprintf(b, sizeof b, "on 60 pc%d", pc); mark(b); s.on(60); s.ms(1200); s.off(60); s.ms(1500);
        }
    } else if (mode == "env") {
        s.set("filter_env_depth", 50); s.set("filter_attack", 20); s.set("filter_decay", 70); s.set("filter_sustain", 40); s.set("filter_release", 60);
        s.set("cutoff", 30);
        s.ms(200);
        mark("on 60"); s.on(60); s.ms(3000); mark("off 60"); s.off(60); s.ms(3000);
    } else {
        mark("on 60"); s.on(60); s.ms(3000); mark("off 60"); s.off(60); s.ms(1500);
    }
    mark("end");
    devices::g_je_uc_write_capture = nullptr;
    fclose(g_out);
    return 0;
}
