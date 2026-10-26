/*
 * Copyright (C) 2026 PortaPack Mayhem
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include "ui_walkie_rx.hpp"

#include "audio.hpp"
#include "baseband_api.hpp"
#include "i2s.hpp"
#include "portapack_persistent_memory.hpp"
#include "string_format.hpp"

#include <iterator>

using namespace portapack;

namespace ui::external_app::walkie_rx {

namespace {

enum ChannelLabel : uint8_t {
    LABEL_NUMBER,    // "07"
    LABEL_PLAIN,     // "7"
    LABEL_WX,        // "WX7"
    LABEL_REPEATER,  // "15R", GMRS repeater input
    LABEL_A,         // "22A", US marine simplex
};

// NFM configs, see nbfm_configs in receiver_model.cpp.
constexpr uint8_t NFM_8K5 = 0;   // 2.5 kHz deviation, 10 kHz channels
constexpr uint8_t NFM_11K = 1;   // 2.5 kHz deviation
constexpr uint8_t NFM_12K5 = 2;  // 2.5 kHz deviation, 12.5 kHz channels
constexpr uint8_t NFM_16K = 3;   // 5 kHz deviation, 20/25 kHz channels
// AM config, see am_configs in receiver_model.cpp.
constexpr uint8_t AM_DSB_6K = 1;
// The block uses the bandwidth of its preset.
constexpr uint8_t BW_PRESET = 0xFF;

constexpr char bandwidth_names[][16] = {
    "NFM 8k5",
    "NFM 11k",
    "NFM 12k5 narrow",
    "NFM 16k wide",
    "AM 9k",
    "AM 6k",
};
constexpr uint8_t AM_NAME_OFFSET = 4;

// CTCSS tones in 0.1 Hz, in EIA code order: the 38 privacy codes used by FRS,
// GMRS and PMR446 radios, then the extended tones numbered as in tone_key.cpp.
constexpr uint16_t ctcss_tones[] = {
    670, 719, 744, 770, 797, 825, 854, 885, 915, 948,
    974, 1000, 1035, 1072, 1109, 1148, 1188, 1230, 1273, 1318,
    1365, 1413, 1462, 1514, 1567, 1622, 1679, 1738, 1799, 1862,
    1928, 2035, 2107, 2181, 2257, 2336, 2418, 2503,
    693, 1598, 1655, 1713, 1773, 1835, 1899, 1966, 1995, 2065, 2291, 2541};
constexpr uint8_t ctcss_count = std::size(ctcss_tones);

// A factory tone is a CTCSS code number, or a DCS code written with its octal digits.
constexpr uint16_t DCS_FLAG = 0x8000;
constexpr uint16_t DCS_INVERTED = 0x4000;
constexpr uint16_t DCS_CODE_MASK = 0x03FF;

constexpr uint16_t dcs(uint16_t octal_digits, bool inverted = false) {
    return DCS_FLAG | (inverted ? DCS_INVERTED : 0) | octal_digits;
}

/* Channel plans *********************************************************/

// FRS: all 22 channels are 12.5 kHz since the 2017 FCC rules.
constexpr ChannelBlock frs_blocks[] = {
    {462562500, 25000, 7, 1, BW_PRESET, LABEL_NUMBER},
    {467562500, 25000, 7, 8, BW_PRESET, LABEL_NUMBER},
    {462550000, 25000, 8, 15, BW_PRESET, LABEL_NUMBER},
};

// GMRS: wide on the 462 MHz channels, the 467 MHz interstitials stay narrow.
constexpr ChannelBlock gmrs_blocks[] = {
    {462562500, 25000, 7, 1, NFM_16K, LABEL_NUMBER},
    {467562500, 25000, 7, 8, NFM_12K5, LABEL_NUMBER},
    {462550000, 25000, 8, 15, NFM_16K, LABEL_NUMBER},
    {467550000, 25000, 8, 15, NFM_16K, LABEL_REPEATER},
};

constexpr ChannelBlock pmr446_blocks[] = {
    {446006250, 12500, 16, 1, BW_PRESET, LABEL_NUMBER},
};

// MURS: 1-3 are limited to 11.25 kHz, 4-5 may be 20 kHz wide.
constexpr ChannelBlock murs_blocks[] = {
    {151820000, 60000, 3, 1, NFM_11K, LABEL_NUMBER},
    {154570000, 30000, 2, 4, NFM_16K, LABEL_NUMBER},
};

// 40 channel CB, shared by the AM and FM presets. Channel 23 sits above 24 and 25.
constexpr ChannelBlock cb40_blocks[] = {
    {26965000, 10000, 3, 1, BW_PRESET, LABEL_NUMBER},
    {27005000, 10000, 4, 4, BW_PRESET, LABEL_NUMBER},
    {27055000, 10000, 4, 8, BW_PRESET, LABEL_NUMBER},
    {27105000, 10000, 4, 12, BW_PRESET, LABEL_NUMBER},
    {27155000, 10000, 4, 16, BW_PRESET, LABEL_NUMBER},
    {27205000, 10000, 3, 20, BW_PRESET, LABEL_NUMBER},
    {27255000, 0, 1, 23, BW_PRESET, LABEL_NUMBER},
    {27235000, 10000, 2, 24, BW_PRESET, LABEL_NUMBER},
    {27265000, 10000, 15, 26, BW_PRESET, LABEL_NUMBER},
};

constexpr ChannelBlock cb_uk_blocks[] = {
    {27601250, 10000, 40, 1, BW_PRESET, LABEL_NUMBER},
};

// 80 channel plan: 41-80 sit halfway between 1-40.
constexpr ChannelBlock uhf_cb_blocks[] = {
    {476425000, 25000, 40, 1, BW_PRESET, LABEL_NUMBER},
    {476437500, 25000, 40, 41, BW_PRESET, LABEL_NUMBER},
};

constexpr ChannelBlock lpd433_blocks[] = {
    {433075000, 25000, 69, 1, BW_PRESET, LABEL_NUMBER},
};

// Simplex voice channels, ship side of the international plan.
constexpr ChannelBlock marine_blocks[] = {
    {156300000, 0, 1, 6, BW_PRESET, LABEL_NUMBER},
    {156400000, 50000, 10, 8, BW_PRESET, LABEL_NUMBER},
    {157100000, 0, 1, 22, BW_PRESET, LABEL_A},
    {156375000, 50000, 3, 67, BW_PRESET, LABEL_NUMBER},
    {156575000, 50000, 4, 71, BW_PRESET, LABEL_NUMBER},
    {156875000, 0, 1, 77, BW_PRESET, LABEL_NUMBER},
};

// In WX number order, which is not frequency order.
constexpr ChannelBlock noaa_blocks[] = {
    {162550000, 0, 1, 1, BW_PRESET, LABEL_WX},
    {162400000, 0, 1, 2, BW_PRESET, LABEL_WX},
    {162475000, 0, 1, 3, BW_PRESET, LABEL_WX},
    {162425000, 0, 1, 4, BW_PRESET, LABEL_WX},
    {162450000, 0, 1, 5, BW_PRESET, LABEL_WX},
    {162500000, 0, 1, 6, BW_PRESET, LABEL_WX},
    {162525000, 0, 1, 7, BW_PRESET, LABEL_WX},
};

// Part 90 itinerant channels are narrow; the MURS and GMRS dots may be wide.
constexpr ChannelBlock business_blocks[] = {
    {151625000, 0, 1, 1, NFM_12K5, LABEL_NUMBER},
    {151955000, 0, 1, 2, NFM_12K5, LABEL_NUMBER},
    {154570000, 30000, 2, 3, NFM_16K, LABEL_NUMBER},
    {462575000, 50000, 3, 5, NFM_16K, LABEL_NUMBER},
    {464500000, 50000, 2, 8, NFM_12K5, LABEL_NUMBER},
    {467762500, 50000, 2, 10, NFM_12K5, LABEL_NUMBER},
    {467850000, 25000, 4, 12, NFM_12K5, LABEL_NUMBER},
};
constexpr char business_names[][14] = {
    "Red Dot",
    "Purple Dot",
    "Blue Dot",
    "Green Dot",
    "White Dot",
    "Black Dot",
    "Orange Dot",
    "Brown Dot",
    "Yellow Dot",
    "J Dot",
    "K Dot",
    "Silver Star",
    "Gold Star",
    "Red Star",
    "Blue Star",
};

// Baofeng BF-888S as shipped (most common factory table, wide).
constexpr ChannelBlock bf888s_blocks[] = {
    {462125000, 100000, 12, 1, BW_PRESET, LABEL_NUMBER},
    {463525000, 0, 1, 13, BW_PRESET, LABEL_NUMBER},
    {450225000, 0, 1, 14, BW_PRESET, LABEL_NUMBER},
    {460325000, 0, 1, 15, BW_PRESET, LABEL_NUMBER},
    {469950000, 0, 1, 16, BW_PRESET, LABEL_NUMBER},
};
constexpr uint16_t bf888s_tones[] = {
    39, 0, 0, 13, 16, 19, 21, 26,
    dcs(25), dcs(51), dcs(125), dcs(155, true), dcs(465, true), dcs(23), 0, 32};

constexpr ChannelBlock toy49_blocks[] = {
    {49830000, 15000, 5, 1, BW_PRESET, LABEL_NUMBER},
};

// Japanese specified low power radio: simplex channels 1-9, then 01-11.
constexpr ChannelBlock japan_blocks[] = {
    {422200000, 12500, 9, 1, BW_PRESET, LABEL_PLAIN},
    {422050000, 12500, 11, 1, BW_PRESET, LABEL_NUMBER},
};

constexpr ChannelBlock china_blocks[] = {
    {409750000, 12500, 20, 1, BW_PRESET, LABEL_NUMBER},
};

constexpr ChannelBlock ham_blocks[] = {
    {145500000, 0, 1, 1, BW_PRESET, LABEL_NUMBER},
    {146520000, 0, 1, 2, BW_PRESET, LABEL_NUMBER},
    {433500000, 0, 1, 3, BW_PRESET, LABEL_NUMBER},
    {446000000, 0, 1, 4, BW_PRESET, LABEL_NUMBER},
};
constexpr char ham_names[][14] = {
    "EU 2m call",
    "US 2m call",
    "EU 70cm call",
    "US 70cm call",
};

constexpr WalkiePreset presets[] = {
    {"FRS", "US/CA 22ch 462/467MHz, narrow", false, NFM_12K5, std::size(frs_blocks), frs_blocks, nullptr, nullptr},
    {"GMRS", "US 22ch + 8 repeater inputs", false, NFM_16K, std::size(gmrs_blocks), gmrs_blocks, nullptr, nullptr},
    {"PMR446", "EU/UK/IN 16ch 446MHz, narrow", false, NFM_12K5, std::size(pmr446_blocks), pmr446_blocks, nullptr, nullptr},
    {"MURS", "US 5ch VHF 151/154MHz", false, NFM_11K, std::size(murs_blocks), murs_blocks, nullptr, nullptr},
    {"CB 40 AM", "US/EU 40ch 27MHz, AM", true, AM_DSB_6K, std::size(cb40_blocks), cb40_blocks, nullptr, nullptr},
    {"CB 40 FM", "EU CEPT 40ch 27MHz, FM", false, NFM_8K5, std::size(cb40_blocks), cb40_blocks, nullptr, nullptr},
    {"UK CB 27/81", "UK 40ch 27.6MHz, FM", false, NFM_8K5, std::size(cb_uk_blocks), cb_uk_blocks, nullptr, nullptr},
    {"UHF CB AU/NZ", "AU/NZ 80ch 477MHz, narrow", false, NFM_12K5, std::size(uhf_cb_blocks), uhf_cb_blocks, nullptr, nullptr},
    {"LPD433", "EU 69ch 433MHz, wide", false, NFM_16K, std::size(lpd433_blocks), lpd433_blocks, nullptr, nullptr},
    {"Marine VHF", "Simplex voice ch, 16 = call", false, NFM_16K, std::size(marine_blocks), marine_blocks, nullptr, nullptr},
    {"NOAA Weather", "US/CA WX1-WX7 162MHz", false, NFM_16K, std::size(noaa_blocks), noaa_blocks, nullptr, nullptr},
    {"Business Dots", "US itinerant color dot freqs", false, NFM_12K5, std::size(business_blocks), business_blocks, nullptr, business_names},
    {"BF-888S", "Baofeng factory 16ch + tones", false, NFM_16K, std::size(bf888s_blocks), bf888s_blocks, bf888s_tones, nullptr},
    {"49MHz Toys", "US 5ch 49.8MHz kids radios", false, NFM_12K5, std::size(toy49_blocks), toy49_blocks, nullptr, nullptr},
    {"Japan 422", "JP license-free 20ch simplex", false, NFM_12K5, std::size(japan_blocks), japan_blocks, nullptr, nullptr},
    {"China 409", "CN public PRS 20ch 409MHz", false, NFM_12K5, std::size(china_blocks), china_blocks, nullptr, nullptr},
    {"Ham Calling", "2m/70cm FM simplex calling", false, NFM_16K, std::size(ham_blocks), ham_blocks, nullptr, ham_names},
};
constexpr uint8_t preset_count = std::size(presets);

/* Squelch and scan timing, in 100 ms audio statistics ticks *************/

constexpr int32_t silent_audio_db = -100;  // squelched audio is all zeros, far below this
constexpr uint8_t carrier_lost_ticks = 3;  // a fade shorter than this keeps the tone squelch open
constexpr uint8_t tone_lost_ticks = 6;     // tone measurements arrive about every 150 ms
constexpr uint8_t tone_lost_measurements = 3;
constexpr uint16_t tone_steady_dhz = 15;  // consecutive measurements of a real tone agree this well
constexpr uint8_t scan_dwell_ticks = 2;
constexpr uint8_t scan_tone_ticks = 8;  // a busy channel gets time for the tone decoder
constexpr uint8_t scan_resume_ticks = 20;

enum Status : uint8_t {
    STATUS_STANDBY,
    STATUS_RECEIVING,
    STATUS_BUSY,
    STATUS_SCANNING,
};

/* Helpers ****************************************************************/

struct Channel {
    uint32_t frequency;
    uint8_t number;
    uint8_t bandwidth;
    uint8_t label;
};

uint8_t channel_count(const WalkiePreset& preset) {
    uint8_t count = 0;
    for (uint8_t i = 0; i < preset.block_count; i++)
        count += preset.blocks[i].count;
    return count;
}

Channel channel_at(const WalkiePreset& preset, size_t index) {
    for (uint8_t i = 0; i < preset.block_count; i++) {
        const auto& block = preset.blocks[i];
        if (index < block.count) {
            return {
                block.first_hz + index * block.step_hz,
                (uint8_t)(block.first_number + index),
                block.bandwidth == BW_PRESET ? preset.bandwidth : block.bandwidth,
                block.label};
        }
        index -= block.count;
    }
    return {preset.blocks[0].first_hz, preset.blocks[0].first_number, preset.bandwidth, LABEL_NUMBER};
}

std::string channel_label(const Channel& channel) {
    switch (channel.label) {
        case LABEL_PLAIN:
            return to_string_dec_uint(channel.number);
        case LABEL_WX:
            return "WX" + to_string_dec_uint(channel.number);
        case LABEL_REPEATER:
            return to_string_dec_uint(channel.number) + "R";
        case LABEL_A:
            return to_string_dec_uint(channel.number) + "A";
        default:
            return to_string_dec_uint(channel.number, 2, '0');
    }
}

// MHz with 3 to 5 decimals, as channel plans write them: 462.5625, 446.00625, 27.185.
std::string frequency_string(uint32_t hz) {
    uint32_t fraction = (hz % 1000000) / 10;
    int32_t digits = 5;
    while (digits > 3 && fraction % 10 == 0) {
        fraction /= 10;
        digits--;
    }
    return to_string_dec_uint(hz / 1000000) + "." + to_string_dec_uint(fraction, digits, '0') + " MHz";
}

std::string code_label(uint8_t code) {
    if (code == 0)
        return "Off";
    const auto tone = ctcss_tones[code - 1];
    return to_string_dec_uint(code) + " " + to_string_dec_uint(tone / 10) + "." + to_string_dec_uint(tone % 10) + "Hz";
}

std::string dcs_label(uint16_t tone) {
    return "DCS " + to_string_dec_uint(tone & DCS_CODE_MASK, 3, '0') + ((tone & DCS_INVERTED) ? "I" : "N");
}

// The code whose tone is nearest to a measured one, or 0 when none is close.
uint8_t code_for_tone(uint16_t tone) {
    uint8_t code = 0;
    int32_t best = INT32_MAX;
    for (uint8_t i = 0; i < ctcss_count; i++) {
        const int32_t diff = abs((int32_t)tone - (int32_t)ctcss_tones[i]);
        if (diff < best) {
            best = diff;
            code = i + 1;
        }
    }
    // About 1.5 % plus rounding; the nearest tone decides between close neighbours.
    return best <= ctcss_tones[code - 1] / 64 + 5 ? code : 0;
}

}  // namespace

/* LabelField *************************************************************/

LabelField::LabelField(Point parent_pos, int length, range_t range)
    : NumberField{parent_pos, length, range, 1, ' ', true},
      length_{(size_t)length} {
}

void LabelField::set_label(std::string label) {
    label.resize(length_, ' ');
    if (label != label_) {
        label_ = std::move(label);
        set_dirty();
    }
}

void LabelField::paint(Painter& painter) {
    painter.draw_string(
        screen_pos(),
        has_focus() ? style().invert() : style(),
        label_);
}

/* WalkieRxView ***********************************************************/

WalkieRxView::WalkieRxView(NavigationView& nav)
    : nav_{nav} {
    add_children({
        &labels,
        &field_squelch,
        &text_monitor,
        &field_rf_amp,
        &field_lna,
        &field_vga,
        &rssi,
        &field_volume,
        &field_preset,
        &text_channel_count,
        &text_info,
        &field_channel,
        &text_channel_name,
        &big_frequency,
        &text_frequency,
        &field_code,
        &text_code_note,
        &text_heard,
        &text_status,
        &button_scan,
        &button_monitor,
    });

    text_monitor.set_style(Theme::getInstance()->fg_yellow);
    text_info.set_style(Theme::getInstance()->fg_light);
    text_code_note.set_style(Theme::getInstance()->fg_yellow);

    field_preset.set_range(0, preset_count - 1);
    field_preset.on_change = [this](int32_t v) {
        select_preset(v, false);
    };

    field_channel.on_change = [this](int32_t v) {
        set_scanning(false);
        select_channel(v, true);
    };

    field_code.set_range(0, ctcss_count);
    field_code.on_change = [this](int32_t v) {
        select_code(v);
    };

    field_squelch.on_change = [this](int32_t) {
        if (monitor_)
            set_monitor(false);
        else
            apply_squelch();
    };

    button_scan.on_select = [this](Button&) {
        set_scanning(!scanning_);
    };

    button_monitor.on_select = [this](Button&) {
        set_monitor(!monitor_);
    };

    if (preset_index_ >= preset_count)
        preset_index_ = 0;
    if (ctcss_code_ > ctcss_count)
        ctcss_code_ = 0;
    select_preset(preset_index_, true);
}

WalkieRxView::~WalkieRxView() {
    // Save the squelch the user set, not the one Monitor opened.
    if (monitor_) {
        monitor_ = false;
        apply_squelch();
    }
    stop_receiver();
}

void WalkieRxView::focus() {
    field_channel.focus();
}

void WalkieRxView::select_preset(size_t index, bool keep_channel) {
    const auto& preset = presets[index];

    set_scanning(false);
    if (monitor_)
        set_monitor(false);

    preset_index_ = index;
    channel_count_ = channel_count(preset);

    field_preset.set_value(index, false);
    field_preset.set_label(preset.name);
    text_info.set(preset.info);
    text_channel_count.set(to_string_dec_uint(channel_count_) + " ch");

    if (!receiver_started_ || preset.am != am_)
        start_receiver(preset.am);

    // Each mode keeps its own squelch level in the receiver model.
    field_squelch.set_value(am_ ? receiver_model.am_squelch_level() : receiver_model.squelch_level(), false);
    // AM radios have no tone squelch.
    field_code.set_focusable(!am_);

    if (!keep_channel) {
        // A stock radio starts on channel 1 with the privacy code off.
        channel_index_ = 0;
        ctcss_code_ = 0;
    }
    if (channel_index_ >= channel_count_)
        channel_index_ = 0;
    field_channel.set_range(0, channel_count_ - 1);
    select_channel(channel_index_, !keep_channel);
}

void WalkieRxView::start_receiver(bool am) {
    stop_receiver();
    am_ = am;

    baseband::run_image(am ? portapack::spi_flash::image_tag_am_audio : portapack::spi_flash::image_tag_nfm_audio);
    receiver_model.set_modulation(am ? ReceiverModel::Mode::AMAudio : ReceiverModel::Mode::NarrowbandFMAudio);
    receiver_model.set_sampling_rate(3072000);
    receiver_model.set_baseband_bandwidth(1750000);
    bandwidth_ = am ? receiver_model.am_configuration() : receiver_model.nbfm_configuration();

    audio::output::start();
    receiver_model.set_headphone_volume(receiver_model.headphone_volume());  // WM8731 hack
    receiver_model.enable();
    receiver_started_ = true;
}

void WalkieRxView::stop_receiver() {
    if (!receiver_started_)
        return;

    audio::output::stop();
    receiver_model.disable();
    baseband::shutdown();
    receiver_started_ = false;
}

void WalkieRxView::select_channel(size_t index, bool load_factory_tone) {
    const auto& preset = presets[preset_index_];
    const auto channel = channel_at(preset, index);
    channel_index_ = index;

    if (channel.bandwidth != bandwidth_) {
        bandwidth_ = channel.bandwidth;
        if (am_)
            receiver_model.set_am_configuration(bandwidth_);
        else
            receiver_model.set_nbfm_configuration(bandwidth_);
    }
    receiver_model.set_target_frequency(channel.frequency);
    ticks_since_tune_ = 0;
    carrier_ = false;

    field_channel.set_value(index, false);
    field_channel.set_label(channel_label(channel));
    text_channel_name.set(preset.names ? preset.names[index] : "");
    big_frequency.set(channel.frequency);
    text_frequency.set(frequency_string(channel.frequency) + " " +
                       bandwidth_names[am_ ? AM_NAME_OFFSET + bandwidth_ : bandwidth_]);

    // DCS can't be decoded from the baseband's tone measurements, so it is only shown.
    const uint16_t factory_tone = preset.tones ? preset.tones[index] : 0;
    text_code_note.set((factory_tone & DCS_FLAG) ? dcs_label(factory_tone) : "");
    if (load_factory_tone && preset.tones)
        ctcss_code_ = (factory_tone & DCS_FLAG) ? 0 : factory_tone;

    select_code(am_ ? 0 : ctcss_code_);
}

void WalkieRxView::select_code(uint8_t code) {
    ctcss_code_ = code;
    field_code.set_value(code, false);
    field_code.set_label(am_ ? "n/a" : code_label(code));
    reset_tone_state();
}

void WalkieRxView::set_monitor(bool enabled) {
    if (enabled)
        set_scanning(false);

    monitor_ = enabled;
    text_monitor.set(enabled ? "MON" : "");
    button_monitor.set_text(enabled ? "Monitor off" : "Monitor");
    apply_squelch();
    update_audio_gate();
}

void WalkieRxView::set_scanning(bool enabled) {
    if (enabled && monitor_)
        set_monitor(false);

    scanning_ = enabled;
    scan_held_ = false;
    scan_hold_ticks_ = 0;
    button_scan.set_text(enabled ? "Stop scan" : "Scan");
}

void WalkieRxView::apply_squelch() {
    const uint8_t level = monitor_ ? 0 : field_squelch.value();
    if (am_)
        receiver_model.set_am_squelch_level(level);
    else
        receiver_model.set_squelch_level(level);
}

void WalkieRxView::reset_tone_state() {
    tone_open_ = false;
    tone_misses_ = 0;
    tone_age_ = 0;
    last_tone_ = 0;
    heard_code_ = 0;
    update_heard_text();
    update_audio_gate();
}

void WalkieRxView::set_tone_open(bool open) {
    if (open != tone_open_) {
        tone_open_ = open;
        update_audio_gate();
    }
}

bool WalkieRxView::audio_gate_open() const {
    return monitor_ || am_ || ctcss_code_ == 0 || tone_open_;
}

void WalkieRxView::update_audio_gate() {
    // Only the I2S output is muted: switching the codec outputs on and off pops.
    if (audio_gate_open() && !persistent_memory::config_audio_mute())
        lpc43xx::i2s::i2s0::tx_unmute();
    else
        lpc43xx::i2s::i2s0::tx_mute();
}

void WalkieRxView::update_heard_text() {
    if (heard_code_)
        text_heard.set(code_label(heard_code_));
    else if (carrier_ && !am_)
        text_heard.set("no tone");
    else
        text_heard.set("--");
}

void WalkieRxView::update_status() {
    const bool heard = carrier_ && audio_gate_open();
    Status status = STATUS_STANDBY;
    if (heard)
        status = STATUS_RECEIVING;
    else if (carrier_)
        status = STATUS_BUSY;
    else if (scanning_)
        status = STATUS_SCANNING;

    if (status == status_)
        return;
    status_ = status;

    switch (status) {
        case STATUS_RECEIVING:
            text_status.set_style(Theme::getInstance()->fg_green);
            text_status.set(" Receiving");
            break;
        case STATUS_BUSY:
            text_status.set_style(Theme::getInstance()->fg_orange);
            text_status.set(" Busy - other code");
            break;
        case STATUS_SCANNING:
            text_status.set_style(Theme::getInstance()->fg_yellow);
            text_status.set(" Scanning...");
            break;
        default:
            text_status.set_style(Theme::getInstance()->fg_light);
            text_status.set(" Standby");
            break;
    }
    big_frequency.set_style(heard ? Theme::getInstance()->fg_green : nullptr);
}

void WalkieRxView::on_tone_measured(uint32_t value) {
    if (am_)
        return;

    // value is in 0.01 Hz. Noise gives scattered values, a real tone a steady one.
    const uint16_t tone = std::min<uint32_t>((value + 5) / 10, UINT16_MAX);
    const bool steady = carrier_ && last_tone_ != 0 &&
                        (uint16_t)abs((int32_t)tone - (int32_t)last_tone_) <= tone_steady_dhz;
    last_tone_ = tone;
    tone_age_ = 0;

    const uint8_t code = steady ? code_for_tone(tone) : 0;
    if (code != heard_code_) {
        heard_code_ = code;
        update_heard_text();
    }

    if (ctcss_code_ == 0)
        return;

    if (code == ctcss_code_) {
        tone_misses_ = 0;
        set_tone_open(true);
    } else if (++tone_misses_ >= tone_lost_measurements) {
        set_tone_open(false);
    }
}

void WalkieRxView::on_audio_level(const AudioStatistics& statistics) {
    // The first report after a retune still holds audio of the previous channel.
    const bool settled = ticks_since_tune_ != 0;
    if (ticks_since_tune_ < UINT8_MAX)
        ticks_since_tune_++;
    if (!settled)
        return;

    if (tone_age_ < UINT8_MAX)
        tone_age_++;

    const bool carrier = statistics.max_db > silent_audio_db;
    if (carrier)
        quiet_ticks_ = 0;
    else if (quiet_ticks_ < UINT8_MAX)
        quiet_ticks_++;

    if (carrier != carrier_) {
        carrier_ = carrier;
        update_heard_text();
    }

    // The tone squelch closes with the carrier, so every transmission is checked anew.
    if (quiet_ticks_ >= carrier_lost_ticks || tone_age_ >= tone_lost_ticks) {
        set_tone_open(false);
        tone_misses_ = 0;
        if (heard_code_) {
            heard_code_ = 0;
            update_heard_text();
        }
    }

    // Also undoes an unmute from the status bar speaker button while the gate is shut.
    update_audio_gate();

    if (scanning_) {
        if (carrier_ && audio_gate_open()) {
            scan_held_ = true;
            scan_hold_ticks_ = 0;
        } else if (scan_held_ && ++scan_hold_ticks_ >= scan_resume_ticks) {
            scan_held_ = false;
        }

        if (!scan_held_) {
            const uint8_t dwell = (carrier_ && ctcss_code_) ? scan_tone_ticks : scan_dwell_ticks;
            if (ticks_since_tune_ >= dwell)
                select_channel((channel_index_ + 1) % channel_count_, true);
        }
    }

    update_status();
}

}  // namespace ui::external_app::walkie_rx
