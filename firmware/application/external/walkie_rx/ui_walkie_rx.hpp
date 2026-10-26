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

/*
    Walkie RX: listens like a stock walkie-talkie. Each preset holds the channel
    plan of a common radio type (FRS, PMR446, CB, ...) with its default
    modulation and bandwidth, so picking a preset and a channel is all it takes.
    The privacy code is a CTCSS tone squelch, decoded from the tone measurements
    of the NFM baseband; DCS codes are shown but not decoded.
*/

#ifndef __UI_WALKIE_RX_H__
#define __UI_WALKIE_RX_H__

#include "ui.hpp"
#include "ui_navigation.hpp"
#include "ui_receiver.hpp"
#include "app_settings.hpp"
#include "radio_state.hpp"
#include "message.hpp"

namespace ui::external_app::walkie_rx {

/* One run of evenly spaced channels of a preset's channel plan. */
struct ChannelBlock {
    uint32_t first_hz;     // frequency of the first channel of the run
    uint32_t step_hz;      // spacing between the channels of the run
    uint8_t count;         // channels in the run
    uint8_t first_number;  // channel number shown for the first channel
    uint8_t bandwidth;     // receiver config index, or BW_PRESET
    uint8_t label;         // ChannelLabel
};

struct WalkiePreset {
    char name[14];  // inline arrays keep the text in the app image
    char info[31];
    bool am;
    uint8_t bandwidth;  // default nbfm (or am) config index of the preset
    uint8_t block_count;
    const ChannelBlock* blocks;
    const uint16_t* tones;    // factory tone of each channel, or nullptr
    const char (*names)[14];  // name of each channel, or nullptr
};

/* A NumberField that shows a text label instead of its value. */
class LabelField : public NumberField {
   public:
    LabelField(Point parent_pos, int length, range_t range);

    void set_label(std::string label);
    void paint(Painter& painter) override;

   private:
    size_t length_;
    std::string label_{};
};

class WalkieRxView : public View {
   public:
    WalkieRxView(NavigationView& nav);
    ~WalkieRxView();

    WalkieRxView(const WalkieRxView&) = delete;
    WalkieRxView(WalkieRxView&&) = delete;
    WalkieRxView& operator=(const WalkieRxView&) = delete;
    WalkieRxView& operator=(WalkieRxView&&) = delete;

    void focus() override;
    std::string title() const override { return "Walkie RX"; };

   private:
    NavigationView& nav_;
    RxRadioState radio_state_{};

    // Bound settings must be declared before settings_ so the loaded values stick.
    uint8_t preset_index_{0};
    uint8_t channel_index_{0};
    uint8_t ctcss_code_{0};

    app_settings::SettingsManager settings_{
        "rx_walkie",
        app_settings::Mode::RX,
        {
            {"preset"sv, &preset_index_},
            {"channel"sv, &channel_index_},
            {"code"sv, &ctcss_code_},
        }};

    bool receiver_started_{false};
    bool am_{false};
    uint8_t bandwidth_{0};
    uint8_t channel_count_{0};

    bool monitor_{false};
    bool scanning_{false};
    bool scan_held_{false};
    uint8_t scan_hold_ticks_{0};

    // Squelch and tone decoder state, advanced by the 10 Hz audio statistics.
    uint8_t ticks_since_tune_{0};
    uint8_t quiet_ticks_{0};
    uint8_t tone_age_{0};
    uint8_t tone_misses_{0};
    bool carrier_{false};
    bool tone_open_{false};
    uint16_t last_tone_{0};  // previous tone measurement, 0.1 Hz
    uint8_t heard_code_{0};
    uint8_t status_{0xFF};

    Labels labels{
        {{UI_POS_X(0), UI_POS_Y(0)}, "SQ", Theme::getInstance()->fg_light->foreground},
        {{UI_POS_X(0), UI_POS_Y(1)}, "Radio", Theme::getInstance()->fg_light->foreground},
        {{UI_POS_X(0), UI_POS_Y(3)}, "Ch", Theme::getInstance()->fg_light->foreground},
        {{UI_POS_X(0), UI_POS_Y(9)}, "Code", Theme::getInstance()->fg_light->foreground},
        {{UI_POS_X(0), UI_POS_Y(10)}, "Heard", Theme::getInstance()->fg_light->foreground},
    };

    NumberField field_squelch{
        {UI_POS_X(3), UI_POS_Y(0)},
        2,
        {0, 99},
        1,
        ' '};
    Text text_monitor{
        {UI_POS_X(6), UI_POS_Y(0), UI_POS_WIDTH(6), UI_POS_HEIGHT(1)},
        ""};
    RFAmpField field_rf_amp{
        {UI_POS_X(13), UI_POS_Y(0)}};
    LNAGainField field_lna{
        {UI_POS_X(15), UI_POS_Y(0)}};
    VGAGainField field_vga{
        {UI_POS_X(18), UI_POS_Y(0)}};
    RSSI rssi{
        {UI_POS_X(21), UI_POS_Y(0), UI_POS_WIDTH_REMAINING(24), 4}};
    AudioVolumeField field_volume{
        {UI_POS_X_RIGHT(2), UI_POS_Y(0)}};

    LabelField field_preset{
        {UI_POS_X(6), UI_POS_Y(1)},
        13,
        {0, 0}};
    Text text_channel_count{
        {UI_POS_X(21), UI_POS_Y(1), UI_POS_WIDTH_REMAINING(21), UI_POS_HEIGHT(1)},
        ""};
    Text text_info{
        {UI_POS_X(0), UI_POS_Y(2), UI_POS_MAXWIDTH, UI_POS_HEIGHT(1)},
        ""};

    LabelField field_channel{
        {UI_POS_X(3), UI_POS_Y(3)},
        3,
        {0, 0}};
    Text text_channel_name{
        {UI_POS_X(7), UI_POS_Y(3), UI_POS_WIDTH_REMAINING(7), UI_POS_HEIGHT(1)},
        ""};

    BigFrequency big_frequency{
        {UI_POS_X(0), UI_POS_Y(4) + 4, UI_POS_MAXWIDTH, 52},
        0};
    Text text_frequency{
        {UI_POS_X(0), UI_POS_Y(8), UI_POS_MAXWIDTH, UI_POS_HEIGHT(1)},
        ""};

    LabelField field_code{
        {UI_POS_X(6), UI_POS_Y(9)},
        10,
        {0, 0}};
    Text text_code_note{
        {UI_POS_X(17), UI_POS_Y(9), UI_POS_WIDTH_REMAINING(17), UI_POS_HEIGHT(1)},
        ""};
    Text text_heard{
        {UI_POS_X(6), UI_POS_Y(10), UI_POS_WIDTH_REMAINING(6), UI_POS_HEIGHT(1)},
        ""};
    Text text_status{
        {UI_POS_X(0), UI_POS_Y(11) + 4, UI_POS_MAXWIDTH, UI_POS_HEIGHT(1)},
        ""};

    Button button_scan{
        {UI_POS_X(1), UI_POS_Y(13), UI_POS_WIDTH(13), UI_POS_HEIGHT(2)},
        "Scan"};
    Button button_monitor{
        {UI_POS_X(16), UI_POS_Y(13), UI_POS_WIDTH(13), UI_POS_HEIGHT(2)},
        "Monitor"};

    void select_preset(size_t index, bool keep_channel);
    void start_receiver(bool am);
    void stop_receiver();
    void select_channel(size_t index, bool load_factory_tone);
    void select_code(uint8_t code);
    void set_monitor(bool enabled);
    void set_scanning(bool enabled);
    void apply_squelch();
    void reset_tone_state();
    void set_tone_open(bool open);
    void update_audio_gate();
    void update_heard_text();
    void update_status();
    bool audio_gate_open() const;

    void on_tone_measured(uint32_t value);
    void on_audio_level(const AudioStatistics& statistics);

    MessageHandlerRegistration message_handler_coded_squelch{
        Message::ID::CodedSquelch,
        [this](const Message* const p) {
            const auto message = *reinterpret_cast<const CodedSquelchMessage*>(p);
            this->on_tone_measured(message.value);
        }};

    MessageHandlerRegistration message_handler_audio_statistics{
        Message::ID::AudioStatistics,
        [this](const Message* const p) {
            this->on_audio_level(static_cast<const AudioStatisticsMessage*>(p)->statistics);
        }};
};

}  // namespace ui::external_app::walkie_rx

#endif /*__UI_WALKIE_RX_H__*/
