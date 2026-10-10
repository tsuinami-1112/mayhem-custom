/*
 * Copyright (C) 2016 Jared Boone, ShareBrained Technology, Inc.
 * Copyright (C) 2016 Furrtek
 * Copyright (C) 2020 Shao
 * Copyright (C) 2026 Multi-GNSS Sim
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

#ifndef __MGNSS_SIM_APP_HPP__
#define __MGNSS_SIM_APP_HPP__

#include "app_settings.hpp"
#include "ui_language.hpp"
#include "radio_state.hpp"
#include "ui_widget.hpp"
#include "ui_navigation.hpp"
#include "ui_receiver.hpp"
#include "ui_freq_field.hpp"
#include "replay_thread.hpp"
#include "ui_transmitter.hpp"

#include <string>
#include <memory>

namespace ui::external_app::mgnss_sim {

/* Replays a GNSS baseband file, in GPS Sim's C8 format or in the packed C2 and C1
 * formats that trade sample precision for bandwidth at the same ~5 MB/s off the
 * card. tools/gnss_sim/FORMAT.md is the contract with the generator.
 *
 * Deliberately no waterfall. GPS Sim carries one but never feeds its spectrum
 * collector, so it displays nothing; and since the PortaPack cannot receive while
 * transmitting, a TX waterfall would only show the file's own baseband, which the
 * PC-side acquisition check already covers. The rows here carry information that
 * cannot be had off the device instead: what the file claims to contain, the time
 * a receiver ought to be reporting, and whether the card kept up. */
class MultiGnssSimView : public View {
   public:
    MultiGnssSimView(NavigationView& nav);
    ~MultiGnssSimView();

    void focus() override;

    std::string title() const override { return "Multi-GNSS Sim"; };

   private:
    NavigationView& nav_;
    TxRadioState radio_state_{
        1575420000 /* frequency */,
        15000000 /* bandwidth */,
        2600000 /* sampling rate */
    };
    app_settings::SettingsManager settings_{
        "tx_mgnss", app_settings::Mode::TX};

    const size_t read_size{16384};
    const size_t buffer_count{3};

    void on_file_changed(const std::filesystem::path& new_file_path);
    void read_sim_metadata(const std::filesystem::path& metadata_path);
    void on_stats(uint32_t bytes_read, uint32_t underruns);
    void update_time(uint32_t bytes_read);
    uint64_t samples_from_bytes(uint64_t bytes) const;

    void toggle();
    void start();
    void stop(const bool do_loop);
    bool is_active() const;
    void set_ready();
    void handle_replay_thread_done(const uint32_t return_code);
    void file_error();

    std::filesystem::path file_path{};
    std::unique_ptr<ReplayThread> replay_thread{};
    bool ready_signal{false};

    /* From the metadata sidecar. 0 = C8, 1 = C2, 2 = C1, matching
     * GNSSSimConfigMessage and the generator's `format=` key. */
    uint8_t sim_format{0};
    std::string sim_gnss{};
    std::string start_date{};
    uint32_t start_seconds{0};
    bool have_start_time{false};
    uint32_t sample_rate{2600000};
    /* The stats message lands 50 times a second; the clock only needs redrawing
     * when its seconds digit moves. */
    uint32_t shown_second{UINT32_MAX};

    Button button_open{
        {UI_POS_X(0), UI_POS_Y(0), 10 * 8, 2 * 16},
        "Open file"};

    Text text_filename{
        {11 * 8, UI_POS_Y(0), 12 * 8, 16},
        "-"};
    Text text_sample_rate{
        {24 * 8, UI_POS_Y(0), 6 * 8, 16},
        "-"};

    Text text_duration{
        {11 * 8, 1 * 16, 6 * 8, 16},
        "-"};
    ProgressBar progressbar{
        {18 * 8, 1 * 16, UI_POS_WIDTH_REMAINING(18), 16}};

    TxFrequencyField field_frequency{
        {UI_POS_X(0), 2 * 16},
        nav_};

    TransmitterView2 tx_view{
        {11 * 8, 2 * 16},
        /*short_ui*/ true};

    Checkbox check_loop{
        {21 * 8, 2 * 16},
        4,
        LanguageHelper::currentMessages[LANG_LOOP],
        true};
    ImageButton button_play{
        {screen_width - 2 * 8, 2 * 16, 2 * 8, 1 * 16},
        &bitmap_play,
        Theme::getInstance()->fg_green->foreground,
        Theme::getInstance()->fg_green->background};

    Text text_gnss{
        {UI_POS_X(0), 3 * 16, screen_width, 16},
        "GNSS  -"};
    Text text_position{
        {UI_POS_X(0), 4 * 16, screen_width, 16},
        "POS   -"};
    Text text_time{
        {UI_POS_X(0), 5 * 16, screen_width, 16},
        "TIME  -"};
    Text text_drops{
        {UI_POS_X(0), 6 * 16, screen_width, 16},
        "DROPS -"};

    MessageHandlerRegistration message_handler_replay_thread_error{
        Message::ID::ReplayThreadDone,
        [this](const Message* const p) {
            const auto message = *reinterpret_cast<const ReplayThreadDoneMessage*>(p);
            this->handle_replay_thread_done(message.return_code);
        }};

    MessageHandlerRegistration message_handler_fifo_signal{
        Message::ID::RequestSignal,
        [this](const Message* const p) {
            const auto message = static_cast<const RequestSignalMessage*>(p);
            if (message->signal == RequestSignalMessage::Signal::FillRequest) {
                this->set_ready();
            }
        }};

    MessageHandlerRegistration message_handler_stats{
        Message::ID::GNSSSimStats,
        [this](const Message* const p) {
            const auto message = *reinterpret_cast<const GNSSSimStatsMessage*>(p);
            this->on_stats(message.bytes_read, message.underruns);
        }};
};

} /* namespace ui::external_app::mgnss_sim */

#endif /*__MGNSS_SIM_APP_HPP__*/
