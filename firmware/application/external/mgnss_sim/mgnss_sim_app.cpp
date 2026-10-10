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

#include "mgnss_sim_app.hpp"
#include "string_format.hpp"

#include "ui_fileman.hpp"
#include "io_file.hpp"
#include "file_reader.hpp"
#include "metadata_file.hpp"
#include "convert.hpp"
#include "utility.hpp"
#include "file_path.hpp"

#include "baseband_api.hpp"
#include "portapack.hpp"
#include "portapack_persistent_memory.hpp"

using namespace portapack;
namespace fs = std::filesystem;
using namespace std::literals;

namespace ui::external_app::mgnss_sim {

void MultiGnssSimView::set_ready() {
    ready_signal = true;
}

/* Samples per byte is the whole point of the packed formats: 0.5 for C8, 2 for
 * C2, 4 for C1. Returned as samples to keep this in integer maths, since the M0
 * has no FPU. */
uint64_t MultiGnssSimView::samples_from_bytes(uint64_t bytes) const {
    switch (sim_format) {
        case 2:
            return bytes * 4;
        case 1:
            return bytes * 2;
        default:
            return bytes / 2;
    }
}

/* The keys the shared read_metadata_file() does not know about. It is kept for
 * center_frequency and sample_rate, which it rejects the file without; the rest
 * is read here so capture_metadata, shared with Capture and Replay, need not
 * grow for one app. */
void MultiGnssSimView::read_sim_metadata(const fs::path& metadata_path) {
    sim_format = 0;
    sim_gnss.clear();
    start_date.clear();
    start_seconds = 0;
    have_start_time = false;

    File f;
    if (f.open(metadata_path))
        return;  // no sidecar: the defaults already describe a plain C8 file

    auto reader = FileLineReader(f);
    for (const auto& line : reader) {
        auto cols = split_string(line, '=');
        if (cols.size() != 2)
            continue;

        /* A CRLF sidecar -- which is what the firmware's own
         * write_metadata_file() emits -- leaves the CR on the value, because
         * read_line() splits on LF alone. */
        const auto key = cols[0];
        const auto value = trim(cols[1]);

        if (key == "format"sv) {
            if (value == "c1")
                sim_format = 2;
            else if (value == "c2")
                sim_format = 1;
            else
                sim_format = 0;
        } else if (key == "gnss"sv) {
            sim_gnss = value;
        } else if (key == "start_time"sv) {
            // "YYYY/MM/DD,hh:mm:ss"
            auto parts = split_string(value, ',');
            if (parts.size() == 2 && parts[1].size() >= 8) {
                uint32_t hh = 0;
                uint32_t mm = 0;
                uint32_t ss = 0;
                if (parse_int(parts[1].substr(0, 2), hh) &&
                    parse_int(parts[1].substr(3, 2), mm) &&
                    parse_int(parts[1].substr(6, 2), ss)) {
                    start_date = std::string{parts[0]};
                    start_seconds = (hh * 3600) + (mm * 60) + ss;
                    have_start_time = true;
                }
            }
        }
    }
}

void MultiGnssSimView::on_file_changed(const fs::path& new_file_path) {
    file_path = new_file_path;
    File::Size file_size{};

    {  // Get the size of the data file.
        File data_file;
        auto error = data_file.open(file_path);
        if (error) {
            file_error();
            return;
        }

        file_size = data_file.size();
    }

    auto metadata_path = get_metadata_path(file_path);
    read_sim_metadata(metadata_path);

    auto metadata = read_metadata_file(metadata_path);
    if (metadata) {
        field_frequency.set_value(metadata->center_frequency);
        transmitter_model.set_sampling_rate(metadata->sample_rate);
    }
    sample_rate = transmitter_model.sampling_rate();

    baseband::set_gnss_sim_format(sim_format);

    static constexpr const char* format_names[3] = {"C8", "C2", "C1"};
    const char* format_name = format_names[sim_format < 3 ? sim_format : 0];

    text_sample_rate.set(unit_auto_scale(sample_rate, 3, 1) + "Hz");
    text_filename.set(truncate(file_path.filename().string(), 12));

    /* ms_duration() takes whole bytes per sample, which the packed formats are
     * not, so the byte count is converted to samples first and fed as 1:1. */
    text_duration.set(to_string_time_ms(
        ms_duration(samples_from_bytes(file_size), sample_rate, 1)));

    /* Scaled to KiB: a FAT32 file reaches 4 GiB - 1, which only just fits a
     * uint32, and exFAT is allowed to go past it. */
    progressbar.set_max(file_size >> 10);
    progressbar.set_value(0);

    text_gnss.set("GNSS  " + (sim_gnss.empty() ? "-"s : sim_gnss) +
                  "   fmt " + format_name);

    if (metadata && metadata->latitude != 0 && metadata->longitude != 0) {
        text_position.set("POS   " + to_string_decimal(metadata->latitude, 4) +
                          " " + to_string_decimal(metadata->longitude, 4));
    } else {
        text_position.set("POS   -");
    }

    shown_second = UINT32_MAX;
    update_time(0);
    text_drops.set("DROPS 0");

    // TODO: fix in UI framework with 'try_focus()'?
    // Hack around focus getting called by ctor before parent is set.
    if (parent())
        button_play.focus();
}

/* The time a receiver should be reporting: the instant baked into the file plus
 * how far playback has got. Seeing it on the device turns a mismatch into
 * something you notice in seconds rather than something you deduce. */
void MultiGnssSimView::update_time(uint32_t bytes_read) {
    if (!have_start_time) {
        text_time.set("TIME  -");
        return;
    }

    const uint32_t elapsed = sample_rate
                                 ? static_cast<uint32_t>(samples_from_bytes(bytes_read) / sample_rate)
                                 : 0;
    if (elapsed == shown_second)
        return;
    shown_second = elapsed;

    const uint32_t total = start_seconds + elapsed;
    const uint32_t time_of_day = total % 86400;

    std::string s = "TIME  " + start_date + " " +
                    to_string_dec_uint(time_of_day / 3600, 2, '0') + ":" +
                    to_string_dec_uint((time_of_day / 60) % 60, 2, '0') + ":" +
                    to_string_dec_uint(time_of_day % 60, 2, '0');

    /* A file can only cross midnight if it started within its own duration of
     * it, so the roll is flagged rather than carried into the date. */
    if (total >= 86400)
        s += " +1d";

    text_time.set(s);
}

void MultiGnssSimView::on_stats(uint32_t bytes_read, uint32_t underruns) {
    progressbar.set_value(bytes_read >> 10);
    update_time(bytes_read);
    text_drops.set("DROPS " + to_string_dec_uint(underruns));
}

void MultiGnssSimView::focus() {
    button_open.focus();
}

void MultiGnssSimView::file_error() {
    nav_.display_modal("Error", "File read error.");
}

bool MultiGnssSimView::is_active() const {
    return (bool)replay_thread;
}

void MultiGnssSimView::toggle() {
    if (is_active()) {
        stop(false);
    } else {
        start();
    }
}

void MultiGnssSimView::start() {
    stop(false);

    std::unique_ptr<stream::Reader> reader;

    auto p = std::make_unique<FileReader>();
    auto open_error = p->open(file_path);
    if (open_error.is_valid()) {
        file_error();
    } else {
        reader = std::move(p);
    }

    if (reader) {
        button_play.set_bitmap(&bitmap_stop);

        /* Resent on every pass: the underrun count resets with it, so what is on
         * screen describes the pass you are watching. */
        baseband::set_gnss_sim_format(sim_format);
        shown_second = UINT32_MAX;

        replay_thread = std::make_unique<ReplayThread>(
            std::move(reader),
            read_size, buffer_count,
            &ready_signal,
            [](uint32_t return_code) {
                ReplayThreadDoneMessage message{return_code};
                EventDispatcher::send_message(message);
            });
    }

    transmitter_model.enable();
}

void MultiGnssSimView::stop(const bool do_loop) {
    if (is_active())
        replay_thread.reset();

    if (do_loop && check_loop.value()) {
        start();
    } else {
        transmitter_model.disable();
        button_play.set_bitmap(&bitmap_play);
    }

    ready_signal = false;
}

void MultiGnssSimView::handle_replay_thread_done(const uint32_t return_code) {
    if (return_code == ReplayThread::END_OF_FILE) {
        stop(true);
    } else if (return_code == ReplayThread::READ_ERROR) {
        stop(false);
        file_error();
    }

    progressbar.set_value(0);
}

MultiGnssSimView::MultiGnssSimView(
    NavigationView& nav)
    : nav_(nav) {
    baseband::run_prepared_image(portapack::memory::map::m4_code.base());

    add_children({
        &button_open,
        &text_filename,
        &text_sample_rate,
        &text_duration,
        &progressbar,
        &field_frequency,
        &tx_view,
        &check_loop,
        &button_play,
        &text_gnss,
        &text_position,
        &text_time,
        &text_drops,
    });

    field_frequency.set_step(5000);

    button_play.on_select = [this](ImageButton&) {
        this->toggle();
    };

    button_open.on_select = [this, &nav](Button&) {
        /* FileLoadView takes a single extension, and C8 is the only format a
         * generator emits so far, so it is the only one worth listing. Showing
         * .C2 and .C1 alongside it needs either a multi-extension picker or a
         * new wildcard beside the existing ".C*" (which resolves through
         * is_cxx_capture_file() and deliberately matches only C8/CU8/C16). That
         * belongs with the change that starts producing packed files. */
        auto open_view = nav.push<FileLoadView>(".C8");
        ensure_directory(gps_dir);
        open_view->push_dir(gps_dir);
        open_view->on_changed = [this](std::filesystem::path new_file_path) {
            on_file_changed(new_file_path);
        };
    };
}

MultiGnssSimView::~MultiGnssSimView() {
    transmitter_model.disable();
    baseband::shutdown();
}

} /* namespace ui::external_app::mgnss_sim */
