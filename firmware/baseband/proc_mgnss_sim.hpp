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

#ifndef __PROC_MGNSS_SIM_HPP__
#define __PROC_MGNSS_SIM_HPP__

#include "baseband_processor.hpp"
#include "baseband_thread.hpp"

#include "spectrum_collector.hpp"

#include "stream_output.hpp"

#include <array>
#include <memory>

/* Replays a GNSS baseband file in one of three sample formats. C8 is the format
 * GPS Sim already uses and is a straight copy; C2 and C1 pack two and four
 * complex samples into a byte, trading sample precision for bandwidth at a
 * constant ~5 MB/s off the SD card. tools/gnss_sim/FORMAT.md is the contract. */
class MultiGNSSSimProcessor : public BasebandProcessor {
   public:
    MultiGNSSSimProcessor();

    void execute(const buffer_c8_t& buffer) override;
    void on_message(const Message* const message) override;

   private:
    /* Must match GNSSSimConfigMessage::Format and the generator's `format=` key. */
    enum class Format : uint8_t {
        C8 = 0,
        C2 = 1,
        C1 = 2,
    };

    /* The TX buffer the baseband hands us is 2048 samples; C8 needs two bytes per
     * sample, which is the largest read of the three formats. */
    static constexpr size_t max_block_samples = 2048;

    size_t baseband_fs = 3072000;
    static constexpr auto spectrum_rate_hz = 50.0f;

    Format format{Format::C8};

    /* Byte -> samples tables, filled by build_tables() at construction. They live
     * in .bss rather than the image because an external app's code and its
     * baseband image share the 40 KiB m4_code region on the LPC4320. */
    std::array<complex8_t, 256 * 4> table_c1{};
    std::array<complex8_t, 256 * 2> table_c2{};

    std::array<uint8_t, max_block_samples * 2> read_buffer{};

    std::unique_ptr<StreamOutput> stream{};

    SpectrumCollector channel_spectrum{};
    size_t spectrum_interval_samples = 0;
    size_t spectrum_samples = 0;

    bool configured{false};
    uint32_t bytes_read{0};
    uint32_t underruns{0};

    void build_tables();
    size_t bytes_for(size_t samples) const;
    size_t unpack(const uint8_t* src, size_t bytes, complex8_t* dst, size_t max_samples) const;

    void sample_rate_config(const SampleRateConfigMessage& message);
    void replay_config(const ReplayConfigMessage& message);

    GNSSSimStatsMessage stats_message{};
    RequestSignalMessage sig_message{RequestSignalMessage::Signal::FillRequest};

    /* NB: Threads should be the last members in the class definition. */
    BasebandThread baseband_thread{
        baseband_fs, this, baseband::Direction::Transmit, /*auto_start*/ false};
};

#endif /*__PROC_MGNSS_SIM_HPP__*/
