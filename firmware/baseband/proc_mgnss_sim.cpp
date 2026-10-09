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

#include "proc_mgnss_sim.hpp"
#include "portapack_shared_memory.hpp"

#include "event_m4.hpp"

#include "utility.hpp"

#include <cstring>

MultiGNSSSimProcessor::MultiGNSSSimProcessor() {
    build_tables();

    spectrum_samples = 0;
    channel_spectrum.set_decimation_factor(1);

    configured = false;
    baseband_thread.start();
}

/* Expand each possible input byte once, so unpacking a block is a table lookup
 * and a store per byte instead of per-sample shifting. Bit layout and levels are
 * fixed by tools/gnss_sim/FORMAT.md: the earliest sample sits in the least
 * significant bits, and I precedes Q within a sample. */
void MultiGNSSSimProcessor::build_tables() {
    static constexpr int8_t levels_c1[2] = {-23, 23};
    static constexpr int8_t levels_c2[4] = {-36, -12, 12, 36};

    for (size_t byte = 0; byte < 256; byte++) {
        for (size_t k = 0; k < 4; k++) {
            const size_t shift = 2 * k;
            table_c1[byte * 4 + k] = {levels_c1[(byte >> shift) & 1],
                                      levels_c1[(byte >> (shift + 1)) & 1]};
        }
        for (size_t k = 0; k < 2; k++) {
            const size_t shift = 4 * k;
            table_c2[byte * 2 + k] = {levels_c2[(byte >> shift) & 3],
                                      levels_c2[(byte >> (shift + 2)) & 3]};
        }
    }
}

size_t MultiGNSSSimProcessor::bytes_for(size_t samples) const {
    switch (format) {
        case Format::C1:
            return samples / 4;
        case Format::C2:
            return samples / 2;
        default:
            return samples * sizeof(complex8_t);
    }
}

size_t MultiGNSSSimProcessor::unpack(const uint8_t* src, size_t bytes, complex8_t* dst, size_t max_samples) const {
    size_t n = 0;

    switch (format) {
        case Format::C1:
            while (n + 4 <= max_samples && n / 4 < bytes) {
                const complex8_t* entry = &table_c1[src[n / 4] * 4];
                dst[n] = entry[0];
                dst[n + 1] = entry[1];
                dst[n + 2] = entry[2];
                dst[n + 3] = entry[3];
                n += 4;
            }
            break;

        case Format::C2:
            while (n + 2 <= max_samples && n / 2 < bytes) {
                const complex8_t* entry = &table_c2[src[n / 2] * 2];
                dst[n] = entry[0];
                dst[n + 1] = entry[1];
                n += 2;
            }
            break;

        default:
            n = bytes / sizeof(complex8_t);
            if (n > max_samples)
                n = max_samples;
            memcpy(dst, src, n * sizeof(complex8_t));
            break;
    }

    return n;
}

void MultiGNSSSimProcessor::execute(const buffer_c8_t& buffer) {
    if (!configured || !stream) return;

    const size_t wanted = bytes_for(buffer.count);
    const size_t got = stream->read(read_buffer.data(), wanted);
    bytes_read += got;

    const size_t filled = unpack(read_buffer.data(), got, buffer.p, buffer.count);

    /* A short read means the SD card did not keep up, or the file ended. Zero the
     * rest of the OUTPUT buffer, never the input bytes: in a packed format a zero
     * byte decodes to the lowest level on both axes, so zero-filling the input
     * would transmit a full-amplitude DC tone instead of silence. A gap in carrier
     * is also easier for a tracking loop to ride out than stale samples, which is
     * what GPS Sim leaves behind here. */
    if (filled < buffer.count) {
        /* Assigned rather than memset: complex8_t is a std::complex specialisation,
         * so clearing it bytewise trips -Wclass-memaccess. */
        for (size_t i = filled; i < buffer.count; i++)
            buffer.p[i] = {0, 0};
        if (got < wanted)
            underruns++;
    }

    spectrum_samples += filled;
    if (spectrum_samples >= spectrum_interval_samples) {
        spectrum_samples -= spectrum_interval_samples;

        stats_message.bytes_read = bytes_read;
        stats_message.underruns = underruns;
        shared_memory.application_queue.push(stats_message);
    }
}

void MultiGNSSSimProcessor::on_message(const Message* const message) {
    switch (message->id) {
        case Message::ID::UpdateSpectrum:
        case Message::ID::SpectrumStreamingConfig:
            channel_spectrum.on_message(message);
            break;

        case Message::ID::SampleRateConfig:
            sample_rate_config(*reinterpret_cast<const SampleRateConfigMessage*>(message));
            break;

        case Message::ID::GNSSSimConfig:
            format = static_cast<Format>(
                reinterpret_cast<const GNSSSimConfigMessage*>(message)->format);
            break;

        case Message::ID::ReplayConfig:
            configured = false;
            bytes_read = 0;
            underruns = 0;
            replay_config(*reinterpret_cast<const ReplayConfigMessage*>(message));
            break;

        // App has prefilled the buffers, we're ready to go now
        case Message::ID::FIFOData:
            configured = true;
            break;

        default:
            break;
    }
}

void MultiGNSSSimProcessor::sample_rate_config(const SampleRateConfigMessage& message) {
    baseband_fs = message.sample_rate;
    baseband_thread.set_sampling_rate(baseband_fs);
    spectrum_interval_samples = baseband_fs / spectrum_rate_hz;
}

void MultiGNSSSimProcessor::replay_config(const ReplayConfigMessage& message) {
    if (message.config) {
        stream = std::make_unique<StreamOutput>(message.config);

        // Tell application that the buffers and FIFO pointers are ready, prefill
        shared_memory.application_queue.push(sig_message);
    } else {
        stream.reset();
    }
}

int main() {
    EventDispatcher event_dispatcher{std::make_unique<MultiGNSSSimProcessor>()};
    event_dispatcher.run();
    return 0;
}
