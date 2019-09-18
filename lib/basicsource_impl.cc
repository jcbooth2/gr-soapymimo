/* -*- c++ -*- */
/*
 * Copyright 2019 <Jayde>.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdexcept>
#include <vector>
#include <string>
#include <gnuradio/io_signature.h>
#include <soapymimo/basicsource.h>
#include "basicsource_impl.h"

const pmt::pmt_t CMD_CHAN_KEY = pmt::mp ("chan");
const pmt::pmt_t CMD_FREQ_KEY = pmt::mp ("freq");
const pmt::pmt_t CMD_GAIN_KEY = pmt::mp ("gain");
const pmt::pmt_t CMD_ANTENNA_KEY = pmt::mp ("antenna");
const pmt::pmt_t CMD_RATE_KEY = pmt::mp ("samp_rate");
const pmt::pmt_t CMD_BW_KEY = pmt::mp ("bw");

namespace gr {
  namespace soapymimo {

    basicsource::sptr
    basicsource::make(size_t nchan, const std::string device1,
      const std::string device2, const std::string args, float sampleRate,
      const std::string type)
    {
      return gnuradio::get_initial_sptr (
        new basicsource_impl (nchan, device1, device2, args, sampleRate, type));
    }

    /*
     * The private constructor
     */
    basicsource_impl::basicsource_impl(size_t nchan, const std::string device1,
      const std::string device2, const std::string args, float sampleRate,
      const std::string type)
      : gr::sync_block("basicsource", gr::io_signature::make(0, 0, 0),
              args_to_io_sig(type, nchan)),
        d_mtu1 (0),
        d_mtu2 (0),
        d_message_port (pmt::mp ("command")),
        d_nchan (nchan),
        d_type ("fc32"),
        d_sampleRate (sampleRate)
    {
      if (type == "fc32")
      {
        d_type_size = 8;
        d_type = "CF32";
      }

      if (type == "s16")
      {
        d_type_size = 2;
        d_type = "s16";
      }

      // Make Device 1
      std::string str_args = device1 + ", " + args;
      SoapySDR::Kwargs d_args = SoapySDR::KwargsFromString(str_args);
      makeDevice(d_device1, str_args);
      std::vector<size_t> channs;
      channs.resize (d_nchan);

      // Make Device 2
      if (device2 != device1)
      {
        str_args = device2;
        d_args = SoapySDR::KwargsFromString(str_args);
        makeDevice(d_device2, str_args);
        std::vector<size_t> channs2;
        channs2.resize (d_nchan);

        // Setup channels on devices
        for (int i = 0; i < d_nchan; i++)
        {
          channs[i] = i;
          set_sample_rate(i, d_sampleRate);
        }

        // Make Stream 1
        d_stream1 = d_device1->setupStream (SOAPY_SDR_RX, d_type, channs, d_args);
        d_device1->activateStream (d_stream1);
        d_mtu1 = d_device1->getStreamMTU(d_stream1);

        // Make Stream 2
        d_stream2 = d_device2->setupStream (SOAPY_SDR_RX, d_type, channs, d_args);
        d_device2->activateStream (d_stream2);
        d_mtu2 = d_device2->getStreamMTU(d_stream2);
      }

      // Setup Channels on the devices
      for (int i = 0; i < d_nchan; i++)
      {
        channs[i] = i;
      }

      message_port_register_in (d_message_port);
      set_msg_handler (
          d_message_port,
          boost::bind (&basicsource_impl::msg_handler_command, this, _1));

      register_msg_cmd_handler (
          CMD_FREQ_KEY,
          boost::bind (&basicsource_impl::cmd_handler_frequency, this, _1, _2));
      register_msg_cmd_handler (
          CMD_GAIN_KEY,
          boost::bind (&basicsource_impl::cmd_handler_gain, this, _1, _2));
      register_msg_cmd_handler (
          CMD_RATE_KEY,
          boost::bind (&basicsource_impl::cmd_handler_samp_rate, this, _1, _2));
      register_msg_cmd_handler (
          CMD_BW_KEY, boost::bind (&basicsource_impl::cmd_handler_bw, this, _1, _2));
      register_msg_cmd_handler (
          CMD_ANTENNA_KEY,
          boost::bind (&basicsource_impl::cmd_handler_antenna, this, _1, _2));

      set_max_noutput_items (d_mtu1+d_mtu2);
    }

    /*
     * Our virtual destructor.
     */
    basicsource_impl::~basicsource_impl()
    {
      unmakeDevice(d_device1);
      unmakeDevice(d_device2);
    }

    void
    basicsource_impl::register_msg_cmd_handler (const pmt::pmt_t &cmd,
                                           cmd_handler_t handler)
    {
      d_cmd_handlers[cmd] = handler;
    }

    int
    basicsource_impl::makeDevice (SoapySDR::Device* d_device, const std::string &argStr)
    {
      try {
        d_device = SoapySDR::Device::make (argStr);
      }
      catch (const std::exception &ex) {
        std::cerr << "Error making device: " << ex.what () << std::endl;
        return EXIT_FAILURE;
      }
      return EXIT_SUCCESS;
    }

    int
    basicsource_impl::unmakeDevice (SoapySDR::Device* dev)
    {
      try {
        SoapySDR::Device::unmake (dev);
      }
      catch (const std::exception &ex) {
        std::cerr << "Error unmaking device: " << ex.what () << std::endl;
        return EXIT_FAILURE;
      }
      return EXIT_SUCCESS;
    }

    void
    basicsource_impl::set_frequency (size_t channel, float frequency)
    {
      d_device1->setFrequency (SOAPY_SDR_RX, channel, frequency);
      d_device2->setFrequency (SOAPY_SDR_RX, channel, frequency);
      d_frequency = d_device1->getFrequency (SOAPY_SDR_RX, channel);
    }

    void
    basicsource_impl::set_frequency (size_t channel, const std::string &name, float frequency)
    {
      d_device1->setFrequency(SOAPY_SDR_RX, channel, name, frequency);
      d_device2->setFrequency(SOAPY_SDR_RX, channel, name, frequency);
      d_frequency = d_device1->getFrequency (SOAPY_SDR_RX, channel);
    }

    void
    basicsource_impl::set_gain (size_t channel, float gain)
    {
      d_device1->setGain (SOAPY_SDR_RX, channel, gain);
      d_device2->setGain (SOAPY_SDR_RX, channel, gain);
      d_gain = d_device1->getGain (SOAPY_SDR_RX, channel);
    }

    void
    basicsource_impl::set_gain (size_t channel, const std::string name, float gain)
    {
      d_device1->setGain (SOAPY_SDR_RX, channel, name, gain);
      d_device2->setGain (SOAPY_SDR_RX, channel, name ,gain);
      d_gain = d_device1->getGain (SOAPY_SDR_RX, channel);
    }

    void
    basicsource_impl::set_sample_rate (size_t channel, float sample_rate)
    {
      d_device1->setSampleRate (SOAPY_SDR_RX, channel, sample_rate);
      d_device2->setSampleRate (SOAPY_SDR_RX, channel, sample_rate);
      d_sampleRate = sample_rate;
    }

    void
    basicsource_impl::set_bandwidth (size_t channel, float bandwidth)
    {
      d_device1->setBandwidth (SOAPY_SDR_RX, channel, bandwidth);
      d_device2->setBandwidth (SOAPY_SDR_RX, channel, bandwidth);
      d_bandwidth = bandwidth;
    }

    void
    basicsource_impl::set_antenna (const size_t channel, const std::string &name)
    {
      d_device1->setAntenna (SOAPY_SDR_RX, channel, name);
      d_device2->setAntenna (SOAPY_SDR_RX, channel, name);
      d_antenna = name;
    }

    void
    basicsource_impl::set_master_clock_rate (double clock_rate)
    {
      d_device1->setMasterClockRate (clock_rate);
      d_device2->setMasterClockRate (clock_rate);
      d_clockRate = clock_rate;
    }

    void
    basicsource_impl::set_clock_source (const std::string &clock_source)
    {
      if (clock_source == "DEVICE1")
      {

      }

      else if (clock_source == "DEVICE2")
      {

      }

      else if (clock_source == "EXTERNAL")
      {
        
      }

      else
      {
        throw std::invalid_argument("Invalid Clock Source");
      }
    }

    double
    basicsource_impl::get_frequency (size_t channel)
    {
      return d_device1->getFrequency (SOAPY_SDR_RX, channel);
    }

    double
    basicsource_impl::get_gain (size_t channel)
    {
      return d_device1->getGain (SOAPY_SDR_RX, channel);
    }

    double
    basicsource_impl::get_sampling_rate (size_t channel)
    {
      return d_device1->getSampleRate (SOAPY_SDR_RX, channel);
    }

    double
    basicsource_impl::get_bandwidth (size_t channel)
    {
      return d_device1->getBandwidth (SOAPY_SDR_RX, channel);
    }

    std::string
    basicsource_impl::get_antenna (size_t channel)
    {
      return d_device1->getAntenna (SOAPY_SDR_RX, channel);
    }

    double
    basicsource_impl::get_master_clock_rate ()
    {
      return d_clockRate;
    }

    std::string
    basicsource_impl::get_clock_source ()
    {
      return d_clockSource;
    }

    void
    basicsource_impl::cmd_handler_frequency (pmt::pmt_t val, size_t chann)
    {
      set_frequency (chann, pmt::to_float (val));
      set_frequency (chann, pmt::to_float(val));
    }

    void
    basicsource_impl::cmd_handler_gain (pmt::pmt_t val, size_t chann)
    {
      set_gain (chann, pmt::to_float (val));
    }

    void
    basicsource_impl::cmd_handler_samp_rate (pmt::pmt_t val, size_t chann)
    {
      set_sample_rate (chann, pmt::to_float (val));
    }

    void
    basicsource_impl::cmd_handler_bw (pmt::pmt_t val, size_t chann)
    {
      set_bandwidth (chann, pmt::to_float (val));
    }

    void
    basicsource_impl::cmd_handler_antenna (pmt::pmt_t val, size_t chann)
    {
      set_antenna (chann, pmt::symbol_to_string (val));
    }

    int
    basicsource_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      int flags1 = 0;
      long long timeNs1 = 0;
      int flags2 = 0;
      long long timeNs2 = 0;

      int read1 = d_device1->readStream (d_stream1, &output_items[0],
        noutput_items/2, flags1, timeNs1);

      int read2 = d_device1->readStream (d_stream1, &output_items[2],
        noutput_items/2, flags2, timeNs2);

      // TO DO: Compare timeNs1 to timeNs2 and match phase of both devices

      // Tell runtime system how many output items we produced.
      if (read1 < 0 || read2 < 0) return 0;
      return read1+read2;
    }

    void
    basicsource_impl::msg_handler_command (pmt::pmt_t msg)
    {
      if (!pmt::is_dict (msg)) {
        return;
      }
      size_t chann = 0;
      if (pmt::dict_has_key (msg, CMD_CHAN_KEY)) {
        chann = pmt::to_long (
            pmt::dict_ref (msg, CMD_CHAN_KEY, pmt::from_long (0)));
        pmt::dict_delete (msg, CMD_CHAN_KEY);
      }
      for (size_t i = 0; i < pmt::length (msg); i++) {
        d_cmd_handlers[pmt::car (pmt::nth (i, msg))] (
            pmt::cdr (pmt::nth (i, msg)), chann);
      }
    }

  } /* namespace soapymimo */
} /* namespace gr */
