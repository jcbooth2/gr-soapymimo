/* -*- c++ -*- */
/*
 * Copyright 2019 <Jayden Booth>.
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

#ifndef INCLUDED_SOAPYMIMO_BASICSOURCE_IMPL_H
#define INCLUDED_SOAPYMIMO_BASICSOURCE_IMPL_H

#include <string>
#include <map>

#include <soapymimo/basicsource.h>
#include <boost/bind.hpp>

#include <SoapySDR/Version.hpp>
#include <SoapySDR/Modules.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Device.hpp>

typedef boost::function<void(pmt::pmt_t , size_t)> cmd_handler_t;

namespace gr {
  namespace soapymimo {

    /*!
    * \brief Test source block implementation for 2 LimeSDR devices
    */

    class basicsource_impl : public basicsource
    {
     private:
      SoapySDR::Device* d_device1;
      SoapySDR::Stream* d_stream1;

      SoapySDR::Device* d_device2;
      SoapySDR::Stream* d_stream2;

      size_t d_mtu1;
      size_t d_mtu2;
      pmt::pmt_t d_message_port;

      float d_frequency;
      float d_gain;
      float d_sampleRate;
      float d_bandwidth;
      std::string d_antenna;
      size_t d_nchan;
      double d_clockRate;
      std::string d_clockSource;
      std::string d_type;
      uint8_t d_type_size;

      void register_msg_cmd_handler(const pmt::pmt_t &cmd, cmd_handler_t handler);
      std::map<pmt::pmt_t, cmd_handler_t> d_cmd_handlers;

      inline io_signature::sptr
      args_to_io_sig(const std::string type, size_t nchan)
      {
        size_t size = 0;
        if(type == "fc32")
          size = 8;
        if(type == "s16")
          size = 2;
        return io_signature::make(2*nchan, 2*nchan, size);
      }

     public:
      basicsource_impl(size_t nchan, const std::string device1,
        const std::string device2, const std::string args, float sampling_rate,
        const std::string type);
      ~basicsource_impl();

      // Where all the action really happens
      int work(int noutput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);

       /*!
        * Create and store a new Device object using the make function of SoapySDR
        * API.
        * For every makeDevice call an unmakeDevice call is also made.
        * \param dev a pointer to a device object
        * \param argStr device construction key/value argument string
        * \return integer indicating success or failure
        */
       int makeDevice(SoapySDR::Device* dev, const std::string &argStr);

       /*!
        * Destroy a Device object created by makeDevice call.
        * Called for every makeDevice call.
        * \param dev a pointer to a Device object
        * \return integer indicating success or failure
        */
       int unmakeDevice(SoapySDR::Device* dev);

       /*!
        * Set the center frequency for the specified RX chain.
        * Default implementation tunes RF component frequency as close as
        * possible to the requested frequency. See specific device module
        * for more information.
        * \param channel an available channel on the device
        * \param frequency center frequency in Hz
        */
       void set_frequency (size_t channel, float frequency);

       /*!
        * Set the center frequency for the specified RX chain.
        * Default implementation tunes RF component frequency as close as
        * possible to the requested frequency. See specific device Module
        * for more information.
        * \param channel an available channel on the device
        * \param name an available element name
        * \param frequency center frequency in Hz
        */
       void set_frequency(size_t channel, const std::string &name, float frequency);

       /*!
        * Set the overall gain for the specified RX chain.
        * The gain will be distributed automatically across available
        * elements according to Soapy API.
        * \param dev a pointer to a device object
        * \param channel an available channel on the device
        * \param gain the new amplification value in dB
        */
       void set_gain(size_t channel, float gain);

       /*!
        * Set the value for the specified gain for the specified TX chain.
        * \param dev a pointer to a device object
        * \param channel an available channel on the device
        * \param name an available gain on the device
        * \param gain gain the new amplification value in dB
        */
       void set_gain(size_t channel, const std::string name, float gain);

       /*!
        * Set the baseband sample rate for the RX chain.
        * \param dev a pointer to a device object
        * \param channel an available channel on the device
        * \param sample_rate the sample rate samples per second
        */
       void set_sample_rate(size_t channel, float sample_rate);

       /*!
        * Set the baseband filter width of the RX chain
        * \param dev a pointer to a device object
        * \param channel an available channel on the device
        * \param bandwidth the baseband filter width in Hz
        */
       void set_bandwidth(size_t channel, float bandwidth);

       /*!
        * Set the antenna element for the RX chain.
        * \param dev a pointer to a device object
        * \param channel an available channel on the device
        * \param name the name of an available antenna
        */
       void set_antenna(size_t channel, const std::string &name);

       /*!
        * Set the master clock rate of the device
        * \param dev a pointer to a device object
        * \param clock_rate the clock rate in Hz
        */
       void set_master_clock_rate(double clock_rate);

       /*!
        * Set the clock source of the device
        * \param dev a pointer to a device object
        * \param clock_source the name of clock source
        */
       void set_clock_source(const std::string &clock_source);

       /*!
        * Get the down conversion frequency of the chain.
        * \param dev a pointer to a device object
        * \param channel an available channel on the device
        * \return the center frequency in Hz
        */
       double get_frequency(size_t channel);

       /*!
        * Get the overall value of the gain elements in a chain
        * \param dev a pointer to a device object
        * \param channel an available channel on the device
        * \return the value of the gain in dB
        */
       double get_gain(size_t channel);

       /*!
        * Get the baseband sample rate of the RX chain.
        * \param dev a pointer to a device object
        * \param channel an available channel on the device
        * \return the sample rate in samples per second
        */
       double get_sampling_rate(size_t channel);

       /*!
        * Get baseband filter width of the RX chain.
        * \param dev a pointer to a device object
        * \param channel an available channel on the device
        * \return the baseband filter width in Hz
        */
       double get_bandwidth(size_t channel);

       /*!
        * Get the selected antenna on RX chain.
        * \param dev a pointer to a device object
        * \param channel an available channel on the device
        * \return the name of the selected antenna
        */
       std::string get_antenna(size_t channel);

       /*!
        * Get the master clock rate of the device.
        * \return the clock rate in Hz
        */
       double get_master_clock_rate();

       /*!
        * Get the clock source of the device
        * \return the name of the clock source
        */
       std::string get_clock_source();

       /*!
        * Calls the correct message handler according to the received message symbol.
        * A dictionary with key the handler name is used in order to call the
        * corresponding handler.
        * \param msg a PMT dictionary
        */
       void msg_handler_command(pmt::pmt_t msg);

       /*!
        * Set the center frequency of the RX chain.
        * @param val center frequency in Hz
        * @param chann an available channel on the device
        */
       void cmd_handler_frequency(pmt::pmt_t val, size_t chann);

       /*!
        * Set the overall gain for the specified chain.
        * The gain will be distributed automatically across available
        * elements according to Soapy API.
        * @param val the new amplification value in dB
        * @param chann an avalaible channel on the device
        */
       void cmd_handler_gain(pmt::pmt_t val, size_t chann);

       /*!
        * Set the baseband sample rate for the RX chain.
        * @param val the sample rate samples per second
        * @param chann an available channel on the device
        */
       void cmd_handler_samp_rate(pmt::pmt_t val, size_t chann);

       /*!
        * Set the baseband filter width for the RX chain.
        * @param val baseband filter width in Hz
        * @param chann an available channel on the device
        */
       void cmd_handler_bw(pmt::pmt_t val, size_t chann);

       /*!
        * Set the anntena element for the RX chain.
        * @param val name of the anntena
        * @param chann an available channel on the device
        */
       void cmd_handler_antenna(pmt::pmt_t val, size_t chann);
    };

  } // namespace soapymimo
} // namespace gr

#endif /* INCLUDED_SOAPYMIMO_BASICSOURCE_IMPL_H */
