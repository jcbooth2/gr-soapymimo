/* -*- c++ -*- */
/*
 * Copyright 2019 <+YOU OR YOUR COMPANY+>.
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


#ifndef INCLUDED_SOAPYMIMO_BASICSOURCE_H
#define INCLUDED_SOAPYMIMO_BASICSOURCE_H

#include <soapymimo/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
  namespace soapymimo {

    /*!
     * \brief <+description of block+>
     * \ingroup soapymimo
     *
     */
    class SOAPYMIMO_API basicsource : virtual public gr::sync_block
    {
     public:
      typedef boost::shared_ptr<basicsource> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of soapymimo::basicsource.
       *
       * To avoid accidental use of raw pointers, soapymimo::basicsource's
       * constructor is in a private implementation
       * class. soapymimo::basicsource::make is the public interface for
       * creating new instances.
       * \param nchan number of channels on each device
       * \param device1 the first device driver and type
       * \param device2 the second device driver and type
       * \param args arguments passed to device on initialization
       * \param sampleRate the sample rate of the device
       * \param type the output type of the stream
       */
      static sptr make(size_t nchan, const std::string device1,
        const std::string device2, const std::string args, float sampleRate,
        const std::string type);

      /*!
       * Callback to set overall gain
       * \param channel an available channel of the device
       * \param gain the overall gain value
       */
      virtual void set_gain(size_t channel, float gain) = 0;

      /*!
       * Callback to set specific gain value
       * \param channel an available channel on the device
       * \param name the gain name to set value
       * \param gain the gain value
       */
      virtual void set_gain(size_t channel, const std::string name, float gain) = 0;

      /*!
       * Callback to change the RF frequency of the device
       * \param channel an available channel of the device
       * \param freq the frequency to be set in Hz
       */
      virtual void set_frequency(size_t channel, float freq) = 0;

      /*!
       * Callback to change center frequency of a tunable element
       * \param channel an available channel of the device
       * \param name an available element name
       * \param frequency the frequency to be set in Hz
       */
      virtual void set_frequency(size_t channel, const std::string &name, float frequency) = 0;

      /*!
       * Callback to set sample rate
       * \param channel an available channel of the device
       * \param sample_rate number of samples in samples per second
       */
      virtual void set_sample_rate(size_t channel, float sample_rate) = 0;

      /*!
       * Callback to set digital filter bandwidth
       * \param channel an available channel on the device
       * \param bandwidth filter width in Hz
       */
      virtual void set_bandwidth(size_t channel, float bandwidth) = 0;

      /*!
       * Callback to set antenna for RF chain
       * \param channel an available channel of the device
       * \param name an available antenna string name
       */
      virtual void set_antenna(size_t channel, const std::string &name) = 0;

      /*!
       * Callback to change master clock rate
       * \param clock_rate the clock rate in Hz
       */
      virtual void set_master_clock_rate(double clock_rate) = 0;

      /*!
       * Callback to set the clock source
       * \param clock_source an available clock source
       */
      virtual void set_clock_source(const std::string &clock_source) = 0;
    };
  } // namespace soapymimo
} // namespace gr

#endif /* INCLUDED_SOAPYMIMO_BASICSOURCE_H */
