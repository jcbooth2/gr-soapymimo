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
       */
      static sptr make(device1 device2 sampleRate numChannels clockRate clockSource centerFreq gain antenna bandwidth);
    };

  } // namespace soapymimo
} // namespace gr

#endif /* INCLUDED_SOAPYMIMO_BASICSOURCE_H */

