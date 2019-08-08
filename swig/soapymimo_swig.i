/* -*- c++ -*- */

#define SOAPYMIMO_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "soapymimo_swig_doc.i"

%{
#include "soapymimo/basicsource.h"
%}

%include "soapymimo/basicsource.h"
GR_SWIG_BLOCK_MAGIC2(soapymimo, basicsource);
