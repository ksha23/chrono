#ifndef CHRONO_SYNCHRONO_DDS_IDL_SYNDDSMESSAGE_WRAPPER_H
#define CHRONO_SYNCHRONO_DDS_IDL_SYNDDSMESSAGE_WRAPPER_H

#include "chrono_synchrono/SynConfig.h"

#if defined(CHRONO_SYNCHRONO_FASTDDS_API) && CHRONO_SYNCHRONO_FASTDDS_API >= 3
    #include "fastdds3/SynDDSMessage.hpp"
#else
    #include "fastdds2/SynDDSMessage.h"
#endif

#endif  // CHRONO_SYNCHRONO_DDS_IDL_SYNDDSMESSAGE_WRAPPER_H
