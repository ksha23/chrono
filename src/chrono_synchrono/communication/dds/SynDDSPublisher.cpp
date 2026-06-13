// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// =============================================================================
//
// Class that handles publishing on a specified topic. "Publishing" is
// essentially synonomous with "Sending". A topic is basically an abstract
// connection between a publisher and a receiver (known as a subscriber in DDS
// terms).
//
// Currently, a SynDDSPublisher can only be responsible for one topic. In the
// future, a SynDDSPublisher should be able to send the same message on more
// than one topic.
//
// Simply, a publisher is responsible for managing data writers. Data writers
// actually write data to the topics desired by the user.
//
// =============================================================================

#include "chrono_synchrono/communication/dds/SynDDSPublisher.h"

#include "chrono_synchrono/communication/dds/SynDDSTopic.h"
#include "chrono_synchrono/communication/dds/SynDDSListener.h"

#include "chrono_synchrono/communication/dds/idl/SynDDSMessage.h"

#include <fastdds/dds/domain/DomainParticipant.hpp>

#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#if defined(CHRONO_SYNCHRONO_FASTDDS_API) && CHRONO_SYNCHRONO_FASTDDS_API >= 3
    #include <fastdds/dds/core/Time_t.hpp>
#else
    #include <fastdds/rtps/common/Time_t.h>
#endif

using namespace eprosima::fastdds::dds;
#if !defined(CHRONO_SYNCHRONO_FASTDDS_API) || CHRONO_SYNCHRONO_FASTDDS_API < 3
using namespace eprosima::fastrtps::types;
#endif

namespace chrono {
namespace synchrono {

SynDDSPublisher::SynDDSPublisher(Publisher* publisher,
                                 DataWriter* writer,
                                 SynDDSDataWriterListener* listener,
                                 std::shared_ptr<SynDDSTopic> topic)
    : m_publisher(publisher), m_writer(writer), m_listener(listener), m_topic(topic) {}

SynDDSPublisher::~SynDDSPublisher() {}

void SynDDSPublisher::DeleteDDSEntities(DomainParticipant* participant) {
    if (m_topic)
        m_topic->DeleteDDSTopic(participant);

    if (m_publisher)
        participant->delete_publisher(m_publisher);
}

bool SynDDSPublisher::Publish(void* message) {
#if defined(CHRONO_SYNCHRONO_FASTDDS_API) && CHRONO_SYNCHRONO_FASTDDS_API >= 3
    auto ret = m_writer->write(message);

    if (ret != RETCODE_OK) {
        std::cerr << "DataWriter failed to write on topic" << m_writer->get_topic()->get_name() << std::endl;
    }

    return ret == RETCODE_OK;
#else
    bool ret = m_writer->write(message);

    if (!ret) {
        std::cerr << "DataWriter failed to write on topic" << m_writer->get_topic()->get_name() << std::endl;
    }

    return ret;
#endif
}

bool SynDDSPublisher::WaitForAcknowledgments(double max_wait) {
#if defined(CHRONO_SYNCHRONO_FASTDDS_API) && CHRONO_SYNCHRONO_FASTDDS_API >= 3
    Duration_t timeout(max_wait);
    auto ret = m_writer->wait_for_acknowledgments(timeout);

    if (ret != RETCODE_OK) {
        std::cerr << "DataWriter acknowledgments timed out on topic " << m_writer->get_topic()->get_name()
                  << std::endl;
    }

    return ret == RETCODE_OK;
#else
    eprosima::fastrtps::Duration_t timeout(max_wait);
    auto ret = m_writer->wait_for_acknowledgments(timeout);

    if (ret != ReturnCode_t::RETCODE_OK) {
        std::cerr << "DataWriter acknowledgments timed out on topic " << m_writer->get_topic()->get_name()
                  << std::endl;
    }

    return ret == ReturnCode_t::RETCODE_OK;
#endif
}

///@brief Wait for the specified number of matches
/// Each subscriber listener has a callback that will be called when a subscriber is matched with
/// a DataWriter. This function blocks until that the matches are achieved. By default,
/// a subscriber will just wait for a single listener.
///
void SynDDSPublisher::WaitForMatches(unsigned int matches) {
    m_listener->BlockUntilMatches(matches);
}

}  // namespace synchrono
}  // namespace chrono
