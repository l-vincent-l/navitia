/* Copyright © 2001-2014, Canal TP and/or its affiliates. All rights reserved.
  
This file is part of Navitia,
    the software to build cool stuff with public transport.
 
Hope you'll enjoy and contribute to this project,
    powered by Canal TP (www.canaltp.fr).
Help us simplify mobility and open public transport:
    a non ending quest to the responsive locomotion way of traveling!
  
LICENCE: This program is free software; you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
   
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Affero General Public License for more details.
   
You should have received a copy of the GNU Affero General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>.
  
Stay tuned using
twitter @navitia 
IRC #navitia on freenode
https://groups.google.com/d/forum/navitia
www.navitia.io
*/

#pragma once

#include <boost/date_time/posix_time/posix_time.hpp>


#include <boost/serialization/weak_ptr.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/date_time/gregorian/greg_serialize.hpp>
#include <boost/date_time/posix_time/time_serialize.hpp>
#include <boost/serialization/bitset.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include <boost/variant.hpp>
#include <boost/serialization/variant.hpp>

#include <atomic>
#include <map>
#include <vector>
#include <string>
#include "utils/serialization_unique_ptr.h"
#include "utils/serialization_unique_ptr_container.h"

#include "type/type.h"

namespace navitia { namespace type {

namespace new_disruption {

enum class Effect {
  NO_SERVICE,
  REDUCED_SERVICE,
  SIGNIFICANT_DELAYS,
  DETOUR,
  ADDITIONAL_SERVICE,
  MODIFIED_SERVICE,
  OTHER_EFFECT,
  UNKNOWN_EFFECT,
  STOP_MOVED
};

inline std::string to_string(Effect effect) {
    switch (effect) {
    case Effect::NO_SERVICE: return "NO_SERVICE";
    case Effect::REDUCED_SERVICE: return "REDUCED_SERVICE";
    case Effect::SIGNIFICANT_DELAYS: return "SIGNIFICANT_DELAYS";
    case Effect::DETOUR: return "DETOUR";
    case Effect::ADDITIONAL_SERVICE: return "ADDITIONAL_SERVICE";
    case Effect::MODIFIED_SERVICE: return "MODIFIED_SERVICE";
    case Effect::OTHER_EFFECT: return "OTHER_EFFECT";
    case Effect::UNKNOWN_EFFECT: return "UNKNOWN_EFFECT";
    case Effect::STOP_MOVED: return "STOP_MOVED";
    default:
        throw navitia::exception("unhandled effect case");
    }
}


struct Cause {
    std::string uri;
    std::string wording;
    boost::posix_time::ptime created_at;
    boost::posix_time::ptime updated_at;

    template<class Archive>
    void serialize(Archive& ar, const unsigned int) {
        ar & uri & wording & created_at & updated_at;
    }
};

struct Severity {
    std::string uri;
    std::string wording;
    boost::posix_time::ptime created_at;
    boost::posix_time::ptime updated_at;
    std::string color;

    int priority;

    Effect effect = Effect::UNKNOWN_EFFECT;

    template<class Archive>
    void serialize(Archive& ar, const unsigned int) {
        ar & uri & wording & created_at & updated_at & color & priority & effect;
    }
};

struct UnknownPtObj {
    template<class Archive>
    void serialize(Archive&, const unsigned int) {}
};
struct LineSection {
    const Line* line = nullptr;
    const StopArea* start_point = nullptr;
    const StopArea* end_point = nullptr;
    template<class Archive>
    void serialize(Archive& ar, const unsigned int) {
        ar & line & start_point & end_point;
    }
};
typedef boost::variant<
    UnknownPtObj,
    const Network *,
    const StopArea *,
    LineSection,
    const Line *,
    const Route *
    > PtObj;

PtObj make_pt_obj(Type_e type,
                  const std::string &uri,
                  const PT_Data& pt_data,
                  const boost::shared_ptr<Impact> &impact = {});

struct Disruption;

struct Message {
    std::string text;
    std::string channel_id;
    std::string channel_name;
    std::string channel_content_type;

    boost::posix_time::ptime created_at;
    boost::posix_time::ptime updated_at;

    template<class Archive>
    void serialize(Archive& ar, const unsigned int) {
        ar & text & created_at & updated_at;
    }
};

struct Impact {
    std::string uri;
    boost::posix_time::ptime created_at;
    boost::posix_time::ptime updated_at;

    // the application period define when the impact happen
    std::vector<boost::posix_time::time_period> application_periods;

    boost::shared_ptr<Severity> severity;

    std::vector<PtObj> informed_entities;

    std::vector<Message> messages;

    // Used when an informed entity is a stop_area
    // We need to store it, to be able to delete it afterward
    std::vector<const JourneyPattern*> impacted_journey_patterns;

    //link to the parent disruption
    //Note: it is a raw pointer because an Impact is owned by it's disruption
    //(even if the impact is stored as a share_ptr in the disruption to allow for weak_ptr towards it)
    Disruption* disruption;

    template<class Archive>
    void serialize(Archive& ar, const unsigned int) {
        ar & uri & created_at & updated_at & application_periods & severity & informed_entities & messages & impacted_journey_patterns & disruption;
    }

    bool is_valid(const boost::posix_time::ptime& current_time, const boost::posix_time::time_period& action_period) const;
};

struct Tag {
    std::string uri;
    std::string name;
    boost::posix_time::ptime created_at;
    boost::posix_time::ptime updated_at;

    template<class Archive>
    void serialize(Archive& ar, const unsigned int) {
        ar & uri & name & created_at & updated_at;
    }
};

struct Disruption {
    std::string uri;

    // it's the title of the disruption as shown in the backoffice
    std::string reference;

    // the publication period specify when an information can be displayed to
    // the customer, if a request is made before or after this period the
    // disruption must not be shown
    boost::posix_time::time_period publication_period {
        boost::posix_time::not_a_date_time, boost::posix_time::seconds(1)
    };//no default constructor for time_period, we must provide a value

    boost::posix_time::ptime created_at;
    boost::posix_time::ptime updated_at;

    boost::shared_ptr<Cause> cause;

    // the place where the disruption happen, the impacts can be in anothers places
    std::vector<PtObj> localization;

    //additional informations on the disruption
    std::vector<boost::shared_ptr<Tag>> tags;

    std::string note;

    template<class Archive>
    void serialize(Archive& ar, const unsigned int) {
        ar & uri & reference & publication_period
           & created_at & updated_at & cause & impacts & localization & tags & note;
    }

    void add_impact(const boost::shared_ptr<Impact>& impact);
    const std::vector<boost::shared_ptr<Impact>>& get_impacts() const {
        return impacts;
    }

    bool is_publishable(const boost::posix_time::ptime& current_time) const;

private:
    //Disruption have the ownership of the Impacts.  Impacts are
    //shared_ptr and not unique_ptr because there are weak_ptr
    //pointing to them in the impacted objects
    std::vector<boost::shared_ptr<Impact>> impacts;
};

struct DisruptionHolder {
    std::vector<std::unique_ptr<Disruption>> disruptions;

    // causes, severities and tags are a pool (weak_ptr because the owner ship
    // is in the linked disruption or impact)
    std::map<std::string, boost::weak_ptr<Cause>> causes; //to be wrapped
    std::map<std::string, boost::weak_ptr<Severity>> severities; //to be wrapped too
    std::map<std::string, boost::weak_ptr<Tag>> tags; //to be wrapped too

    template<class Archive>
    void serialize(Archive& ar, const unsigned int) {
        ar & disruptions & causes & severities & tags;
    }
};
}

}}//namespace
