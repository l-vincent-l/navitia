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

#include "type.h"
#include "pt_data.h"
#include <iostream>
#include <boost/assign.hpp>
#include "utils/functions.h"
#include "utils/logger.h"

//they need to be included for the BOOST_CLASS_EXPORT_GUID macro
#include "third_party/eos_portable_archive/portable_iarchive.hpp"
#include "third_party/eos_portable_archive/portable_oarchive.hpp"
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

namespace navitia { namespace type {

std::string VehicleJourney::get_direction() const {
    if ((this->journey_pattern != nullptr) && (!this->journey_pattern->journey_pattern_point_list.empty())){
        const auto jpp = this->journey_pattern->journey_pattern_point_list.back();
        if(jpp->stop_point != nullptr){
            for (auto admin: jpp->stop_point->admin_list){
                if (admin->level == 8){
                    return jpp->stop_point->name + " (" + admin->name + ")";
                }
            }
        return jpp->stop_point->name;
        }
    }
    return "";
}

std::vector<boost::weak_ptr<new_disruption::Impact>> HasMessages::get_applicable_messages(
        const boost::posix_time::ptime& current_time,
        const boost::posix_time::time_period& action_period) const {
    std::vector<boost::weak_ptr<new_disruption::Impact>> result;

    //we cleanup the released pointer (not in the loop for code clarity)
    clean_up_weak_ptr(impacts);

    for (auto impact : this->impacts) {
        auto impact_acquired = impact.lock();
        if (! impact_acquired) {
            continue; //pointer might still have become invalid
        }
        if (impact_acquired->is_valid(current_time, action_period)) {
            result.push_back(impact);
        }
    }

    return result;

}

bool HasMessages::has_applicable_message(
        const boost::posix_time::ptime& current_time,
        const boost::posix_time::time_period& action_period) const {
    //we cleanup the released pointer (not in the loop for code clarity)
    clean_up_weak_ptr(impacts);

    for (auto impact : this->impacts) {
        auto impact_acquired = impact.lock();
        if (! impact_acquired) {
            continue; //pointer might still have become invalid
        }
        if (impact_acquired->is_valid(current_time, action_period)) {
            return true;
        }
    }
    return false;
}

uint32_t StopTime::f_arrival_time(const u_int32_t hour, bool clockwise) const {
    if(clockwise) {
        if (this == &this->vehicle_journey->stop_time_list.front())
            return hour;
        const auto& prec_st = this->vehicle_journey->stop_time_list[this->journey_pattern_point->order-1];
        return hour + this->arrival_time - prec_st.arrival_time;
    } else {
        if (this == &this->vehicle_journey->stop_time_list.back())
            return hour;
        const auto& next_st = this->vehicle_journey->stop_time_list[this->journey_pattern_point->order+1];
        return hour - (next_st.arrival_time - this->arrival_time);
    }
}

bool StopTime::is_valid_day(u_int32_t day, const bool is_arrival, const bool is_adapted) const{
    if((is_arrival && arrival_time >= DateTimeUtils::SECONDS_PER_DAY)
       || (!is_arrival && departure_time >= DateTimeUtils::SECONDS_PER_DAY)) {
        if(day == 0)
            return false;
        --day;
    }
    if(!is_adapted) {
        return vehicle_journey->validity_pattern->check(day);
    } else {
        return vehicle_journey->adapted_validity_pattern->check(day);
    }
}

bool StopTime::operator<(const StopTime& other) const {
    if(this->vehicle_journey == other.vehicle_journey){
        return this->journey_pattern_point->order < other.journey_pattern_point->order;
    } else {
        return *this->vehicle_journey < *other.vehicle_journey;
    }
}

uint32_t StopTime::f_departure_time(const u_int32_t hour, bool clockwise) const {
    if(clockwise) {
        if (this == &this->vehicle_journey->stop_time_list.front())
            return hour;
        const auto& prec_st = this->vehicle_journey->stop_time_list[this->journey_pattern_point->order-1];
        return hour + this->departure_time - prec_st.departure_time;
    } else {
        if (this == &this->vehicle_journey->stop_time_list.back())
            return hour;
        const auto& next_st = this->vehicle_journey->stop_time_list[this->journey_pattern_point->order+1];
        return hour - (next_st.departure_time - this->departure_time);
    }
}

bool FrequencyVehicleJourney::is_valid(int day, const bool is_adapted) const {
    if (day < 0)
        return false;
    if (! is_adapted) {
        return validity_pattern->check(day);
    } else {
        return adapted_validity_pattern->check(day);
    }
}

bool VehicleJourney::has_date_time_estimated() const{
    for(const StopTime& st : this->stop_time_list){
        if (st.date_time_estimated()){
            return true;
        }
    }
    return false;
}

bool VehicleJourney::has_boarding() const{
    std::string physical_mode;
    if ((this->journey_pattern != nullptr) && (this->journey_pattern->physical_mode != nullptr))
        physical_mode = this->journey_pattern->physical_mode->name;
    if (! physical_mode.empty()){
        boost::to_lower(physical_mode);
        return (physical_mode == "boarding");
    }
    return false;

}

bool VehicleJourney::has_landing() const{
    std::string physical_mode;
    if ((this->journey_pattern != nullptr) && (this->journey_pattern->physical_mode != nullptr))
        physical_mode = this->journey_pattern->physical_mode->name;
    if (! physical_mode.empty()){
        boost::to_lower(physical_mode);
        return (physical_mode == "landing");
    }
    return false;

}

type::OdtLevel_e VehicleJourney::get_odt_level() const{
    type::OdtLevel_e result = type::OdtLevel_e::none;
    if (this->stop_time_list.empty()){
        return result;
    }

    if (stop_time_list.front().is_odt_and_date_time_estimated()){
        result = type::OdtLevel_e::zonal;
    }
    for(const auto& st: stop_time_list){
        if (st.is_odt_and_date_time_estimated()){
            if (result != type::OdtLevel_e::zonal){
                result = type::OdtLevel_e::mixt;
                break;
            }
        }else{
            if(result == type::OdtLevel_e::zonal){
                result = type::OdtLevel_e::mixt;
                break;
            }
        }
    }
    return result;
}

bool ValidityPattern::is_valid(int day) const {
    if(day < 0) {
        LOG4CPLUS_DEBUG(log4cplus::Logger::getInstance("log"), "Validity pattern not valid, the day "
                       << day << " is too early");
        return false;
    }
    if(size_t(day) >= days.size()) {
        LOG4CPLUS_DEBUG(log4cplus::Logger::getInstance("log"), "Validity pattern not valid, the day "
                       << day << " is late");
        return false;
    }
    return true;
}

int ValidityPattern::slide(boost::gregorian::date day) const {
    return (day - beginning_date).days();
}

void ValidityPattern::add(boost::gregorian::date day){
    long duration = slide(day);
    add(duration);
}

void ValidityPattern::add(int duration){
    if(is_valid(duration))
        days[duration] = true;
}

void ValidityPattern::add(boost::gregorian::date start, boost::gregorian::date end, std::bitset<7> active_days){
    for (auto current_date = start; current_date < end; current_date = current_date + boost::gregorian::days(1)) {
        navitia::weekdays week_day = navitia::get_weekday(current_date);
        if (active_days[week_day]) {
            add(current_date);
        } else {
            remove(current_date);
        }
    };
}

void ValidityPattern::remove(boost::gregorian::date date){
    long duration = slide(date);
    remove(duration);
}

void ValidityPattern::remove(int day){
    if(is_valid(day))
        days[day] = false;
}

std::string ValidityPattern::str() const {
    return days.to_string();
}

bool ValidityPattern::check(boost::gregorian::date day) const {
    long duration = slide(day);
    return ValidityPattern::check(duration);
}

bool ValidityPattern::check(unsigned int day) const {
//    BOOST_ASSERT(is_valid(day));
    return days[day];
}

bool ValidityPattern::check2(unsigned int day) const {
//    BOOST_ASSERT(is_valid(day));
    if(day == 0)
        return days[day] || days[day+1];
    else
        return days[day-1] || days[day] || days[day+1];
}

bool ValidityPattern::uncheck2(unsigned int day) const {
//    BOOST_ASSERT(is_valid(day));
    if(day == 0)
        return !days[day] && !days[day+1];
    else
        return !days[day-1] && !days[day] && !days[day+1];
}

static_data * static_data::instance = 0;
static_data * static_data::get() {
    if (instance == 0) {
        static_data* temp = new static_data();

        boost::assign::insert(temp->types_string)
                (Type_e::ValidityPattern, "validity_pattern")
                (Type_e::Line, "line")
                (Type_e::JourneyPattern, "journey_pattern")
                (Type_e::VehicleJourney, "vehicle_journey")
                (Type_e::StopPoint, "stop_point")
                (Type_e::StopArea, "stop_area")
                (Type_e::Network, "network")
                (Type_e::PhysicalMode, "physical_mode")
                (Type_e::CommercialMode, "commercial_mode")
                (Type_e::Connection, "connection")
                (Type_e::JourneyPatternPoint, "journey_pattern_point")
                (Type_e::Company, "company")
                (Type_e::Way, "way")
                (Type_e::Coord, "coord")
                (Type_e::Address, "address")
                (Type_e::Route, "route")
                (Type_e::POI, "poi")
                (Type_e::POIType, "poi_type")
                (Type_e::Contributor, "contributor")
                (Type_e::Calendar, "calendar");

        boost::assign::insert(temp->modes_string)
                (Mode_e::Walking, "walking")
                (Mode_e::Bike, "bike")
                (Mode_e::Car, "car")
                (Mode_e::Bss, "bss");
        instance = temp;

    }
    return instance;
}

Type_e static_data::typeByCaption(const std::string & type_str) {
    return instance->types_string.right.at(type_str);
}

std::string static_data::captionByType(Type_e type){
    return instance->types_string.left.at(type);
}

Mode_e static_data::modeByCaption(const std::string & mode_str) {
    return instance->modes_string.right.at(mode_str);
}

template<typename T> std::vector<idx_t> indexes(const std::vector<T*>& elements){
    std::vector<idx_t> result;
    for(T* element : elements){
        result.push_back(element->idx);
    }
    return result;
}

Calendar::Calendar(boost::gregorian::date beginning_date) : validity_pattern(beginning_date) {}

bool VehicleJourney::operator<(const VehicleJourney& other) const {
    if (this->journey_pattern->uri != other.journey_pattern->uri)
        return this->journey_pattern->uri < other.journey_pattern->uri;
    return this->uri < other.uri;

    if (this == &other) return false;
    if(this->journey_pattern == other.journey_pattern){
        return this->stop_time_list.front() < other.stop_time_list.front();
    }else{
        return this->journey_pattern->uri < other.journey_pattern->uri;
    }
}
std::vector<idx_t> Calendar::get(Type_e type, const PT_Data & data) const{
    std::vector<idx_t> result;
    switch(type) {
    case Type_e::Line:{
        // if the method is slow, adding a list of lines in calendar
        for(Line* line: data.lines) {
            for(Calendar* cal : line->calendar_list) {
                if(cal == this) {
                    result.push_back(line->idx);
                    break;
                }
            }
        }
    }
    break;
    default : break;
    }
    return result;
}
std::vector<idx_t> StopArea::get(Type_e type, const PT_Data &) const {
    std::vector<idx_t> result;
    switch(type) {
    case Type_e::StopPoint: return indexes(this->stop_point_list);
    default: break;
    }
    return result;
}

std::vector<idx_t> Network::get(Type_e type, const PT_Data &) const {
    std::vector<idx_t> result;
    switch(type) {
    case Type_e::Line: return indexes(line_list);
    default: break;
    }
    return result;
}


std::vector<idx_t> Company::get(Type_e type, const PT_Data &) const {
    std::vector<idx_t> result;
    switch(type) {
    case Type_e::Line: return indexes(line_list);
    default: break;
    }
    return result;
}

std::vector<idx_t> CommercialMode::get(Type_e type, const PT_Data &) const {
    std::vector<idx_t> result;
    switch(type) {
    case Type_e::Line: return indexes(line_list);
    default: break;
    }
    return result;
}

std::vector<idx_t> PhysicalMode::get(Type_e type, const PT_Data & data) const {
    std::vector<idx_t> result;
    switch(type) {
        case Type_e::VehicleJourney:
            for (auto jp: data.journey_patterns) {
                if (jp->physical_mode != this) {
                    continue;
                }
                for (const auto& vj: jp->discrete_vehicle_journey_list) {
                    result.push_back(vj->idx);
                }
                for (const auto& vj: jp->frequency_vehicle_journey_list) {
                    result.push_back(vj->idx);
                }
            }
            break;
    default: break;
    }
    return result;
}

std::vector<idx_t> Line::get(Type_e type, const PT_Data&) const {
    std::vector<idx_t> result;
    switch(type) {
    case Type_e::CommercialMode: result.push_back(commercial_mode->idx); break;
    case Type_e::Company: return indexes(company_list);
    case Type_e::Network: result.push_back(network->idx); break;
    case Type_e::Route: return indexes(route_list);
    case Type_e::Calendar: return indexes(calendar_list);
    default: break;
    }
    return result;
}

type::hasOdtProperties Line::get_odt_properties() const{
    type::hasOdtProperties result;
    if (!this->route_list.empty()){
        for (const auto route : this->route_list) {
            result |= route->get_odt_properties();
        }
    }
    return result;
}

std::vector<idx_t> Route::get(Type_e type, const PT_Data &) const {
    std::vector<idx_t> result;
    switch(type) {
    case Type_e::Line: result.push_back(line->idx); break;
    case Type_e::JourneyPattern: return indexes(journey_pattern_list);
    default: break;
    }
    return result;
}

idx_t Route::main_destination() const {
   // StopPoint_idx, count
    std::map<idx_t, size_t> stop_point_map;
    std::pair<idx_t, size_t> best{invalid_idx, 0};
    for(const JourneyPattern* jp : this->journey_pattern_list) {

        jp->for_each_vehicle_journey([&](const VehicleJourney& vj) {
            if((!vj.stop_time_list.empty())
                && (vj.stop_time_list.back().journey_pattern_point != nullptr)
                    && (vj.stop_time_list.back().journey_pattern_point->stop_point != nullptr)){
                const StopPoint* sp = vj.stop_time_list.back().journey_pattern_point->stop_point;
                stop_point_map[sp->idx] += 1;
                size_t val = stop_point_map[sp->idx];
                if (( best.first == invalid_idx) || (best.second < val)){
                    best = {sp->idx, val};
                }
            }
            return true;
        });
    }
    return best.first;
}


type::hasOdtProperties Route::get_odt_properties() const{
    type::hasOdtProperties result;
    if (!this->journey_pattern_list.empty()){
        for (const auto jp : this->journey_pattern_list) {
            result.odt_properties |= jp->odt_properties.odt_properties;
        }
    }
    return result;
}

void JourneyPattern::build_odt_properties(){
    for_each_vehicle_journey([&](const VehicleJourney& vj) {
        if (vj.is_virtual_odt()) {
            this->odt_properties.set_virtual_odt();
        }
        if (vj.is_zonal_odt()) {
            this->odt_properties.set_zonal_odt();
        }
        return true;
    });
}

std::vector<idx_t> JourneyPattern::get(Type_e type, const PT_Data &) const {
    std::vector<idx_t> result;
    switch(type) {
    case Type_e::Route: result.push_back(route->idx); break;
    case Type_e::CommercialMode: result.push_back(commercial_mode->idx); break;
    case Type_e::JourneyPatternPoint: return indexes(journey_pattern_point_list);
    case Type_e::VehicleJourney:
        for(const auto& f_vj: frequency_vehicle_journey_list) {
            result.push_back(f_vj->idx);
        }
        for(const auto& d_vj: discrete_vehicle_journey_list) {
            result.push_back(d_vj->idx);
        }
        break;
    default: break;
    }
    return result;
}


std::vector<idx_t> VehicleJourney::get(Type_e type, const PT_Data &) const {
    std::vector<idx_t> result;
    switch(type) {
    case Type_e::JourneyPattern: result.push_back(journey_pattern->idx); break;
    case Type_e::Company: result.push_back(company->idx); break;
    case Type_e::PhysicalMode: result.push_back(journey_pattern->physical_mode->idx); break;
    case Type_e::ValidityPattern: result.push_back(validity_pattern->idx); break;
    default: break;
    }
    return result;
}

VehicleJourney::~VehicleJourney() {}
FrequencyVehicleJourney::~FrequencyVehicleJourney() {}
DiscreteVehicleJourney::~DiscreteVehicleJourney() {}

JourneyPattern::~JourneyPattern() {}

//we need to define the copy constructor because we don't want to copy the vjs
//(idx can be skiped too)
JourneyPattern::JourneyPattern(const JourneyPattern& other):
    is_frequence(other.is_frequence),
    route(other.route),
    commercial_mode(other.commercial_mode),
    physical_mode(other.physical_mode),
    journey_pattern_point_list(other.journey_pattern_point_list),
    odt_properties(other.odt_properties)
{
    name = other.name;
    uri = other.uri;
}

std::vector<idx_t> JourneyPatternPoint::get(Type_e type, const PT_Data &) const {
    std::vector<idx_t> result;
    switch(type) {
    case Type_e::JourneyPattern: result.push_back(journey_pattern->idx); break;
    case Type_e::StopPoint: result.push_back(stop_point->idx); break;
    default: break;
    }
    return result;
}

std::vector<idx_t> StopPoint::get(Type_e type, const PT_Data &) const {
    std::vector<idx_t> result;
    switch(type) {
    case Type_e::StopArea: result.push_back(stop_area->idx); break;
    case Type_e::JourneyPatternPoint: return indexes(journey_pattern_point_list);
    case Type_e::Connection:
    case Type_e::StopPointConnection:
        for (const StopPointConnection* stop_cnx : stop_point_connection_list)
            result.push_back(stop_cnx->idx);
        break;
    default: break;
    }
    return result;
}

std::vector<idx_t> StopPointConnection::get(Type_e type, const PT_Data & ) const {
    std::vector<idx_t> result;
    switch(type) {
    case Type_e::StopPoint:
        result.push_back(this->departure->idx);
        result.push_back(this->destination->idx);
        break;
    default: break;
    }
    return result;
}
bool StopPointConnection::operator<(const StopPointConnection& other) const { return this < &other; }

std::string to_string(ExceptionDate::ExceptionType t) {
    switch (t) {
    case ExceptionDate::ExceptionType::add:
        return "Add";
    case ExceptionDate::ExceptionType::sub:
        return "Sub";
    default:
        throw navitia::exception("unhandled exception type");
    }
}

EntryPoint::EntryPoint(const Type_e type, const std::string &uri, int access_duration) : type(type), uri(uri), access_duration(access_duration) {
   // Gestion des adresses
   if (type == Type_e::Address){
       std::vector<std::string> vect;
       vect = split_string(uri, ":");
       if(vect.size() == 3){
           this->uri = vect[0] + ":" + vect[1];
           this->house_number = str_to_int(vect[2]);
       }
   }
   if(type == Type_e::Coord){
       size_t pos2 = uri.find(":", 6);
       try{
           if(pos2 != std::string::npos) {
               this->coordinates.set_lon(boost::lexical_cast<double>(uri.substr(6, pos2 - 6)));
               this->coordinates.set_lat(boost::lexical_cast<double>(uri.substr(pos2+1)));
           }
       }catch(boost::bad_lexical_cast){
           this->coordinates.set_lon(0);
           this->coordinates.set_lat(0);
       }
   }
}

EntryPoint::EntryPoint(const Type_e type, const std::string &uri) : EntryPoint(type, uri, 0) { }

void StreetNetworkParams::set_filter(const std::string &param_uri){
    size_t pos = param_uri.find(":");
    if(pos == std::string::npos)
        type_filter = Type_e::Unknown;
    else {
        uri_filter = param_uri;
        type_filter = static_data::get()->typeByCaption(param_uri.substr(0,pos));
    }
}

}} //namespace navitia::type

BOOST_CLASS_EXPORT_GUID(navitia::type::DiscreteVehicleJourney, "DiscreteVehicleJourney")
BOOST_CLASS_EXPORT_GUID(navitia::type::FrequencyVehicleJourney, "FrequencyVehicleJourney")
