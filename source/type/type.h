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

#include "type/time_duration.h"
#include "datetime.h"
#include "geographical_coord.h"
#include "utils/flat_enum_map.h"
#include "utils/exception.h"
#include "utils/functions.h"
#include <boost/date_time/gregorian/gregorian.hpp>
#include <vector>
#include <bitset>

#include <boost/weak_ptr.hpp>
#include <boost/date_time/gregorian/greg_serialize.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/bitset.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/export.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/bimap.hpp>
#include <boost/serialization/export.hpp>

namespace navitia { namespace georef {
 struct Admin;
 struct GeoRef;
}}
namespace navitia { namespace type {
typedef uint32_t idx_t;

const idx_t invalid_idx = std::numeric_limits<idx_t>::max();

struct Message;
namespace new_disruption {
struct Impact;
}

#define ITERATE_NAVITIA_PT_TYPES(FUN)\
    FUN(ValidityPattern, validity_patterns)\
    FUN(Line, lines)\
    FUN(JourneyPattern, journey_patterns)\
    FUN(VehicleJourney, vehicle_journeys)\
    FUN(StopPoint, stop_points)\
    FUN(StopArea, stop_areas)\
    FUN(Network, networks)\
    FUN(PhysicalMode, physical_modes)\
    FUN(CommercialMode, commercial_modes)\
    FUN(JourneyPatternPoint, journey_pattern_points)\
    FUN(Company, companies)\
    FUN(Route, routes)\
    FUN(Contributor, contributors)\
    FUN(Calendar, calendars)

enum class Type_e {
    ValidityPattern                 = 0,
    Line                            = 1,
    JourneyPattern                  = 2,
    VehicleJourney                  = 3,
    StopPoint                       = 4,
    StopArea                        = 5,
    Network                         = 6,
    PhysicalMode                    = 7,
    CommercialMode                  = 8,
    Connection                      = 9,
    JourneyPatternPoint             = 10,
    Company                         = 11,
    Route                           = 12,
    POI                             = 13,
    StopPointConnection             = 15,
    Contributor                     = 16,

    // Objets spéciaux qui ne font pas partie du référentiel TC
    eStopTime                       = 17,
    Address                         = 18,
    Coord                           = 19,
    Unknown                         = 20,
    Way                             = 21,
    Admin                           = 22,
    POIType                         = 23,
    Calendar                        = 24
};

enum class Mode_e {
    Walking = 0,    // Marche à pied
    Bike = 1,       // Vélo
    Car = 2,        // Voiture
    Bss = 3         // Vls
    //Note: if a new transportation mode is added, don't forget to update the associated enum_size_trait<type::Mode_e>
};

enum class OdtLevel_e {
    none = 0,
    mixt = 1,
    zonal = 2,
    all = 3
};

struct PT_Data;
template<class T> std::string T::* name_getter(){return &T::name;}
template<class T> int T::* idx_getter(){return &T::idx;}


struct Nameable {
    std::string name;
    std::string comment;
    bool visible = true;
};


struct Header {
    idx_t idx = invalid_idx; // Index of the object in the main structure
    std::string uri; // unique indentifier of the object
    std::vector<idx_t> get(Type_e, const PT_Data &) const {return std::vector<idx_t>();}
};

struct Codes {
    std::map<std::string, std::string> codes;
};


typedef std::bitset<10> Properties;
struct hasProperties {
    static const uint8_t WHEELCHAIR_BOARDING = 0;
    static const uint8_t SHELTERED = 1;
    static const uint8_t ELEVATOR = 2;
    static const uint8_t ESCALATOR = 3;
    static const uint8_t BIKE_ACCEPTED = 4;
    static const uint8_t BIKE_DEPOT = 5;
    static const uint8_t VISUAL_ANNOUNCEMENT = 6;
    static const uint8_t AUDIBLE_ANNOUNVEMENT = 7;
    static const uint8_t APPOPRIATE_ESCORT = 8;
    static const uint8_t APPOPRIATE_SIGNAGE = 9;

    bool wheelchair_boarding() {return _properties[WHEELCHAIR_BOARDING];}
    bool wheelchair_boarding() const {return _properties[WHEELCHAIR_BOARDING];}
    bool sheltered() {return _properties[SHELTERED];}
    bool sheltered() const {return _properties[SHELTERED];}
    bool elevator() {return _properties[ELEVATOR];}
    bool elevator() const {return _properties[ELEVATOR];}
    bool escalator() {return _properties[ESCALATOR];}
    bool escalator() const {return _properties[ESCALATOR];}
    bool bike_accepted() {return _properties[BIKE_ACCEPTED];}
    bool bike_accepted() const {return _properties[BIKE_ACCEPTED];}
    bool bike_depot() {return _properties[BIKE_DEPOT];}
    bool bike_depot() const {return _properties[BIKE_DEPOT];}
    bool visual_announcement() {return _properties[VISUAL_ANNOUNCEMENT];}
    bool visual_announcement() const {return _properties[VISUAL_ANNOUNCEMENT];}
    bool audible_announcement() {return _properties[AUDIBLE_ANNOUNVEMENT];}
    bool audible_announcement() const {return _properties[AUDIBLE_ANNOUNVEMENT];}
    bool appropriate_escort() {return _properties[APPOPRIATE_ESCORT];}
    bool appropriate_escort() const {return _properties[APPOPRIATE_ESCORT];}
    bool appropriate_signage() {return _properties[APPOPRIATE_SIGNAGE];}
    bool appropriate_signage() const {return _properties[APPOPRIATE_SIGNAGE];}

    bool accessible(const Properties &required_properties) const{
        auto mismatched = required_properties & ~_properties;
        return mismatched.none();
    }
    bool accessible(const Properties &required_properties) {
        auto mismatched = required_properties & ~_properties;
        return mismatched.none();
    }

    void set_property(uint8_t property) {
        _properties.set(property, true);
    }

    void set_properties(const Properties &other) {
        this->_properties = other;
    }

    Properties properties() const {
        return this->_properties;
    }

    void unset_property(uint8_t property) {
        _properties.set(property, false);
    }

    bool property(uint8_t property) const {
        return _properties[property];
    }

    idx_t to_ulog(){
        return _properties.to_ulong();
    }

//private: on ne peut pas binaraisé si privé
    Properties _properties;
};

typedef std::bitset<8> VehicleProperties;
struct hasVehicleProperties {
    static const uint8_t WHEELCHAIR_ACCESSIBLE = 0;
    static const uint8_t BIKE_ACCEPTED = 1;
    static const uint8_t AIR_CONDITIONED = 2;
    static const uint8_t VISUAL_ANNOUNCEMENT = 3;
    static const uint8_t AUDIBLE_ANNOUNCEMENT = 4;
    static const uint8_t APPOPRIATE_ESCORT = 5;
    static const uint8_t APPOPRIATE_SIGNAGE = 6;
    static const uint8_t SCHOOL_VEHICLE = 7;

    bool wheelchair_accessible() {return _vehicle_properties[WHEELCHAIR_ACCESSIBLE];}
    bool wheelchair_accessible() const {return _vehicle_properties[WHEELCHAIR_ACCESSIBLE];}
    bool bike_accepted() {return _vehicle_properties[BIKE_ACCEPTED];}
    bool bike_accepted() const {return _vehicle_properties[BIKE_ACCEPTED];}
    bool air_conditioned() {return _vehicle_properties[AIR_CONDITIONED];}
    bool air_conditioned() const {return _vehicle_properties[AIR_CONDITIONED];}
    bool visual_announcement() {return _vehicle_properties[VISUAL_ANNOUNCEMENT];}
    bool visual_announcement() const {return _vehicle_properties[VISUAL_ANNOUNCEMENT];}
    bool audible_announcement() {return _vehicle_properties[AUDIBLE_ANNOUNCEMENT];}
    bool audible_announcement() const {return _vehicle_properties[AUDIBLE_ANNOUNCEMENT];}
    bool appropriate_escort() {return _vehicle_properties[APPOPRIATE_ESCORT];}
    bool appropriate_escort() const {return _vehicle_properties[APPOPRIATE_ESCORT];}
    bool appropriate_signage() {return _vehicle_properties[APPOPRIATE_SIGNAGE];}
    bool appropriate_signage() const {return _vehicle_properties[APPOPRIATE_SIGNAGE];}
    bool school_vehicle() {return _vehicle_properties[SCHOOL_VEHICLE];}
    bool school_vehicle() const {return _vehicle_properties[SCHOOL_VEHICLE];}

    bool accessible(const VehicleProperties &required_vehicles) const{
        auto mismatched = required_vehicles & ~_vehicle_properties;
        return mismatched.none();
    }
    bool accessible(const VehicleProperties &required_vehicles) {
        auto mismatched = required_vehicles & ~_vehicle_properties;
        return mismatched.none();
    }

    void set_vehicle(uint8_t vehicle) {
        _vehicle_properties.set(vehicle, true);
    }

    void set_vehicles(const VehicleProperties &other) {
        this->_vehicle_properties = other;
    }

    VehicleProperties vehicles() const {
        return this->_vehicle_properties;
    }

    void unset_vehicle(uint8_t vehicle) {
        _vehicle_properties.set(vehicle, false);
    }

    bool vehicle(uint8_t vehicle) const {
        return _vehicle_properties[vehicle];
    }

    idx_t to_ulog(){
        return _vehicle_properties.to_ulong();
    }

//private: on ne peut pas binaraisé si privé
    VehicleProperties _vehicle_properties;
};

struct HasMessages{
protected:
    mutable std::vector<boost::weak_ptr<new_disruption::Impact>> impacts;
public:
    void add_impact(const boost::shared_ptr<new_disruption::Impact>& i) {impacts.push_back(i);}

    std::vector<boost::weak_ptr<new_disruption::Impact>> get_applicable_messages(
            const boost::posix_time::ptime& current_time,
            const boost::posix_time::time_period& action_period) const;

    bool has_applicable_message(
            const boost::posix_time::ptime& current_time,
            const boost::posix_time::time_period& action_period) const;

};


enum class ConnectionType {
    StopPoint = 0,
    StopArea = 1,
    Walking = 2,
    VJ = 3,
    Guaranteed = 4,
    Default = 5,
    stay_in,
    guarantee,
    undefined
};

enum class VehicleJourneyType {
    regular = 0,                    // ligne régulière
    virtual_with_stop_time = 1,       // TAD virtuel avec horaires
    virtual_without_stop_time = 2,    // TAD virtuel sans horaires
    stop_point_to_stop_point = 3,     // TAD rabattement arrêt à arrêt
    adress_to_stop_point = 4,         // TAD rabattement adresse à arrêt
    odt_point_to_point = 5            // TAD point à point (Commune à Commune)
};

struct StopPoint;
struct Line;
struct JourneyPattern;
struct ValidityPattern;
struct Route;
struct JourneyPatternPoint;
struct VehicleJourney;
struct StopTime;

struct StopPointConnection: public Header, hasProperties{
    const static Type_e type = Type_e::Connection;
    StopPoint* departure;
    StopPoint* destination;
    int display_duration;
    int duration;
    int max_duration;
    ConnectionType connection_type;

    StopPointConnection() : departure(nullptr), destination(nullptr), display_duration(0), duration(0),
        max_duration(0){}

    template<class Archive> void serialize(Archive & ar, const unsigned int ) {
        ar & idx & uri & departure & destination & display_duration & duration &
            max_duration & connection_type & _properties;
    }

    std::vector<idx_t> get(Type_e type, const PT_Data & data) const;

    bool operator<(const StopPointConnection &other) const;

};

struct ExceptionDate {
    enum class ExceptionType {
        sub = 0,      // remove
        add = 1       // add
    };
    ExceptionType type;
    boost::gregorian::date date;
    template<class Archive> void serialize(Archive & ar, const unsigned int ) {
        ar & type & date;
    }
    inline bool operator<(const ExceptionDate& that) const {
        if (this->type < that.type) return true;
        if (that.type < this->type) return false;
        return this->date < that.date;
    }
    inline bool operator==(const ExceptionDate& that) const {
        return this->type == that.type && this->date == that.date;
    }
};
inline std::ostream& operator<<(std::ostream& os, const ExceptionDate& ed) {
    switch (ed.type) {
    case ExceptionDate::ExceptionType::add: os << "excl "; break;
    case ExceptionDate::ExceptionType::sub: os << "incl "; break;
    }
    return os << ed.date;
}

std::string to_string(ExceptionDate::ExceptionType t);

inline ExceptionDate::ExceptionType to_exception_type(const std::string& str) {
    if (str == "Add") {
        return ExceptionDate::ExceptionType::add;
    }
    if (str == "Sub") {
        return ExceptionDate::ExceptionType::sub;
    }
    throw navitia::exception("unhandled exception type: " + str);
}


struct StopArea : public Header, Nameable, hasProperties, HasMessages, Codes{
    const static Type_e type = Type_e::StopArea;
    GeographicalCoord coord;
    std::string additional_data;
    std::vector<navitia::georef::Admin*> admin_list;
    bool wheelchair_boarding = false;
    std::string label;
    //name of the time zone of the stop area
    //the name must respect the format of the tz db, for example "Europe/Paris"
    std::string timezone;

    template<class Archive> void serialize(Archive & ar, const unsigned int ) {
        ar & idx & label & uri & name & coord & stop_point_list & admin_list
        & _properties & wheelchair_boarding & impacts & visible
                & comment & codes & timezone;
    }

    std::vector<StopPoint*> stop_point_list;
    std::vector<idx_t> get(Type_e type, const PT_Data & data) const;
    bool operator<(const StopArea & other) const { return this < &other; }
};

struct Network : public Header, Nameable, HasMessages, Codes{
    const static Type_e type = Type_e::Network;
    std::string address_name;
    std::string address_number;
    std::string address_type_name;
    std::string phone_number;
    std::string mail;
    std::string website;
    std::string fax;
    int sort = std::numeric_limits<int>::max();

    std::vector<Line*> line_list;

    template<class Archive> void serialize(Archive & ar, const unsigned int ) {
        ar & idx & name & uri & address_name & address_number & address_type_name
            & mail & website & fax & sort & line_list & impacts & codes;
    }

    std::vector<idx_t> get(Type_e type, const PT_Data & data) const;
    bool operator<(const Network & other) const {
        if(this->sort != other.sort) {
            return this->sort < other.sort;
        }
        if(this->name != other.name) {
             return this->name < other.name;
        }
        return this < &other;
    }

};

struct Contributor : public Header, Nameable{
    const static Type_e type = Type_e::Contributor;

    template<class Archive> void serialize(Archive & ar, const unsigned int ) {
        ar & idx & name & uri;
    }
    bool operator<(const Contributor & other) const { return this < &other; }
};

struct Company : public Header, Nameable, Codes{
    const static Type_e type = Type_e::Company;
    std::string address_name;
    std::string address_number;
    std::string address_type_name;
    std::string phone_number;
    std::string mail;
    std::string website;
    std::string fax;

    std::vector<Line*> line_list;

    template<class Archive> void serialize(Archive & ar, const unsigned int ) {
        ar & idx & name & uri & address_name & address_number &
        address_type_name & phone_number & mail & website & fax & codes;
    }
    std::vector<idx_t> get(Type_e type, const PT_Data & data) const;
    bool operator<(const Company & other) const { return this < &other; }
};

struct CommercialMode : public Header, Nameable{
    const static Type_e type = Type_e::CommercialMode;
    std::vector<Line*> line_list;
    template<class Archive> void serialize(Archive & ar, const unsigned int ) {
        ar & idx & name & uri & line_list;
    }
    std::vector<idx_t> get(Type_e type, const PT_Data & data) const;
    bool operator<(const CommercialMode & other) const { return this < &other; }

};

struct PhysicalMode : public Header, Nameable{
    const static Type_e type = Type_e::PhysicalMode;
    double co2_emission = 0.;
    std::vector<JourneyPattern*> journey_pattern_list;

    template<class Archive> void serialize(Archive & ar, const unsigned int ) {
        ar & idx & name & uri & co2_emission & journey_pattern_list;
    }
    std::vector<idx_t> get(Type_e type, const PT_Data & data) const;

    PhysicalMode() {}
    bool operator<(const PhysicalMode & other) const { return this < &other; }

};

struct Calendar;

typedef std::bitset<2> OdtProperties;
struct hasOdtProperties {
    static const uint8_t VIRTUAL_ODT = 0;
    static const uint8_t ZONAL_ODT = 1;
    OdtProperties odt_properties;

    hasOdtProperties() {
        odt_properties.reset();
    }

    void operator=(const type::hasOdtProperties& other) {
        odt_properties = other.odt_properties;
    }

    void operator|=(const type::hasOdtProperties& other) {
        odt_properties |= other.odt_properties;
    }

    void reset_odt() {
        odt_properties.reset();
    }

    void set_regular() {
        odt_properties.reset();
    }

    void set_virtual_odt() {
        odt_properties.set(VIRTUAL_ODT, true);
    }

    void unset_virtual_odt() {
        odt_properties.set(VIRTUAL_ODT, false);
    }

    void set_zonal_odt() {
        odt_properties.set(ZONAL_ODT, true);
    }
    void unset_zonal_odt() {
        odt_properties.set(ZONAL_ODT, false);
    }

    bool is_regular() const {
        return odt_properties.none();
    }

    bool is_odt() const {
        return odt_properties.any();
    }

    bool is_mixed() const {
        return odt_properties.all();
    }

    bool is_virtual_odt() const {
        return odt_properties[VIRTUAL_ODT];
    }
    bool is_zonal_odt() const {
        return odt_properties[ZONAL_ODT];
    }

    template<class Archive> void serialize(Archive & ar, const unsigned int ) {
        ar & odt_properties;
    }
};

struct Line : public Header, Nameable, HasMessages, Codes{
    const static Type_e type = Type_e::Line;
    std::string code;
    std::string forward_name;
    std::string backward_name;

    std::string additional_data;
    std::string color;
    int sort = std::numeric_limits<int>::max();

    CommercialMode* commercial_mode = nullptr;

    std::vector<Company*> company_list;
    Network* network = nullptr;

    std::vector<Route*> route_list;
    std::vector<PhysicalMode*> physical_mode_list;
    std::vector<Calendar*> calendar_list;
    MultiLineString shape;

    template<class Archive> void serialize(Archive & ar, const unsigned int ) {
        ar & idx & name & uri & code & forward_name & backward_name
                & additional_data & color & sort & commercial_mode
                & company_list & network & route_list & physical_mode_list
                & impacts & calendar_list & codes & shape & comment;
    }
    std::vector<idx_t> get(Type_e type, const PT_Data & data) const;

    bool operator<(const Line & other) const {
        if(this->network != other.network){
            return this->network < other.network;
        }
        if(this->sort != other.sort) {
            return this->sort < other.sort;
        }
        if(this->code != other.code) {
            return navitia::pseudo_natural_sort()(this->code, other.code);
        }
        if(this->name != other.name) {
            return this->name < other.name;
        }
        return this < &other;
    }
    type::hasOdtProperties get_odt_properties() const;
};

struct Route : public Header, Nameable, HasMessages, Codes{
    const static Type_e type = Type_e::Route;
    Line* line = nullptr;
    MultiLineString shape;
    std::vector<JourneyPattern*> journey_pattern_list;

    idx_t main_destination() const;
    type::hasOdtProperties get_odt_properties() const;

    template<class Archive> void serialize(Archive & ar, const unsigned int ) {
        ar & idx & name & uri & line & journey_pattern_list & impacts & codes & shape & comment;
    }

    std::vector<idx_t> get(Type_e type, const PT_Data & data) const;
    bool operator<(const Route & other) const { return this < &other; }

};
struct JourneyPattern;
struct MetaVehicleJourney;

/**
 * A VehicleJourney is an abstract class with 2 subclasses
 *
 *  - DiscreteVehicleJourney
 * The 'classic' VJ, with expanded stop times
 *
 *  - FrequencyVehicleJourney
 * A frequency VJ, with a start, an end and frequency (headway)
 *
 * The JourneyPattern owns 2 differents list for the VJs, and both are treated differently in the algorithm (in best_stop_times)
 */
struct VehicleJourney: public Header, Nameable, hasVehicleProperties, HasMessages, Codes {
    const static Type_e type = Type_e::VehicleJourney;
    JourneyPattern* journey_pattern = nullptr;
    Company* company = nullptr;
    ValidityPattern* validity_pattern = nullptr;
    std::vector<StopTime> stop_time_list;

    // These variables are used in the case of an extension of service
    // They indicate what's the vj you can take directly after or before this one
    // They have the same block id
    VehicleJourney* next_vj = nullptr;
    VehicleJourney* prev_vj = nullptr;
    //associated meta vj
    const MetaVehicleJourney* meta_vj = nullptr;
    std::string odt_message; //TODO It seems a VJ can have either a comment or an odt_message but never both, so we could use only the 'comment' to store the odt_message

    VehicleJourneyType vehicle_journey_type = VehicleJourneyType::regular;

    // all times are stored in UTC
    // however, sometime we do not have a date to convert the time to a local value (in jormungandr)
    // For example for departure board over a period (calendar)
    // thus we store the shit needed to convert all stop times of the vehicle journey to local
    int16_t utc_to_local_offset = 0; //in seconds

    bool is_adapted = false; //REMOVE (change to enum ?)
    ValidityPattern* adapted_validity_pattern = nullptr; //REMOVE
    std::vector<VehicleJourney*> adapted_vehicle_journey_list; //REMOVE
    VehicleJourney* theoric_vehicle_journey = nullptr; //REMOVE

    std::string get_direction() const;
    bool has_date_time_estimated() const;

    bool is_odt()  const{
        return vehicle_journey_type != VehicleJourneyType::regular;
    }
    bool is_none_odt() const {return (this->vehicle_journey_type == VehicleJourneyType::regular);}
    bool is_virtual_odt() const {return (this->vehicle_journey_type == VehicleJourneyType::virtual_with_stop_time);}
    bool is_zonal_odt() const {return (this->vehicle_journey_type > VehicleJourneyType::virtual_with_stop_time);}

    bool has_boarding() const;
    bool has_landing() const;
    std::vector<idx_t> get(Type_e type, const PT_Data & data) const;

    bool operator<(const VehicleJourney& other) const;
    template<class Archive> void serialize(Archive& ar, const unsigned int ) {
        ar & name & uri & journey_pattern & company & validity_pattern
            & idx & stop_time_list & is_adapted
            & adapted_validity_pattern & adapted_vehicle_journey_list
            & theoric_vehicle_journey & comment & vehicle_journey_type
            & odt_message & _vehicle_properties & impacts
            & codes & next_vj & prev_vj
            & meta_vj & utc_to_local_offset;
    }

    type::OdtLevel_e get_odt_level() const;
    virtual ~VehicleJourney();
    //TODO remove the virtual there, but to do that we need to remove the prev/next_vj since boost::serialiaze needs to make a virtual call for those
private:
    /*
     * Note: the destructor has not been defined as virtual because we don't need those classes to
     * be virtual.
     * the JP owns 2 differents lists so no virtual call must be made on destruction
     */
    VehicleJourney() = default;
    VehicleJourney(const VehicleJourney&) = default;
    friend class boost::serialization::access;
    friend struct DiscreteVehicleJourney;
    friend struct FrequencyVehicleJourney;
};

struct DiscreteVehicleJourney: public VehicleJourney {
    virtual ~DiscreteVehicleJourney();
    template<class Archive> void serialize(Archive& ar, const unsigned int ) {
        ar & boost::serialization::base_object<VehicleJourney>(*this);
    }
};


struct FrequencyVehicleJourney: public VehicleJourney {

    uint32_t start_time = std::numeric_limits<uint32_t>::max(); // first departure hour
    uint32_t end_time = std::numeric_limits<uint32_t>::max(); // last departure hour
    uint32_t headway_secs = std::numeric_limits<uint32_t>::max(); // Seconds between each departure.
    virtual ~FrequencyVehicleJourney();

    bool is_valid(int day, const bool is_adapted) const;
    template<class Archive> void serialize(Archive& ar, const unsigned int) {
        ar & boost::serialization::base_object<VehicleJourney>(*this);

        ar & start_time & end_time & headway_secs;
    }
};

struct JourneyPattern : public Header, Nameable {
    const static Type_e type = Type_e::JourneyPattern;
    bool is_frequence = false;
    Route* route = nullptr;
    CommercialMode* commercial_mode = nullptr;
    PhysicalMode* physical_mode = nullptr;

    std::vector<JourneyPatternPoint*> journey_pattern_point_list;
    hasOdtProperties odt_properties;

    std::vector<std::unique_ptr<DiscreteVehicleJourney>> discrete_vehicle_journey_list;
    std::vector<std::unique_ptr<FrequencyVehicleJourney>> frequency_vehicle_journey_list;

    JourneyPattern() {}
    ~JourneyPattern();
    JourneyPattern(const JourneyPattern&);
    JourneyPattern operator=(const JourneyPattern&) = delete;

    template <typename T>
    void for_each_vehicle_journey(const T func) const {
        //call the functor for each vj.
        // if func return false, we stop
        for (const auto& vj: discrete_vehicle_journey_list) { if (! func(*vj)) {return;} }
        for (const auto& vj: frequency_vehicle_journey_list) { if (! func(*vj)) {return;} }
    }

    template<class Archive> void serialize(Archive & ar, const unsigned int ) {
        ar & idx & name & uri & is_frequence & odt_properties &  route & commercial_mode
                & physical_mode & journey_pattern_point_list & discrete_vehicle_journey_list
                & frequency_vehicle_journey_list;

    }

    std::vector<idx_t> get(Type_e type, const PT_Data & data) const;
    bool operator<(const JourneyPattern & other) const { return this < &other; }

    void build_odt_properties();
};

struct AssociatedCalendar {
    ///calendar matched
    const Calendar* calendar;

    ///exceptions to this association (not to be mixed up with the exceptions in the calendar)
    ///the calendar exceptions change it's validity pattern
    /// the AssociatedCalendar exceptions are the differences between the vj validity pattern and the calendar's
    std::vector<ExceptionDate> exceptions;

    template<class Archive> void serialize(Archive & ar, const unsigned int ) {
        ar & calendar & exceptions;
    }
};

struct MetaVehicleJourney;


struct StopTime {
    static const uint8_t PICK_UP = 0;
    static const uint8_t DROP_OFF = 1;
    static const uint8_t ODT = 2;
    static const uint8_t IS_FREQUENCY = 3;
    static const uint8_t WHEELCHAIR_BOARDING = 4;
    static const uint8_t DATE_TIME_ESTIMATED = 5;

    std::bitset<8> properties;
    uint16_t local_traffic_zone = std::numeric_limits<uint16_t>::max();
    uint32_t arrival_time = 0; ///< En secondes depuis minuit
    uint32_t departure_time = 0; ///< En secondes depuis minuit //TODO, comment that, with explanation for frequency VJ and non frequency VJ
    VehicleJourney* vehicle_journey = nullptr;
    JourneyPatternPoint* journey_pattern_point = nullptr;

    bool pick_up_allowed() const {return properties[PICK_UP];}
    bool drop_off_allowed() const {return properties[DROP_OFF];}
    bool odt() const {return properties[ODT];}
    bool is_frequency() const {return properties[IS_FREQUENCY];}
    bool date_time_estimated() const {return properties[DATE_TIME_ESTIMATED];}

    inline void set_pick_up_allowed(bool value) {properties[PICK_UP] = value;}
    inline void set_drop_off_allowed(bool value) {properties[DROP_OFF] = value;}
    inline void set_odt(bool value) {properties[ODT] = value;}
    inline void set_is_frequency(bool value) {properties[IS_FREQUENCY] = value;}
    inline void set_date_time_estimated(bool value) {properties[DATE_TIME_ESTIMATED] = value;}



    /// Est-ce qu'on peut finir par ce stop_time : dans le sens avant on veut descendre
    bool valid_end(bool clockwise) const {return clockwise ? drop_off_allowed() : pick_up_allowed();}

    bool is_odt_and_date_time_estimated() const{ return (this->odt() && this->date_time_estimated());}
    /// Heure de fin de stop_time : dans le sens avant, c'est la fin, sinon le départ
    uint32_t section_end_time(bool clockwise, const u_int32_t hour = 0) const {
        if(this->is_frequency())
            return clockwise ? this->f_arrival_time(hour) : this->f_departure_time(hour);
        else
            return clockwise ? arrival_time : departure_time;
    }

    uint32_t f_arrival_time(const u_int32_t hour, bool clockwise = true) const;

    uint32_t f_departure_time(const u_int32_t hour, bool clockwise = false) const;

    uint32_t end_time(const bool is_departure) const;

    uint32_t start_time(const bool is_departure) const;

    DateTime section_end_date(int date, bool clockwise) const {
        return DateTimeUtils::set(date, this->section_end_time(clockwise) % DateTimeUtils::SECONDS_PER_DAY);
    }

    bool is_valid_day(u_int32_t day, const bool is_arrival, const bool is_adapted) const;

    template<class Archive> void serialize(Archive & ar, const unsigned int ) {
            ar & arrival_time & departure_time & vehicle_journey & journey_pattern_point
            & properties & local_traffic_zone;
    }

    bool operator<(const StopTime& other) const;

};


struct ValidityPattern : public Header {
    const static Type_e type = Type_e::ValidityPattern;
private:
    bool is_valid(int duration) const;
public:
    using year_bitset = std::bitset<366>;
    year_bitset days;
    boost::gregorian::date beginning_date;

    ValidityPattern()  {}
    ValidityPattern(const boost::gregorian::date& beginning_date) : beginning_date(beginning_date){}
    ValidityPattern(const boost::gregorian::date& beginning_date, const std::string & vp) : days(vp), beginning_date(beginning_date){}
    ValidityPattern(const ValidityPattern & vp) : days(vp.days), beginning_date(vp.beginning_date){}
    ValidityPattern(const ValidityPattern* vp) : days(vp->days), beginning_date(vp->beginning_date){}

    int slide(boost::gregorian::date day) const;
    void add(boost::gregorian::date day);
    void add(int day);
    void add(boost::gregorian::date start, boost::gregorian::date end, std::bitset<7> active_days);
    void remove(boost::gregorian::date day);
    void remove(int day);
    std::string str() const;
    template<class Archive> void serialize(Archive & ar, const unsigned int ) {
        ar & beginning_date & days & idx & uri;
    }

    bool check(boost::gregorian::date day) const;
    bool check(unsigned int day) const;
    bool check2(unsigned int day) const;
    bool uncheck2(unsigned int day) const;
    //void add(boost::gregorian::date start, boost::gregorian::date end, std::bitset<7> active_days);
    bool operator<(const ValidityPattern & other) const { return this < &other; }
    bool operator==(const ValidityPattern & other) const { return (this->beginning_date == other.beginning_date) && (this->days == other.days);}
};

struct StopPoint : public Header, Nameable, hasProperties, HasMessages, Codes{
    const static Type_e type = Type_e::StopPoint;
    GeographicalCoord coord;
    int fare_zone;
    std::string platform_code;
    std::string label;

    StopArea* stop_area;
    std::vector<navitia::georef::Admin*> admin_list;
    Network* network;
    std::vector<JourneyPatternPoint*> journey_pattern_point_list;
    std::vector<StopPointConnection*> stop_point_connection_list;

    template<class Archive> void serialize(Archive & ar, const unsigned int ) {
        journey_pattern_point_list.resize(0);
        ar & uri & label & name & stop_area & coord & fare_zone & idx & platform_code
            & journey_pattern_point_list & admin_list & _properties & impacts
            & stop_point_connection_list & comment & codes;
    }

    StopPoint(): fare_zone(0),  stop_area(nullptr), network(nullptr) {}

    std::vector<idx_t> get(Type_e type, const PT_Data & data) const;
    bool operator<(const StopPoint & other) const { return this < &other; }

};


struct JourneyPatternPoint : public Header{
    const static Type_e type = Type_e::JourneyPatternPoint;
    JourneyPattern* journey_pattern;
    StopPoint* stop_point;
    uint16_t order;
    LineString shape_from_prev;

    JourneyPatternPoint() : journey_pattern(nullptr), stop_point(nullptr), order(0){}

    // Attention la sérialisation est répartrie dans deux methode: save et load
    template<class Archive> void save(Archive & ar, const unsigned int) const{
        ar & idx & uri & order & journey_pattern & stop_point & order & shape_from_prev;
    }
    template<class Archive> void load(Archive & ar, const unsigned int) {
        ar & idx & uri & order & journey_pattern & stop_point & order & shape_from_prev;
        //on remplit le tableau des stoppoints, bizarrement ca segfault au chargement si on le fait à la bina...
        this->stop_point->journey_pattern_point_list.push_back(this);
    }
    BOOST_SERIALIZATION_SPLIT_MEMBER()
    std::vector<idx_t> get(Type_e type, const PT_Data & data) const;

    bool operator<(const JourneyPatternPoint& jpp2) const {
        return this->journey_pattern < jpp2.journey_pattern  || (this->journey_pattern == jpp2.journey_pattern && this->order < jpp2.order);}

};


struct Calendar : public Nameable, public Header, public Codes {
    const static Type_e type = Type_e::Calendar;
    typedef std::bitset<7> Week;
    Week week_pattern;
    std::vector<boost::gregorian::date_period> active_periods;
    std::vector<ExceptionDate> exceptions;

    ValidityPattern validity_pattern; //computed validity pattern

    Calendar() {}
    Calendar(boost::gregorian::date beginning_date);



    bool operator<(const Calendar & other) const { return this < &other; }

    std::vector<idx_t> get(Type_e type, const PT_Data & data) const;
    template<class Archive> void serialize(Archive & ar, const unsigned int ) {
        ar & name & idx & uri & week_pattern & active_periods & exceptions & validity_pattern & codes;
    }
};

/**
 * A meta vj is a shell around some vehicle journeys
 *
 * It has 2 purposes:
 *
 *  - to store the adapted and real time vj
 *
 *  - sometime we have to split a vj.
 *    For example we have to split a vj because of dst (day saving light see gtfs parser for that)
 *    the meta vj can thus make the link between the split vjs
 *    *NOTE*: An IMPORTANT prerequisite is that ALL theoric vj have the same local time
 *            (even if the UTC time is different because of DST)
 *            That prerequisite is very important for calendar association and departure board over period
 *
 *
 */
struct MetaVehicleJourney {
    //store the name ?
    //TODO if needed use a flat_enum_map
    std::vector<VehicleJourney*> theoric_vj;
    std::vector<VehicleJourney*> adapted_vj;
    std::vector<VehicleJourney*> real_time_vj;

    /// map of the calendars that nearly match union of the validity pattern
    /// of the theoric vj, key is the calendar name
    std::map<std::string, AssociatedCalendar*> associated_calendars;

    template<class Archive> void serialize(Archive & ar, const unsigned int ) {
        ar & theoric_vj & adapted_vj & real_time_vj & associated_calendars;
    }
};

struct static_data {
private:
    static static_data * instance;
public:
    static static_data * get();
    // static std::string getListNameByType(Type_e type);
    static boost::posix_time::ptime parse_date_time(const std::string& s);
    static Type_e typeByCaption(const std::string & type_str);
    static std::string captionByType(Type_e type);
    boost::bimap<Type_e, std::string> types_string;
    static Mode_e modeByCaption(const std::string & mode_str);
    boost::bimap<Mode_e, std::string> modes_string;
};

/**

Gestion des paramètres de rabattement

*/
struct StreetNetworkParams{
    Mode_e mode;
    idx_t offset;
    float speed_factor;
    navitia::time_duration max_duration;
    Type_e type_filter; // filtre sur le départ/arrivée : exemple les arrêts les plus proches à une site type
    std::string uri_filter; // l'uri de l'objet
    float radius_filter; // ce paramètre est utilisé pour le filtre
    StreetNetworkParams():
                mode(Mode_e::Walking),
                offset(0),
                speed_factor(1),
                max_duration(navitia::seconds(1)),
                type_filter(Type_e::Unknown),
                uri_filter(""),
                radius_filter(150){}
    void set_filter(const std::string & param_uri);
};
/**
  Gestion de l'accessibilié
  */
struct AccessibiliteParams{
    Properties properties;  // Accissibilié StopPoint, Correspondance, ..
    VehicleProperties vehicle_properties; // Accissibilié VehicleJourney

    AccessibiliteParams(){}
};

/** Type pour gérer le polymorphisme en entrée de l'API
  *
  * Les objets on un identifiant universel de type stop_area:872124
  * Ces identifiants ne devraient pas être générés par le média
  * C'est toujours NAViTiA qui le génère pour être repris tel quel par le média
  */
struct EntryPoint {
    Type_e type;//< Le type de l'objet
    std::string uri; //< Le code externe de l'objet
    int house_number;
    int access_duration;
    GeographicalCoord coordinates;  // < coordonnées du point d'entrée
    StreetNetworkParams streetnetwork_params;        // < paramètres de rabatement du point d'entrée

    /// Construit le type à partir d'une chaîne
    EntryPoint(const Type_e type, const std::string & uri);
    EntryPoint(const Type_e type, const std::string & uri, int access_duration);

    EntryPoint() : type(Type_e::Unknown), house_number(-1), access_duration(0) {}
    bool set_mode(const std::string& mode) {
        if (mode == "walking") {
            streetnetwork_params.mode = Mode_e::Walking;
        } else if (mode == "bike") {
            streetnetwork_params.mode = Mode_e::Bike;
        } else if (mode == "bss") {
            streetnetwork_params.mode = Mode_e::Bss;
        } else if (mode == "car") {
            streetnetwork_params.mode = Mode_e::Car;
        } else {
            return false;
        }
        return true;
    }
};

template<typename T>
std::string get_admin_name(const T* v) {
    std::string admin_name = "";
    for(auto admin : v->admin_list) {
        if (admin->level == 8){
            admin_name += " (" + admin->name + ")";
        }
    }
    return admin_name;
}
} //namespace navitia::type

//trait to access the number of elements in the Mode_e enum
template <>
struct enum_size_trait<type::Mode_e> {
    static constexpr typename get_enum_type<type::Mode_e>::type size() {
        return 4;
    }
};

} //namespace navitia

