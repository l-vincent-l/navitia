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

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE raptor_odt_test
#include <boost/test/unit_test.hpp>
#include "routing/raptor_api.h"
#include "ed/build_helper.h"
#include "tests/utils_test.h"
#include "routing/raptor.h"
#include "georef/street_network.h"


struct logger_initialized {
    logger_initialized()   { init_logger(); }
};
BOOST_GLOBAL_FIXTURE( logger_initialized )

class Params{
public:
    std::vector<std::string> forbidden;
    ed::builder b;
    navitia::type::EntryPoint origin;
    navitia::type::EntryPoint destination;
    std::unique_ptr<navitia::routing::RAPTOR> raptor;
    std::unique_ptr<navitia::georef::StreetNetwork> street_network;
    pbnavitia::Response resp;
    pbnavitia::Journey journey;
    pbnavitia::Section section;
    pbnavitia::PtDisplayInfo displ;
    navitia::type::VehicleJourney* vj1;
    navitia::type::VehicleJourney* vj2;

    Params():b("20140113") {

        b.vj("network:R", "line:A", "1111111111111", "", true, "vj1")("stop_area:stop1", 10 * 3600 + 15 * 60, 10 * 3600 + 15 * 60)("stop_area:stop2", 11 * 3600 + 10 * 60, 11 * 3600 + 10 * 60);
        b.vj("network:R", "line:C", "1111111111111", "", true, "vj2")("stop_area:stop1", 10 * 3600 + 30 * 60, 10 * 3600 + 30 * 60)("stop_area:stop2", 11 * 3600 + 25 * 60, 11 * 3600 + 25 * 60);
        b.vj("network:R", "line:D", "1111111111111", "", true, "vj3")("stop_area:stop2", 11 * 3600 + 30 * 60, 11 * 3600 + 30 * 60)("stop_area:stop3", 13 * 3600 + 10 * 60 ,13 * 3600 + 10 * 60);
        b.vj("network:R", "line:E", "1111111111111", "", true, "vj4")("stop_area:stop4", 8 * 3600 + 30 * 60, 8* 3600 + 30 * 60)("stop_area:stop1", 9 * 3600 + 10 * 60 , 9 * 3600 + 10 * 60);
        b.vj("network:R", "line:F", "1111111111111", "", true, "vj5")("stop_area:stop5", 11 * 3600 + 30 * 60, 11 * 3600 + 30 * 60)("stop_area:stop6", 13 * 3600 + 10 * 60 ,13 * 3600 + 10 * 60);
        b.vj("network:R", "line:G", "1111111111111", "", true, "vj6")("stop_area:stop7", 8 * 3600 + 30 * 60, 8* 3600 + 30 * 60)("stop_area:stop8", 9 * 3600 + 10 * 60 , 9 * 3600 + 10 * 60);
        b.connection("stop_area:stop2", "stop_area:stop2", 120);
        b.connection("stop_area:stop1", "stop_area:stop1", 120);
        b.connection("stop_area:stop2", "stop_area:stop5", 120);
        b.connection("stop_area:stop8", "stop_area:stop1", 120);


        b.build_relations(*b.data->pt_data);
        b.generate_dummy_basis();
        b.data->pt_data->index();
        b.data->pt_data->sort();
        b.data->build_raptor();
        b.data->build_uri();

        origin = navitia::type::EntryPoint(navitia::type::Type_e::StopPoint, "stop_area:stop1");
        destination = navitia::type::EntryPoint (navitia::type::Type_e::StopPoint, "stop_area:stop2");
        street_network = std::make_unique<navitia::georef::StreetNetwork>(*b.data->geo_ref);
        raptor = std::make_unique<navitia::routing::RAPTOR>(*b.data);

        b.data->meta->production_date = boost::gregorian::date_period(boost::gregorian::date(2014,01,13), boost::gregorian::date(2014,01,25));
        vj1 = b.data->pt_data->vehicle_journeys_map["vj1"];
        vj2 = b.data->pt_data->vehicle_journeys_map["vj2"];

    }

    pbnavitia::Response make_response(bool allow_odt) {
        return make_response(allow_odt, origin, destination);
    }

    pbnavitia::Response make_response(bool allow_odt, navitia::type::EntryPoint from,
                                      navitia::type::EntryPoint to) {
        return navitia::routing::make_response(*raptor, from, to,
                               {navitia::test::to_posix_timestamp("20140114T101000")}, true,
                               navitia::type::AccessibiliteParams(),
                               forbidden,
                               *street_network, false, allow_odt);
    }
    void unset_odt_jp_and_vj(){
        for(navitia::type::JourneyPattern* jp : b.data->pt_data->journey_patterns){
            jp->odt_properties.reset_odt();
        }
        for (navitia::type::VehicleJourney* vj : b.data->pt_data->vehicle_journeys){
            vj->vehicle_journey_type = navitia::type::VehicleJourneyType::regular;
        }
    }
};

BOOST_FIXTURE_TEST_SUITE(allow_odt, Params)


/*
   Testing the behavior if :
   VJ1 : is regular
   VJ2 : is regular
*/
BOOST_AUTO_TEST_CASE(test1){
    vj1->vehicle_journey_type = navitia::type::VehicleJourneyType::regular;
    vj2->vehicle_journey_type = navitia::type::VehicleJourneyType::regular;
    b.data->aggregate_odt();
    resp = make_response(true);
    BOOST_CHECK_EQUAL(resp.response_type(), pbnavitia::ITINERARY_FOUND);
    BOOST_REQUIRE_EQUAL(resp.journeys_size(), 1);
    journey = resp.journeys(0);
    section = journey.sections(0);
    displ = section.pt_display_informations();
    BOOST_CHECK_EQUAL(displ.uris().vehicle_journey(), "vj1");
    BOOST_CHECK_EQUAL(journey.duration(), 3300);
    BOOST_CHECK_EQUAL(journey.departure_date_time(), navitia::test::to_posix_timestamp("20140114T101500")); //date are now posix timestamp
    BOOST_CHECK_EQUAL(journey.arrival_date_time(), navitia::test::to_posix_timestamp("20140114T111000"));

    resp = make_response(false);
    BOOST_REQUIRE_EQUAL(resp.journeys_size(), 1);
    journey = resp.journeys(0);
    section = journey.sections(0);
    displ = section.pt_display_informations();
    BOOST_CHECK_EQUAL(displ.uris().vehicle_journey(), "vj1");
    BOOST_CHECK_EQUAL(journey.duration(), 3300);
    BOOST_CHECK_EQUAL(journey.departure_date_time(), navitia::test::to_posix_timestamp("20140114T101500"));
    BOOST_CHECK_EQUAL(journey.arrival_date_time(), navitia::test::to_posix_timestamp("20140114T111000"));
}

/*
   Testing the behavior if :
   VJ1 : is virtual
   VJ2 : is virtual

*/
BOOST_AUTO_TEST_CASE(test2){
    vj1->vehicle_journey_type = navitia::type::VehicleJourneyType::virtual_with_stop_time;
    vj2->vehicle_journey_type = navitia::type::VehicleJourneyType::virtual_with_stop_time;
    b.data->aggregate_odt();
    resp = make_response(true);
    BOOST_CHECK_EQUAL(resp.response_type(), pbnavitia::ITINERARY_FOUND);
    BOOST_REQUIRE_EQUAL(resp.journeys_size(), 1);
    journey = resp.journeys(0);
    section = journey.sections(0);
    displ = section.pt_display_informations();
    BOOST_CHECK_EQUAL(displ.uris().vehicle_journey(), "vj1");
    BOOST_CHECK_EQUAL(journey.duration(), 3300);
    BOOST_CHECK_EQUAL(journey.departure_date_time(), navitia::test::to_posix_timestamp("20140114T101500"));
    BOOST_CHECK_EQUAL(journey.arrival_date_time(), navitia::test::to_posix_timestamp("20140114T111000"));

    resp = make_response(false);
    BOOST_CHECK_EQUAL(resp.response_type(), pbnavitia::ITINERARY_FOUND);
    BOOST_REQUIRE_EQUAL(resp.journeys_size(), 1);
    journey = resp.journeys(0);
    section = journey.sections(0);
    displ = section.pt_display_informations();
    BOOST_CHECK_EQUAL(displ.uris().vehicle_journey(), "vj1");
    BOOST_CHECK_EQUAL(journey.duration(), 3300);
    BOOST_CHECK_EQUAL(journey.departure_date_time(), navitia::test::to_posix_timestamp("20140114T101500"));
    BOOST_CHECK_EQUAL(journey.arrival_date_time(), navitia::test::to_posix_timestamp("20140114T111000"));
    unset_odt_jp_and_vj();
}

/*
   Testing the behavior if :
   VJ1 : is virtual
   VJ2 : is regular
*/
BOOST_AUTO_TEST_CASE(test3){
    vj1->stop_time_list.front().set_odt(true);
    vj1->stop_time_list.front().set_date_time_estimated(true);
    vj1->vehicle_journey_type = navitia::type::VehicleJourneyType::virtual_with_stop_time;
    vj2->vehicle_journey_type = navitia::type::VehicleJourneyType::regular;
    b.data->aggregate_odt();
    resp = make_response(true);
    BOOST_CHECK_EQUAL(resp.response_type(), pbnavitia::ITINERARY_FOUND);
    BOOST_REQUIRE_EQUAL(resp.journeys_size(), 1);
    journey = resp.journeys(0);
    section = journey.sections(0);
    displ = section.pt_display_informations();
    BOOST_CHECK_EQUAL(displ.uris().vehicle_journey(), "vj1");
    BOOST_CHECK_EQUAL(journey.duration(), 3300);
    BOOST_CHECK_EQUAL(journey.departure_date_time(), navitia::test::to_posix_timestamp("20140114T101500"));
    BOOST_CHECK_EQUAL(journey.arrival_date_time(), navitia::test::to_posix_timestamp("20140114T111000"));

    resp = make_response(false);
    BOOST_CHECK_EQUAL(resp.response_type(), pbnavitia::ITINERARY_FOUND);
    BOOST_REQUIRE_EQUAL(resp.journeys_size(), 1);
    journey = resp.journeys(0);
    section = journey.sections(0);
    displ = section.pt_display_informations();
    BOOST_CHECK_EQUAL(displ.uris().vehicle_journey(), "vj1");
    BOOST_CHECK_EQUAL(journey.duration(), 3300);
    BOOST_CHECK_EQUAL(journey.departure_date_time(), navitia::test::to_posix_timestamp("20140114T101500"));
    BOOST_CHECK_EQUAL(journey.arrival_date_time(), navitia::test::to_posix_timestamp("20140114T111000"));
    unset_odt_jp_and_vj();
}

/*
   Testing the behavior if :
   VJ1 : is regular
   VJ2 : is virtual
*/
BOOST_AUTO_TEST_CASE(test4){
    vj1->stop_time_list.front().set_odt(true);
    vj1->stop_time_list.front().set_date_time_estimated(true);
    vj1->stop_time_list.back().set_odt(true);
    vj1->vehicle_journey_type = navitia::type::VehicleJourneyType::regular;
    vj2->vehicle_journey_type = navitia::type::VehicleJourneyType::virtual_with_stop_time;
    b.data->aggregate_odt();
    resp = make_response(true);
    BOOST_CHECK_EQUAL(resp.response_type(), pbnavitia::ITINERARY_FOUND);
    BOOST_REQUIRE_EQUAL(resp.journeys_size(), 1);
    journey = resp.journeys(0);
    section = journey.sections(0);
    displ = section.pt_display_informations();
    BOOST_CHECK_EQUAL(displ.uris().vehicle_journey(), "vj1");
    BOOST_CHECK_EQUAL(journey.duration(), 3300);
    BOOST_CHECK_EQUAL(journey.departure_date_time(), navitia::test::to_posix_timestamp("20140114T101500"));
    BOOST_CHECK_EQUAL(journey.arrival_date_time(), navitia::test::to_posix_timestamp("20140114T111000"));

    resp = make_response(false);
    BOOST_CHECK_EQUAL(resp.response_type(), pbnavitia::ITINERARY_FOUND);
    BOOST_REQUIRE_EQUAL(resp.journeys_size(), 1);
    journey = resp.journeys(0);
    section = journey.sections(0);
    displ = section.pt_display_informations();
    BOOST_CHECK_EQUAL(displ.uris().vehicle_journey(), "vj1");
    BOOST_CHECK_EQUAL(journey.duration(), 3300);
    BOOST_CHECK_EQUAL(journey.departure_date_time(), navitia::test::to_posix_timestamp("20140114T101500"));
    BOOST_CHECK_EQUAL(journey.arrival_date_time(), navitia::test::to_posix_timestamp("20140114T111000"));
    unset_odt_jp_and_vj();
}

/*
   Testing the behavior if :
   VJ1 : is zonal (virtual_without_stop_time)
   VJ2 : is regular
*/
BOOST_AUTO_TEST_CASE(test5){
    vj1->stop_time_list.front().set_odt(true);
    vj1->stop_time_list.front().set_date_time_estimated(true);
    vj1->stop_time_list.back().set_odt(true);
    vj1->stop_time_list.back().set_date_time_estimated(true);
    vj1->vehicle_journey_type = navitia::type::VehicleJourneyType::virtual_without_stop_time;
    vj2->vehicle_journey_type = navitia::type::VehicleJourneyType::regular;
    b.data->aggregate_odt();
    resp = make_response(true);
    BOOST_CHECK_EQUAL(resp.response_type(), pbnavitia::ITINERARY_FOUND);
    BOOST_REQUIRE_EQUAL(resp.journeys_size(), 1);
    journey = resp.journeys(0);
    section = journey.sections(0);
    displ = section.pt_display_informations();
    BOOST_CHECK_EQUAL(displ.uris().vehicle_journey(), "vj1");
    BOOST_CHECK_EQUAL(journey.duration(), 3300);
    BOOST_CHECK_EQUAL(journey.departure_date_time(), navitia::test::to_posix_timestamp("20140114T101500"));
    BOOST_CHECK_EQUAL(journey.arrival_date_time(), navitia::test::to_posix_timestamp("20140114T111000"));

    resp = make_response(false);
    BOOST_CHECK_EQUAL(resp.response_type(), pbnavitia::ITINERARY_FOUND);
    BOOST_REQUIRE_EQUAL(resp.journeys_size(), 1);
    journey = resp.journeys(0);
    section = journey.sections(0);
    displ = section.pt_display_informations();
    BOOST_CHECK_EQUAL(displ.uris().vehicle_journey(), "vj2");
    BOOST_CHECK_EQUAL(journey.duration(), 3300);
    BOOST_CHECK_EQUAL(journey.departure_date_time(), navitia::test::to_posix_timestamp("20140114T103000"));
    BOOST_CHECK_EQUAL(journey.arrival_date_time(), navitia::test::to_posix_timestamp("20140114T112500"));
    vj1->stop_time_list.front().set_odt(false);
    vj1->stop_time_list.front().set_date_time_estimated(false);
    vj1->stop_time_list.back().set_odt(false);
    vj1->stop_time_list.back().set_date_time_estimated(false);
    unset_odt_jp_and_vj();
}

/*
   Testing the behavior if :
   VJ1 : is regular
   VJ2 : is zonal (virtual_without_stop_time)
*/
BOOST_AUTO_TEST_CASE(test6){
    vj2->stop_time_list.front().set_odt(true);
    vj1->vehicle_journey_type = navitia::type::VehicleJourneyType::regular;
    vj2->vehicle_journey_type = navitia::type::VehicleJourneyType::virtual_without_stop_time;
    b.data->aggregate_odt();
    resp = make_response(true);
    BOOST_CHECK_EQUAL(resp.response_type(), pbnavitia::ITINERARY_FOUND);
    BOOST_REQUIRE_EQUAL(resp.journeys_size(), 1);
    journey = resp.journeys(0);
    section = journey.sections(0);
    displ = section.pt_display_informations();
    BOOST_CHECK_EQUAL(displ.uris().vehicle_journey(), "vj1");
    BOOST_CHECK_EQUAL(journey.duration(), 3300);
    BOOST_CHECK_EQUAL(journey.departure_date_time(), navitia::test::to_posix_timestamp("20140114T101500"));
    BOOST_CHECK_EQUAL(journey.arrival_date_time(), navitia::test::to_posix_timestamp("20140114T111000"));

    resp = make_response(false);
    BOOST_CHECK_EQUAL(resp.response_type(), pbnavitia::ITINERARY_FOUND);
    BOOST_REQUIRE_EQUAL(resp.journeys_size(), 1);
    journey = resp.journeys(0);
    section = journey.sections(0);
    displ = section.pt_display_informations();
    BOOST_CHECK_EQUAL(displ.uris().vehicle_journey(), "vj1");
    BOOST_CHECK_EQUAL(journey.duration(), 3300);
    BOOST_CHECK_EQUAL(journey.departure_date_time(), navitia::test::to_posix_timestamp("20140114T101500"));
    BOOST_CHECK_EQUAL(journey.arrival_date_time(), navitia::test::to_posix_timestamp("20140114T111000"));
    unset_odt_jp_and_vj();
}

/*
   Testing the behavior if :
   VJ1 : is virtual
   VJ2 : is zonal (virtual_without_stop_time)
*/
BOOST_AUTO_TEST_CASE(test7){
    vj2->stop_time_list.front().set_odt(true);
    vj2->stop_time_list.front().set_date_time_estimated(true);
    vj1->vehicle_journey_type = navitia::type::VehicleJourneyType::virtual_with_stop_time;
    vj2->vehicle_journey_type = navitia::type::VehicleJourneyType::virtual_without_stop_time;
    b.data->aggregate_odt();
    resp = make_response(true);
    BOOST_CHECK_EQUAL(resp.response_type(), pbnavitia::ITINERARY_FOUND);
    BOOST_REQUIRE_EQUAL(resp.journeys_size(), 1);
    journey = resp.journeys(0);
    section = journey.sections(0);
    displ = section.pt_display_informations();
    BOOST_CHECK_EQUAL(displ.uris().vehicle_journey(), "vj1");
    BOOST_CHECK_EQUAL(journey.duration(), 3300);
    BOOST_CHECK_EQUAL(journey.departure_date_time(), navitia::test::to_posix_timestamp("20140114T101500"));
    BOOST_CHECK_EQUAL(journey.arrival_date_time(), navitia::test::to_posix_timestamp("20140114T111000"));

    resp = make_response(false);
    BOOST_CHECK_EQUAL(resp.response_type(), pbnavitia::ITINERARY_FOUND);
    BOOST_REQUIRE_EQUAL(resp.journeys_size(), 1);
    journey = resp.journeys(0);
    section = journey.sections(0);
    displ = section.pt_display_informations();
    BOOST_CHECK_EQUAL(displ.uris().vehicle_journey(), "vj1");
    BOOST_CHECK_EQUAL(journey.duration(), 3300);
    BOOST_CHECK_EQUAL(journey.departure_date_time(), navitia::test::to_posix_timestamp("20140114T101500"));
    BOOST_CHECK_EQUAL(journey.arrival_date_time(), navitia::test::to_posix_timestamp("20140114T111000"));
    unset_odt_jp_and_vj();
}

/*
   Testing the behavior if :
   VJ1 : is virtual
   VJ2 : is zonal (stop_point_to_stop_point)
*/
BOOST_AUTO_TEST_CASE(test8){
    vj2->stop_time_list.front().set_odt(true);
    vj2->stop_time_list.front().set_date_time_estimated(true);
    vj2->stop_time_list.back().set_odt(true);
    vj1->vehicle_journey_type = navitia::type::VehicleJourneyType::virtual_with_stop_time;
    vj2->vehicle_journey_type = navitia::type::VehicleJourneyType::stop_point_to_stop_point;
    b.data->aggregate_odt();
    resp = make_response(true);
    BOOST_CHECK_EQUAL(resp.response_type(), pbnavitia::ITINERARY_FOUND);
    BOOST_REQUIRE_EQUAL(resp.journeys_size(), 1);
    journey = resp.journeys(0);
    section = journey.sections(0);
    displ = section.pt_display_informations();
    BOOST_CHECK_EQUAL(displ.uris().vehicle_journey(), "vj1");
    BOOST_CHECK_EQUAL(journey.duration(), 3300);
    BOOST_CHECK_EQUAL(journey.departure_date_time(), navitia::test::to_posix_timestamp("20140114T101500"));
    BOOST_CHECK_EQUAL(journey.arrival_date_time(), navitia::test::to_posix_timestamp("20140114T111000"));

    resp = make_response(false);
    BOOST_CHECK_EQUAL(resp.response_type(), pbnavitia::ITINERARY_FOUND);
    BOOST_REQUIRE_EQUAL(resp.journeys_size(), 1);
    journey = resp.journeys(0);
    section = journey.sections(0);
    displ = section.pt_display_informations();
    BOOST_CHECK_EQUAL(displ.uris().vehicle_journey(), "vj1");
    BOOST_CHECK_EQUAL(journey.duration(), 3300);
    BOOST_CHECK_EQUAL(journey.departure_date_time(), navitia::test::to_posix_timestamp("20140114T101500"));
    BOOST_CHECK_EQUAL(journey.arrival_date_time(), navitia::test::to_posix_timestamp("20140114T111000"));
    unset_odt_jp_and_vj();
}

/*
   Testing the behavior if :
   VJ1 : is virtual
   VJ2 : is zonal (adress_to_stop_point)
*/
BOOST_AUTO_TEST_CASE(test9){
    vj2->stop_time_list.front().set_odt(true);
    vj2->stop_time_list.front().set_date_time_estimated(true);
    vj2->stop_time_list.back().set_odt(true);
    vj2->stop_time_list.back().set_date_time_estimated(true);
    vj1->vehicle_journey_type = navitia::type::VehicleJourneyType::virtual_with_stop_time;
    vj2->vehicle_journey_type = navitia::type::VehicleJourneyType::adress_to_stop_point;
    b.data->aggregate_odt();
    resp = make_response(true);
    BOOST_CHECK_EQUAL(resp.response_type(), pbnavitia::ITINERARY_FOUND);
    BOOST_REQUIRE_EQUAL(resp.journeys_size(), 1);
    journey = resp.journeys(0);
    section = journey.sections(0);
    displ = section.pt_display_informations();
    BOOST_CHECK_EQUAL(displ.uris().vehicle_journey(), "vj1");
    BOOST_CHECK_EQUAL(journey.duration(), 3300);
    BOOST_CHECK_EQUAL(journey.departure_date_time(), navitia::test::to_posix_timestamp("20140114T101500"));
    BOOST_CHECK_EQUAL(journey.arrival_date_time(), navitia::test::to_posix_timestamp("20140114T111000"));

    resp = make_response(false);
    BOOST_CHECK_EQUAL(resp.response_type(), pbnavitia::ITINERARY_FOUND);
    BOOST_REQUIRE_EQUAL(resp.journeys_size(), 1);
    journey = resp.journeys(0);
    section = journey.sections(0);
    displ = section.pt_display_informations();
    BOOST_CHECK_EQUAL(displ.uris().vehicle_journey(), "vj1");
    BOOST_CHECK_EQUAL(journey.duration(), 3300);
    BOOST_CHECK_EQUAL(journey.departure_date_time(), navitia::test::to_posix_timestamp("20140114T101500"));
    BOOST_CHECK_EQUAL(journey.arrival_date_time(), navitia::test::to_posix_timestamp("20140114T111000"));
    unset_odt_jp_and_vj();
}

/*
   Testing the behavior if :
   VJ1 : is virtual
   VJ2 : is zonal (odt_point_to_point)
*/
BOOST_AUTO_TEST_CASE(test10){
    vj1->stop_time_list.front().set_odt(true);
    vj2->stop_time_list.front().set_odt(true);
    vj1->vehicle_journey_type = navitia::type::VehicleJourneyType::virtual_with_stop_time;
    vj2->vehicle_journey_type = navitia::type::VehicleJourneyType::odt_point_to_point;
    b.data->aggregate_odt();
    resp = make_response(true);
    BOOST_CHECK_EQUAL(resp.response_type(), pbnavitia::ITINERARY_FOUND);
    BOOST_REQUIRE_EQUAL(resp.journeys_size(), 1);
    journey = resp.journeys(0);
    section = journey.sections(0);
    displ = section.pt_display_informations();
    BOOST_CHECK_EQUAL(displ.uris().vehicle_journey(), "vj1");
    BOOST_CHECK_EQUAL(journey.duration(), 3300);
    BOOST_CHECK_EQUAL(journey.departure_date_time(), navitia::test::to_posix_timestamp("20140114T101500"));
    BOOST_CHECK_EQUAL(journey.arrival_date_time(), navitia::test::to_posix_timestamp("20140114T111000"));

    resp = make_response(false);
    BOOST_CHECK_EQUAL(resp.response_type(), pbnavitia::ITINERARY_FOUND);
    BOOST_REQUIRE_EQUAL(resp.journeys_size(), 1);
    journey = resp.journeys(0);
    section = journey.sections(0);
    displ = section.pt_display_informations();
    BOOST_CHECK_EQUAL(displ.uris().vehicle_journey(), "vj1");
    BOOST_CHECK_EQUAL(journey.duration(), 3300);
    BOOST_CHECK_EQUAL(journey.departure_date_time(), navitia::test::to_posix_timestamp("20140114T101500"));
    BOOST_CHECK_EQUAL(journey.arrival_date_time(), navitia::test::to_posix_timestamp("20140114T111000"));
    unset_odt_jp_and_vj();
}

/*
   Testing the behavior if :
   VJ1 : is zonal
   VJ2 : is zonal
*/
BOOST_AUTO_TEST_CASE(test13){
    vj1->stop_time_list.front().set_odt(true);
    vj1->stop_time_list.front().set_date_time_estimated(true);
    vj1->stop_time_list.back().set_odt(true);
    vj1->stop_time_list.back().set_date_time_estimated(true);

    vj2->stop_time_list.front().set_odt(true);
    vj2->stop_time_list.front().set_date_time_estimated(true);
    vj2->stop_time_list.back().set_odt(true);
    vj2->stop_time_list.back().set_date_time_estimated(true);
    vj1->vehicle_journey_type = navitia::type::VehicleJourneyType::odt_point_to_point;
    vj2->vehicle_journey_type = navitia::type::VehicleJourneyType::odt_point_to_point;

    b.data->aggregate_odt();
    resp = make_response(true);
    BOOST_CHECK_EQUAL(resp.response_type(), pbnavitia::ITINERARY_FOUND);
    BOOST_REQUIRE_EQUAL(resp.journeys_size(), 1);
    journey = resp.journeys(0);
    section = journey.sections(0);
    displ = section.pt_display_informations();
    BOOST_CHECK_EQUAL(displ.uris().vehicle_journey(), "vj1");
    BOOST_CHECK_EQUAL(journey.duration(), 3300);
    BOOST_CHECK_EQUAL(journey.departure_date_time(), navitia::test::to_posix_timestamp("20140114T101500"));
    BOOST_CHECK_EQUAL(journey.arrival_date_time(), navitia::test::to_posix_timestamp("20140114T111000"));

    resp = make_response(false);
    BOOST_CHECK_EQUAL(resp.response_type(), pbnavitia::NO_SOLUTION);
    unset_odt_jp_and_vj();
}

/*
   Testing the behavior if :
   VJ1 : is rugular
*/

BOOST_AUTO_TEST_CASE(test14){
    vj1->stop_time_list.front().set_odt(true);
    vj1->vehicle_journey_type = navitia::type::VehicleJourneyType::regular;
    b.data->aggregate_odt();
    BOOST_CHECK_EQUAL((vj1->get_odt_level() == navitia::type::OdtLevel_e::none), true);
    unset_odt_jp_and_vj();
}

/*
   Testing the behavior if :
   VJ1 : is zonal
*/
BOOST_AUTO_TEST_CASE(test17){
    vj1->stop_time_list.front().set_odt(true);
    vj1->stop_time_list.front().set_date_time_estimated(true);
    vj1->stop_time_list.back().set_odt(true);
    vj1->stop_time_list.back().set_date_time_estimated(true);
    vj1->vehicle_journey_type = navitia::type::VehicleJourneyType::virtual_without_stop_time;
    b.data->aggregate_odt();
    BOOST_CHECK_EQUAL((vj1->get_odt_level() == navitia::type::OdtLevel_e::zonal), true);
    unset_odt_jp_and_vj();
}

/*
   Testing the behavior if :
   VJ1 : first stoptime is not odt && not estimated_date_time, last stoptime is odt && not estimated_date_time
   VJ2 : not odt
*/

BOOST_AUTO_TEST_CASE(test18){
    vj1->stop_time_list.back().set_odt(true);
    b.data->aggregate_odt();
    BOOST_CHECK_EQUAL((vj1->get_odt_level() == navitia::type::OdtLevel_e::none), true);
    unset_odt_jp_and_vj();
}


BOOST_AUTO_TEST_CASE(waiting_after){
    vj2->stop_time_list.front().set_odt(true);
    vj2->stop_time_list.front().set_date_time_estimated(true);
    b.data->aggregate_odt();
    resp = make_response(true, navitia::type::EntryPoint(navitia::type::Type_e::StopPoint, "stop_area:stop1"),
            navitia::type::EntryPoint(navitia::type::Type_e::StopPoint, "stop_area:stop3"));
    BOOST_CHECK_EQUAL(resp.response_type(), pbnavitia::ITINERARY_FOUND);
    BOOST_REQUIRE_EQUAL(resp.journeys_size(), 1);
    journey = resp.journeys(0);
    BOOST_REQUIRE_EQUAL(journey.sections_size(), 3);
    section = journey.sections(0);
    displ = section.pt_display_informations();
    BOOST_CHECK_EQUAL(displ.uris().vehicle_journey(), "vj2");

    section = journey.sections(1);
    BOOST_CHECK_EQUAL(section.type(), pbnavitia::SectionType::WAITING);
    BOOST_CHECK(section.add_info_vehicle_journey().has_date_time_estimated());
    unset_odt_jp_and_vj();
}
BOOST_AUTO_TEST_CASE(waiting_before){
    vj1->stop_time_list.front().set_odt(true);
    vj1->stop_time_list.front().set_date_time_estimated(true);
    b.data->aggregate_odt();
    resp = make_response(true, navitia::type::EntryPoint(navitia::type::Type_e::StopPoint, "stop_area:stop4"),
            navitia::type::EntryPoint(navitia::type::Type_e::StopPoint, "stop_area:stop2"));
    BOOST_REQUIRE_EQUAL(resp.response_type(), pbnavitia::ITINERARY_FOUND);
    BOOST_REQUIRE_EQUAL(resp.journeys_size(), 1);
    journey = resp.journeys(0);
    BOOST_REQUIRE_EQUAL(journey.sections_size(), 3);
    section = journey.sections(0);
    displ = section.pt_display_informations();
    BOOST_CHECK_EQUAL(displ.uris().vehicle_journey(), "vj4");

    section = journey.sections(1);
    BOOST_CHECK_EQUAL(section.type(), pbnavitia::SectionType::WAITING);
    BOOST_CHECK(section.add_info_vehicle_journey().has_date_time_estimated());
    unset_odt_jp_and_vj();
}
BOOST_AUTO_TEST_CASE(waiting_after_with_transfer){
    vj2->stop_time_list.front().set_odt(true);
    vj2->stop_time_list.front().set_date_time_estimated(true);
    b.data->aggregate_odt();
    resp = make_response(true, navitia::type::EntryPoint(navitia::type::Type_e::StopPoint, "stop_area:stop1"),
            navitia::type::EntryPoint(navitia::type::Type_e::StopPoint, "stop_area:stop6"));
    BOOST_CHECK_EQUAL(resp.response_type(), pbnavitia::ITINERARY_FOUND);
    BOOST_REQUIRE_EQUAL(resp.journeys_size(), 1);
    journey = resp.journeys(0);
    BOOST_REQUIRE_EQUAL(journey.sections_size(), 4);
    section = journey.sections(0);
    displ = section.pt_display_informations();
    BOOST_CHECK_EQUAL(displ.uris().vehicle_journey(), "vj2");

    section = journey.sections(2);
    BOOST_CHECK_EQUAL(section.type(), pbnavitia::SectionType::WAITING);
    BOOST_CHECK(section.add_info_vehicle_journey().has_date_time_estimated());
    unset_odt_jp_and_vj();
}
BOOST_AUTO_TEST_CASE(waiting_before_with_transfer){
    vj1->stop_time_list.front().set_odt(true);
    vj1->stop_time_list.front().set_date_time_estimated(true);
    b.data->aggregate_odt();
    resp = make_response(true, navitia::type::EntryPoint(navitia::type::Type_e::StopPoint, "stop_area:stop7"),
            navitia::type::EntryPoint(navitia::type::Type_e::StopPoint, "stop_area:stop2"));
    BOOST_REQUIRE_EQUAL(resp.response_type(), pbnavitia::ITINERARY_FOUND);
    BOOST_REQUIRE_EQUAL(resp.journeys_size(), 1);
    journey = resp.journeys(0);
    BOOST_REQUIRE_EQUAL(journey.sections_size(), 4);
    section = journey.sections(0);
    displ = section.pt_display_informations();
    BOOST_CHECK_EQUAL(displ.uris().vehicle_journey(), "vj6");

    section = journey.sections(2);
    BOOST_CHECK_EQUAL(section.type(), pbnavitia::SectionType::WAITING);
    BOOST_CHECK(section.add_info_vehicle_journey().has_date_time_estimated());
    unset_odt_jp_and_vj();
}
BOOST_AUTO_TEST_SUITE_END()
