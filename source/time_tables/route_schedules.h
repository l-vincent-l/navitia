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
#include "type/type.h"
#include "routing/routing.h"
#include "get_stop_times.h"
#include "type/pb_converter.h"

namespace navitia { namespace timetables {

typedef std::vector<std::string> vector_string;
typedef std::pair<DateTime, const type::StopTime*> vector_date_time;

pbnavitia::Response route_schedule(const std::string & line_externalcode,
        const std::vector<std::string>& forbidden_uris,
        const boost::posix_time::ptime datetime, uint32_t duration, size_t max_stop_date_times, uint32_t interface_version,
        const uint32_t max_depth, int count, int start_page, const type::Data &d, bool disruption_active,
        const bool show_codes);

}}
