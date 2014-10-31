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

#include "raptor_solutions.h"
#include "raptor_path.h"
#include "raptor.h"
#include "raptor_path_defs.h"


namespace bt = boost::posix_time;
namespace navitia { namespace routing {

Solutions
get_solutions(const std::vector<std::pair<type::idx_t, navitia::time_duration> > &departs,
             const std::vector<std::pair<type::idx_t, navitia::time_duration> > &destinations,
             bool clockwise, const type::AccessibiliteParams & accessibilite_params,
             bool disruption_active, const RAPTOR& raptor) {
      Solutions result;
      auto pareto_front = get_pareto_front(clockwise, departs, destinations,
              accessibilite_params, disruption_active, raptor);
      result.insert(pareto_front.begin(), pareto_front.end());

      //if(!pareto_front.empty()) {
      //    auto walking_solutions = get_walking_solutions(clockwise, departs, destinations,
      //            *pareto_front.rbegin(), disruption_active, accessibilite_params, raptor);
      //    if(!walking_solutions.empty()) {
      //      result.insert(walking_solutions.begin(), walking_solutions.end());
      //    }
      //}
      return result;
}


Solutions
get_solutions(const std::vector<std::pair<type::idx_t, navitia::time_duration> > &departs,
              const DateTime &dep, bool clockwise, const type::Data & , bool) {
    Solutions result;
    for(auto dep_dist : departs) {
        Solution d;
        d.count = 0;
        d.stop_point_idx = dep_dist.first;
        d.walking_time = dep_dist.second;
        if(clockwise)
            d.arrival = dep + d.walking_time.total_seconds();
        else
            d.arrival = dep - d.walking_time.total_seconds();
        d.total_arrival = d.arrival;
        result.insert(d);
    }
    return result;
}

// Does the current date improves compared to best_so_far – we must not forget to take the walking duration
bool improves(const DateTime & best_so_far, bool clockwise, const DateTime & current, int walking_duration) {
    if(clockwise) {
        return (current - walking_duration) > best_so_far;
    } else {
        return (current + walking_duration) < best_so_far;
    }
}

Solutions
get_pareto_front(bool clockwise, const std::vector<std::pair<type::idx_t, navitia::time_duration> > &departs,
               const std::vector<std::pair<type::idx_t, navitia::time_duration> > &destinations,
               const type::AccessibiliteParams & accessibilite_params, bool disruption_active, const RAPTOR& raptor) {
    Solutions result;

    DateTime best_dt, best_dt_stop_point;
    if(clockwise) {
        best_dt = DateTimeUtils::min;
        best_dt_stop_point = DateTimeUtils::min;
    } else {
        best_dt = DateTimeUtils::inf;
        best_dt_stop_point = DateTimeUtils::inf;
    }
    for(unsigned int round=1; round <= raptor.count; ++round) {
        // For every round with look for the best journey pattern point that belongs to one of the destination stop points
        // We must not forget to walking duration
        type::idx_t best_stop_point_idx = type::invalid_idx;
        for(auto spid_dist : destinations) {
            const auto stop_point_idx = spid_dist.first;
            auto& l = raptor.labels[round][stop_point_idx];
            if(l.pt_is_initialized() &&
               improves(best_dt, clockwise, l.dt_pt, spid_dist.second.total_seconds()) ) {
                best_stop_point_idx = stop_point_idx;
                best_dt_stop_point = l.dt_pt;
                // Dans le sens horaire : lors du calcul on gardé que l’heure de départ, mais on veut l’arrivée
                // Il faut donc retrouver le stop_time qui nous intéresse avec best_stop_time
                const type::StopTime* st = l.st;
                DateTime dt = l.dt_pt;

                if(clockwise) {
                    auto arrival_time = !st->is_frequency() ? st->arrival_time : st->f_arrival_time(DateTimeUtils::hour(dt));
                    DateTimeUtils::update(best_dt_stop_point, arrival_time, false);
                } else {
                    auto departure_time = !st->is_frequency() ? st->departure_time : st->f_departure_time(DateTimeUtils::hour(dt));
                    DateTimeUtils::update(best_dt_stop_point, departure_time, true);
                }
                if(clockwise)
                    best_dt = dt - spid_dist.second.total_seconds();
                else
                    best_dt = dt + spid_dist.second.total_seconds();
            }
        }
        if(best_stop_point_idx != type::invalid_idx) {
            Solution s;
            s.stop_point_idx = best_stop_point_idx;
            s.count = round;
            s.walking_time = getWalkingTime(round, best_stop_point_idx, departs, destinations, clockwise, disruption_active,
                                            accessibilite_params, raptor);
            s.arrival = best_dt_stop_point;
            s.ratio = 0;
            s.total_arrival = best_dt;
            type::idx_t final_stop_point_idx;
            std::tie(final_stop_point_idx, s.upper_bound) = get_final_stop_point_idx_and_date(round, best_stop_point_idx, clockwise,
                                                                             disruption_active, accessibilite_params, raptor);
            for(auto spid_dep : departs) {
                if(final_stop_point_idx == spid_dep.first) {
                    if(clockwise) {
                        s.upper_bound = s.upper_bound + spid_dep.second.total_seconds();
                    }else {
                        s.upper_bound = s.upper_bound - spid_dep.second.total_seconds();
                    }
                }
            }
            result.insert(s);
        }
    }

    return result;
}





Solutions
get_walking_solutions(bool clockwise, const std::vector<std::pair<type::idx_t, navitia::time_duration> > &departs,
                      const std::vector<std::pair<type::idx_t, navitia::time_duration> > &destinations, const Solution& best,
                      const bool disruption_active, const type::AccessibiliteParams &accessibilite_params,
                      const RAPTOR& raptor) {
    Solutions result;

    std::/*unordered_*/map<type::idx_t, Solution> tmp;
    // We start at 1 because we don't want results of the first round
    for(uint32_t i=1; i <= raptor.count; ++i) {
        for(auto spid_dist : destinations) {
            Solution best_departure;
            best_departure.ratio = 2;
            best_departure.stop_point_idx = type::invalid_idx;
            type::idx_t stop_point_idx = spid_dist.first;
            const auto label = raptor.labels[i][stop_point_idx];
            // We only want solution ending by a vehicle journey or a stay_in
            if(label.pt_is_initialized()) {
                navitia::time_duration walking_time = getWalkingTime(i, stop_point_idx, departs, destinations, clockwise,
                                                                     disruption_active, accessibilite_params, raptor);
                if(best.walking_time < walking_time) {
                    continue;
                }
                float lost_time;
                if(clockwise)
                    lost_time = best.total_arrival - (label.dt_pt - best.walking_time.total_seconds());
                else
                    lost_time = (label.dt_pt + spid_dist.second.total_seconds()) - best.total_arrival;


                //Si je gagne 5 minutes de marche a pied, je suis pret à perdre jusqu'à 10 minutes.
                int walking_time_diff_in_s = (best.walking_time - walking_time).total_seconds();
                if (walking_time_diff_in_s > 0) {
                    float ratio = lost_time / walking_time_diff_in_s;
                    if( ratio >= best_departure.ratio) {
                        continue;
                    }
                    Solution s;
                    s.stop_point_idx = stop_point_idx;
                    s.count = i;
                    s.ratio = ratio;
                    s.walking_time = walking_time;
                    s.arrival = raptor.labels[i][stop_point_idx].dt_pt;
                    type::idx_t final_stop_point_idx;
                    DateTime last_time;
                    std::tie(final_stop_point_idx, last_time) = get_final_stop_point_idx_and_date(i, stop_point_idx, clockwise,
                                        disruption_active, accessibilite_params, raptor);

                    s.upper_bound = last_time;
                    for(auto spid_dep : departs) {
                        if(final_stop_point_idx == spid_dep.first) {
                            if (clockwise) {
                                s.upper_bound = s.upper_bound + (spid_dep.second.total_seconds());
                            } else {
                                s.upper_bound = s.upper_bound - (spid_dep.second.total_seconds());
                            }
                        }
                    }
                    best_departure = s;
                }
            }
            if(best_departure.stop_point_idx != type::invalid_idx) {
                if(tmp.find(best_departure.stop_point_idx) == tmp.end()) {
                    tmp.insert(std::make_pair(best_departure.stop_point_idx, best_departure));
                } else if(tmp[best_departure.stop_point_idx].ratio > best_departure.ratio) {
                    tmp[best_departure.stop_point_idx] = best_departure;
                }
            }
        }
    }


    std::vector<Solution> to_sort;
    for(auto p : tmp) {
        to_sort.push_back(p.second);
    }
    std::sort(to_sort.begin(), to_sort.end(),
            [](const Solution s1, const Solution s2) { return s1.ratio < s2.ratio;});

    if(to_sort.size() > 2)
        to_sort.resize(2);
    result.insert(to_sort.begin(), to_sort.end());

    return result;
}

struct VisitorFinalStopPointAndDate : public BasePathVisitor {
    type::idx_t current_stop_point_idx = type::invalid_idx;
    DateTime last_time = DateTimeUtils::inf;
    void final_step(const type::idx_t current_stop_point, size_t count, const std::vector<label_vector_t> &labels) {
        this->current_stop_point_idx = current_stop_point;
        last_time = labels[count][current_stop_point].dt_pt;
    }
};

// Reparcours l’itinéraire rapidement pour avoir le JPP et la date de départ (si on cherchait l’arrivée au plus tôt)
std::pair<type::idx_t, DateTime>
get_final_stop_point_idx_and_date(int count, type::idx_t stop_point_idx, bool clockwise, bool disruption_active,
                          const type::AccessibiliteParams & accessibilite_params, const RAPTOR& raptor) {
    VisitorFinalStopPointAndDate v;
    read_path(v, stop_point_idx, count, !clockwise, disruption_active, accessibilite_params, raptor);
    return std::make_pair(v.current_stop_point_idx, v.last_time);
}


struct VisitorWalkingTime : public BasePathVisitor {
    navitia::time_duration walking_time = {};
    type::idx_t departure_stop_point_idx = type::invalid_idx;
    void connection(type::StopPoint* , type::StopPoint* ,
                boost::posix_time::ptime dep_time, boost::posix_time::ptime arr_time,
                type::StopPointConnection*) {

        walking_time += navitia::seconds((arr_time - dep_time).total_seconds());
    }

    void final_step(type::idx_t current_stop_point, size_t , const std::vector<std::vector<Label>> &){
        departure_stop_point_idx = current_stop_point;
    }

};


navitia::time_duration getWalkingTime(int count, type::idx_t stop_point_idx, const std::vector<std::pair<type::idx_t, navitia::time_duration> > &departs,
                     const std::vector<std::pair<type::idx_t, navitia::time_duration> > &destinations,
                     bool clockwise, bool disruption_active, const type::AccessibiliteParams & accessibilite_params,
                     const RAPTOR& raptor) {
    navitia::time_duration walking_time = {};

    //Marche à la fin
    for(auto dest_dist : destinations) {
        if(dest_dist.first == stop_point_idx) {
            walking_time = dest_dist.second;
            break;
        }
    }

    VisitorWalkingTime v;
    read_path(v, stop_point_idx, count, !clockwise, disruption_active, accessibilite_params, raptor);
    walking_time += v.walking_time;
    if (v.departure_stop_point_idx == type::invalid_idx) {
        return walking_time;
    }
    //Marche au départ
    for(auto dep_dist : departs) {
        if(dep_dist.first == v.departure_stop_point_idx) {
            walking_time += dep_dist.second;
            break;
        }
    }

    return walking_time;
}

}}

