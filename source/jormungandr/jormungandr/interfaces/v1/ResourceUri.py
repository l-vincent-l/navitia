# Copyright (c) 2001-2014, Canal TP and/or its affiliates. All rights reserved.
#
# This file is part of Navitia,
#     the software to build cool stuff with public transport.
#
# Hope you'll enjoy and contribute to this project,
#     powered by Canal TP (www.canaltp.fr).
# Help us simplify mobility and open public transport:
#     a non ending quest to the responsive locomotion way of traveling!
#
# LICENCE: This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Affero General Public License for more details.
#
# You should have received a copy of the GNU Affero General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
#
# Stay tuned using
# twitter @navitia
# IRC #navitia on freenode
# https://groups.google.com/d/forum/navitia
# www.navitia.io

from flask.ext.restful import Resource
from converters_collection_type import collections_to_resource_type
from converters_collection_type import resource_type_to_collection
from jormungandr import utils
from jormungandr.interfaces.v1.StatedResource import StatedResource
from jormungandr.stat_manager import manage_stat_caller
from make_links import add_id_links, clean_links, add_pagination_links
from functools import wraps
from collections import OrderedDict, deque
from flask import url_for
from flask.ext.restful.utils import unpack
from jormungandr.authentication import authentication_required
import navitiacommon.type_pb2 as type_pb2


class ResourceUri(StatedResource):

    def __init__(self, authentication=True, *args, **kwargs):
        StatedResource.__init__(self, *args, **kwargs)
        self.region = None
        self.method_decorators.append(add_id_links())
        self.method_decorators.append(add_address_poi_id(self))
        self.method_decorators.append(add_computed_resources(self))
        self.method_decorators.append(add_pagination_links())
        self.method_decorators.append(clean_links())
        if authentication:
            #some rare API (eg journey) must handle the authenfication by themself, thus deactivate it
            #by default ALWAYS use authentication=True
            self.method_decorators.append(authentication_required)

    def get_filter(self, items):
        filter_list = []
        if len(items) % 2 != 0:
            items = items[:-1]
        type_ = None

        for item in items:
            if not type_:
                if item != "coord":
                    if(item == "calendars"):
                        type_ = 'calendar'
                    else:
                        type_ = collections_to_resource_type[item]
                else:
                    type_ = "coord"
            else:
                if type_ == "coord" or type_ == "address":
                    splitted_coord = item.split(";")
                    if len(splitted_coord) == 2:
                        lon, lat = splitted_coord
                        object_type = "stop_point"
                        if(self.collection == "pois"):
                            object_type = "poi"
                        filter_ = object_type + ".coord DWITHIN(" + lon + ","
                        filter_ += lat + ",200)"
                        filter_list.append(filter_)
                    else:
                        filter_list.append(type_ + ".uri=" + item)
                elif type_ == 'poi':
                    filter_list.append(type_ + '.uri=' + item.split(":")[-1])
                else:
                    filter_list.append(type_ + ".uri=" + item)
                type_ = None
        return " and ".join(filter_list)


class add_address_poi_id(object):

    def __init__(self, resource):
        self.resource = resource

    def __call__(self, f):
        @wraps(f)
        def wrapper(*args, **kwargs):
            objects = f(*args, **kwargs)

            def add_id(objects, region, type_=None):
                if isinstance(objects, (list, tuple)):
                    for item in objects:
                        add_id(item, region, type_)
                elif hasattr(objects, 'keys'):
                    for v in objects.keys():
                        add_id(objects[v], region, v)
                    if 'address' == type_ and 'coord' in objects:
                        lon = objects['coord']['lon']
                        lat = objects['coord']['lat']
                        objects['id'] = lon + ';' + lat
                    if 'embedded_type' in objects.keys() and\
                        (objects['embedded_type'] == 'address' or
                         objects['embedded_type'] == 'poi' or
                         objects['embedded_type'] == 'administrative_region') and\
                         objects['embedded_type'] in objects:
                        objects["id"] = objects[objects['embedded_type']]["id"]
            if self.resource.region:
                add_id(objects, self.resource.region)
            return objects
        return wrapper


class add_computed_resources(object):

    def __init__(self, resource):
        self.resource = resource

    def __call__(self, f):
        @wraps(f)
        def wrapper(*args, **kwargs):
            response = f(*args, **kwargs)
            if isinstance(response, tuple):
                data, code, header = unpack(response)
            else:
                data = response
            collection = None
            kwargs["_external"] = True
            templated = True
            for key in data.keys():
                if key in collections_to_resource_type.keys():
                    collection = key
                if key in resource_type_to_collection.keys():
                    collection = resource_type_to_collection[key]
            if collection is None:
                return response
            kwargs["uri"] = collection + '/'
            if "id" in kwargs.keys():
                kwargs["uri"] += kwargs["id"]
                del kwargs["id"]
                templated = False
            else:
                kwargs["uri"] += '{' + collection + ".id}"
            if collection in ['stop_areas', 'stop_points', 'lines', 'routes',
                              'addresses'] and "region" in kwargs:
                for api in ['route_schedules', 'stop_schedules',
                            'arrivals', 'departures', "places_nearby"]:
                    data['links'].append({
                        "href": url_for("v1." + api, **kwargs),
                        "rel": api,
                        "templated": templated
                    })
            if collection in ['stop_areas', 'stop_points', 'addresses']:
                data['links'].append({
                    "href": url_for("v1.journeys", **kwargs),
                    "rel": "journeys",
                    "templated": templated
                })
            #for lines we add the link to the calendars
            if 'region' in kwargs:
                if collection == 'lines':
                    data['links'].append({
                        "href": url_for("v1.calendars", **kwargs),
                        "rel": "calendars",
                        "templated": templated
                    })
                if collection in ['stop_areas', 'lines', 'networks']:
                    data['links'].append({
                        "href": url_for("v1.disruptions", **kwargs),
                        "rel": "disruptions",
                        "templated": templated
                    })
            if isinstance(response, tuple):
                return data, code, header
            else:
                return data
        return wrapper

class complete_links(object):
    def __init__(self, resource):
        self.resource = resource
    def complete(self, data, collect):
        queue = deque()
        result = []
        for v in data.itervalues():
            queue.append(v)
        collect_type = collect["type"]
        while queue:
            elem = queue.pop()
            if isinstance(elem, (list, tuple)):
                queue.extend(elem)
            elif hasattr(elem, 'iterkeys'):
                if 'type' in elem and elem['type'] == collect_type:
                    if collect_type == "notes":
                        result.append({"id": elem['id'], "value": elem['value'], "type": collect_type})
                    elif collect_type == "exceptions":
                        type_= "Remove"
                        if elem['except_type'] == 0:
                            type_ = "Add"
                        result.append({"id": elem['id'], "date": elem['date'], "type" : type_})
                    for del_ in collect["del"]:
                        del elem[del_]
                else:
                    queue.extend(elem.itervalues())
        return result

    def __call__(self, f):
        @wraps(f)
        def wrapper(*args, **kwargs):
            objects = f(*args, **kwargs)
            if isinstance(objects, tuple):
                data, code, header = unpack(objects)
            else:
                data = objects

            if self.resource.region:
                collection = {
                    "notes" : {"type" : "notes",
                              "del":["value"]},
                    "exceptions" : {"type" : "exceptions",
                              "del":["date","except_type"]}
                }
                # Add notes exceptions
                for col in collection:
                    if not col in data or not isinstance(data[col], list):
                        data[col] = []
                        res = []
                        note = self.complete(data, collection[col])
                        [res.append(item) for item in note if not item in res]
                        data[col].extend(res)

            if isinstance(objects, tuple):
                return data, code, header
            else:
                return data

        return wrapper

class update_journeys_status(object):

    def __init__(self, resource):
        self.resource = resource

    def update_status(self, journey, disruptions):

        class find_most_dangerous_disruption:
            def __init__(self):
                self.max_priority_label = None
                self.max_priority = []

            def __call__(self, key, obj):
                try:
                    if 'links' not in obj:
                        return
                except TypeError:
                    return
                real_disruptions = [disruptions[d['id']] for d in obj['links'] if d['type'] == 'disruption']
                for d in real_disruptions:
                    prio = d['severity']['priority']
                    self.max_priority = prio if not self.max_priority else max(self.max_priority, prio)
                    if self.max_priority == prio:
                        self.max_priority_label = d['severity']['effect']

        v = find_most_dangerous_disruption()
        utils.walk_dict(journey, v)

        if v.max_priority_label:
            journey['status'] = v.max_priority_label

    def __call__(self, f):
        @wraps(f)
        def wrapper(*args, **kwargs):
            objects = f(*args, **kwargs)
            if isinstance(objects, tuple):
                data, code, header = unpack(objects)
            else:
                data = objects

            if self.resource.region and "journeys" in data:

                all_disruptions = {d['id']: d for d in data['disruptions']}

                for journey in data["journeys"]:
                    self.update_status(journey, all_disruptions)

            if isinstance(objects, tuple):
                return data, code, header
            else:
                return data

        return wrapper
