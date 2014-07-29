#coding: utf-8

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

from flask import abort, current_app, url_for
import flask_restful
from flask_restful import fields, marshal_with, marshal, reqparse, types
import sqlalchemy
from validate_email import validate_email

import logging

from navitiacommon import models
from navitiacommon.models import db

__ALL__ = ['Api', 'Instance', 'User', 'Key']


class FieldDate(fields.Raw):
    def format(self, value):
        if value:
            return value.isoformat()
        else:
            return 'null'


key_fields = {'id': fields.Raw, 'token': fields.Raw, 'valid_until': FieldDate}

instance_fields = {'id': fields.Raw, 'name': fields.Raw, "is_free": fields.Raw}
api_fields = {'id': fields.Raw, 'name': fields.Raw}

user_fields = {'id': fields.Raw, 'login': fields.Raw, 'email': fields.Raw}
user_fields_full = {'id': fields.Raw, 'login': fields.Raw, \
        'email': fields.Raw, 'keys': fields.List(fields.Nested(key_fields)),
        'authorizations': fields.List(fields.Nested(
            {'instance': fields.Nested(instance_fields),
                'api': fields.Nested(api_fields)}))}

jobs_fields = {'jobs': fields.List(fields.Nested({
        'id': fields.Raw,
        'state': fields.Raw,
        'created_at': FieldDate,
        'updated_at': FieldDate,
        'data_sets': fields.List(fields.Nested({
            'type': fields.Raw,
            'name': fields.Raw
        })),
        'instance': fields.Nested(instance_fields)
}))}


class Api(flask_restful.Resource):
    def __init__(self):
        pass

    def get(self):
        return marshal(models.Api.query.all(), api_fields)


class Index(flask_restful.Resource):
    def get(self):
        return {'jobs': {'href': url_for('jobs', _external=True)}}

class Job(flask_restful.Resource):
    @marshal_with(jobs_fields)
    def get(self, instance_name=None):
        query = models.Job.query
        if instance_name:
            query = query.filter(models.Instance.name == instance_name)
        return {'jobs': query.order_by(models.Job.created_at.desc()).limit(30)}

class Instance(flask_restful.Resource):
    def __init__(self):
        pass

    @marshal_with(instance_fields)
    def get(self):
        parser = reqparse.RequestParser()
        parser.add_argument('is_free', type=types.boolean, required=False,
                case_sensitive=False, help='boolean for returning only free '
                'or private instances')
        args = parser.parse_args()
        if args['is_free'] != None:
            return models.Instance.query.filter_by(**args).all()
        else:
            return models.Instance.query.all()


class User(flask_restful.Resource):
    def __init__(self):
        pass

    def get(self, user_id=None):
        if user_id:
            return marshal(models.User.query.get_or_404(user_id),
                    user_fields_full)
        else:
            parser = reqparse.RequestParser()
            parser.add_argument('login', type=unicode, required=False,
                    case_sensitive=False, help='login')
            parser.add_argument('email', type=unicode, required=False,
                    case_sensitive=False, help='email')
            args = parser.parse_args()

            # dict comprehension would be better, but it's not in python 2.6
            filter_params = dict((k, v) for k, v in args.items() if v)

            if filter_params:
                users = models.User.query.filter_by(**filter_params).all()
                return marshal(users, user_fields)
            else:
                return marshal(models.User.query.all(), user_fields)

    def post(self):
        user = None
        parser = reqparse.RequestParser()
        parser.add_argument('login', type=unicode, required=True,
                case_sensitive=False, help='login is required')
        parser.add_argument('email', type=unicode, required=True,
                case_sensitive=False, help='email is required')
        args = parser.parse_args()

        if not validate_email(args['email'],
                          check_mx=current_app.config['EMAIL_CHECK_MX'],
                          verify=current_app.config['EMAIL_CHECK_SMTP']):
            return ({'error': 'email invalid'}, 400)

        try:
            user = models.User(login=args['login'], email=args['email'])
            db.session.add(user)
            db.session.commit()
            return marshal(user, user_fields_full)
        except sqlalchemy.exc.IntegrityError:
            return ({'error': 'duplicate user'}, 409)
        except Exception:
            logging.exception("fail")
            raise

    def put(self, user_id):
        user = models.User.query.get_or_404(user_id)
        parser = reqparse.RequestParser()
        parser.add_argument('email', type=unicode, required=True,
                case_sensitive=False, help='email is required')
        args = parser.parse_args()

        if not validate_email(args['email'],
                          check_mx=current_app.config['EMAIL_CHECK_MX'],
                          verify=current_app.config['EMAIL_CHECK_SMTP']):
            return ({'error': 'email invalid'}, 400)

        try:
            user.email = args['email']
            db.session.commit()
            return marshal(user, user_fields_full)
        except sqlalchemy.exc.IntegrityError:
            return ({'error': 'duplicate user'}, 409)  # Conflict
        except Exception:
            logging.exception("fail")
            raise

    def delete(self, user_id):
        user = models.User.query.get_or_404(user_id)
        try:
            db.session.delete(user)
            db.session.commit()
        except Exception:
            logging.exception("fail")
            raise
        return ({}, 204)


class Key(flask_restful.Resource):
    def __init__(self):
        pass

    @marshal_with(key_fields)
    def get(self, user_id, key_id=None):
        try:
            return models.User.query.get_or_404(user_id).keys.all()
        except Exception:
            logging.exception("fail")
            raise

    @marshal_with(user_fields_full)
    def post(self, user_id):
        parser = reqparse.RequestParser()
        parser.add_argument('valid_until', type=types.date, required=False,
                case_sensitive=False, help='end validity date of the key')
        args = parser.parse_args()
        user = models.User.query.get_or_404(user_id)
        try:
            user.add_key(valid_until=args['valid_until'])
            db.session.commit()
        except Exception:
            logging.exception("fail")
            raise
        return user

    @marshal_with(user_fields_full)
    def delete(self, user_id, key_id):
        user = models.User.query.get_or_404(user_id)
        try:
            key = user.keys.filter_by(id=key_id).first()
            if not key:
                abort(404)
            db.session.delete(key)
            db.session.commit()
        except Exception:
            logging.exception("fail")
            raise
        return user

    @marshal_with(user_fields_full)
    def put(self, user_id, key_id):
        parser = reqparse.RequestParser()
        parser.add_argument('valid_until', type=types.date, required=True,
                case_sensitive=False, help='end validity date of the key')
        args = parser.parse_args()
        user = models.User.query.get_or_404(user_id)
        try:
            key = user.keys.filter_by(id=key_id).first()
            if not key:
                abort(404)
            key.valid_until = args['valid_until']
            db.session.commit()
        except Exception:
            logging.exception("fail")
            raise
        return user


class Authorization(flask_restful.Resource):
    def __init__(self):
        pass

    def delete(self, user_id):
        parser = reqparse.RequestParser()
        parser.add_argument('api_id', type=int, required=True,
                case_sensitive=False, help='api_id is required')
        parser.add_argument('instance_id', type=int, required=True,
                case_sensitive=False, help='instance_id is required')
        args = parser.parse_args()

        try:
            user = models.User.query.get_or_404(user_id)
            authorization = user.authorizations.filter_by(
                    api_id=args['api_id'], instance_id=args['instance_id'])
            if not authorization:
                abort(404)
            db.session.delete(authorization)
            db.session.commit()
        except Exception:
            logging.exception("fail")
            raise
        return user

    def post(self, user_id):
        parser = reqparse.RequestParser()
        parser.add_argument('api_id', type=int, required=True,
                case_sensitive=False, help='api_id is required')
        parser.add_argument('instance_id', type=int, required=True,
                case_sensitive=False, help='instance_id is required')
        args = parser.parse_args()

        user = models.User.query.get_or_404(user_id)
        api = models.Api.query.get_or_404(args['api_id'])
        instance = models.Instance.query.get_or_404(args['instance_id'])

        try:
            authorization = models.Authorization()
            authorization.user = user
            authorization.api = api
            authorization.instance = instance
            user.authorizations.append(authorization)
            db.session.add(authorization)
            db.session.commit()
        except sqlalchemy.exc.IntegrityError:
            return ({'error': 'duplicate entry'}, 409)
        except Exception:
            logging.exception("fail")
            raise
        return marshal(user, user_fields_full)
