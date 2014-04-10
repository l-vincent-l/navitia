# encoding: utf-8
import logging
import logging.config
import os
from flask import Flask, got_request_exception
from flask.ext.restful import Api
import sys
from jormungandr.exceptions import log_exception

app = Flask(__name__)
app.config.from_object('jormungandr.default_settings')
if 'JORMUNGANDR_CONFIG_FILE' in os.environ:
    app.config.from_envvar('JORMUNGANDR_CONFIG_FILE')

if 'LOGGER' in app.config:
    logging.config.dictConfig(app.config['LOGGER'])
else:  # Default is std out
    handler = logging.StreamHandler(stream=sys.stdout)
    app.logger.addHandler(handler)
    app.logger.setLevel('INFO')

got_request_exception.connect(log_exception, app)

rest_api = Api(app, catch_all_404s=True)

from navitiacommon.models import db
db.init_app(app)
if not app.config['CACHE_DISABLED']:
    from navitiacommon.cache import init_cache
    init_cache(host=app.config['REDIS_HOST'], port=app.config['REDIS_PORT'],
               db=app.config['REDIS_DB'],
               password=app.config['REDIS_PASSWORD'],
               default_ttl=app.config['AUTH_CACHE_TTL'])


from jormungandr.instance_manager import InstanceManager
i_manager = InstanceManager()
i_manager.initialisation(start_ping=app.config['START_MONITORING_THREAD'])

from jormungandr.stat_manager import StatManager
i_stat_manager = StatManager()
i_stat_manager.init(save_stat=app.config['SAVE_STAT'],
                    broker_url=app.config['BROKER_URL'],
                    exchange_name=app.config['EXCHANGE_NAME'],
                    topic_name=app.config['TOPIC_NAME'])

from jormungandr import api


def setup_package():
    i_manager.stop()
