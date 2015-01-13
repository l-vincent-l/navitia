"""Add association meta_vj calendar

Revision ID: 570f3e987e5f
Revises: 2d86200bcb93
Create Date: 2015-01-13 10:11:38.939751

"""

# revision identifiers, used by Alembic.
revision = '570f3e987e5f'
down_revision = '2d86200bcb93'

from alembic import op
import sqlalchemy as sa
import geoalchemy2 as ga


def upgrade():
    ### commands auto generated by Alembic - please adjust! ###
    op.create_table('rel_metavj_calendar',
    sa.Column('meta_vj', sa.BIGINT(), nullable=True),
    sa.Column('calendar', sa.BIGINT(), nullable=True),
    sa.ForeignKeyConstraint(['calendar'], [u'navitia.calendar.id'], name=u'rel_metavj_meta_vj_calendar_fkey'),
    sa.ForeignKeyConstraint(['meta_vj'], [u'navitia.meta_vj.id'], name=u'rel_metavj_vj_meta_vj_fkey'),
    schema='navitia'
    )
    ### end Alembic commands ###


def downgrade():
    ### commands auto generated by Alembic - please adjust! ###
    op.drop_table('rel_metavj_calendar', schema='navitia')
    ### end Alembic commands ###
