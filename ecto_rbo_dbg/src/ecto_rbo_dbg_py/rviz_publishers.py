#!/usr/bin/env python
import ecto
import ecto_pcl
import ecto_ros
import ecto_rbo_pcl
from ecto import BlackBoxCellInfo as CellInfo, BlackBoxForward as Forward

class PublishClusters(ecto.BlackBox):
    """A black blox which publishes clusters as colored point clouds."""
    @staticmethod
    def declare_cells(_p):
        cells = {}

        cells['colorizer'] = CellInfo(ecto_pcl.ColorizeClusters, name='Colorizer')
        cells['convert2msg'] = CellInfo(ecto_rbo_pcl.PointCloud2Message, name='Converter')
        cells['publisher'] = CellInfo(ecto_ros.ecto_sensor_msgs.Publisher_PointCloud2, name='Publisher')

        return cells

    @staticmethod
    def declare_forwards(_p):
        p = {'publisher': [ Forward('topic_name', new_key='namespace') ]}

        i = {'colorizer': [ Forward('input', new_key='pointcloud'),
                            Forward('clusters', new_key='input')]}
        o = {}

        return (p, i, o)

    def connections(self, _p):
        return [ self.colorizer['output'] >> self.convert2msg['input'],
                 self.convert2msg['output'] >> self.publisher['input'] ]


class PublishPointCloud(ecto.BlackBox):
    """A black blox which publishes a point cloud as a point cloud message."""
    @staticmethod
    def declare_cells(_p):
        cells = {}

        cells['convert2msg'] = CellInfo(ecto_rbo_pcl.PointCloud2Message, name='Converter')
        cells['publisher'] = CellInfo(ecto_ros.ecto_sensor_msgs.Publisher_PointCloud2, name='Publisher')

        return cells

    @staticmethod
    def declare_forwards(_p):
        p = {'publisher': [ Forward('topic_name', new_key='namespace') ]}
        i = {'convert2msg': [ Forward('input', new_key='input')]}
        o = {}

        return (p, i, o)

    def connections(self, _p):
        return [ self.convert2msg['output'] >> self.publisher['input'] ]
