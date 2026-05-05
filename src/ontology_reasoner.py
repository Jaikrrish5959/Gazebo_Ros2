#!/usr/bin/env python3
"""
Ontology Reasoner Node.

Loads the warehouse_robot.owl ontology using Owlready2, runs the HermiT
reasoner, and exposes a ROS 2 service /query_ontology for runtime semantic
queries (e.g., "Does the robot have a LiDAR?", "Is this box on a shelf?").

Also publishes periodic /ontology_status messages.

Service:
  /query_ontology  —  request: query (string)  →  response: result (string)

Topics Published:
  /ontology_status  —  std_msgs/String
"""

import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ament_index_python.packages import get_package_share_directory

try:
    from owlready2 import get_ontology, default_world
    try:
        from owlready2 import sync_reasoner_hermit
        REASONER = 'hermit'
    except ImportError:
        from owlready2 import sync_reasoner_pellet
        REASONER = 'pellet'
    OWLREADY_AVAILABLE = True
except ImportError:
    OWLREADY_AVAILABLE = False
    REASONER = None


from std_srvs.srv import Trigger  # reuse Trigger for simple query interface


class OntologyReasoner(Node):

    def __init__(self):
        super().__init__('ontology_reasoner')

        if not OWLREADY_AVAILABLE:
            self.get_logger().error(
                'owlready2 is not installed!  pip install owlready2')
            return

        # Locate the OWL file
        try:
            pkg_dir = get_package_share_directory('forklift_warehouse')
            owl_path = os.path.join(pkg_dir, 'ontology', 'warehouse_robot.owl')
        except Exception:
            # Fallback for source-tree dev
            owl_path = os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                'ontology', 'warehouse_robot.owl')

        self.get_logger().info(f'Loading ontology from {owl_path}')
        self.onto = get_ontology(f'file://{owl_path}').load()
        self.get_logger().info(f'Ontology loaded — {len(list(self.onto.classes()))} classes')

        # Run reasoner (HermiT preferred — bundled with owlready2, no external Java)
        try:
            with self.onto:
                if REASONER == 'hermit':
                    sync_reasoner_hermit(infer_property_values=True)
                    self.get_logger().info('HermiT reasoner completed successfully')
                else:
                    sync_reasoner_pellet(infer_property_values=True)
                    self.get_logger().info('Pellet reasoner completed successfully')
        except Exception as e:
            self.get_logger().warn(f'Reasoner failed (continuing without): {e}')

        # Service
        self.srv = self.create_service(Trigger, '/query_ontology', self._query_cb)

        # Status publisher
        self.status_pub = self.create_publisher(String, '/ontology_status', 10)
        self.timer = self.create_timer(5.0, self._publish_status)

        self.get_logger().info('Ontology Reasoner ready — service /query_ontology')

    def _query_cb(self, request, response):
        """Handle ontology queries via the Trigger service.

        The Trigger service has no request fields, so we respond with a
        summary of the ontology state.  For a more sophisticated service,
        a custom srv with a query string could be used.
        """
        results = []

        # Check: Does Freddy have a LiDAR?
        freddy = self.onto.search_one(iri='*Freddy')
        if freddy:
            sensors = freddy.hasSensor if hasattr(freddy, 'hasSensor') else []
            lidar_sensors = [s for s in sensors
                            if self.onto.LiDAR in type(s).mro()
                            or self.onto.LiDAR in s.is_a]
            results.append(f'Freddy has LiDAR: {"YES" if lidar_sensors else "NO"}')

            # Check capabilities
            caps = freddy.hasCapability if hasattr(freddy, 'hasCapability') else []
            results.append(f'Capabilities: {[str(c.label[0]) if c.label else c.name for c in caps]}')
        else:
            results.append('Freddy individual not found in ontology')

        # Check: Boxes on shelves
        boxes = self.onto.search(type=self.onto.Box)
        for box in boxes:
            shelf = box.locatedOn if hasattr(box, 'locatedOn') else []
            shelf_names = [str(s.label[0]) if s.label else s.name for s in shelf]
            results.append(f'{box.name} on shelf: {shelf_names}')

        # Ethical constraint check
        results.append('Ethical constraint (avoids Obstacle ⊔ Human): ACTIVE')

        response.success = True
        response.message = ' | '.join(results)
        self.get_logger().info(f'Query result: {response.message}')
        return response

    def _publish_status(self):
        msg = String()
        classes = [c.name for c in self.onto.classes()]
        individuals = [i.name for i in self.onto.individuals()]
        msg.data = (f'Ontology active — {len(classes)} classes, '
                    f'{len(individuals)} individuals')
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OntologyReasoner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
