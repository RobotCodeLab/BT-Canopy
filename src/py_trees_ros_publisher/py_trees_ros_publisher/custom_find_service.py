
import py_trees_ros_interfaces.msg as py_trees_msgs  # noqa
import py_trees_ros_interfaces.srv as py_trees_srvs  # noqa

import rclpy
import time

from py_trees_ros import exceptions

def find_service(node: rclpy.node.Node,
                 service_type: str,
                 namespace: str=None,
                 timeout: float=0.5):
    """
    Discover a service of the specified type and if necessary, under the specified
    namespace.
    Args:
        node (:class:`~rclpy.node.Node`): nodes have the discovery methods
        service_type (:obj:`str`): primary lookup hint
        namespace (:obj:`str`): secondary lookup hint
        timeout: immediately post node creation, can take time to discover the graph (sec)
    Returns:
        :obj:`str`: fully expanded service name
    Raises:
        :class:`~py_trees_ros.exceptions.NotFoundError`: if no services were found
        :class:`~py_trees_ros.exceptions.MultipleFoundError`: if multiple services were found
    """

    loop_period = 0.1  # seconds
    clock = rclpy.clock.Clock()
    start_time = clock.now()
    service_names = []
    while clock.now() - start_time < rclpy.time.Duration(seconds=timeout):
        # Returns a list of the form: [('exchange/blackboard', ['std_msgs/String'])
        service_names_and_types = node.get_service_names_and_types()
        service_names = [name for name, types in service_names_and_types if service_type in types]
        if namespace is not None:
            service_names = [name for name in service_names if namespace in name]
        if service_names:
            break
        time.sleep(loop_period)


    if not service_names:
        raise exceptions.NotFoundError("service not found [type: {}]".format(service_type))
    elif len(service_names) == 1:
        return service_names[0]
    else:
        raise exceptions.MultipleFoundError("multiple services found [type: {}]".format(service_type))
