import rclpy
from rclpy.node import Node

from tree_msgs.msg import StatusChangeLog
from tree_msgs.msg import NodeStatus

import py_trees_ros

# https://github.com/splintered-reality/py_trees_ros/blob/d7e12ed24f6d298dd70b6625e6ae2f3f15afdcdd/py_trees_ros/trees.py#L744
# https://github.com/splintered-reality/py_trees_ros/blob/d7e12ed24f6d298dd70b6625e6ae2f3f15afdcdd/py_trees_ros/programs/tree_watcher.py

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. argparse::
   :module: py_trees_ros.programs.tree_watcher
   :func: command_line_argument_parser
   :prog: py-trees-tree-watcher

Command line utility that introspects on a running
:class:`~py_trees_ros.trees.BehaviourTree` instance over it's snapshot
stream interfaces. Use to visualise the tree as a dot graph or
track tree changes, timing statistics and blackboard variables visited
by the tree on each tick.

.. image:: images/tree-watcher.gif

"""

##############################################################################
# Imports
##############################################################################

import argparse
import py_trees
import py_trees.console as console
import py_trees_ros
import rclpy
import sys

##############################################################################
# Classes
##############################################################################

from py_trees_ros import conversions
from py_trees_ros.trees import SnapshotStream
from py_trees_ros.trees import WatcherMode
import unique_identifier_msgs.msg as unique_identifier_msgs
import math

class CustomWatcher(py_trees_ros.trees.Watcher):

    def __init__(self, topic_name: str = None, namespace_hint: str = None, parameters: SnapshotStream.Parameters = None, statistics: bool = False, mode: WatcherMode = ...):
        super().__init__(topic_name, namespace_hint, parameters, statistics, mode)

    def callback_snapshot(self, msg):
        """
        Formats the string message coming in.

        Args
            msg (:class:`py_trees_ros_interfaces.msg.BehaviourTree`):serialised snapshot
        """
        ####################
        # Processing
        ####################
        self.snapshot_visitor.previously_visited = self.snapshot_visitor.visited
        self.snapshot_visitor.visited = {}
        serialised_behaviours = {}
        root_id = None
        for serialised_behaviour in msg.behaviours:
            if serialised_behaviour.parent_id == unique_identifier_msgs.UUID():
                root_id = conversions.msg_to_uuid4(serialised_behaviour.own_id)
            serialised_behaviours[
                conversions.msg_to_uuid4(serialised_behaviour.own_id)
            ] = serialised_behaviour

        def deserialise_tree_recursively(msg):
            behaviour = conversions.msg_to_behaviour(msg)
            for serialised_child_id in msg.child_ids:
                child_id = conversions.msg_to_uuid4(serialised_child_id)
                child = deserialise_tree_recursively(
                    serialised_behaviours[child_id]
                )
                # invasive hack to revert the dummy child we added in msg_to_behaviour
                if isinstance(behaviour, py_trees.decorators.Decorator):
                    behaviour.children = [child]
                    behaviour.decorated = behaviour.children[0]
                else:
                    behaviour.children.append(child)
                child.parent = behaviour
            # set the current child so tip() works properly everywhere
            if behaviour.children:
                if msg.current_child_id != unique_identifier_msgs.UUID():
                    current_child_id = conversions.msg_to_uuid4(msg.current_child_id)
                    for index, child in enumerate(behaviour.children):
                        if child.id == current_child_id:
                            # somewhat ugly not having a consistent api here
                            if isinstance(behaviour, py_trees.composites.Selector):
                                behaviour.current_child = child
                            elif isinstance(behaviour, py_trees.composites.Sequence):
                                behaviour.current_index = index
                            # else Parallel, nothing to do since it infers
                            # the current child from children's status on the fly
                            break
            if msg.is_active:
                self.snapshot_visitor.visited[behaviour.id] = behaviour.status
            return behaviour

        # we didn't set the tip in any behaviour, but nothing depends
        # on that right now
        root = deserialise_tree_recursively(serialised_behaviours[root_id])

        ####################
        # Streaming
        ####################
        if self.mode == WatcherMode.SNAPSHOTS:
            if msg.changed:
                colour = console.green
            else:
                colour = console.bold
            ####################
            # Banner
            ####################
            title = "Tick {}".format(msg.statistics.count)
            print(colour + "\n" + 80 * "*" + console.reset)
            print(colour + "* " + console.bold + title.center(80) + console.reset)
            print(colour + 80 * "*" + "\n" + console.reset)
            ####################
            # Tree Snapshot
            ####################
            print(
                py_trees.display.unicode_tree(
                    root=root,
                    visited=self.snapshot_visitor.visited,
                    previously_visited=self.snapshot_visitor.previously_visited
                )
            )
            print(colour + "-" * 80 + console.reset)

            print("\n############# Bingus ##############\n")

        #     ####################
        #     # Stream Variables
        #     ####################
        #     if self.parameters.blackboard_data:
        #         print("")
        #         print(colour + "Blackboard Data" + console.reset)
        #         indent = " " * 4
        #         if not msg.blackboard_on_visited_path:
        #             print(
        #                 console.cyan + indent + "-" + console.reset + " : " +
        #                 console.yellow + "-" + console.reset
        #             )
        #         else:
        #             # could probably re-use the unicode_blackboard by passing a dict to it
        #             # like we've done for the activity stream
        #             max_length = 0
        #             for variable in msg.blackboard_on_visited_path:
        #                 max_length = len(variable.key) if len(variable.key) > max_length else max_length
        #             for variable in msg.blackboard_on_visited_path:
        #                 print(
        #                     console.cyan + indent +
        #                     '{0: <{1}}'.format(variable.key, max_length + 1) + console.reset + ": " +
        #                     console.yellow + '{0}'.format(variable.value) + console.reset
        #                 )
        #     ####################
        #     # Stream Activity
        #     ####################
        #     if self.parameters.blackboard_activity:
        #         print("")
        #         print(colour + "Blackboard Activity Stream" + console.reset)
        #         if msg.blackboard_activity:
        #             print(py_trees.display.unicode_blackboard_activity_stream(
        #                 msg.blackboard_activity,
        #                 indent=0,
        #                 show_title=False
        #             ))
        #         else:
        #             indent = " " * 4
        #             print(
        #                 console.cyan + indent + "-" + console.reset + " : " +
        #                 console.yellow + "-" + console.reset
        #             )
        #     ####################
        #     # Stream Statistics
        #     ####################
        #     if self.statistics:
        #         print("")
        #         print(colour + "Statistics" + console.reset)
        #         print(
        #             console.cyan + "    Timestamp: " + console.yellow +
        #             "{}".format(
        #                 conversions.rclpy_time_to_float(
        #                     rclpy.time.Time.from_msg(
        #                         msg.statistics.stamp
        #                     )
        #                 )
        #             )
        #         )
        #         print(
        #             console.cyan + "    Duration : " + console.yellow +
        #             "{:.3f}/{:.3f}/{:.3f} (ms) [time/avg/stddev]".format(
        #                 msg.statistics.tick_duration * 1000,
        #                 msg.statistics.tick_duration_average * 1000,
        #                 math.sqrt(msg.statistics.tick_duration_variance) * 1000
        #             )
        #         )
        #         print(
        #             console.cyan + "    Interval : " + console.yellow +
        #             "{:.3f}/{:.3f}/{:.3f} (s) [time/avg/stddev]".format(
        #                 msg.statistics.tick_interval,
        #                 msg.statistics.tick_interval_average,
        #                 math.sqrt(msg.statistics.tick_interval_variance)
        #             )
        #         )
        #         print(console.reset)

        # ####################
        # # Dot Graph
        # ####################
        # elif self.mode == WatcherMode.DOT_GRAPH and not self.rendered:
        #     self.rendered = True
        #     directory_name = tempfile.mkdtemp()
        #     py_trees.display.render_dot_tree(
        #         root=root,
        #         target_directory=directory_name,
        #         with_blackboard_variables=self.parameters.blackboard_data
        #     )
        #     xdot_program = py_trees.utilities.which('xdot')

        #     if not xdot_program:
        #         print("")
        #         console.logerror("No xdot viewer found [hint: sudo apt install xdot]")
        #         print("")
        #         print(py_trees.display.dot_tree(root=root).to_string())
        #         self.done = True
        #         self.xdot_process = None
        #         return

        #     filename = py_trees.utilities.get_valid_filename(root.name) + '.dot'
        #     if xdot_program:
        #         try:
        #             self.xdot_process = subprocess.Popen(
        #                 [
        #                     xdot_program,
        #                     os.path.join(directory_name, filename)
        #                 ]
        #             )
        #         except KeyboardInterrupt:
        #             pass
        #     self.done = True



        

def description(formatted_for_sphinx):
    short = "Open up a window onto the behaviour tree!\n"
    long = ("\nRender a oneshot snapshot of the tree as a dot graph, or\n"
            "stream it and it's state continuously as unicode art on your console.\n"
            "This utility automatically discovers the running tree and opens\n"
            "interfaces to that, but if there is more than one tree executing\n"
            "use the namespace argument to differentiate between trees.\n"
            )
    examples = {
        "--dot-graph": "render the tree as a dot graph (does not include runtime information)",
        "/tree/snapshots": "connect to an existing snapshot stream (e.g. the default, if it is enabled)",
        "": "open and connect to a snapshot stream, visualise the tree graph and it's changes only",
        "-b": "open a snapshot stream and include visited blackboard variables",
        "-a": "open a snapshot stream and include blackboard access details (activity)",
        "-s": "open a snapshot stream and include timing statistics",
    }
    script_name = "py-trees-tree-watcher"

    if formatted_for_sphinx:
        # for sphinx documentation (doesn't like raw text)
        s = short
        s += long
        s += "\n"
        s += "**Examples:**\n\n"
        s += ".. code-block:: bash\n"
        s += "    \n"
        for command, comment in examples.items():
            s += "    # {}\n".format(comment)
            s += "    $ " + script_name + " {}\n".format(command)
        s += "\n"
    else:
        banner_line = console.green + "*" * 79 + "\n" + console.reset
        s = "\n"
        s += banner_line
        s += console.bold_white + "Tree Watcher".center(79) + "\n" + console.reset
        s += banner_line
        s += "\n"
        s += short
        s += long
        s += "\n"
        s += console.bold + "Examples" + console.reset + "\n\n"
        for command, comment in examples.items():
            s += "    # {}\n".format(comment)
            s += "    $ " + console.cyan + script_name + console.yellow + " {}\n".format(command) + console.reset
        s += "\n\n"
        s += banner_line
    return s


def epilog(formatted_for_sphinx):
    if formatted_for_sphinx:
        return None
    else:
        return console.cyan + "And his noodly appendage reached forth to tickle the blessed...\n" + console.reset


def command_line_argument_parser(formatted_for_sphinx=True):
    # formatted_for_sphinx is an ugly hack to make sure sphinx does not pick up the colour codes.
    # works only by assuming that the only callee who calls it without setting the arg is sphinx's argparse
    parser = argparse.ArgumentParser(description=description(formatted_for_sphinx),
                                     epilog=epilog(formatted_for_sphinx),
                                     formatter_class=argparse.RawDescriptionHelpFormatter,
                                     )
    # common arguments
    parser.add_argument('topic_name', nargs='?', default=None, help='snapshot stream to connect to, will create a temporary stream if none specified')
    parser.add_argument('-n', '--namespace-hint', nargs='?', const=None, default=None, help='namespace hint snapshot stream services (if there should be more than one tree)')
    parser.add_argument('-a', '--blackboard-activity', action='store_true', help="show logged activity stream (streaming mode only)")
    parser.add_argument('-b', '--blackboard-data', action='store_true', help="show visited path variables (streaming mode only)")
    parser.add_argument('-s', '--statistics', action='store_true', help="show tick timing statistics (streaming mode only)")
    # don't use 'required=True' here since it forces the user to expclitly type out one option
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        '--snapshots',
        dest='mode',
        action='store_const',
        const=py_trees_ros.trees.WatcherMode.SNAPSHOTS,
        help='render ascii/unicode snapshots from a snapshot stream')
    group.add_argument(
        '--dot-graph',
        dest='mode',
        action='store_const',
        const=py_trees_ros.trees.WatcherMode.DOT_GRAPH,
        help='render the tree as a dot graph')
    return parser


def pretty_print_variables(variables):
    s = "\n"
    s += console.bold + console.cyan + "Blackboard Variables:" + console.reset + console.yellow + "\n"
    for variable in variables:
        variable = variable.split('/')
        if len(variable) > 1:
            sep = "/"
        else:
            sep = ""
        s += "    " * len(variable) + sep + variable[-1] + "\n"
    s += console.reset
    print("{}".format(s))


def echo_blackboard_contents(contents):
    """
    Args:
        contents (:obj:`str`): blackboard contents

    .. note::
        The string comes pre-formatted with bash color identifiers and newlines.
        This is currently not especially good for anything other than debugging.
    """
    print("{}".format(contents))

##############################################################################
# Main
##############################################################################


def main():
    """
    Entry point for the tree watcher script.
    """
    ####################
    # Arg Parsing
    ####################

    # command_line_args = rclpy.utilities.remove_ros_args(command_line_args)[1:]
    command_line_args = None
    parser = command_line_argument_parser(formatted_for_sphinx=False)
    args = parser.parse_args(command_line_args)

    # mode is None if the user didn't specify any option in the exclusive group
    if args.mode is None:
        args.mode = py_trees_ros.trees.WatcherMode.SNAPSHOTS
    args.snapshot_period = 2.0 if (args.statistics or args.blackboard_data or args.blackboard_activity) else py_trees.common.Duration.INFINITE.value
    # tree_watcher = py_trees_ros.trees.Watcher(
    tree_watcher = CustomWatcher(
        namespace_hint=args.namespace_hint,
        topic_name=args.topic_name,
        parameters=py_trees_ros.trees.SnapshotStream.Parameters(
            blackboard_data=args.blackboard_data,
            blackboard_activity=args.blackboard_activity,
            snapshot_period=args.snapshot_period
        ),
        mode=args.mode,
        statistics=args.statistics,
    )

    ####################
    # Setup
    ####################
    rclpy.init(args=None)
    try:
        tree_watcher.setup(timeout_sec=5.0)
    # setup discovery fails
    except py_trees_ros.exceptions.NotFoundError as e:
        print(console.red + "\nERROR: {}\n".format(str(e)) + console.reset)
        sys.exit(1)
    # setup discovery finds duplicates
    except py_trees_ros.exceptions.MultipleFoundError as e:
        print(console.red + "\nERROR: {}\n".format(str(e)) + console.reset)
        if args.namespace is None:
            print(console.red + "\nERROR: select one with the --namespace argument\n" + console.reset)
        else:
            print(console.red + "\nERROR: but none matching the requested '{}'\n".format(args.namespace) + console.reset)
        sys.exit(1)
    except py_trees_ros.exceptions.TimedOutError as e:
        print(console.red + "\nERROR: {}\n".format(str(e)) + console.reset)
        sys.exit(1)

    ####################
    # Execute
    ####################
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node=tree_watcher.node)
    try:
        while True:
            if not rclpy.ok():
                break
            if tree_watcher.done:
                if tree_watcher.xdot_process is None:
                    # no xdot found on the system, just break out and finish
                    break
                elif tree_watcher.xdot_process.poll() is not None:
                    # xdot running, wait for it to terminate
                    break
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if tree_watcher.xdot_process is not None:
            if tree_watcher.xdot_process.poll() is not None:
                tree_watcher.xdot_process.terminate()
        tree_watcher.shutdown()
        rclpy.shutdown()
