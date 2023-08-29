import * as d3 from "https://cdn.jsdelivr.net/npm/d3@7/+esm";
import "https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.min.js";


export default {
    template: `
    <button v-for="(tree, treeName) in forest" @click="setTree(treeName)">
        {{ treeName }}
    </button>
    <template v-for="(tree, treeName) in forest">
        <svg v-show="activeTree === treeName" height="100%" width="100%">
            <g :id="treeName">
                <template v-for="node in tree.descendants()">
                    <text v-if="node.data.isText" text-anchor="middle" :y="node.y">
                        <tspan :x="node.x" dy="1.2em">{{ node.parent.data.instance_name }}</tspan>
                        <tspan :x="node.x" dy="1.2em">{{ node.parent.data.registration_name }}</tspan>
                        <tspan :x="node.x" dy="1.2em">{{ node.parent.data.visits }}</tspan>
                    </text>
                    <circle v-else :cx="node.x" :cy="node.y" r="10"/>
                </template>
                <template v-for="edge in tree.links()">
                    <line v-if="!(edge.target.data.isText || edge.target.data.isText)" :x1="edge.source.x" :y1="edge.source.y" :x2="edge.target.x" :y2="edge.target.y"/>
                </template>
            </g>
        </svg>
    </template>
    `,
    created() {
        // Setup ROS connection.
        const ros = new ROSLIB.Ros({
            url: "ws://localhost:9090"
        });
        ros.on("connection", () => {
            console.log("Connected to rosbridge server.");
        });
        ros.on("error", (error) => {
            console.log("Error connecting to rosbridge server: ", error);
        });
        ros.on("close", () => {
            console.log("Connection to rosbridge server closed.");
        });

        // Create ROS subscriber for behavior tree changes.
        const listener = new ROSLIB.Topic({
            ros: this.$ros,
            name: "/bt_status_change_log",
            messageType: "tree_msgs/StatusChangeLog"
        });
        // listener.subscribe(this.update);
    },
    mounted() {
        this.update({ "behavior_tree": { "tree_uid": "/home/entran/code/research/ros2_ws/install/nav2_bt_navigator/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml", "root_uid": 1, "nodes": [{ "uid": 1, "child_uids": [2, 14], "type": 3, "instance_name": "NavigateRecovery", "registration_name": "RecoveryNode", "params": [] }, { "uid": 2, "child_uids": [3, 9], "type": 3, "instance_name": "NavigateWithReplanning", "registration_name": "PipelineSequence", "params": [] }, { "uid": 3, "child_uids": [4], "type": 4, "instance_name": "RateController", "registration_name": "RateController", "params": [] }, { "uid": 4, "child_uids": [5, 6], "type": 3, "instance_name": "ComputePathToPose", "registration_name": "RecoveryNode", "params": [] }, { "uid": 5, "child_uids": [], "type": 1, "instance_name": "ComputePathToPose", "registration_name": "ComputePathToPose", "params": [] }, { "uid": 6, "child_uids": [7, 8], "type": 3, "instance_name": "Sequence", "registration_name": "Sequence", "params": [] }, { "uid": 7, "child_uids": [], "type": 2, "instance_name": "WouldAPlannerRecoveryHelp", "registration_name": "WouldAPlannerRecoveryHelp", "params": [] }, { "uid": 8, "child_uids": [], "type": 1, "instance_name": "ClearGlobalCostmap-Context", "registration_name": "ClearEntireCostmap", "params": [] }, { "uid": 9, "child_uids": [10, 11], "type": 3, "instance_name": "FollowPath", "registration_name": "RecoveryNode", "params": [] }, { "uid": 10, "child_uids": [], "type": 1, "instance_name": "FollowPath", "registration_name": "FollowPath", "params": [] }, { "uid": 11, "child_uids": [12, 13], "type": 3, "instance_name": "Sequence", "registration_name": "Sequence", "params": [] }, { "uid": 12, "child_uids": [], "type": 2, "instance_name": "WouldAControllerRecoveryHelp", "registration_name": "WouldAControllerRecoveryHelp", "params": [] }, { "uid": 13, "child_uids": [], "type": 1, "instance_name": "ClearLocalCostmap-Context", "registration_name": "ClearEntireCostmap", "params": [] }, { "uid": 14, "child_uids": [15, 18], "type": 3, "instance_name": "Sequence", "registration_name": "Sequence", "params": [] }, { "uid": 15, "child_uids": [16, 17], "type": 3, "instance_name": "Fallback", "registration_name": "Fallback", "params": [] }, { "uid": 16, "child_uids": [], "type": 2, "instance_name": "WouldAControllerRecoveryHelp", "registration_name": "WouldAControllerRecoveryHelp", "params": [] }, { "uid": 17, "child_uids": [], "type": 2, "instance_name": "WouldAPlannerRecoveryHelp", "registration_name": "WouldAPlannerRecoveryHelp", "params": [] }, { "uid": 18, "child_uids": [19, 20], "type": 3, "instance_name": "RecoveryFallback", "registration_name": "ReactiveFallback", "params": [] }, { "uid": 19, "child_uids": [], "type": 2, "instance_name": "GoalUpdated", "registration_name": "GoalUpdated", "params": [] }, { "uid": 20, "child_uids": [21, 24, 25, 26], "type": 3, "instance_name": "RecoveryActions", "registration_name": "RoundRobin", "params": [] }, { "uid": 21, "child_uids": [22, 23], "type": 3, "instance_name": "ClearingActions", "registration_name": "Sequence", "params": [] }, { "uid": 22, "child_uids": [], "type": 1, "instance_name": "ClearLocalCostmap-Subtree", "registration_name": "ClearEntireCostmap", "params": [] }, { "uid": 23, "child_uids": [], "type": 1, "instance_name": "ClearGlobalCostmap-Subtree", "registration_name": "ClearEntireCostmap", "params": [] }, { "uid": 24, "child_uids": [], "type": 1, "instance_name": "Spin", "registration_name": "Spin", "params": [] }, { "uid": 25, "child_uids": [], "type": 1, "instance_name": "Wait", "registration_name": "Wait", "params": [] }, { "uid": 26, "child_uids": [], "type": 1, "instance_name": "BackUp", "registration_name": "BackUp", "params": [] }] }, "state_changes": [{ "uid": 10, "prev_status": { "value": 1 }, "status": { "value": 2 }, "timestamp": { "sec": 1693262512, "nanosec": 277161987 } }, { "uid": 10, "prev_status": { "value": 2 }, "status": { "value": 0 }, "timestamp": { "sec": 1693262512, "nanosec": 277171221 } }, { "uid": 9, "prev_status": { "value": 1 }, "status": { "value": 2 }, "timestamp": { "sec": 1693262512, "nanosec": 277171807 } }, { "uid": 3, "prev_status": { "value": 1 }, "status": { "value": 0 }, "timestamp": { "sec": 1693262512, "nanosec": 277172594 } }, { "uid": 9, "prev_status": { "value": 2 }, "status": { "value": 0 }, "timestamp": { "sec": 1693262512, "nanosec": 277173047 } }, { "uid": 2, "prev_status": { "value": 1 }, "status": { "value": 2 }, "timestamp": { "sec": 1693262512, "nanosec": 277173402 } }, { "uid": 2, "prev_status": { "value": 2 }, "status": { "value": 0 }, "timestamp": { "sec": 1693262512, "nanosec": 277173772 } }, { "uid": 1, "prev_status": { "value": 1 }, "status": { "value": 2 }, "timestamp": { "sec": 1693262512, "nanosec": 277174122 } }, { "uid": 1, "prev_status": { "value": 2 }, "status": { "value": 0 }, "timestamp": { "sec": 1693262512, "nanosec": 277175579 } }] });
    },
    data() {
        return {
            activeTree: "",
            forest: {}
        }
    },
    methods: {
        update(message) {
            let treeUid = message.behavior_tree.tree_uid.split("/").pop();

            // Add tree if not exists.
            if (!(treeUid in this.$data.forest)) {
                // Create backlink from child nodes to parent nodes and create
                // text labels.
                let nodes = message.behavior_tree.nodes;
                const texts = [];
                nodes.forEach(node => {
                    const parentUid = node.uid;
                    node.child_uids.forEach(childUid => {
                        const childIdx = nodes.findIndex(n => n.uid === childUid);
                        nodes[childIdx].parent_uid = parentUid;
                    });

                    node.visits = 0;
                    node.idle = 0;
                    node.running = 0;
                    node.success = 0;
                    node.failure = 0;
                    node.isText = false;

                    texts.push({
                        parent_uid: parentUid,
                        isText: true
                    });
                });
                nodes = nodes.concat(texts);

                // Generate D3 graph.
                const root = d3.stratify()
                    .id(node => { return node.uid; })
                    .parentId(node => { return node.parent_uid; })
                    (nodes);
                const treeLayout = d3.tree().size([2000, 500]);
                treeLayout(root);
                this.$data.forest[treeUid] = root;

                // Prevent text overlap. 
                d3.forceSimulation(this.$data.forest[treeUid].descendants())
                    .force("charge", d3.forceManyBody())
                    .force("link", d3.forceLink(this.$data.forest[treeUid].links()
                        .filter(link => link.target.data.isText || link.target.data.isText)
                    ).distance(30));
            }

            // Keep track of state changes.
            if (message.state_changes) {
                message.state_changes.forEach(transition => {
                    const node = this.$data.forest[treeUid].descendants().find(d => d.data.uid === transition.uid);
                    node.data.visits += 1;
                })
            }
        },
        handleZoom(e) {
            d3.selectAll("svg g").attr("transform", e.transform);
        },
        setTree(treeName) {
            this.$data.activeTree = treeName;

            // Zoom and panning behavior.
            d3.select("svg").call(d3.zoom().on("zoom", this.handleZoom));
        }
    }
}
