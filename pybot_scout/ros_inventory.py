# -*- coding: utf-8 -*-
"""
Enumerate the full ROS graph (nodes, topics with types, services) and log it.

Uses the ROS master XMLRPC API so it works with any ROS node or service,
not just the topics that have active publishers.
"""


def _log(logger, event, **fields):
    if logger is not None:
        logger.log(event, **fields)


def log_ros_inventory(logger=None):
    """Query the ROS master for the full graph and emit structured log events.

    Logs the following events:
      ros_inventory_nodes       – list of all known ROS node names
      ros_inventory_topics      – list of {topic, topic_type} dicts
      ros_inventory_services    – list of service names
      ros_inventory_failed      – if the master is unreachable

    Returns a dict with keys 'nodes', 'topics', 'services' (empty lists on failure).
    """
    inventory = {"nodes": [], "topics": [], "services": []}

    try:
        import rospy

        master = rospy.get_master()
        code, msg, state = master.getSystemState()
        if code != 1:
            _log(logger, "ros_inventory_failed", reason="getSystemState returned code %d: %s" % (code, msg))
            return inventory

        publishers, subscribers, services = state

        # Collect all unique node names across publishers and subscribers
        node_set = set()
        for _topic, nodes in publishers + subscribers:
            for n in nodes:
                node_set.add(n)
        inventory["nodes"] = sorted(node_set)

        # Collect all unique service names
        service_set = set()
        for svc, _nodes in services:
            service_set.add(svc)
        inventory["services"] = sorted(service_set)

        # Collect topics with their types via get_published_topics
        try:
            published = rospy.get_published_topics()
        except Exception as exc:
            published = []
            _log(logger, "ros_inventory_topic_types_failed", error=str(exc))

        type_map = {topic: topic_type for topic, topic_type in published}

        pub_counts = {}
        for topic, nodes in publishers:
            pub_counts[topic] = len(nodes or [])

        sub_counts = {}
        for topic, nodes in subscribers:
            sub_counts[topic] = len(nodes or [])

        # Union of all topics seen in publishers/subscribers
        topic_set = set()
        for topic, _nodes in publishers + subscribers:
            topic_set.add(topic)

        topics_with_types = []
        for topic in sorted(topic_set):
            topics_with_types.append({
                "topic": topic,
                "topic_type": type_map.get(topic, "unknown"),
                "publisher_count": int(pub_counts.get(topic, 0)),
                "subscriber_count": int(sub_counts.get(topic, 0)),
                "has_publishers": bool(pub_counts.get(topic, 0) > 0),
                "has_subscribers": bool(sub_counts.get(topic, 0) > 0),
            })
        inventory["topics"] = topics_with_types

    except Exception as exc:
        _log(logger, "ros_inventory_failed", error=str(exc))
        return inventory

    _log(logger, "ros_inventory_nodes", count=len(inventory["nodes"]), nodes=inventory["nodes"])
    _log(logger, "ros_inventory_topics", count=len(inventory["topics"]), topics=inventory["topics"])
    _log(logger, "ros_inventory_services", count=len(inventory["services"]), services=inventory["services"])

    return inventory
