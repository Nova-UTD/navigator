const rclnodejs = require('rclnodejs');

rclnodejs.init().then(() => {
  const node = new rclnodejs.Node('publisher_example_node');
  const publisher = node.createPublisher('std_msgs/msg/String', 'topic');

  publisher.publish(`Hello ROS 2 from rclnodejs`);

  node.spin();
});