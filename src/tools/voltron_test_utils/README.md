# voltron_test_utils

Contains various utilities used to test our code. The different
features included are listed below. Currently the only available
features are TestPublisher and TestSubscriber:

## TestPublisher

This is a templatized utility class to allow test cases to publish
messages of any type to a topic.

### Usage

First, make sure that you call `rclcpp::init()` and
`rclcpp::shutdown()` in your code - the TestPublisher will not do that
for you! To use, simply include the TestPublisher.hpp header in your
code and instantiate it as `TestPublisher<MyMessageType>
my_publisher("my_topic_name");`. You can then call
`my_publisher.send_message(MyMessageType message) to publish 
messages - simple!

### Notes

Do not try to call `rclcpp::spin()` on a TestPublisher - it is not a
ROS node. Rather, it wraps a ROS node a spins when necessary.

## TestSubscriber

The slightly more complex counterpart of TestPublisher, TestSubscriber allows your test cases to receive messages of any type on arbitrary topics.

### Usage

Instantiate it the same way as TestPublisher:
`TestSubscriber<MyMessageType> my_subscriber("my_topic_name");`. You
can then call `my_subscriber.has_message_ready()` to see if any
messages are in the queue, and `my_subscriber.get_message()` to get a
pointer to the message.

### Notes

Again, this is not a ROS node. Don't try to call `rclcpp::spin()` on
it. **DO** call `rclcpp::init()` and `rclcpp::shutdown()` before and
after using!
