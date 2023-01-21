# can_interface

Provides a ROS node that allows CAN messages to be sent and received
as messages on ROS topics. The custom library used internally to
interact with the CAN bus is also available.

## Node Features

- Simple interface - CAN frames are represented by simple ROS messages
  that only include two values - one for the identifier and one for
  the data. Simply subscribe to or publish on the provided topics to
  interface with the CAN bus.

- No echo - Messages being sent are published on a different topic
  than messages being received. This means that messages we send will
  not be "echoed" back to us by the CAN interface.

- Multiple interfaces - This node obviously needs to be provided with
  an interface name (e.g., "can0"). It then names the used topics
  based on that interface ("outgoing_can_messages_<interface_name>"
  and "incoming_can_messages_<interface_name>"). This means that we
  can start multiple copies of this node on different interfaces
  without them interfering with one another.

- Error handling - This node includes procedures to deal with errors
  that arise during operation. These may include re-initializing the
  CAN bus, contacting a safety node, waiting and retrying, etc. (not
  yet implemented!)

## API Features

- Object-oriented interface - CAN buses are instantiated as objects
  and therefore come with all of the nice features of C++ objects. The
  bus is automatically closed on object destruction.

- Simplicity - Connecting to a bus is as easy as `new
  CanBus("can0");`. Supports checking whether there is a frame ready
  for reading (to facilitate non-blocking I/O) through a simple
  instance method. Reading and writing frames are as simple as a
  function call each.

- Exceptions - Rather than setting arcane error bits and returning -1,
  this library will throw actual, useful exceptions when things go
  wrong. This leaves your code free to handle errors as you please
  without extra overhead. (not yet implemented!)

## Notes

- When testing this package (using `colcon test`), you must have a
  virtual CAN bus named "vcan0" available on your system. Otherwise,
  the provided tests will fail. It's also worth noting that the tests
  may fail if the bus is noisy when the tests are run.
