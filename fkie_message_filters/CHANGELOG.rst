^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fkie_message_filters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.2 (2025-04-16)
------------------
* Use tf2_ros native function for timestamp conversions
* Inherit additional constructors for Buffer, Sequencer, and TfFilter
* Contributors: Timo Röhling

3.0.1 (2025-04-13)
------------------
* Recheck image_transport version when library is used
* Improve exception message for noncopyable types
* Enable BUILD_SHARED_LIBS by default
* Fix prefix for namespace macro
* Contributors: Timo Röhling

3.0.0 (2025-04-01)
------------------
* Improve test coverage
* Add explicit ABI version
* Add support for test coverage
* Support sanitizer plugings for tests
* Add quality declaration
* Make Signals class movable
* Rename headers from *.h to *.hpp
* Use #pragma once to help clangd not choke on _impl headers
* Add rosdoc2 configuration
* Contributors: Timo Röhling

2.1.0 (2025-03-31)
------------------
* Clean up includes
* Add logging with configurable ROS logger
* Use FKIE_MF as macro prefix (instead of FKIE_MESSAGE_FILTERS)
* Contributors: Timo Röhling

2.0.1 (2025-03-27)
------------------
* Minor bugfixes for buffer/callback group interaction
* Improve support for older image_transport API versions
* Add pre-commit hook for code formatting
* Contributors: Timo Röhling

2.0.0 (2025-03-24)
------------------
* Reimplementation for ROS 2
* Contributors: Timo Röhling

1.1.2 (2020-09-03)
------------------
* Fix compiler warnings with Clang 9
* Fix compilation error due to namespace problems with friend class
* Contributors: Timo Röhling

1.1.1 (2020-09-01)
------------------
* Fix compilation bug with RosMessage adapter
* Contributors: Timo Röhling

1.1.0 (2020-08-31)
------------------
* Rewrite documentation
* Add test for subscriber callbacks
* Use "nullptr" instead of "0"
* Add rostest testsuite for tests involving ROS publish/subscribe
* Add subscriber status callback support
* Contributors: Timo Röhling

1.0.1 (2019-09-18)
------------------
* Bugfix for ODR violation
* Improve documentation
* Rename helper function for more clarity
* Reword documentation to be a bit more precise
* Contributors: Timo Röhling

1.0.0
-----
* Initial release
