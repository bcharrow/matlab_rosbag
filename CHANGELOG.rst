^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package matlab_rosbag
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2014-03-31)
------------------
* Add ability to 1) filter messages by time and 2) access bag start/stop time.
* New class, ros.TFTree, for working with stored TF messages
* Autocomplete paths to bags using ros.Bag.load()
* Update build to use Hydro

0.3.0 (2013-06-09)
------------------
* Add equivalents of 'rosbag info', 'rosmsg show', and 'rosmsg show --raw' to ros.Bag
* Rename ros.Bag functions readMessage() to read() and readAllMessages() to readAll()
* By default, return simple messages as matrices instead of structs
* Improve overall performance; depending on the message type, reading messages is 2x-20x faster than v0.2
* Update build to use Groovy; much easier to compile everything

0.2.0 (2012-02-11)
------------------
* Improve performance for fixed size messages
* Various bugfixes (constants, primitive aliases, 0 length arrays)

0.1.0 (2012-02-04)
------------------
* Initial release using ROS electric
