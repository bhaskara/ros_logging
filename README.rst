Usage
=====

1. In the background run mongod.  If you have the warehousewg stack, this can be done using::

  $ rosrun mongodb wrapper.py _database_path:=/path/to/db

2. In a terminal::
  $ rosrun ros_logging rosout_logger
3. Do your ros thing
4. To query the log, use grep_log, e.g., if you want to find out when exactly your robot caught fire::
  $ rosrun ros_logging grep_log -m fier