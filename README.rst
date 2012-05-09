Installation
============

You need to have a base ROS `installation <http://www.ros.org/wiki/electric/Installation>`_ as well as the `warehousewg <http://www.ros.org/wiki/warehousewg>`_ stack installed.  Then, clone this repo into your ``ROS_PACKAGE_PATH`` and do a ``make``.

Setup
=====

The simplest way to run the logging nodes is::

    $ roslaunch logging_tools ros_logging.launch run_db:=true db_path:=/path/to/db

You can leave out ``db_path``, in which case it will default to ``/tmp/mongo``.  If you're already running a Mongo db server for other purposes (on port 27017), use ``run_db:=false`` instead.

Usage
=====

Querying rosout
---------------

Use the ``grep_log`` tool, e.g., if you want to find out when exactly your robot caught fire::

    $ rosrun logging_tools grep_log -m fire
    
For general help, see::
 
    $ rosrun logging_tools grep_log -h

If you use this often, you can add an alias to your ``.bashrc``::

    alias gl="rosrun logging_tools grep_log"
    

Querying the action history
---------------------------

For each ROS action that was performed, the goal message, goal time, and result time and status are logged.  You can query by specifying action name and optionally a max and min age.  E.g., the following gets all move_base actions that happened between one and two hours ago::

    $ rosrun logging_tools search_actions -x 120 -m 60 -a move_base
