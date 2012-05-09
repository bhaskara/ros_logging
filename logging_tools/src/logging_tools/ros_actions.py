import roslib

def normalize_name(name):
    "Normalize an action name: /foo/bar/baz -> foo_bar_baz"
    if name.startswith('/'):
        name = name[1:]
    return name.replace('/','_')

def get_classes(pkg, action_type):
    "Get goal and result action class objects given action type name"
    # Import the message modules for this ros package
    mod = import_msg_pkg(pkg)
    
    # Get the message class objects
    goal_type = action_type+'Goal'
    result_type = action_type+'Result'

    goal_class = getattr(mod, goal_type)
    result_class = getattr(mod, result_type)
    
    return goal_class, result_class

def get_goal_class(pkg, action_type):
    "Returns FooGoal as opposed to FooActionGoal"
    mod = import_msg_pkg(pkg)
    goal_type = action_type[:-6]+'Goal'
    return getattr(mod, goal_type)
    

def import_msg_pkg(pkg):
    roslib.load_manifest(pkg)
    m = __import__(pkg+'.msg')
    return getattr(m, 'msg')
