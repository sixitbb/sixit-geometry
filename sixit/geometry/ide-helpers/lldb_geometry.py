# LLDB pretty-printer for geometry classes
import lldb

def dump_valobj(valobj, dict):
    s = ''
    for i in range(valobj.GetNumChildren()):
        child = valobj.GetChildAtIndex(i)
        s = s + f'Name: {child.GetName()}; '
        s = s + f'Type: {child.GetType()}; '
        s = s + f'Value: {child.GetValue()}'
    return s
    
def Vector2_Summary(valobj, dict):
    v = valobj.GetChildAtIndex(0);
    data = v.GetChildAtIndex(0).GetChildAtIndex(0).GetChildMemberWithName('data');
    x = data.GetChildAtIndex(0).GetValue();
    y = data.GetChildAtIndex(1).GetValue();
    return f'{{{x}, {y}}}'

def Point2_Summary(valobj, dict):
    return Vector2_Summary(valobj.GetChildMemberWithName('vector'), dict)

def Direction2_Summary(valobj, dict):
    return Vector2_Summary(valobj.GetChildMemberWithName('vector'), dict)
    
def Vector3_Summary(valobj, dict):
    vec4 = valobj.GetChildAtIndex(0)
    xyzw = vec4.GetChildAtIndex(0).GetChildAtIndex(0);
    x = xyzw.GetChildMemberWithName('x').GetValue()
    y = xyzw.GetChildMemberWithName('y').GetValue()
    z = xyzw.GetChildMemberWithName('z').GetValue()
    return f'{{{x}, {y}, {z}}}'

def Direction3_Summary(valobj, dict):
    vector = valobj.GetChildMemberWithName('vector')
    return f'{Vector3_Summary(vector, dict)}'

def Point3_Summary(valobj, dict):
    vector = valobj.GetChildMemberWithName('vector')
    return f'{Vector3_Summary(vector, dict)}'

def Triangle2_Summary(valobj, dict):
    output = []
    points = valobj.GetChildAtIndex(0).GetChildMemberWithName('points').GetChildMemberWithName('__elems_');
    for i in range(points.GetNumChildren()):
        point = points.GetChildAtIndex(i)
        output.append(Point2_Summary(point, dict))
    return '{' + ', '.join(output) + '}'

def Triangle3_Summary(valobj, dict):
    output = []
    points = valobj.GetChildAtIndex(0).GetChildMemberWithName('points').GetChildMemberWithName('__elems_');
    for i in range(points.GetNumChildren()):
        point = points.GetChildAtIndex(i)
        output.append(Point3_Summary(point, dict))
    return '{' + ', '.join(output) + '}'
    
def Grid2Cell_Summary(valobj, dict):
    x = valobj.GetChildMemberWithName('x').GetValue()
    y = valobj.GetChildMemberWithName('y').GetValue()
    return f'{{{x}, {y}}}'

def Grid2_Summary(valobj, dict):
    step_x = valobj.GetChildMemberWithName('step_x_').GetValue()
    step_y = valobj.GetChildMemberWithName('step_y').GetValue()
    return f'{{{step_x} x {step_y}}}'

def LineSegment2_Summary(valobj, dict):
    points = valobj.GetChildAtIndex(0).GetChildAtIndex(0).GetChildMemberWithName('points').GetChildMemberWithName('__elems_');
    p1 = points.GetChildAtIndex(0);
    p2 = points.GetChildAtIndex(1);
    return f'{{{Point2_Summary(p1, dict)} - {Point2_Summary(p2, dict)}}}'
    
def Ray2_Summary(valobj, dict):
    origin = valobj.GetChildMemberWithName('origin_data')
    dir = valobj.GetChildMemberWithName('direction_data')
    return f'{{o:{Point2_Summary(origin,dict)} d:{Direction2_Summary(dir,dict)}}}'
    
def TwoPointsHolder_Summary(valobj, dict):
    points = valobj.GetChildAtIndex(0).GetChildMemberWithName('points').GetChildMemberWithName('__elems_')
    p1 = points.GetChildAtIndex(0)
    p2 = points.GetChildAtIndex(1)
    return f'{{{p1.GetSummary()}, {p2.GetSummary()}}}'
    
def Line3_Summary(valobj, dict):
    return f'{valobj.GetChildAtIndex(0).GetSummary()}'

def Line2_Summary(valobj, dict):
    return f'{valobj.GetChildAtIndex(0).GetSummary()}'

# auto-register summary printers using `type summary add ...` command
# in practice, it appears that sometimes XCode fails to pick the type using 
# qualified-name, so it may be worth duplicating it without the namespace
# NOTE: python module name is the current file name without extension
def __lldb_init_module(debugger, dict):
    debugger.HandleCommand('type summary add -F lldb_geometry.Vector2_Summary "sixit::geometry::low_level::vector2"')
    debugger.HandleCommand('type summary add -F lldb_geometry.Vector2_Summary "vector2"')
    debugger.HandleCommand('type summary add -F lldb_geometry.Point2_Summary "sixit::geometry::point2"')
    debugger.HandleCommand('type summary add -F lldb_geometry.Point2_Summary "point2"')
    debugger.HandleCommand('type summary add -F lldb_geometry.Triangle2_Summary "sixit::geometry::triangle2"')
    debugger.HandleCommand('type summary add -F lldb_geometry.Triangle3_Summary "sixit::geometry::triangle3"')
    debugger.HandleCommand('type summary add -F lldb_geometry.Grid2Cell_Summary "sixit::geometry::grid2_cell"')
    debugger.HandleCommand('type summary add -F lldb_geometry.Grid2_Summary "sixit::geometry::grid2"')
    debugger.HandleCommand('type summary add -F lldb_geometry.LineSegment2_Summary "sixit::geometry::line_segment2"')
    debugger.HandleCommand('type summary add -F lldb_geometry.Direction2_Summary "sixit::geometry::direction2"')
    debugger.HandleCommand('type summary add -F lldb_geometry.Direction2_Summary "direction2"')
    debugger.HandleCommand('type summary add -F lldb_geometry.Ray2_Summary "sixit::geometry::ray2"')
    debugger.HandleCommand('type summary add -F lldb_geometry.Ray2_Summary "ray2"')
    debugger.HandleCommand('type summary add -F lldb_geometry.Vector3_Summary "sixit::geometry::low_level::vector3"')
    debugger.HandleCommand('type summary add -F lldb_geometry.Direction3_Summary "sixit::geometry::direction3"')
    debugger.HandleCommand('type summary add -F lldb_geometry.Direction3_Summary "sixit::geometry::direction3::averaging"')
    debugger.HandleCommand('type summary add -F lldb_geometry.Point3_Summary "sixit::geometry::point3"')
    debugger.HandleCommand('type summary add -F lldb_geometry.Line3_Summary "sixit::geometry::line3"')
    debugger.HandleCommand('type summary add -F lldb_geometry.Line2_Summary "sixit::geometry::line2"')
    debugger.HandleCommand('type summary add -F lldb_geometry.TwoPointsHolder_Summary "sixit::geometry::low_level::two_points_holder<sixit::geometry::*>"')
