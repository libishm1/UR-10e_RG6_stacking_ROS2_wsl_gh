"""
Generate a Grasshopper .ghx with a single GhPython component plus a panel of
boolean toggles / panels / sliders. Best-effort — open in Rhino 8, right-click
the component header and switch to "Python 3 (CPython)" if it isn't already.

If Grasshopper refuses the file, fall back to pasting ur10e_rg6_gh.py into a
fresh GhPython component on a blank canvas (see README.md for input names).
"""
import os, uuid, html

HERE = os.path.dirname(os.path.abspath(__file__))
PY_PATH = os.path.join(HERE, "ur10e_rg6_gh.py")
OUT_PATH = os.path.join(HERE, "ur10e_rg6.ghx")

with open(PY_PATH, "r", encoding="utf-8") as f:
    SCRIPT = f.read()

# Stable Grasshopper component GUIDs
GH_PYTHON     = "410755B1-224A-4C1E-A407-BF32FB45EA7E"
BOOL_TOGGLE   = "2E78987B-9DCB-42EA-A7A6-2B68BA0E8B79"
PANEL         = "59E0B89A-E487-49F8-BAB8-B5BAB16BE14C"

def new_guid():
    return str(uuid.uuid4()).upper()

# Instance GUIDs for placed components
ID_PY        = new_guid()
ID_HOST      = new_guid()
ID_PORT      = new_guid()
ID_CONNECT   = new_guid()
ID_MODE      = new_guid()
ID_TARGET    = new_guid()
ID_DURATION  = new_guid()
ID_VELSCALE  = new_guid()
ID_MOVE      = new_guid()
ID_GRIPPER   = new_guid()
ID_TRIGGER   = new_guid()
ID_OK_OUT    = new_guid()
ID_LOG_OUT   = new_guid()

# Per-input/per-output parameter GUIDs of the GhPython component
def io_ids(n):
    return [new_guid() for _ in range(n)]

INPUT_NAMES  = ["host", "port", "connect", "mode", "target", "duration",
                "vel_scale", "move", "gripper", "trigger_gr", "tick"]
OUTPUT_NAMES = ["ok", "names", "positions", "tcp_pos", "tcp_quat", "log"]
PY_IN_IDS    = io_ids(len(INPUT_NAMES))
PY_OUT_IDS   = io_ids(len(OUTPUT_NAMES))

# Helpers
def gh_str(name, val, code="10"):
    return ('<item name="' + name + '" type_name="gh_string" type_code="' + code + '">'
            + html.escape(str(val)) + '</item>')

def gh_int(name, val):
    return '<item name="' + name + '" type_name="gh_int32" type_code="3">' + str(int(val)) + '</item>'

def gh_bool(name, val):
    return '<item name="' + name + '" type_name="gh_bool" type_code="1">' + ('true' if val else 'false') + '</item>'

def gh_guid(name, val):
    return '<item name="' + name + '" type_name="gh_guid" type_code="9">' + val + '</item>'

def gh_drawing_rect(name, x, y, w, h):
    return ('<item name="' + name + '" type_name="gh_drawing_rectanglef" type_code="35">'
            '<X>' + str(x) + '</X><Y>' + str(y) + '</Y><W>' + str(w) + '</W><H>' + str(h) + '</H></item>')

def gh_drawing_point(name, x, y):
    return ('<item name="' + name + '" type_name="gh_drawing_pointf" type_code="31">'
            '<X>' + str(x) + '</X><Y>' + str(y) + '</Y></item>')

# Build the GhPython component
def py_input_param(pid, name):
    access = 1 if name == "target" else 0  # list vs item
    items = [
        gh_guid("InstanceGuid", pid),
        gh_str("Name", name),
        gh_str("NickName", name),
        gh_str("Description", ""),
        gh_bool("Optional", True),
        gh_bool("Reverse", False),
        gh_bool("Simplify", False),
        gh_int("Access", access),
        gh_int("SourceCount", 0),
        gh_int("TypeHintID", 0),
    ]
    return ('<chunk name="param_input">'
            '<items count="' + str(len(items)) + '">' + "".join(items) + '</items>'
            '</chunk>')

def py_output_param(pid, name):
    items = [
        gh_guid("InstanceGuid", pid),
        gh_str("Name", name),
        gh_str("NickName", name),
        gh_str("Description", ""),
        gh_bool("Optional", True),
    ]
    return ('<chunk name="param_output">'
            '<items count="' + str(len(items)) + '">' + "".join(items) + '</items>'
            '</chunk>')

py_attr = ('<chunk name="Attributes"><items count="2">'
           + gh_drawing_point("Pivot", 600, 320)
           + gh_drawing_rect("Bounds", 540, 240, 120, 160)
           + '</items></chunk>')

py_container_items = [
    gh_guid("InstanceGuid", ID_PY),
    gh_str("Name", "GhPython Script"),
    gh_str("NickName", "ur10e_rg6"),
    gh_str("Description", "UR10e + RG6 ROS 2 bridge runtime"),
    gh_bool("HideInput", False),
    gh_bool("HideOutput", False),
    gh_bool("IsAdvancedMode", True),
    gh_int("IconDisplay", 0),
    gh_str("CodeInput", SCRIPT),
]

input_chunks  = "".join(py_input_param(pid, n) for pid, n in zip(PY_IN_IDS, INPUT_NAMES))
output_chunks = "".join(py_output_param(pid, n) for pid, n in zip(PY_OUT_IDS, OUTPUT_NAMES))

py_object = (
    '<chunk name="Object">'
    '<items count="2">'
    + gh_guid("GUID", GH_PYTHON)
    + gh_str("Name", "GhPython Script")
    + '</items>'
    '<chunks count="1">'
    '<chunk name="Container">'
    '<items count="' + str(len(py_container_items)) + '">' + "".join(py_container_items) + '</items>'
    '<chunks count="' + str(1 + len(INPUT_NAMES) + len(OUTPUT_NAMES)) + '">'
    + py_attr + input_chunks + output_chunks +
    '</chunks></chunk></chunks></chunk>'
)

# Satellite components
def make_toggle(iid, nick, x, y):
    items = [
        gh_guid("InstanceGuid", iid),
        gh_str("Name", "Boolean Toggle"),
        gh_str("NickName", nick),
        gh_str("Description", nick),
        gh_bool("Optional", False),
        gh_bool("Value", False),
    ]
    attr = ('<chunk name="Attributes"><items count="2">'
            + gh_drawing_point("Pivot", x + 35, y + 11)
            + gh_drawing_rect("Bounds", x, y, 70, 22)
            + '</items></chunk>')
    return ('<chunk name="Object">'
            '<items count="2">'
            + gh_guid("GUID", BOOL_TOGGLE)
            + gh_str("Name", "Boolean Toggle")
            + '</items>'
            '<chunks count="1">'
            '<chunk name="Container">'
            '<items count="' + str(len(items)) + '">' + "".join(items) + '</items>'
            '<chunks count="1">' + attr + '</chunks></chunk></chunks></chunk>')

def make_panel(iid, text, x, y, w=160, h=22):
    items = [
        gh_guid("InstanceGuid", iid),
        gh_str("Name", "Panel"),
        gh_str("NickName", text[:20]),
        gh_str("UserText", text),
        gh_bool("MultiLine", True),
        gh_bool("DrawIndices", False),
        gh_bool("DrawPaths", False),
        gh_bool("Stream", False),
        gh_bool("Wrap", True),
    ]
    attr = ('<chunk name="Attributes"><items count="2">'
            + gh_drawing_point("Pivot", x + w/2, y + h/2)
            + gh_drawing_rect("Bounds", x, y, w, h)
            + '</items></chunk>')
    return ('<chunk name="Object">'
            '<items count="2">'
            + gh_guid("GUID", PANEL)
            + gh_str("Name", "Panel")
            + '</items>'
            '<chunks count="1">'
            '<chunk name="Container">'
            '<items count="' + str(len(items)) + '">' + "".join(items) + '</items>'
            '<chunks count="1">' + attr + '</chunks></chunk></chunks></chunk>')

objects = [
    py_object,
    make_panel(ID_HOST, "localhost", 340, 160),
    make_panel(ID_PORT, "9090", 340, 185),
    make_toggle(ID_CONNECT, "connect", 350, 210),
    make_panel(ID_MODE, "direct", 340, 235),
    make_panel(ID_TARGET, "0, -1.57, 1.57, -1.57, -1.57, 0", 320, 260, w=190),
    make_panel(ID_DURATION, "4.0", 340, 285),
    make_panel(ID_VELSCALE, "0.2", 340, 310),
    make_toggle(ID_MOVE, "move", 350, 335),
    make_toggle(ID_GRIPPER, "close?", 350, 360),
    make_toggle(ID_TRIGGER, "trigger_gr", 350, 385),
    make_panel(ID_OK_OUT, "(ok)", 730, 240),
    make_panel(ID_LOG_OUT, "(log)", 730, 320, w=220, h=60),
]

doc_objects_chunks = "".join(objects)

ghx = (
    '<?xml version="1.0" encoding="utf-8"?>\n'
    '<Archive name="Root">\n'
    '  <items count="1">\n'
    '    <item name="ArchiveVersion" type_name="gh_version" type_code="80">\n'
    '      <Major>0</Major><Minor>2</Minor><Revision>2</Revision>\n'
    '    </item>\n'
    '  </items>\n'
    '  <chunks count="2">\n'
    '    <chunk name="Definition">\n'
    '      <items count="1">\n'
    '        <item name="plugin_version" type_name="gh_version" type_code="80">\n'
    '          <Major>1</Major><Minor>0</Minor><Revision>0007</Revision>\n'
    '        </item>\n'
    '      </items>\n'
    '      <chunks count="3">\n'
    '        <chunk name="DocumentHeader">\n'
    '          <items count="3">\n'
    '            ' + gh_guid("DocumentID", new_guid()) + '\n'
    '            ' + gh_str("Preview", "Shaded") + '\n'
    '            ' + gh_int("PreviewMeshType", 1) + '\n'
    '          </items>\n'
    '        </chunk>\n'
    '        <chunk name="DefinitionProperties">\n'
    '          <items count="5">\n'
    '            <item name="Date" type_name="gh_date" type_code="8">638540000000000000</item>\n'
    '            ' + gh_str("Description", "UR10e + RG6 ROS 2 bridge") + '\n'
    '            ' + gh_bool("KeepOpen", False) + '\n'
    '            ' + gh_str("Name", "ur10e_rg6") + '\n'
    '            ' + gh_int("Revision", 1) + '\n'
    '          </items>\n'
    '          <chunks count="2">\n'
    '            <chunk name="Projection">\n'
    '              <items count="2">\n'
    '                ' + gh_drawing_point("Target", 700, 350) + '\n'
    '                <item name="Zoom" type_name="gh_single" type_code="6">1.0</item>\n'
    '              </items>\n'
    '            </chunk>\n'
    '            <chunk name="Views"><items count="1"><item name="ViewCount" type_name="gh_int32" type_code="3">0</item></items></chunk>\n'
    '          </chunks>\n'
    '        </chunk>\n'
    '        <chunk name="RcpLayout"><items count="1"><item name="RcpCount" type_name="gh_int32" type_code="3">0</item></items></chunk>\n'
    '      </chunks>\n'
    '    </chunk>\n'
    '    <chunk name="DefinitionObjects">\n'
    '      <items count="1">\n'
    '        <item name="ObjectCount" type_name="gh_int32" type_code="3">' + str(len(objects)) + '</item>\n'
    '      </items>\n'
    '      <chunks count="' + str(len(objects)) + '">\n'
    '        ' + doc_objects_chunks + '\n'
    '      </chunks>\n'
    '    </chunk>\n'
    '  </chunks>\n'
    '</Archive>\n'
)

with open(OUT_PATH, "w", encoding="utf-8") as f:
    f.write(ghx)

print("wrote", OUT_PATH, "(" + str(len(ghx)) + " bytes,", len(objects), "objects)")
