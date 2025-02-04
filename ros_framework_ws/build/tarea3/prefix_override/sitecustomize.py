import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/usuario/Documentos/GitHub/IR2136-RAYS/ros_framework_ws/install/tarea3'
