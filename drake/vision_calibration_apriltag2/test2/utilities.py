import argparse

from kortex_api.TCPTransport import TCPTransport
from kortex_api.UDPTransport import UDPTransport
from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
from kortex_api.SessionManager import SessionManager
from kortex_api.autogen.messages import Session_pb2, DeviceConfig_pb2, DeviceManager_pb2, VisionConfig_pb2

#
# Dictionary of all Sensor strings
#
all_sensor_strings = {
    VisionConfig_pb2.SENSOR_UNSPECIFIED : "Unspecified sensor",
    VisionConfig_pb2.SENSOR_COLOR       : "Color",
    VisionConfig_pb2.SENSOR_DEPTH       : "Depth"
}

#
# Dictionary of all Resolution strings
#
all_resolution_strings = {
    VisionConfig_pb2.RESOLUTION_UNSPECIFIED : "Unspecified resolution",
    VisionConfig_pb2.RESOLUTION_320x240     : "320x240",
    VisionConfig_pb2.RESOLUTION_424x240     : "424x240",
    VisionConfig_pb2.RESOLUTION_480x270     : "480x270",
    VisionConfig_pb2.RESOLUTION_640x480     : "640x480",
    VisionConfig_pb2.RESOLUTION_1280x720    : "1280x720",
    VisionConfig_pb2.RESOLUTION_1920x1080   : "1920x1080"
}

def parseConnectionArguments(parser = argparse.ArgumentParser()):
    parser.add_argument("--ip", type=str, help="IP address of destination", default="192.168.1.10")
    parser.add_argument("-u", "--username", type=str, help="username to login", default="admin")
    parser.add_argument("-p", "--password", type=str, help="password to login", default="admin")
    return parser.parse_args()

class DeviceConnection:
    
    TCP_PORT = 10000
    UDP_PORT = 10001

    @staticmethod
    def createTcpConnection(args): 
        """
        returns RouterClient required to create services and send requests to device or sub-devices,
        """

        return DeviceConnection(args.ip, port=DeviceConnection.TCP_PORT, credentials=(args.username, args.password))

    @staticmethod
    def createUdpConnection(args): 
        """        
        returns RouterClient that allows to create services and send requests to a device or its sub-devices @ 1khz.
        """

        return DeviceConnection(args.ip, port=DeviceConnection.UDP_PORT, credentials=(args.username, args.password))

    def __init__(self, ipAddress, port=TCP_PORT, credentials = ("","")):

        self.ipAddress = ipAddress
        self.port = port
        self.credentials = credentials

        self.sessionManager = None

        # Setup API
        self.transport = TCPTransport() if port == DeviceConnection.TCP_PORT else UDPTransport()
        self.router = RouterClient(self.transport, RouterClient.basicErrorCallback)

    # Called when entering 'with' statement
    def __enter__(self):
        
        self.transport.connect(self.ipAddress, self.port)

        if (self.credentials[0] != ""):
            session_info = Session_pb2.CreateSessionInfo()
            session_info.username = self.credentials[0]
            session_info.password = self.credentials[1]
            session_info.session_inactivity_timeout = 10000   # (milliseconds)
            session_info.connection_inactivity_timeout = 2000 # (milliseconds)

            self.sessionManager = SessionManager(self.router)
            print("Logging as", self.credentials[0], "on device", self.ipAddress)
            self.sessionManager.CreateSession(session_info)

        return self.router

    # Called when exiting 'with' statement
    def __exit__(self, exc_type, exc_value, traceback):
    
        if self.sessionManager != None:

            router_options = RouterClientSendOptions()
            router_options.timeout_ms = 1000 
            
            self.sessionManager.CloseSession(router_options)

        self.transport.disconnect()

#
# Returns a string matching the requested sensor
#
def sensor_to_string(sensor):
    return all_sensor_strings.get(sensor, "Unknown sensor")

#
# Returns a string matching the requested resolution
#
def resolution_to_string(resolution):
    return all_resolution_strings.get(resolution, "Unknown resolution")

#
# Prints the intrinsic parameters on stdout
#
def print_intrinsic_parameters(intrinsics):
    # Import the utilities helper module
    # sys.path.insert(0, os.path.join(os.path.dirname(__file__), "."))
    import utilities

    print("Sensor: {0} ({1})".format(intrinsics.sensor, utilities.sensor_to_string(intrinsics.sensor)))
    print("Resolution: {0} ({1})".format(intrinsics.resolution, utilities.resolution_to_string(intrinsics.resolution)))
    print("Principal point x: {0:.6f}".format(intrinsics.principal_point_x))
    print("Principal point y: {0:.6f}".format(intrinsics.principal_point_y))
    print("Focal length x: {0:.6f}".format(intrinsics.focal_length_x))
    print("Focal length y: {0:.6f}".format(intrinsics.focal_length_y))
    print("Distortion coefficients: [{0:.6f} {1:.6f} {2:.6f} {3:.6f} {4:.6f}]".format( \
                                    intrinsics.distortion_coeffs.k1, \
                                    intrinsics.distortion_coeffs.k2, \
                                    intrinsics.distortion_coeffs.p1, \
                                    intrinsics.distortion_coeffs.p2, \
                                    intrinsics.distortion_coeffs.k3))

#
# Returns the device identifier of the Vision module, 0 if not found
#
def example_vision_get_device_id(device_manager):
    vision_device_id = 0
    
    # Getting all device routing information (from DeviceManagerClient service)
    all_devices_info = device_manager.ReadAllDevices()

    vision_handles = [ hd for hd in all_devices_info.device_handle if hd.device_type == DeviceConfig_pb2.VISION ]
    if len(vision_handles) == 0:
        print("Error: there is no vision device registered in the devices info")
    elif len(vision_handles) > 1:
        print("Error: there are more than one vision device registered in the devices info")
    else:
        handle = vision_handles[0]
        vision_device_id = handle.device_identifier
        print("Vision module found, device Id: {0}".format(vision_device_id))

    return vision_device_id