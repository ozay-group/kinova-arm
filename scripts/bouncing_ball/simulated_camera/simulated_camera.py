from pydrake.all import *
from kinova_drake.kinova_station.common import (CameraPosePublisher)


def addSimulatedCamera(name, X_camera, scene_graph, builder, show_window=False):
    """
    Add a simulated camera to the simulated KinovaStation while it is being built.
    This must be called after its scene_graph is created but before the station is finalized.
    
    The rgb image is accessible with the port name ``name + "_rgb_image"``.
    The depth image is accessible with the port name ``name + "_depth_image"``.
    
    Parameters
    ----------
    name : str
        The name of the camera to created
    X_camera : RigidTransform
        The fixed camera orientation in world coordinates
    scene_graph : SceneGraph
        The scene graph of the KinovaStation
    builder : DiagramBuilder
        The builder of the KinovaStation
    show_window : bool
        If True, then display the image rendered by the camera to the screen.
        
    Returns
    -------
    RgbdSensor
        The created camera object that was added to the KinovaStation
    """
    renderer_name = name + "_renderer"
    scene_graph.AddRenderer(renderer_name, 
                            MakeRenderEngineVtk(RenderEngineVtkParams()))
    # Set camera properites. These roughly correspond
    # to the camera on the hardware (Intel Realsense D410).
    intrinsics = CameraInfo(width=480, height=270, fov_y=np.radians(40))
    #intrinsics = CameraInfo(width=160, height=90, fov_y=np.radians(40))
    clipping = ClippingRange(0.01,3.0)
    X_lens = RigidTransform()
    camera_core = RenderCameraCore(renderer_name, intrinsics, clipping, X_lens)
    depth_range = DepthRange(0.1, 2.0)

    # Create the camera model
    color_camera = ColorRenderCamera(camera_core, show_window=show_window)
    depth_camera = DepthRenderCamera(camera_core, depth_range)
    
    camera = builder.AddSystem(RgbdSensor(scene_graph.world_frame_id(),
                                          X_camera,
                                          color_camera,
                                          depth_camera))
    camera.set_name(name)
    
    # Output camera images
    builder.ExportOutput(camera.color_image_output_port(),
                         name + "_rgb_image")
    builder.ExportOutput(camera.depth_image_32F_output_port(),
                         name + "_depth_image")
    
    # Connect input from scene_graph         
    builder.Connect(
            scene_graph.get_query_output_port(),
            camera.query_object_input_port())
            
    return camera
