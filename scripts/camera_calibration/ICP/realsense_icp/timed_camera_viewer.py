from pydrake.all import *
import cv2

class TimedCameraViewer(LeafSystem):
    """
    An observer which makes visualizations of camera data at regular intervals
                        -------------------------
                        |                       |
    color_image ------> |                       |
                        |     CameraViewer      |
    depth_image ------> |                       |
                        |                       |
                        -------------------------
    """
    def __init__(self, fps):
        """
        Parameters
        ----------
        fps : float
            The number of frames per second that images are produced at
        """
        LeafSystem.__init__(self)
        self.period = 1.0 / fps;
        self.offset = 0.0;
        # Direct Drake to call the publishImages method at a regular period
        self.DeclarePeriodicPublishEvent(self.period, self.offset, self._publishImages);

        # Create example images which will be used to define the 
        # (abstract) import port types
        sample_color_image = Image[PixelType.kRgba8U](width=640,height=480)
        sample_depth_image = Image[PixelType.kDepth32F](width=640,height=480)

        # Declare input ports
        self.color_image_port = self.DeclareAbstractInputPort(
                "color_image",
                AbstractValue.Make(sample_color_image))
        self.depth_image_port = self.DeclareAbstractInputPort(
                "depth_image",
                AbstractValue.Make(sample_depth_image))

    def _publishImages(self, context):
        """
        This method is called whenever an image should be processed from the camera.
        It is called automatically with a fixed period given by the fps
        """
        color_image = self.color_image_port.Eval(context)
        depth_image = self.depth_image_port.Eval(context)

        # color_image.data and depth_image.data contain raw np arrays
        # with the image. So we can do things like load into opencv, etc
        
        # Example of displaying the depth image (then waiting for a keystroke
        # to move to the next timestep:
        #cv2.imshow("depth_image", depth_image.data)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
        
        # Return the status of processing this event
        # For simplicity, just return success
        return EventStatus.Succeeded()
