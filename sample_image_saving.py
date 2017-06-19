import time
import numpy as np
import cv2
import pyrealsense as pyrs
import atexit

from scipy import ndimage
from scipy.ndimage import morphology as scmorph

# set up logging level
logging.basicConfig(level=logging.INFO)


def main():
    # Start realsense (if available)
    
    pyrs.start()
    
    # Set up device for colour and depth streaming.
    # How might you add a raw IR stream?
    py_dev = pyrs.Device(device_id = 0, streams = [pyrs.ColourStream(fps = 30), pyrs.DepthStream(fps=30)])

    # Run forever! (until quitting)
    while True:
        
        py_dev.wait_for_frame()
        c_im = py_dev.colour
        
        # Convert colour frame into a single RGB image
        rgb_im = c_im[..., ::-1]
        
        # scale depth frames to avoid colour wrapping. Is this the best scaling factor for our purposes? Try changing it
        d_im = py_dev.depth * 0.05

        # Convert the 3D depth frame to a 2D colour map:
        d_im_col = cv2.applyColorMap(d_im.astype(np.uint8), cv2.COLORMAP_HOT)
        # what other colourmaps exist? are some more intuitive than others? Any that would be a problem for colourblind people?

        # combine the two images into one array, for display
        cd = np.concatenate((rgb_im, d_im_col), axis=1)

        # show two images in one frame (default name)
        # To-do: what if we want to display multiple frames? look at the OpenCV imshow documentation for tips
        cv2.imshow('', cd)

        # To-do: how about displaying a point-cloud using VTK, in a different window?

        input = cv2.waitKey(1):
        # quit if we hit 'q'
        if (input == ord('q')):
            break
        elif (input == ord('a')):
            # save depth and colour images if we hit 'a'.
            # Note that there are many, many ways to save images in Python.
            # OpenCV has one method in its library, but for faster image saving we could look into something like PIL.
            cv2.imwrite('depth_image.png',d_im_col);
            cv2.imwrite('colour_image.png', rgb_im);
            # to-do: how do we save images to a specific folder? How can we let the user choose this folder on start up?
            # Can we add an incrementing number to the filename of colour/depth?
            # What about having the user type a new filename every time we save?
        elif (input == -1):
            continue



if __name__ == '__main__':
    main()