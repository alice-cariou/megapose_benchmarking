import numpy as np
from PIL import Image as im
import ast
import cv2
from cv_bridge import CvBridge

def main():
    with open('compres_image') as f:
        test = f.read()
    array = test[test.find("["):test.find("]")+1]
    array = ast.literal_eval(array)
    print(array)
    PILimage = Image.fromarray(... bgr8data[:,:,::-1]... )
  #  array = np.arange(0, 307200, 1, np.uint8)
  #  print(type(array))
  #  array = np.reshape(array, (480, 640))
  #  print(array.shape)
  #  print(array)
  #  data = im.fromarray(array)
  #  data.save('gfg_dummy_pic.png')"

if __name__ == "__main__":
    main()