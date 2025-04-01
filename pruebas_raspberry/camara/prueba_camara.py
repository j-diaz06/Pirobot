import cv2
import picamera2
import platform
from importlib.metadata import version
import time
import numpy as np

print("Python:", platform.python_version())
print(picamera2.__name__ + ":", version(picamera2.__name__))
print(cv2.__name__ + ":", cv2.__version__)
print()
time.sleep(1)

cv2.startWindowThread()

picam2 = picamera2.Picamera2(0)

# Configuracin explita con YUV420 y tamao especfico
config = picam2.create_still_configuration(main={"format": 'YUV420', "size": (640, 480)})
picam2.configure(config)

picam2.start()

while True:
    yuv420_array = picam2.capture_array()

    # Convertir YUV420 a RGB
    rgb_array = cv2.cvtColor(yuv420_array, cv2.COLOR_YUV2BGR_I420)

    cv2.imshow("Camera", rgb_array)

    key = cv2.waitKey(1)
    if key == ord('s'):
        timeStamp = time.strftime("%Y%m%d-%H%M%S")
        targetPath = "/home/pirobot/Desktop/img" + "_" + timeStamp + ".jpg"
        cv2.imwrite(targetPath, rgb_array)
        print("- Saved:", targetPath)

    elif key == ord('q'):
        print("Quit")
        break

cv2.destroyAllWindows()
