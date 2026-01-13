import cv2

def test_camera():
    for index in range(5):  # Test camera indices 0 to 4
        print(f"Testing camera index {index}...")
        cap = cv2.VideoCapture(index)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                print(f"Camera index {index} is working. Frame captured successfully.")
            else:
                print(f"Camera index {index} is working, but failed to capture frame.")
        else:
            print(f"Camera index {index} failed to open.")
        cap.release()

if __name__ == "__main__":
    test_camera()