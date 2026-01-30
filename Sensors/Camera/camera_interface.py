import cv2

class Webcam:
    """
    A simple class to handle webcam operations: open, capture images, 
    show video feed, and close.
    """

    def __init__(self, index=0):
        """
        Initializes the webcam.

        Args:
            index (int): Camera index. Default is 0 which should be all we need while we only have one camera.
        """
        self.index = index
        self.camera = None

    def open(self):
        """
        Opens the webcam resources.
        """
        self.camera = cv2.VideoCapture(self.index)
        if not self.camera.isOpened():
            raise RuntimeError(f"Could not open webcam with index {self.index}")

    def take_picture(self, save_path=None, rgb=True):
        """
        Captures a single picture from the webcam.

        Args:
            save_path (str, optional): Path to save the image. If None, then the image is not saved.
            rgb (bool): If True, converts image from BGR to RGB. Added this because YOLO takes in RGB images

        Returns:
            frame (ndarray): Captured image (RGB if rgb=True, else BGR). YOLO needs RGB to work.
        """
        if self.camera is None or not self.camera.isOpened():
            raise RuntimeError("Webcam is not opened. Call open() first.")

        ret, frame = self.camera.read()
        if not ret:
            raise RuntimeError("Failed to capture image from webcam")

        if rgb:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        if save_path:
            cv2.imwrite(save_path, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) if rgb else frame)

        return frame

    def show_feed(self, rgb=True):
        """
        Opens a live video feed from the webcam. The user pressing q quits the feed.
        This is mostly just for debugging right now.

        Args:
            rgb (bool): If True, converts frames from BGR to RGB before showing.
        """
        if self.camera is None or not self.camera.isOpened():
            self.open()

        while True:
            ret, frame = self.camera.read()
            if not ret:
                print("Failed to grab frame")
                break

            display_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) if rgb else frame
            cv2.imshow("Webcam Feed", display_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.close()

    def close(self):
        """
        Closes the webcam and releases resources.
        """
        if self.camera is not None and self.camera.isOpened():
            self.camera.release()
        cv2.destroyAllWindows()

    def __del__(self):
        """
        Ensures the webcam is released when the object is deleted.
        """
        self.close()
