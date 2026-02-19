import os
import cv2


def process_image(img):
    h, w = img.shape[:2]

    # Center crop to square
    if w > h:
        offset = (w - h) // 2
        img = img[:, offset:offset + h]
    elif h > w:
        offset = (h - w) // 2
        img = img[offset:offset + w, :]

    # Resize to 256x256
    img = cv2.resize(img, (256, 256), interpolation=cv2.INTER_AREA)
    return img


class VideoToImagesProcessor:
    def __init__(self, input_dir, output_dir, frame_skip=1):
        """
        :param input_dir: Directory containing video files
        :param output_dir: Directory where processed images will be saved
        :param frame_skip: Save every Nth frame
        """
        self.input_dir = input_dir
        self.output_dir = output_dir
        self.frame_skip = frame_skip

        os.makedirs(self.output_dir, exist_ok=True)

    def process_all_videos(self):
        video_extensions = (".mp4", ".avi", ".mov", ".mkv")
        global_saved_count = 0

        for filename in os.listdir(self.input_dir):
            if filename.lower().endswith(video_extensions):
                video_path = os.path.join(self.input_dir, filename)
                video_name = os.path.splitext(filename)[0]

                print(f"Processing video: {filename}")
                global_saved_count = self.process_video(
                    video_path,
                    video_name,
                    global_saved_count
                )

        print(f"\nDone. Total images saved: {global_saved_count}")

    def process_video(self, video_path, video_name, start_index):
        cap = cv2.VideoCapture(video_path)

        if not cap.isOpened():
            print(f"Cannot open video file: {video_path}")
            return start_index

        frame_count = 0
        saved_count = start_index

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            if frame_count % self.frame_skip == 0:
                processed = process_image(frame)

                output_path = os.path.join(
                    self.output_dir,
                    f"{video_name}_frame_{saved_count:06d}.png"
                )

                cv2.imwrite(output_path, processed)
                saved_count += 1

            frame_count += 1

        cap.release()
        print(f"Saved {saved_count - start_index} frames from {video_name}")
        return saved_count

if __name__ == "__main__":
    input_folder = "Sensors/Camera/Dataset Files/Image Proccessing/RawVideos"
    output_folder = "Sensors/Camera/Dataset Files/Image Proccessing/RawImages"

    processor = VideoToImagesProcessor(
        input_dir=input_folder,
        output_dir=output_folder
    )

    processor.process_all_videos()
