import picamera

class Camera:
    def __init__(self):
        # Initializing camera
        self.camera = picamera.PiCamera(resolution=(500, 500), framerate = 90)
        # Initializing streams
        self.stream = picamera.PiCameraCircularIO(self.camera, seconds=5, bitrate=10000000)
        self.stream2 = picamera.PiCameraCircularIO(self.camera, seconds=5, bitrate=10000000)
        self.camera.vflip = True
        self.camera.hflip = True

    def start(self):
        """
        Starts the camera preview.
        """
        self.camera.start_recording(self.stream, format='h264')
        self.camera.start_preview()

    def start_stream2(self):
        self.camera.start_recording(self.stream2, format='h264')

    def stop(self):
        """
        Stops the camera preview.
        """
        self.camera.stop_recording()
        self.camera.stop_preview()

    def clear_streams(self):
        self.stream.clear()
        self.stream2.clear()

    def write_video(self, mouse_name, timestamp, code):
        """
        Writes video from stream either on an external drive or
        the internal storage; timestamp is the datetime of the trial, code is the trial code.
        """
        print('WRITING VIDEO')
        date = (f"{timestamp.month}-{timestamp.day}-{timestamp.year}-"
                f"{timestamp.hour:02d}{timestamp.minute:02d}{timestamp.second:02d}")

        if self.stream.tell():
            video_file_before = (
                f'data/{mouse_name}/Videos/'
                f'{mouse_name}_{code}_{date}_BEFORE.h264'
            )
            with open(video_file_before, 'wb') as output:
                self.stream.copy_to(output)

        if self.stream2.tell():
            video_file_after = (
                f'data/{mouse_name}/Videos/'
                f'{mouse_name}_{code}_{date}_AFTER.h264'
            )
            with open(video_file_after, 'wb') as output2:
                self.stream2.copy_to(output2)

        self.clear_streams()