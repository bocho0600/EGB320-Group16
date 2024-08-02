#!/usr/bin/env python
from importlib import import_module
import os
from flask import Flask, render_template, Response
import sys


class WebStreamApp:

    app = Flask(__name__)
    camera_instance = None

    @staticmethod
    @app.route('/')
    def index():
        """Video streaming home page."""
        return render_template('index.html')

    @staticmethod
    def gen():
        """Video streaming generator function."""
        yield b'--frame\r\n'
        while True:
            frame = WebStreamApp.camera_instance.get_frame()
            yield b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n--frame\r\n'

    @staticmethod
    @app.route('/video_feed')
    def video_feed():
        """Video streaming route. Put this in the src attribute of an img tag."""
        return Response(WebStreamApp.gen(),
                        mimetype='multipart/x-mixed-replace; boundary=frame')

    @staticmethod
    def set_camera(Camera):
        # import camera driver
        print("App Camera set to " + str(Camera))
        WebStreamApp.camera_instance = Camera

    @staticmethod
    def send_frame(frame):
        if WebStreamApp.camera_instance is not None:
            WebStreamApp.camera_instance.send_frame(frame)

    @staticmethod
    def start():
        WebStreamApp.app.run(host='0.0.0.0', threaded=True)

if __name__ == "__main__":
    if len(sys.argv) > 1:
        WebStreamApp.set_camera(import_module('camera_' + sys.argv[1]).Camera())
    else:
        WebStreamApp.set_camera(import_module('camera').Camera())
    WebStreamApp.start()
