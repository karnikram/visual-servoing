<h1 align="center">Visual Servoing of Mobile Robot</h1>

This repo contains code for controlling a differential drive mobile robot within a map using images from an overhead camera. Uses thresholding and template matching for localization and a PID controller for generating the steering commands. The mobile robot uses an Arduino as its controller and LibSerial is used for communicating with it.

Also included are the PCB files for the mobile robot controller board.


<div align="center">
<img src="https://github.com/karnikram/visual-servoing/blob/master/image.png" width="60%"/>
</div>

<a href="https://www.youtube.com/watch?v=kU-pIHazJII">Video</a>

### Libraries Used
* [OpenCV](https://github.com/opencv/opencv)
* [LibSerial](http://libserial.sourceforge.net/x27.html)

Based on [LOCK](https://github.com/QuinAsura/LOCK).
