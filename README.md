Arkanoid
========

A simple Arkanoid clone that uses webcam video as input.

Requires OpenCV 2.

Calibration
-----------

The game recognized motion by tracking a specific color, which can be customized at any time. After starting the application, you'll see a rectangle at the center of a window: that means the calibration is in progress. Hold the object to be tracked in such a way that it fills the whole rectangle and press SPACE. The game will remember its color and start tracking it.

Controls
--------

- 1/q, 2/w, 3/e - adjust various motion detection parameters
- a - toggle background visibility
- s - toggle debug contours visibility
- d - toggle marker position visibility
- f - toggle debug frame visibility
- c/z/c/v - toggle logs
- space - start/stop color calibration

