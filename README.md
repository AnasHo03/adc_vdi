# adc

### Lane Detection
* Inspired by:
<https://automaticaddison.com/the-ultimate-guide-to-real-time-lane-detection-using-opencv/>
* Algorithim
  * Thresholding
  * Bird's eye view
  * Histogram: locate areas of the image that have high concentrations of white pixels
  * Sliding window: returns a polynomial fitting for right and left lines
  * Fill in the lane lines: fill gaps between lines
  * Overlay lane lines on original image
  * Calculate curvatre: return radius of imaginary circle completing left and right lines
  * Calculate car position: return offset from center of two lines


#### lane.py Parameter Tweaking
* Sobel Thresholding: adjust the original value 80

`_, s_binary = edge.threshold(s_channel, (80, 255))`

* ROI: adjust from constructor

`self.roi_points = np.float32`

#### TODO
* Tweak position
* Synthesize a second line in case original is not in frame
