
# **Finding Lane Lines on the Road** 


---

**Finding Lane Lines on the Road**

The goals / steps of this project are the following:
* Make a pipeline that finds lane lines on the road
* Reflect on your work in a written report

---

### Reflection

### 1. Describe your pipeline. As part of the description, explain how you modified the draw_lines() function.

My pipeline consisted of 5 steps:
1. Convert the images to grayscale
2. Apply Gaussian blur with size 3
3. Use Canny's method to detect edges with `low_threshold, high_threshold = 20, 50`
4. Apply a mask to filter region of interest with `vertices = [(0, y_len), (x_len, y_len), (0.55*x_len, 0.6*y_len), (0.45*x_len, 0.6*y_len)]`
5. Apply hought transform to detect lines with `hought_parameters = {'rho': 2, 'theta': 2*np.pi/180, 'threshold': 2, 'min_line_len': 2, 'max_line_gap': 3}`

In order to draw a single line on the left and right lanes. I modified the draw_lines() function. I also set line `thickness` to `10` and `α=.8`, `β=0.5` in function `weighted_img` to make lane lines thicker and semi-transparent.

At first my function `draw_lines_continuous` looked like this:
1. Group all lines into seperate groups according to their distance to each other, record in a dictionary `line_groups` with slope and interception as keys `(m, b)` and `(x1,y1,x2,y2,length)` as values, where `length` is the sum of lengths of all line segments in each group.
2. Sort line_groups with descending order in total length. Pick the first group with negative slope as the lane line on the left, and pick the first group with positive slope as the lane line on the right.
3. Draw lane lines according to the chosen slope and interception, and the y limit `330` and `540`.

This worked great for videos `solidWhiteRight.mp4` and `solidYellowLeft.mp4`, but did very bad on the chanllenge video. So I modified the draw_lines function as in `draw_lines_challenge` in the notebook.
1. Fixed two points at the bottom that two lane lines have to pass thorough: `left_bot_x = 0.17 * x_len`, `right_bot_x = 0.9 * x_len`.
2. Regard a line as `left` if -0.9 < m < -0.5 and its horizontal interception at bottom is close enough to `left_bot_x`; Regard a line as `right` if 0.5 < m < 0.9 and its horizontal interception at bottom is close enough to `right_bot_x`.
3.  Compute the weighted average of slope and interception in `left` and `right` according lenghs of line segments:
    ```{python}
    ml = np.average([x[0] for x in left], weights = left_lens)
    bl = np.average([x[1] for x in left], weights = left_lens)
    mr = np.average([x[0] for x in right], weights = right_lens)
    br = np.average([x[1] for x in right], weights = right_lens)
    ```
4. Draw lane lines using `ml`, `bl`, `mr`, `br`, `left_bot_x` and `right_bot_x`.
5. Also, in the challenge test video, image size is changing over time, sometimes it's (720, 1280) and sometimes (540, 960). So I had to change a lot of parameters from fixed values to proportional values according to `x_len` and `y_len`.

This version of the draw_lines function worked well on all three test videos.

### 2. Identify potential shortcomings with your current pipeline

The most obvious problem is that the lane lines are "shaking" in the video. This may because the lines identified include too much noise and after averging all slopes and interceptions, the result would shift from the correct direction. This is especially the case in the challenge video, where there is a car on the right and a lot of line segments on the left coming from trees.

One potential shortcoming would be what would happen when there are more closed vehicles or other objects in front of the camera, it would produce much more line segments which shouldn't be included as lane lines.

### 3. Suggest possible improvements to your pipeline

A possible improvement would be to modify the mask carefully so that it doesn't take into account objects that are too far away or outside the lane lines.

Another potential improvement I guess could be to take into account lane lines at previous time in a video, a similar idea as in "momentum" for stochastic gradient descent. Averaging over time would make lane lines more stable.
