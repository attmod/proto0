# PCA analysis - orientation

https://docs.opencv.org/3.4/d3/d8d/classcv_1_1PCA.html

Example:

https://docs.opencv.org/4.x/d1/dee/tutorial_introduction_to_pca.html

https://automaticaddison.com/how-to-determine-the-orientation-of-an-object-using-opencv/

# Segmentation, background

https://docs.opencv.org/3.4/d5/de8/samples_2cpp_2segment_objects_8cpp-example.html#a4

Thresholds:

https://docs.opencv.org/3.4/db/d8e/tutorial_threshold.html

https://docs.opencv.org/4.x/d7/d4d/tutorial_py_thresholding.html

https://learnopencv.com/opencv-threshold-python-cpp/

Contours:

https://docs.opencv.org/3.4/d4/d73/tutorial_py_contours_begin.html

https://docs.opencv.org/3.4/d3/dc0/group__imgproc__shape.html#ga17ed9f5d79ae97bd4c7cf18403e1689a

# Convolution filters

https://learnopencv.com/image-filtering-using-convolution-in-opencv/

# Blobs

https://learnopencv.com/blob-detection-using-opencv-python-c/

```cpp
using namespace cv;
// Read image
Mat im = imread( "blob.jpg", IMREAD_GRAYSCALE );

// Set up the detector with default parameters.
SimpleBlobDetector detector;

// Detect blobs.
std::vector<KeyPoint> keypoints;
detector.detect( im, keypoints);

// Draw detected blobs as red circles.
// DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
Mat im_with_keypoints;
drawKeypoints( im, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

// Show blobs
imshow("keypoints", im_with_keypoints );
waitKey(0);
```

# Blur

https://docs.opencv.org/3.4/dc/dd3/tutorial_gausian_median_blur_bilateral_filter.html

```cpp
int MAX_KERNEL_LENGTH = 31;
Mat src; Mat dst;
    for ( int i = 1; i < MAX_KERNEL_LENGTH; i = i + 2 )
    {
        blur( src, dst, Size( i, i ), Point(-1,-1) );
    }
```
or
```cpp
    GaussianBlur( src, dst, Size( i, i ), 0, 0 );
    medianBlur ( src, dst, i );
    bilateralFilter ( src, dst, i, i*2, i/2 );
```


## Key points

https://docs.opencv.org/4.x/d2/d29/classcv_1_1KeyPoint.html

```
cv::KeyPoint

 	KeyPoint (Point2f pt, float size, float angle=-1, float response=0, int octave=0, int class_id=-1)
 
 	KeyPoint (float x, float y, float size, float angle=-1, float response=0, int octave=0, int class_id=-1)

Public Attributes
float 	angle
 
int 	class_id
 	object class (if the keypoints need to be clustered by an object they belong to) More...
 
int 	octave
 	octave (pyramid layer) from which the keypoint has been extracted More...
 
Point2f 	pt
 	coordinates of the keypoints More...
 
float 	response
 	the response by which the most strong keypoints have been selected. Can be used for the further sorting or subsampling More...
 
float 	size
 	diameter of the meaningful keypoint neighborhood More...
```

Python syntax

    cv.KeyPoint(x, y, size[, angle[, response[, octave[, class_id]]]])



# Edge detection

https://learnopencv.com/edge-detection-using-opencv/

