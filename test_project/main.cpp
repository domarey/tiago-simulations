#include <iostream>
#include <librealsense2/rs.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>


using namespace std;
using namespace cv;

// from rs2::frame to cv::Mat
static cv::Mat frame_to_mat(const rs2::frame& frame){
    const int w = frame.as<rs2::video_frame>().get_width();
    const int h = frame.as<rs2::video_frame>().get_height();

    if (frame.get_profile().format() == RS2_FORMAT_BGR8){
        return Mat(Size(w, h), CV_8UC3, (void*)frame.get_data(), Mat::AUTO_STEP);
    }
    else if (frame.get_profile().format() == RS2_FORMAT_RGB8){
        auto r_rgb = Mat(Size(w, h), CV_8UC3, (void*)frame.get_data(), Mat::AUTO_STEP);
        Mat r_bgr;
        cvtColor(r_rgb, r_bgr, COLOR_RGB2BGR);
        return r_bgr;
    }
    else if (frame.get_profile().format() == RS2_FORMAT_Z16){
        return Mat(Size(w, h), CV_16UC1, (void*)frame.get_data(), Mat::AUTO_STEP);
    }
    else if (frame.get_profile().format() == RS2_FORMAT_Y8){
        return Mat(Size(w, h), CV_8UC1, (void*)frame.get_data(), Mat::AUTO_STEP);
    }
    else if (frame.get_profile().format() == RS2_FORMAT_DISPARITY32){
        return Mat(Size(w, h), CV_32FC1, (void*)frame.get_data(), Mat::AUTO_STEP);
    }

    throw std::runtime_error("Frame format is not supported yet!");
}

/*
cv::Mat create_mask_from_depth(Mat& depth, int thresh, ThresholdTypes type){
        threshold(depth, depth, thresh, 255, type);
        dilate(depth, depth, erode_less);
        erode(depth, depth, erode_more);
}
*/

// Converts depth frame to a matrix of doubles with distances in meters
static cv::Mat depth_frame_to_meters( const rs2::depth_frame & f ){
    cv::Mat dm = frame_to_mat(f);
    dm.convertTo( dm, CV_64F );
    dm = dm * f.get_units();
    return dm;
}


void stream_from_camera(rs2::pipeline pipeline){
    rs2::colorizer color_map;
    rs2::disparity_transform depth_to_disparity(true);

    const auto win_name = "Camera stream";
    namedWindow( win_name, WINDOW_AUTOSIZE );

    while (waitKey(1) < 0 && getWindowProperty(win_name, WND_PROP_AUTOSIZE) >= 0){
        rs2::frameset frames = pipeline.wait_for_frames();

        // Make the frameset spatially aligned
        rs2::align align_to(RS2_STREAM_COLOR);
        rs2::frameset aligned_set = align_to.process(frames);
        rs2::frame depth = aligned_set.get_depth_frame();
        Mat color_mat = frame_to_mat(aligned_set.get_color_frame());

        // color_map: colorize such that white is near and black is far
        // This takes advantage of histogram equalization done by the colorizer
        // color_map.set_option(RS2_OPTION_COLOR_SCHEME, 2);
        depth = depth_to_disparity.process(depth);
        depth = depth.apply_filter(color_map);
        Mat depth_mat = frame_to_mat(depth);

        const int w = depth.as<rs2::video_frame>().get_width();
        const int h = depth.as<rs2::video_frame>().get_height();
        Mat resized_color_mat;
        resize(color_mat, resized_color_mat, Size(w, h), INTER_LINEAR);

        //Mat to_concatenate[] = {depth_mat, resized_color_mat};
        Mat concatenated;
        hconcat(depth_mat, resized_color_mat, concatenated);
        imshow(win_name, concatenated);
    }
}



/*
void background_processing(){
    // Generate "near" mask image:
    auto near = frame_to_mat(depth);
    cvtColor(near, near, COLOR_BGR2GRAY);
    // Take just values within range [180-255]
    // These will roughly correspond to near objects due to histogram equalization
    create_mask_from_depth(near, 180, THRESH_BINARY);

    // Generate "far" mask image:
    auto far = frame_to_mat(bw_depth);
    cvtColor(far, far, COLOR_BGR2GRAY);
    far.setTo(255, far == 0); // Note: 0 value does not indicate pixel near the camera, and requires special attention
    create_mask_from_depth(far, 100, THRESH_BINARY_INV);

    // GrabCut algorithm needs a mask with every pixel marked as either:
    // BGD, FGB, PR_BGD, PR_FGB
    Mat mask;
    mask.create(near.size(), CV_8UC1);
    mask.setTo(Scalar::all(GC_BGD)); // Set "background" as default guess
    mask.setTo(GC_PR_BGD, far == 0); // Relax this to "probably background" for pixels outside "far" region
    mask.setTo(GC_FGD, near == 255); // Set pixels within the "near" region to "foreground"

    // Run Grab-Cut algorithm:
    Mat bgModel, fgModel;
    grabCut(color_mat, mask, Rect(), bgModel, fgModel, 1, GC_INIT_WITH_MASK);

    // Extract foreground pixels based on refined mask from the algorithm
    Mat3b foreground = Mat3b::zeros(color_mat.rows, color_mat.cols);
    color_mat.copyTo(foreground, (mask == GC_FGD) | (mask == GC_PR_FGD));
    imshow(window_name, foreground);
}
*/

void stream_depth(rs2::pipeline pipeline){
    rs2::colorizer color_map;
    rs2::disparity_transform depth_to_disparity(true);

    const auto win_name = "Disparity stream";
    namedWindow( win_name, WINDOW_AUTOSIZE );

    while (waitKey(1) < 0 && getWindowProperty(win_name, WND_PROP_AUTOSIZE) >= 0){
        // Block program until frames arrive
        rs2::frameset frames = pipeline.wait_for_frames();

        // get the frames
        rs2::frame depth = frames.get_depth_frame();
        depth = depth_to_disparity.process(depth);
        depth = depth.apply_filter(color_map);

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        Mat image = frame_to_mat(depth);

        // Show the image inside it.
        imshow(win_name, image);
    }
}

void stream_rgb(rs2::pipeline pipeline){

    const auto win_name = "Regular stream";
    namedWindow( win_name, WINDOW_AUTOSIZE );


    while (waitKey(1) < 0 && getWindowProperty(win_name, WND_PROP_AUTOSIZE) >= 0){
        // Block program until frames arrive
        rs2::frameset frames = pipeline.wait_for_frames();

        // Try to get a frame of a depth image
        rs2::frame color_frame = frames.get_color_frame();

        Mat image = frame_to_mat(color_frame);

        imshow(win_name, image);
    }
}



int main(int argc, char* argv[])
{   // encapsulates the device and sensors
    rs2::pipeline p;

    // start streaming with the defaul ocnfiguration
    p.start();

    stream_from_camera(p);
    stream_rgb(p);
    stream_depth(p);

    return 0;
}
