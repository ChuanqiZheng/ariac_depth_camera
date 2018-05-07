//analyze_pcd_file.cpp
// prompts for a pcd file name, reads the file, and displays to rviz on topic "pcd"
// tries to find "interesting" points above the support surface
//hard-code transform between bin and camera
//wsn April 2018

#include<ros/ros.h> //generic C++ stuff
#include <stdlib.h>
#include <math.h>
#include <sensor_msgs/PointCloud2.h> //useful ROS message types
#include <pcl_ros/point_cloud.h> //to convert between PCL a nd ROS

//#include <pcl/ros/conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/PCLHeader.h>
#include <pcl_utils/pcl_utils.h>

//for opencv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <sstream> //need this to change numbers in names


using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcd_interpreter"); //node name
    ros::NodeHandle nh;

    //hard-coded identification of bin plane from depth camera:
    //these were found from a plane fit of a camera viewing an empty bin;
    // expect same transform for all  5 bins
    Eigen::Vector3f plane_normal;
    double nom_tilt_ang = atan2(-0.285, -0.958);
    cout<<"nom_tilt_ang: "<<nom_tilt_ang<<endl;
    
    plane_normal << cos(nom_tilt_ang), sin(nom_tilt_ang), 0; //-0.958, -0.285, 0;
    double plane_dist = -0.4675;

    //library that provides some helper functions:
    PclUtils pclUtils(&nh);
    //create a coordinate frame on the identified plane using the plane's parameters
    Eigen::Affine3f affine_plane_frame;
    affine_plane_frame = pclUtils.make_affine_from_plane_params(plane_normal, plane_dist);


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //output cloud
    cout << "enter pcd file name: ";
    string fname;
    cin>>fname;

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (fname, *pcl_clr_ptr) == -1) //* load the file
    {
        ROS_ERROR("Couldn't read file \n");
        return (-1);
    }
    std::cout << "Loaded "
            << pcl_clr_ptr->width * pcl_clr_ptr->height
            << " data points from file " << fname << std::endl;

    //    pcl::transformPointCloud(*pclKinect_clr_ptr, *transformed_cloud_ptr, A_plane_wrt_camera.inverse());
    pcl::transformPointCloud(*pcl_clr_ptr, *output_cloud_ptr, affine_plane_frame.inverse());

    //pclUtils.transform_cloud(affine_plane_frame, pcl_clr_ptr,output_cloud_ptr);
    int npts_cloud = output_cloud_ptr->width * output_cloud_ptr->height;
    int width = output_cloud_ptr->width;
    int height = output_cloud_ptr->height;
    ROS_INFO("output cloud has %d points, %d x %d", npts_cloud, width, height);

    //analyze the transformed cloud:
    //evaluate how many 3-D points lie within 1mm slabs parallel to the bin surface
    double dz = 0.001;
    double z, zmax;
    int npts;
    double z_sum = 0.0;
    Eigen::Vector3f cloud_pt;
    int ntop = 0;
    //sort the z-values of points in the range from -2cm to +3cm w/rt bin surface
    for (double zmin = -0.0205; zmin < 0.03; zmin += dz) {
        npts = 0;
        zmax = zmin + dz;
        for (int ipt = 0; ipt < npts_cloud; ipt++) {
            //eval z-coord of every point
            cloud_pt = output_cloud_ptr->points[ipt].getVector3fMap();//get information from each pixel of pointcloud!!!!!
            z = cloud_pt[2];
            //does it lie between the current upper and lower bounds?
            // if so, increment the count of points in this range
            if ((z > zmin)&&(z < zmax)) npts++;
        }
        ROS_INFO("at zmin = %f, zmax = %f: npts = %d", zmin, zmax, npts);

    }

    //search through the points again: how many points have z-values that are
    // above the bin surface?  What is the average of these z values?
    for (int ipt = 0; ipt < npts_cloud; ipt++) {
        cloud_pt = output_cloud_ptr->points[ipt].getVector3fMap();
        z = cloud_pt[2];
        if (z > 0.0) {    //if (z > 0.0) {
            ntop++;
            z_sum += z;
        }
    }
    double z_top = z_sum / ntop; //compute average z value for z values>0
    ROS_INFO("mean z for values>0 = %f", z_top);

    //convert the 3-D image to a 2-D image, where illumination is proportional to z-value=
    // height above plane;
    // Alternatively, create a binary 2-d image where z values >0 map to 1, all others to 0
    
    //choose the resolution for projected image;
    //source is 50x50, BUT resolution is higher in y-direction than x-direction;
    //need to scale transform to pixels correspondingly so circular objects are
    //circular in the projection
    int Nv = 50; //number of pixels of synthesized image in vertical direction
    int Nu = 35; //number of pixels in horizontal direction
    double min_y = -0.229132; //these values were observed from camera outputs
    double max_y = 0.233651;
    double min_x = -0.005231;
    double max_x = 0.315519;
    double zval_black = 0.0; // in ouput image, assign intensity=0 at this z-value (or less)
    double zval_white = 0.01; //in output image, assign intensity=255 at this z-value (or above)      
    int u, v;
    //point-cloud data is in units of meters, but image will be indexed by pixels;
    // scale factor converts meters to pixels; will need to go back in other direction as well
    double meters_to_pixels = 110; //100 //convert pointcloud data to pixel numbers
    cv::Mat depthmap_image(Nu, Nv, CV_8U, cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
    uchar gray_level; //uchar 0 - 255
    double x, y;
    //for each pointcloud point, compute which pixel it maps to and find its z value
    for (int ipt = 0; ipt < npts_cloud; ipt++) {
        cloud_pt = output_cloud_ptr->points[ipt].getVector3fMap();
        z = cloud_pt[2];
        y = cloud_pt[1];
        x = cloud_pt[0];
        //careful: some pointclouds include bad data; test for this
        if ((z == z)&&(x == x)&&(y == y)) { // test for Nan 
            //compute pixel values from metric values
            v = round((y - min_y) * meters_to_pixels);
            u = round((x - min_x) * meters_to_pixels);
            //flip/invert these so image makes sense visually
            u = Nu - u;
            v = Nv - v;
            // convert z to an intensity:
            if (z > zval_white) gray_level = 255; //max intensity
            else if (z < zval_black) gray_level = 0; //min intensity
            else {
                gray_level = (uchar) (255 * (z - zval_black) / (zval_white - zval_black));
            }
            if ((u > 0)&&(u < Nu - 1)&&(v > 0)&&(v < Nv - 1)) {
                depthmap_image.at<uchar>(u, v) = gray_level;
            }
        }

    }
    //ROS_INFO("min_x,max_x, min_y,max_y = %f, %f, %f, %f",min_x,max_x,min_y,max_y);
    std::cout << "output image size: " << depthmap_image.size().height << " , "
            << depthmap_image.size().width << std::endl; //initially, 0x0
    //save the synthesized  image to  disk
    cv::imwrite("output.bmp", depthmap_image);
    
    //interpret image w/rt template:
    //see OpenCV: void matchTemplate(InputArray image, InputArray templ, OutputArray result, int method)
	//cv::Mat src_img, template_img;
	//cv::Mat result_mat;




/*
    int Ntu = 23; //define template 10x10 (arbitrary)
    int Ntv = 23; 
    int tu_c = 11; //(7,7) is central pixel of template 15x15 template
    int tv_c = 11;
    int template_u_halfwidth = 11; //template is +/-5 pixels about center
    int template_v_halfwidth = 11;
    
    //here is the pattern of interest: a circle of diameter Ntu = Ntv pixels
    //this template is for the "disk" part
    cv::Mat template_img(Ntu, Ntv, CV_8U, cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
    int Rpix = 7; //make a template of a circle of radius 7 pixels
    double r,theta;
    for (r=0;r<=Rpix;r+=0.2) {
        for (theta=0;theta<6.28;theta+=0.01) {
            u= round(r*sin(theta))+tu_c;
            v= round(r*cos(theta))+tv_c;
            cout<<"tu= "<<u<<", tv = "<<v<<endl;
            template_img.at<uchar>(u, v) = 255;
        }
    }*/

int Ntu, Ntv, tu_c, tv_c, template_u_halfwidth, template_v_halfwidth, Rpix;
double r,theta;
double bin_x, bin_y, bin_z, bin_roll, bin_pitch, bin_yall;
double z_difference = 0.012;//amout of z value difference from bin to part placed in middle of box.
//Basicly a fixed number combined by thickness of box bottom and box tilt angle. Checked from Gazebo

//Create template depending on part type
if((z_top<0.012)&&(z_top>0.006))
{
    cout<<"Gear part detected!"<<endl;
    Ntu = 13; //define template 10x10 (arbitrary)
    Ntv = 13; 
    tu_c = 6; //(7,7) is central pixel of template 15x15 template
    tv_c = 6;
    template_u_halfwidth = 6; //template is +/-5 pixels about center
    template_v_halfwidth = 6;
    Rpix = 5;
    //hard code bin pose here, more scientific way is to depend on which depth camera being used
    //further adjustment need to be done, with specific names of pcd file taken from specific depth cameras
    bin_x = -0.775;
    bin_y = -0.49;
    bin_z = 0.75;
    bin_roll = 0.0;
    bin_pitch = -0.19;
    bin_yall = 3.141591;
}
else if((z_top<0.023)&&(z_top>0.017))
{
    cout<<"Disk part detected!"<<endl;
    Ntu = 19;
    Ntv = 19; 
    tu_c = 9;
    tv_c = 9;
    template_u_halfwidth = 9;
    template_v_halfwidth = 9;
    Rpix = 7.0;

    bin_x = -0.775;
    bin_y = 0.32;
    bin_z = 0.75;
    bin_roll = 0.0;
    bin_pitch = -0.19;
    bin_yall = 3.141591;
}
else if((z_top<0.075)&&(z_top>0.069))
{
    cout<<"Pulley part detected!"<<endl;
    Ntu = 31;
    Ntv = 31; 
    tu_c = 15;
    tv_c = 15;
    template_u_halfwidth = 15;
    template_v_halfwidth = 15;
    Rpix = 12.5;

    bin_x = -0.775;
    bin_y = 1.94;
    bin_z = 0.75;
    bin_roll = 0.0;
    bin_pitch = -0.19;
    bin_yall = 3.141591;
}
else
{
    cout<<"No bin/ Empty bin/ Not gear or disk or pulley"<<endl;
    return 0;
}
cv::Mat template_img(Ntu, Ntv, CV_8U, cv::Scalar(0));
for (r=0;r<=Rpix;r+=0.2) {
        for (theta=0;theta<6.28;theta+=0.01) {
            u= round(r*sin(theta))+tu_c;
            v= round(r*cos(theta))+tv_c;
            //cout<<"tu= "<<u<<", tv = "<<v<<endl;
            template_img.at<uchar>(u, v) = 255;
        }
    }
    cout<<"created template"<<endl;
    cv::imwrite("template_img.bmp", template_img);



    //create a larger image with room for padding around the edges
    //cv::Mat padded_img(Nu+2*Ntu-1, Nv+2*Ntv-2, CV_8U, cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
    cv::Mat padded_img(Nu+2*Ntu, Nv+2*Ntv, CV_8U, cv::Scalar(0));
    //make a checkerboard background; want to pad the image w/ unbiased background
    //start by making this entire image a checkerboard
    /* try w/o checkerboard
    for (u=0;u<Nu+2*Ntu-1;u+=2) {
        for (v=0;v<Nv+2*Ntv-1;v+=2) {
            padded_img.at<uchar>(u, v) = 255;
        }
    }
    */
    //now copy over the image to be embedded (centered) in checkerboard; edges will still be unbiased
    for (u=0;u<Nu;u++) {
        for (v=0;v<Nv;v++) {
            //leaves edges of halfwidth of template with unbiased (checkerboard) padding
            //padded_img.at<uchar>(u+Ntu-1, v+Ntv-1) = embedded_img.at<uchar>(u, v);
            //padded_img.at<uchar>(u+Ntu-1, v+Ntv-1) = depthmap_image.at<uchar>(u, v); //use real image   
            padded_img.at<uchar>(u+Ntu, v+Ntv) = depthmap_image.at<uchar>(u, v); 
        }
    }    
    cout<<"padded image with checkerboard"<<endl;
    cv::imwrite("padded_img_1.bmp", padded_img);
    
    //now compute distance between template and snippets of image, and put results into another image
    cv::Mat result_mat[20];  //put result of template matching in this output matrix; 
    /*		// method: CV_TM_SQDIFF, CV_TM_SQDIFF_NORMED, CV_TM _CCORR, CV_TM_CCORR_NORMED, CV_TM_CCOEFF, CV_TM_CCOEFF_NORMED
		int match_method = CV_TM_CCORR_NORMED;
		cv::matchTemplate(src_img, template_img, result_mat, match_method);
		cv::normalize(result_mat, result_mat, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());*/
    //int match_method = CV_TM_SQDIFF_NORMED; //CV_TM_SQDIFF
    int match_method = CV_TM_SQDIFF;
    double minVal, maxVal; 
    cv::Point minLoc, maxLoc, matchLoc;
    int npart, x_fit[20], y_fit[20];
    for(int ipart = 0;ipart<20;ipart++) //set a maximum number of parts in one single bin
    {
      cv::matchTemplate(padded_img, template_img, result_mat[ipart], match_method);
      cv::normalize(result_mat[ipart], result_mat[ipart], 0, 255, cv::NORM_MINMAX, -1, cv::Mat());
      //cv::imwrite("result_mat.bmp", result_mat[ipart]);
      std::ostringstream result_mat_name;
      result_mat_name << "result_mat_" << ipart+1 << ".png";
      cv::imwrite(result_mat_name.str(), result_mat[ipart]);
      ros::Duration(0.01).sleep();
      cv::minMaxLoc(result_mat[ipart], &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );
      if ((minVal == 0)||(abs(minVal)>0.0001))
      {
        npart = ipart;
        cout<<ipart<<" parts in total."<<endl;
        break;
      }
      cout<<"maxVal match: "<<maxVal<<endl;
      cout<<"minVal match: "<<minVal<<endl;
      cout<<"Part number "<<ipart+1<<", best match at: "<<minLoc<<endl;
      x_fit[ipart] = minLoc.y;
      y_fit[ipart] = minLoc.x;
      cout<<"x_fit = "<<x_fit[ipart]<<"; y_fit = "<<y_fit[ipart]<<endl;
      //erase this match and try again:
      for (int i=x_fit[ipart]-Rpix+tu_c-1;i<=x_fit[ipart]+Rpix+tu_c+1;i++) {
          for (int j=y_fit[ipart]-Rpix+tv_c-1;j<=y_fit[ipart]+Rpix+tv_c+1;j++) {
              padded_img.at<uchar>(i,j) = 0;
          }
      }
      std::ostringstream padded_img_name;
      padded_img_name << "padded_img_" << ipart+2 << ".png";
      cv::imwrite(padded_img_name.str(), padded_img);
      ros::Duration(0.01).sleep();
      //cv::imwrite("padded_img2.bmp", padded_img);
    }



    //computing world frame of detected parts.
    double pitch_bin = 0.19; //check from gazebo
    double sin_bin_tilt = sin(pitch_bin); //0.1888
    double cos_bin_tilt = cos(pitch_bin); //0.982
    double middle_x = (double)Ntv + 0.5*(Nv+1); //middle pixel coordinate on padded img 
    double middle_y = (double)Ntu + 0.5*(Nu+1);
    for(int ipart = 0; ipart<npart; ipart++)
    {
      //y_fit = minLoc.x so that erasing formula is right, now converge it back, same for x_fit
      double x_wrt_bin_pixel = (double)y_fit[ipart] - middle_x; 
      double y_wrt_bin_pixel = (double)x_fit[ipart] - middle_y;
      double x_wrt_bin_metric = (double)x_wrt_bin_pixel/meters_to_pixels; 
      double y_wrt_bin_metric = (double)y_wrt_bin_pixel/meters_to_pixels;

      //now from bin frame to world frame
      double x_wrt_wrd = bin_x + y_wrt_bin_metric*cos_bin_tilt;
      double y_wrt_wrd = bin_y + x_wrt_bin_metric;
      double z_wrt_wrd = bin_z + z_difference - y_wrt_bin_metric*sin_bin_tilt;
      //keep roll, pitch, yall same with bin frame
      double part_roll = bin_roll;
      double part_pitch = bin_pitch;
      double part_yall = bin_yall;
      cout<<"part number "<<ipart+1<<", in world frame: "<<endl;
      cout<<"x: "<<x_wrt_wrd<<"  y: "<<y_wrt_wrd<<"  z: "<<z_wrt_wrd<<endl;
      cout<<"roll: "<<part_roll<<"  pitch: "<<part_pitch<<"  yall: "<<part_yall<<endl;
    }









    //cv::namedWindow( "Display_window" );// Create a window for display.
    //cv::imshow( "Display window", src_img );                   // Show our image inside it.

    //cv::waitKey(0);      
    //publish the point cloud in a ROS-compatible message; here's a publisher:
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/pcd", 1);
    sensor_msgs::PointCloud2 ros_cloud; //here is the ROS-compatible message
    pcl::toROSMsg(*pcl_clr_ptr, ros_cloud); //convert from PCL to ROS type this way
    ros_cloud.header.frame_id = "depth_camera_1_frame";

    //publish the transformed cloud as well:
    ros::Publisher pubCloud2 = nh.advertise<sensor_msgs::PointCloud2> ("/transformed_cloud", 1);
    sensor_msgs::PointCloud2 ros_cloud2; //here is the ROS-compatible message
    pcl::toROSMsg(*output_cloud_ptr, ros_cloud2); //convert from PCL to ROS type this way
    ros_cloud2.header.frame_id = "bin_frame";

    cout << "view in rviz; choose: topic= pcd; and fixed frame= depth_camera_1_frame" << endl;
    cout << "view transformed image with: topic= /transformed_cloud; and fixed frame= bin_frame" << endl;

    //publish the ROS-type message on topic "/ellipse"; can view this in rviz
    while (ros::ok()) {

        pubCloud.publish(ros_cloud);
        pubCloud2.publish(ros_cloud2);

        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    return 0;
}

