/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Gonçalo Cabrita on 28/02/2014
*
* This is ment to be an example file for the HRATC2104 Challenge.
* In this example we show how to read from the metal detector
* and index the readings to a different referential than that of
* the coils.
*
*
*********************************************************************/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <metal_detector_msgs/Coil.h>

class MetalDetector
{
public:
    MetalDetector() : pn_("~"), tf_(),  target_frame_("odom")
    {
        pn_.param<std::string>("file_name", file_name_, "file.csv");

        file_ = fopen(file_name_.c_str(), "w");
        if(file_ == NULL)
        {
            ROS_FATAL("Error creating file %s", file_name_.c_str());
            ROS_BREAK();
        }

        md_sub_.subscribe(n_, "coils", 10);
        tf_filter_ = new tf::MessageFilter<metal_detector_msgs::Coil>(md_sub_, tf_, target_frame_, 10);
        tf_filter_->registerCallback( boost::bind(&MetalDetector::msgCallback, this, _1) );
    }

    ~MetalDetector()
    {
        fclose(file_);
    }

private:
    message_filters::Subscriber<metal_detector_msgs::Coil> md_sub_;
    tf::TransformListener tf_;
    tf::MessageFilter<metal_detector_msgs::Coil> * tf_filter_;
    ros::NodeHandle n_;
    ros::NodeHandle pn_;
    std::string target_frame_;

    FILE * file_;
    std::string file_name_;

    //  Callback to register with tf::MessageFilter to be called when transforms are available
    void msgCallback(const boost::shared_ptr<const metal_detector_msgs::Coil>& coil_ptr)
    {
        geometry_msgs::PointStamped point_in;
        point_in.header.frame_id = coil_ptr->header.frame_id;
        point_in.header.stamp = coil_ptr->header.stamp;
        point_in.point.x = 0.0;
        point_in.point.y = 0.0;
        point_in.point.z = 0.0;

        geometry_msgs::PointStamped point_out;
        try
        {
            tf_.transformPoint(target_frame_, point_in, point_out);

             fprintf(file_, "%d,%d,%d,%f,%f,%f\n",
                     coil_ptr->channel[0],
                     coil_ptr->channel[1],
                     coil_ptr->channel[2],
                     point_out.point.x,
                     point_out.point.y,
                     point_out.point.z);
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("Failure %s\n", ex.what());
        }
    }
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "csv_creator");

    MetalDetector md;

    ros::spin();

    return 0;
};
