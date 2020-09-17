//
// Created by maxandre on 17.09.20.
//
#include <ros/ros.h>
#include <ros/console.h>
#include <delphi_esr_msgs/EsrTrack.h>
#include <vehicle_platform/EsrTrackArray.h>
#include "radar_topic_merger.h"

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "radar_topic_merger");
    ros::NodeHandle nh("~");

    radar_topic_merger radar_topic_merger(&nh);

    ros::spin();
    return 0;
}

radar_topic_merger::radar_topic_merger(ros::NodeHandle *nodehandle) : nh(*nodehandle) {
    this->nh.param<string>("esrtrack_topic", this->esrTrackTopic, "/radar_fc/parsed_tx/radartrack");
    this->esrTrackSub = this->nh.subscribe(this->esrTrackTopic, 10, &radar_topic_merger::esrtrack_callback, this);
    this->esrTrackArrayPub = nh.advertise<vehicle_platform::EsrTrackArray>("/radar_fc/merged/radartracks", 10);
}

void radar_topic_merger::esrtrack_callback(const delphi_esr_msgs::EsrTrackConstPtr &esrTrack) {
    bool skip = false;
    if (!this->esrTrackArray.EsrTracks.empty()
        && esrTrack->header.seq < this->esrTrackArray.EsrTracks.back().header.seq) { // We make sure messages are coming in the right order
        skip = true;
    }

    if ((int)esrTrack->track_status == 0 // We skip empty tracks
       || esrTrack->header.seq < this->endOfScanSeq) { /* We make sure we only keep current scan messages */
        skip = true;
    }

    if (!skip) {
        this->esrTrackArray.EsrTracks.push_back(*esrTrack);
    }
    // Nothing more to do if we haven't reached the end of the array.
    if (esrTrack->track_id < 63) return;

    this->endOfScanSeq = esrTrack->header.seq;

    // We set the timestamp of the array msg to the timestamp of the last element to allow sync.
    this->esrTrackArray.header.stamp = esrTrack->header.stamp;

    this->esrTrackArrayPub.publish(this->esrTrackArray);
    this->esrTrackArray.EsrTracks.clear();
}