#!/usr/bin/env python3
import rospy
import message_filters
from sensor_msgs.msg import Image


def stereo_callback(left: Image, right: Image) -> None:
  dt = (left.header.stamp - right.header.stamp).to_sec()
  rospy.loginfo(
    f"[SYNC] Left seq={left.header.seq}, Right seq={right.header.seq}, dt={dt:.4f} s"
  )


def main():
  rospy.init_node("stereo_sync_sub")

  # ROS node parameters
  left_topic  = rospy.get_param("~left_topic",  "/left/image_raw")
  right_topic = rospy.get_param("~right_topic", "/right/image_raw")

  queue_size  = rospy.get_param("~queue_size",  10)
  slop        = rospy.get_param("~slop",        0.05) # 0.05 = 50 milliseconds
  use_approx  = rospy.get_param("~use_approx",  True)

  rospy.loginfo(
    "Starting stereo_sync_sub:\n"
    f"\t - left_topic:  {left_topic}\n"
    f"\t - right_topic: {right_topic}\n"
    f"\t - queue_size:  {queue_size}\n"
    f"\t - slop:        {slop}\n"
    f"\t - use_approx:  {use_approx}"
  )

  # create filter subscribers (special class)
  left_sub  = message_filters.Subscriber(left_topic, Image)
  right_sub = message_filters.Subscriber(right_topic, Image)

  if use_approx:
    sync = message_filters.ApproximateTimeSynchronizer(
      [left_sub, right_sub],
      queue_size=queue_size,
      slop=slop
    )
    rospy.loginfo("Using ApproximateTimeSynchronizer")
  else:
    sync = message_filters.TimeSynchronizer(
      [left_sub, right_sub],
      queue_size=queue_size
    )
    rospy.loginfo("Using TimeSynchronizer (exact)")

  sync.registerCallback(stereo_callback)
  rospy.spin()


if __name__ == "__main__":
  main()

# Lab tasks
#
# 1. Implement frames visualization (merge images into one, side-by-side visualization with stats: difference in time to microseconds + sequence).
# 2. Launch
