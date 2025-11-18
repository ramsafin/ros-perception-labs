#!/usr/bin/env python3

import cv2
import argparse
import numpy as np
from typing import Tuple, NamedTuple

import rospy

from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge


class FontOptions(NamedTuple):
  font_face: int = cv2.FONT_HERSHEY_SIMPLEX
  font_scale: float = 3.0
  thickness: int = 5


def generate_image(
  text: str, shape: Tuple[int, int] = (512, 512),
  font_opts: FontOptions = FontOptions()
) -> np.ndarray:
  assert text is not None, "text must be provided"

  image = np.zeros(shape, dtype=np.uint8)
  font_face, font_scale, thickness = font_opts

  (text_width, text_height), _ = cv2.getTextSize(text, font_face, font_scale, thickness)

  cv2.putText(
    image, text, ((shape[1] - text_width) // 2, (shape[0] + text_height) // 2),
    font_face, font_scale, 255, thickness, cv2.LINE_AA
  )

  return image


def image_msg_from(seq: int, stamp: rospy.Time, frame_id: str, data: np.ndarray, bridge: CvBridge) -> Image:
  header = Header()
  header.seq = seq
  header.stamp = stamp
  header.frame_id = frame_id
  return bridge.cv2_to_imgmsg(data, encoding="mono8", header=header)


def parse_args():
  parser = argparse.ArgumentParser(
    description="Fake camera publisher."
  )

  parser.add_argument(
    "-f", "--frame", type=str, default="camera",
    help="TF frame_id for the camera (default: %(default)s)"
  )

  parser.add_argument(
    "-r", "--rate", type=float, default=1.0,
    help="Average publish rate in Hz (default: %(default)s)"
  )

  parser.add_argument("-x", "--width", type=int, default=512, help="(default: %(default)s)")
  parser.add_argument("-y", "--height", type=int, default=512, help="(default: %(default)s)")

  return parser.parse_known_args()[0]


def main():
  args = parse_args()
  rospy.init_node(f"{args.frame}_pub", anonymous=True)

  if args.rate <= 0:
    rospy.logwarn("rate <= 0, defaulting to 1 Hz")
    args.rate = 1.0

  rate = rospy.Rate(args.rate)
  topic = f"{args.frame}/image_raw"

  rospy.loginfo(
    "Starting fake camera publisher:\n"
    f"\t- topic: '{rospy.resolve_name(topic)}'\n"
    f"\t- frame_id: '{args.frame}'\n"
    f"\t- rate: {args.rate} Hz\n"
    f"\t- size: {args.width}x{args.height} pixels"
  )

  seq = 0
  bridge = CvBridge()
  pub = rospy.Publisher(topic, Image, queue_size=10)

#   while pub.get_num_connections() == 0 and not rospy.is_shutdown():
#     rospy.loginfo_throttle(10.0, "Waiting for subscriber...")
#     rospy.sleep(0.1)

  while not rospy.is_shutdown():
    image = generate_image(f"{seq}", shape=(args.height, args.width))

    stamp = rospy.Time().now()
    msg = image_msg_from(seq, stamp, args.frame, image, bridge)

    pub.publish(msg)
    seq += 1

    rate.sleep()

  rospy.loginfo(f"Fake camera publisher stopped: {args.frame}")


if __name__ == "__main__":
  main()
