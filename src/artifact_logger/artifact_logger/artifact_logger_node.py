#!/usr/bin/env python3
"""
Artifact Logger (with temporal + spatial voting)

Subscribes to /detection_info (object_detection_msgs/ObjectDetectionInfoArray),
transforms each detection's position from the camera frame into the `map` frame
(anchored at the robot's start = (0,0,0), so map coords are start-relative), and
groups detections into spatial "tracks". A track is only written to the CSV once
it has been seen enough times (min_sightings) and has a clear dominant class; the
logged name is the MAJORITY class across all sightings, not the first one.

Why voting: YOLO on low-res, untextured sim meshes frequently misclassifies a
single frame (e.g. a backpack flashing up as "umbrella"). A one-frame fluke must
not land in the CSV. Requiring N sightings + a majority class filters transient
false positives and self-corrects the label.

CSV columns:  name,x,y,z

Run (inside container, with sim + a2 detect running):
  a2 source
  ros2 run artifact_logger artifact_logger_node
  # or with options:
  ros2 run artifact_logger artifact_logger_node --ros-args \
      -p min_confidence:=0.6 -p min_sightings:=5 -p dedup_radius:=1.0
"""

import csv
import math

import rclpy
from rclpy.node import Node

from object_detection_msgs.msg import ObjectDetectionInfoArray
from geometry_msgs.msg import PointStamped

import tf2_ros
from tf2_geometry_msgs import do_transform_point


class Track:
    """A spatial cluster of detections that may be the same physical object."""
    __slots__ = ('x', 'y', 'z', 'n', 'votes')

    def __init__(self, x, y, z, name):
        self.x = x
        self.y = y
        self.z = z
        self.n = 1               # total sightings
        self.votes = {name: 1}   # class -> count

    def update(self, x, y, z, name):
        # running mean of position so the logged point is stable, not the first hit
        self.n += 1
        a = 1.0 / self.n
        self.x += (x - self.x) * a
        self.y += (y - self.y) * a
        self.z += (z - self.z) * a
        self.votes[name] = self.votes.get(name, 0) + 1

    def majority(self):
        """(name, count) of the most-voted class."""
        return max(self.votes.items(), key=lambda kv: kv[1])

    def dominant_fraction(self):
        return self.majority()[1] / self.n


class ArtifactLogger(Node):
    def __init__(self):
        super().__init__('artifact_logger')

        # --- parameters ---
        self.declare_parameter('csv_path', '/a2_ros_ws/bags/artifacts.csv')
        self.declare_parameter('target_frame', 'map')      # frame to log in (start-relative)
        self.declare_parameter('min_confidence', 0.6)      # ignore weak detections
        self.declare_parameter('dedup_radius', 1.0)        # detections within this dist = same track (m)
        self.declare_parameter('max_range', 3.0)           # ignore detections farther than this from cam (m)
        self.declare_parameter('min_sightings', 5)         # sightings before a track may be logged
        self.declare_parameter('min_dominant', 0.5)        # majority class must be >= this fraction of votes
        self.declare_parameter('detection_topic', '/detection_info')

        self.csv_path = self.get_parameter('csv_path').value
        self.target_frame = self.get_parameter('target_frame').value
        self.min_conf = self.get_parameter('min_confidence').value
        self.dedup_radius = self.get_parameter('dedup_radius').value
        self.max_range = self.get_parameter('max_range').value
        self.min_sightings = self.get_parameter('min_sightings').value
        self.min_dominant = self.get_parameter('min_dominant').value
        topic = self.get_parameter('detection_topic').value

        # --- state: list of Track ---
        self.tracks = []

        # --- TF ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- io ---
        self.create_subscription(ObjectDetectionInfoArray, topic, self.detection_cb, 10)
        self._write_csv()   # start with a header

        self.get_logger().info(
            f'artifact_logger up | sub {topic} -> map-frame CSV at {self.csv_path} '
            f'| min_conf={self.min_conf} dedup_radius={self.dedup_radius}m '
            f'max_range={self.max_range}m min_sightings={self.min_sightings} '
            f'min_dominant={self.min_dominant}')

    # ---------- detection handling ----------
    def detection_cb(self, msg):
        for det in msg.info:
            if det.confidence < self.min_conf:
                continue

            # reject far detections — bbox->lidar depth is unreliable at distance
            rng = math.sqrt(det.position.x**2 + det.position.y**2 + det.position.z**2)
            if rng > self.max_range:
                continue

            p_cam = PointStamped()
            p_cam.header.frame_id = msg.header.frame_id
            p_cam.header.stamp = msg.header.stamp
            p_cam.point.x = det.position.x
            p_cam.point.y = det.position.y
            p_cam.point.z = det.position.z

            # camera -> map, using the detection's OWN timestamp (rotating robot
            # otherwise puts a stationary object at different map coords).
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.target_frame, p_cam.header.frame_id,
                    rclpy.time.Time.from_msg(msg.header.stamp),
                    timeout=rclpy.duration.Duration(seconds=0.2))
                p_map = do_transform_point(p_cam, tf)
            except tf2_ros.TransformException:
                try:
                    tf = self.tf_buffer.lookup_transform(
                        self.target_frame, p_cam.header.frame_id, rclpy.time.Time())
                    p_map = do_transform_point(p_cam, tf)
                except tf2_ros.TransformException as e:
                    self.get_logger().warn(
                        f'TF {p_cam.header.frame_id}->{self.target_frame} failed: {e}',
                        throttle_duration_sec=2.0)
                    continue

            self.add_detection(det.class_id, p_map.point.x, p_map.point.y, p_map.point.z)

    def add_detection(self, name, x, y, z):
        """Assign a detection to the nearest track (any class) or start a new one."""
        best, best_d = None, self.dedup_radius
        for t in self.tracks:
            d = math.dist((x, y, z), (t.x, t.y, t.z))
            if d < best_d:
                best, best_d = t, d

        if best is None:
            self.tracks.append(Track(x, y, z, name))
        else:
            before = self._confirmed_name(best)
            best.update(x, y, z, name)
            after = self._confirmed_name(best)
            # rewrite only when the confirmed set/label actually changes
            if after is not None and after != before:
                top, cnt = best.majority()
                self.get_logger().info(
                    f'CONFIRMED: {top} @ map ({best.x:.2f}, {best.y:.2f}, {best.z:.2f}) '
                    f'[{cnt}/{best.n} votes, classes={dict(best.votes)}]')
                self._write_csv()

    def _confirmed_name(self, track):
        """Logged name for a track, or None if it has not met the confirmation bar."""
        if track.n < self.min_sightings:
            return None
        if track.dominant_fraction() < self.min_dominant:
            return None
        return track.majority()[0]

    def _write_csv(self):
        rows = []
        for t in self.tracks:
            name = self._confirmed_name(t)
            if name is not None:
                rows.append((name, t.x, t.y, t.z))
        try:
            with open(self.csv_path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['name', 'x', 'y', 'z'])
                for (n, x, y, z) in rows:
                    w.writerow([n, f'{x:.4f}', f'{y:.4f}', f'{z:.4f}'])
        except OSError as e:
            self.get_logger().error(f'cannot write CSV {self.csv_path}: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ArtifactLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        confirmed = sum(1 for t in node.tracks if node._confirmed_name(t) is not None)
        node.get_logger().info(
            f'shutting down — {confirmed} confirmed artifacts '
            f'({len(node.tracks)} raw tracks) in {node.csv_path}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
