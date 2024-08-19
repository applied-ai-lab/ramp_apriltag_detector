import rospy

from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion
import yaml
from dataclasses import dataclass
import numpy as np
import math

import tf2_ros


import logging

logger = logging.Logger(__name__)


@dataclass
class NumpyTransform:
    translation: np.array
    rotation: np.array

    def __str__(self) -> str:
        p_str = ", ".join(
            f"p.{c}: {v:.3f}" for c, v in zip(["x", "y", "z"], self.translation)
        )
        q_str = ", ".join(
            f"q.{c}: {v:.3f}" for c, v in zip(["x", "y", "z", "w"], self.rotation)
        )
        return f"{p_str}, {q_str}"


def apply_quat(quat_a, quat_b):
    x0, y0, z0, w0 = quat_a
    x1, y1, z1, w1 = quat_b
    return np.array(
        [
            -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
            x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
            -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
            x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0,
        ],
        dtype=np.float64,
    )


def quat_conj(quat):
    x0, y0, z0, w0 = quat
    return np.array([-x0, -y0, -z0, w0], dtype=np.float64)


def quat_inv(quat):
    return quat_conj(np.real(quat)) / math.sqrt(
        np.real(sum(quat[None, :] @ quat[:, None]))
    )


def quat_diff(p, q):
    """returns pq^-1"""
    # this is the rotation from q to p
    return apply_quat(p, quat_inv(q))


def mean_quaternion(q_list: list):
    # https://ntrs.nasa.gov/api/citations/20070017872/downloads/20070017872.pdf
    # Page 4

    n = len(q_list)
    qq_sum = sum(q[:, None] @ q[None, :] for q in q_list) / n

    eig_val, v = np.linalg.eig(qq_sum)
    max_col = np.argmax(np.real(eig_val))

    q_avg = v[:, max_col]

    return q_avg

def dct_to_NumpyTransform(dct):
    if dct is None:
        return None
    x = dct
    lin = np.array((x['x'], x['y'], x['z']))
    rot = np.array((x['qx'], x['qy'], x['qz'], x['qw']))
    return NumpyTransform(lin, rot)
@dataclass
class Beam:
    name: str
    beam_tags: list  # list[Tag]
    transform: Transform

    def offsets(self):

        if all(t.offset is not None for t in self.beam_tags):
            return {t.name: t.offset
                            for t in self.beam_tags}

    
        

    def calc_origin(self, mean_rot=False) -> NumpyTransform:
        n = len(self.beam_tags)

        transforms = [t.transform for t in self.beam_tags]
        if any(t is None for t in transforms):
            return
        p = np.array(
            [0.0, 0.0, 0.0],
            dtype=np.float64,
        )
        for t in transforms:
            p += t.translation
        p /= n

        if mean_rot:
            q = mean_quaternion([t.rotation for t in transforms])
        else:

            # use the transform of the first tag
            q = self.beam_tags[0].transform.rotation
            # q = np.array([0.0, 0.0, 0.0, 1.0])

        self.transform = NumpyTransform(p, q)

        return self.transform

    def beam_to_tags(self) -> dict:
        """requires calc_origin to be called_first"""
        """returns dictionary of offsets"""
        if self.transform is None:
            return

        return {
            tag.name: NumpyTransform(
                tag.transform.translation - self.transform.translation,
                quat_diff(tag.transform.rotation, self.transform.rotation,),
            )
            for tag in self.beam_tags
        }


@dataclass
class Tag:
    name: str
    transform: NumpyTransform
    parent: Beam = None
    offset: NumpyTransform = None


            

class Beamtracker:
    def __init__(self) -> None:
        import os

        self.path = os.path.join(
            "/".join(__file__.split("/")[:-1]), "../../config/beams.yaml"
        )
        self.beam_config = yaml.safe_load(open(self.path))

        self.beams = {}
        self.beam_tags = {}

        for b in self.beam_config["beams"]:
            beam_obj = Beam(b["name"], None, None)
            tags = [(t["name"], Tag(t["name"], None, beam_obj, offset=dct_to_NumpyTransform(t.get('offset', None)))) for t in b["tags"]]
            beam_obj.beam_tags = list(zip(*tags))[1]

            self.beams[b["name"]] = beam_obj
            for t, T in tags:
                self.beam_tags[t] = T

        self.link_tags = {
            link["name"]: Tag(link["name"], None) for link in self.beam_config["links"]
        }

    def tags(self):
        return tuple(list(self.beam_tags.keys()) + list(self.link_tags.keys()))


def tf_to_NumpyTransform(tf):
    p = tf.translation
    q = tf.rotation
    return NumpyTransform(np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w]))

def NumpyTransform_to_tf(npt:NumpyTransform):
    tf = Transform()
    tf.translation = Vector3(*npt.translation)
    tf.rotation = Quaternion(*npt.rotation)
    return tf


# def tf_callback(data: TfMessage, beam_tracker: Beamtracker):

#     tags_names = beam_tracker.tags()

#     frames = (t for t in data.transforms() if t.child_frame_id in tags_names)

#     for tf in frames:
#         process_tf(tf)

def NumpyTransform_to_dct(tf: NumpyTransform) -> dict:
    keys = ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')
    return {k:round(float(v), 4) for k,v in zip(keys, (*tf.translation, *tf.rotation))}


def configure_beam(data: TransformStamped, beam_tracker: Beamtracker):
    tf = data.transform
    tag_id = data.child_frame_id

    is_link = bool(tag_id in beam_tracker.link_tags)
    # store position
    # if both positions exist publish relative transform
    # filter with Finite MA(?)

    if is_link:
        beam_tracker.link_tags[tag_id].transform = tf_to_NumpyTransform(tf)
        return

    tag = beam_tracker.beam_tags[tag_id]
    tag.transform = tf_to_NumpyTransform(tf)

    beam:Beam = tag.parent
    res = beam.calc_origin()
    if res is not None:
        offsets = beam.beam_to_tags()
        overwrite_beamconfig(beam_tracker.path, beam.name,  offsets)
        offsets = {t: str(v) for t, v in offsets.items()}
        logging.info(f"{beam.name}: {offsets}")

def get_beamposition(data: TransformStamped, beam_tracker: Beamtracker,
                    #   tf_broadcaster: tf2_ros.TransformBroadcaster
                      ):
    tf = data.transform
    tag_id = data.child_frame_id

    is_link = bool(tag_id in beam_tracker.link_tags)
    # store position
    # if both positions exist publish relative transform
    # filter with Finite MA(?)

    if is_link:
        beam_tracker.link_tags[tag_id].transform = tf_to_NumpyTransform(tf)
        return

    tag = beam_tracker.beam_tags[tag_id]
    
    
    tag_world = tf_to_NumpyTransform(tf)

    beam: Beam = tag.parent
    

    beam_tfstamped = TransformStamped()
    beam_tfstamped.header.stamp = rospy.Time.now()
    beam_tfstamped.header.frame_id = data.header.frame_id
    beam_tfstamped.child_frame_id = beam.name

    beam_to_tag = beam.offsets()[tag_id]

    # invert the local offset
    beam_pose = NumpyTransform(
                tag_world.translation + beam_to_tag.translation,
                apply_quat(tag_world.rotation, beam_to_tag.rotation),
                # np.array([0.0, 0.0, 0.0, 1.0])
                    )

    beam_tfstamped.transform = NumpyTransform_to_tf(beam_pose)

    
    
    return beam_tfstamped


def overwrite_beamconfig(config_file, beam_name, offsets):
    with open(config_file) as f:
        beams_dct = yaml.safe_load(f)
    beam_list:list = beams_dct.get('beams')
    beam_index = [beam.get('name') for beam in beam_list].index(beam_name)

    
    for tag_dct in beam_list[beam_index].get('tags'):
        for k,v in offsets.items():
            if k == tag_dct.get('name'):
                tag_dct['offset'] = NumpyTransform_to_dct(v)

    with open(config_file, 'w') as f:
        yaml.safe_dump(beams_dct, f)

def listen_to(beam_tracker, mode:str= 'detect'):
    tags_names = beam_tracker.tags()

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    if mode == 'detect':
        br = tf2_ros.TransformBroadcaster()

    while not rospy.is_shutdown():
        rate = rospy.Rate(30.0)
        try:
            transforms = (
                tf_buffer.lookup_transform("camera_link", tf_id, rospy.Time())
                for tf_id in tags_names
            )
            for tf in transforms:
                if mode == 'detect':
                    tf_to_publish = get_beamposition(tf, beam_tracker)
                    if tf_to_publish is not None:
                        # 
                        
                        br.sendTransform(tf_to_publish)

                elif mode == 'config':
                    configure_beam(tf, beam_tracker)

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("beam_perception")
    # rospy.subscriber('/tag_detections')

    beam_tracker = Beamtracker()
    listen_to(beam_tracker, mode='config')

    # rospy.Subscriber("tf", TfMessage, callback=tf_callback, callback_args=beam_tracker)
    # rospy.spin()
    # while True:
    #     import time
    #     time.sleep(60.)
