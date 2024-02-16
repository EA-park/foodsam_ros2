#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from foodsam_msgs.msg import BBox as BBoxMsg
from foodsam_msgs.msg import BBoxList as BBoxListMsg
from sensor_msgs.msg import Image
from std_msgs.msg import String

import datetime

import cv2
import numpy as np
from cv_bridge import CvBridge
from segment_anything import SamAutomaticMaskGenerator, sam_model_registry

import sys
sys.path.append("/home/koras/FoodSAM")
from mmseg.apis.inference import init_segmentor, inference_segmentor
from FoodSAM.mmcv_deprecated.runner.checkpoint import load_checkpoint
from FoodSAM.semantic import parser, create_logger, get_amg_kwargs
from FoodSAM.FoodSAM_tools.predict_semantic_mask import save_result

bridge = CvBridge()
args = parser.parse_args()

def categorize(category_txt):
    category_lists = []
    with open(category_txt, 'r') as f:
        category_lines = f.readlines()
        category_lists = [(' '.join(line_data.split('\t')[1:])).strip() for line_data in category_lines]
        f.close()
    return category_lists


class Subscriber(Node):

    def __init__(self):
        super().__init__('pubsub')
        qos_profile = QoSProfile(depth=10)
        self.publisher = self.create_publisher(BBoxListMsg, 'detection', 10)
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_rect_raw',
            self.listener_callback,
            qos_profile)

        self.logger = create_logger(args.output)
        self.categories = categorize(args.category_txt)

        sam = sam_model_registry[args.model_type](checkpoint=args.SAM_checkpoint)
        _ = sam.to(device=args.device)        
        self.generator = SamAutomaticMaskGenerator(
            sam,            
            output_mode="binary_mask",
            **get_amg_kwargs(args)
            )
        
        checkpoint_ = args.semantic_checkpoint
        self.segmentor = init_segmentor(args.semantic_config, checkpoint_)
        checkpoint_ = load_checkpoint(self.segmentor, checkpoint_, map_location='cpu')

        
    def listener_callback(self, data):
        msg = self.predict(data)
        self.publisher.publish(msg)


    def predict(self, data, img_path=None):
        """
        Inference and ROS2 msgs
        """
        np_img = bridge.imgmsg_to_cv2(data, "bgr8")
        image = cv2.cvtColor(np_img, cv2.COLOR_BGR2RGB)

        # SAM
        self.logger.info("running sam!")
        masks = self.generator.generate(image)        
        self.logger.info("sam done!\n")

        # Semantic seg model
        self.logger.info("running semantic seg model!")
        result = inference_segmentor(self.segmentor, np_img)

        time_ = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        save_result(        # Saving debugging images
            image,
            result,
            color_list_path=args.color_list_path,
            show=False,
            out_file=args.output,
            vis_save_name=f"pred_vis_{time_}.png",
            mask_save_name=f"pred_mask_{time_}.png"
            )
        self.logger.info("semantic predict done!\n")

        # Enhance semantic masks
        self.logger.info("enhance semantic masks")
        sam_mask_data = masks
        pred_mask_img = result[0]
        msg_array = BBoxListMsg()
        
        for i in range(len(sam_mask_data)):
            single_mask = sam_mask_data[i]
            single_mask_labels = pred_mask_img[single_mask['segmentation']]
            unique_values, counts = np.unique(single_mask_labels, return_counts=True, axis=0)
            max_idx = np.argmax(counts)
            single_mask_category_label = unique_values[max_idx]
            class_name = self.categories[single_mask_category_label]

            # ROS2 msg
            msg = BBoxMsg()

            msg_box_id = String()
            msg_box_id.data = str(i)
            msg.box_id = msg_box_id

            msg_class_label = String()            
            msg_class_label.data = str(class_name)
            msg.class_label = msg_class_label
            
            msg.x0, msg.y0, msg.w, msg.h = single_mask['bbox']
            msg_array.bboxes.append(msg)

        self.logger.info("enhance semantic masks done!\n")
        return msg_array
            

def main(args=None):
    rclpy.init(args=args)
    subscriber = Subscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
