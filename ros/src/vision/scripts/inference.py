import os
import cv2
import numpy as np
import tensorflow as tf
import label_map_util
import visualization_utils as vis_util

class detector:
    def __init__(self):
        # Get paths
        TFWD_PATH = os.path.join(os.path.dirname(__file__), 'tf')
        PATH_TO_CKPT = os.path.join(TFWD_PATH,'frozen_inference_graph.pb')
        PATH_TO_LABELS = os.path.join(TFWD_PATH,'annotation.pbtxt')
        
        # get our label map
        label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=3, use_display_name=True)
        self.category_index = label_map_util.create_category_index(categories)

        # Load the Tensorflow model into memory.
        detection_graph = tf.Graph()
        with detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
            # get our graph and save to class
            self.sess = tf.Session(graph=detection_graph)

        # Define input and output tensors for the object detection classifier
        self.it = detection_graph.get_tensor_by_name('image_tensor:0')
        self.db = detection_graph.get_tensor_by_name('detection_boxes:0')
        self.ds = detection_graph.get_tensor_by_name('detection_scores:0')
        self.dc = detection_graph.get_tensor_by_name('detection_classes:0')
        self.nd = detection_graph.get_tensor_by_name('num_detections:0')

    def detect(self, frame):
        frame_expanded = np.expand_dims(frame, axis=0)

        # Perform the actual detection by running the model with the image as input
        (boxes, scores, classes, num) = sess.run(
            [self.db, self.ds, self.dc, self.nd],
            feed_dict={image_tensor: frame_expanded})

        # Draw the results of the detection (aka 'visulaize the results')
        vis_util.visualize_boxes_and_labels_on_image_array(
            frame,
            np.squeeze(boxes),
            np.squeeze(classes).astype(np.int32),
            np.squeeze(scores),
            category_index,
            use_normalized_coordinates=True,
            line_thickness=5,
            min_score_thresh=0.95)

        # publish what we think we saw
        return frame