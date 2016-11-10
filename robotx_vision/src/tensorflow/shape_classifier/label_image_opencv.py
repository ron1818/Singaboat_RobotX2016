#! /usr/bin/python
""" use tensorflow's inception and retrained classifier to
identify circle, cruciform and triangle,
Ren Ye, 2016-11-02
renye@ntu.edu.sg
"""

import cv2
import tensorflow as tf
import inspect, os, sys

# read the image by opencv
image_names = ["triangle0.jpg", "circle0.jpg", "triangle0.jpg", "cross0.jpg"]
os_path = os.path.dirname(os.path.realpath(__file__))
# image_path = os_path + "/test_photos/" + image_names[0]
# img = cv2.imread(image_path)  # , cv2.IMREAD_GRAYSCALE)
# image = tf.placeholder("uint8", [None, None, 3])

# Read in the image_data
# image_data = tf.gfile.FastGFile(image_path, 'rb').read()

# Loads label file, strips off carriage return
label_lines = [line.rstrip() for line
                   in tf.gfile.GFile(os_path + "/retrained_labels.txt")]

# Unpersists graph from file
with tf.gfile.FastGFile(os_path + "/retrained_graph.pb", 'rb') as f:
    graph_def = tf.GraphDef()
    graph_def.ParseFromString(f.read())
    _ = tf.import_graph_def(graph_def, name='')

with tf.Session() as sess:
    for i in range(len(image_names)):
        image_path = os_path + "/test_photos/" + image_names[i]
        img = cv2.imread(image_path)
        image = tf.placeholder("uint8", [None, None, 3])

        # Feed the image_data as input to the graph and get first prediction
        softmax_tensor = sess.graph.get_tensor_by_name('final_result:0')

        predictions = sess.run(softmax_tensor, feed_dict={image: img})

        # Sort to show labels of first prediction in order of confidence
        top_k = predictions[0].argsort()[-len(predictions[0]):][::-1]

        for node_id in top_k:
            human_string = label_lines[node_id]
            score = predictions[0][node_id]
            print('%s (score = %.5f)' % (human_string, score))
